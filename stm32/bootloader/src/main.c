#include "stm32f1xx_hal.h"
#include "can_boot_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UID_ADDRESS UINT32_C(0x1FFFF7E8)
#define RAM_START   UINT32_C(0x20000000)
#define RAM_END     UINT32_C(0x20002800)
#define STARTUP_LISTEN_MS UINT32_C(100)
#define CAN_TX_TIMEOUT_MS UINT32_C(20)

_Static_assert(APP_ADDR == BOOT_ADDR + BOOT_SIZE, "boot/app layout gap");
_Static_assert(META_ADDR == APP_ADDR + APP_MAX_SIZE, "app/meta layout gap");
_Static_assert(META_ADDR + META_SIZE == BOOT_ADDR + UINT32_C(0x8000),
               "layout must exactly fit STM32F103C6 Flash");
_Static_assert(sizeof(can_boot_metadata_t) == 16U, "metadata ABI changed");

typedef struct {
    bool active;
    uint32_t image_size;
    uint32_t expected_crc32;
    uint32_t received_bytes;
    uint16_t expected_sequence;
} update_state_t;

static CAN_HandleTypeDef hcan;
static uint32_t device_id;
static uint32_t boot_ctrl_id;
static uint32_t boot_data_id;
static uint32_t boot_resp_id;
static update_state_t update_state;

static void drive_disable_early(void);
static bool system_clock_config(void);
static bool can_init(void);
static bool boot_request_take(void);
static bool application_valid(void);
static bool process_pending_frame(bool *stay_in_bootloader);
static void jump_to_app(void) __attribute__((noreturn, noinline));
static void fatal_stop(void) __attribute__((noreturn));

/**
 * @brief Bootloader entry point.
 * @return Never returns.
 */
int main(void)
{
    bool stay_in_bootloader;

    /* This is intentionally before HAL_Init and clock switching. */
    drive_disable_early();
    if (HAL_Init() != HAL_OK || !system_clock_config()) {
        fatal_stop();
    }
    drive_disable_early();

    stay_in_bootloader = boot_request_take();
    if (!application_valid()) {
        stay_in_bootloader = true;
    }

    if (!can_init()) {
        fatal_stop();
    }

    if (!stay_in_bootloader) {
        const uint32_t started_at = HAL_GetTick();
        while ((HAL_GetTick() - started_at) < STARTUP_LISTEN_MS) {
            if (!process_pending_frame(&stay_in_bootloader)) {
                fatal_stop();
            }
            if (stay_in_bootloader) {
                break;
            }
        }
        if (!stay_in_bootloader) {
            jump_to_app();
        }
    }

    for (;;) {
        if (!process_pending_frame(&stay_in_bootloader)) {
            fatal_stop();
        }
    }
}

/** Disable the motor driver using reset-default HSI timing. */
static void drive_disable_early(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIOA->BRR = GPIO_PIN_3;
    GPIOA->CRL = (GPIOA->CRL & ~(UINT32_C(0xF) << 12)) |
                 (UINT32_C(0x2) << 12); /* Output push-pull, 2 MHz. */
    GPIOA->BRR = GPIO_PIN_3;
}

/**
 * @brief Configure the same 64 MHz SYSCLK and 32 MHz PCLK1 as the application.
 * @return true on success, otherwise false.
 */
static bool system_clock_config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    oscillator.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        return false;
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) == HAL_OK;
}

/**
 * @brief Generate the same non-zero 26-bit FNV-1a ID as the application.
 * @return The device-specific CAN identifier suffix.
 */
static uint32_t make_device_id(void)
{
    const volatile uint8_t *uid = (const volatile uint8_t *)UID_ADDRESS;
    uint32_t hash = UINT32_C(2166136261);
    uint8_t index;

    for (index = 0; index < 12U; ++index) {
        hash ^= uid[index];
        hash *= UINT32_C(16777619);
    }
    hash &= CAN_DEVICE_ID_MASK;
    return (hash == 0U) ? 1U : hash;
}

/**
 * @brief Configure one exact extended-data-frame filter.
 * @return true on HAL success, otherwise false.
 */
static bool configure_filter(uint32_t bank, uint32_t id)
{
    CAN_FilterTypeDef filter = {0};
    const uint32_t encoded_id = ((id & UINT32_C(0x1FFFFFFF)) << 3) |
                                (UINT32_C(1) << 2);
    const uint32_t encoded_mask = (UINT32_C(0x1FFFFFFF) << 3) |
                                  (UINT32_C(1) << 2) |
                                  (UINT32_C(1) << 1);

    filter.FilterBank = bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(encoded_id >> 16);
    filter.FilterIdLow = (uint16_t)encoded_id;
    filter.FilterMaskIdHigh = (uint16_t)(encoded_mask >> 16);
    filter.FilterMaskIdLow = (uint16_t)encoded_mask;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    return HAL_CAN_ConfigFilter(&hcan, &filter) == HAL_OK;
}

/**
 * @brief Initialize CAN1 PA11/PA12 for polling at 1 Mbps.
 * @return true when CAN and both exact filters started successfully.
 */
static bool can_init(void)
{
    device_id = make_device_id();
    boot_ctrl_id = CAN_BOOT_CTRL_BASE | device_id;
    boot_data_id = CAN_BOOT_DATA_BASE | device_id;
    boot_resp_id = CAN_BOOT_RESP_BASE | device_id;

    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 2;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = ENABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK ||
        !configure_filter(0U, boot_ctrl_id) ||
        !configure_filter(1U, boot_data_id)) {
        return false;
    }
    return HAL_CAN_Start(&hcan) == HAL_OK;
}

/**
 * @brief Read and atomically consume the backup-register boot request.
 * @return true only when both request words match.
 */
static bool boot_request_take(void)
{
    bool requested;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    requested = ((uint16_t)BKP->DR1 == CAN_BOOT_BKP_DR1_MAGIC) &&
                ((uint16_t)BKP->DR2 == CAN_BOOT_BKP_DR2_MAGIC);
    BKP->DR1 = 0U;
    BKP->DR2 = 0U;
    HAL_PWR_DisableBkUpAccess();
    return requested;
}

/**
 * @brief Calculate CRC-32/IEEE in the zlib-compatible reflected form.
 * @return CRC32 of exactly size bytes.
 */
static uint32_t crc32_ieee(const uint8_t *data, uint32_t size)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    uint32_t index;

    for (index = 0; index < size; ++index) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0; bit < 8U; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return crc ^ UINT32_C(0xFFFFFFFF);
}

/**
 * @brief Validate metadata, vectors, and the complete application CRC.
 * @return true only for a fully committed application image.
 */
static bool application_valid(void)
{
    const can_boot_metadata_t *metadata =
        (const can_boot_metadata_t *)META_ADDR;
    const uint32_t initial_msp = *(const uint32_t *)APP_ADDR;
    const uint32_t reset_handler = *(const uint32_t *)(APP_ADDR + 4U);
    const uint32_t reset_address = reset_handler & ~UINT32_C(1);

    if (metadata->magic != CAN_BOOT_METADATA_MAGIC ||
        metadata->image_size == 0U || metadata->image_size > APP_MAX_SIZE) {
        return false;
    }
    if (initial_msp < RAM_START || initial_msp > RAM_END ||
        (initial_msp & 3U) != 0U) {
        return false;
    }
    if ((reset_handler & 1U) == 0U ||
        reset_address < APP_ADDR || reset_address >= META_ADDR) {
        return false;
    }
    return crc32_ieee((const uint8_t *)APP_ADDR, metadata->image_size) ==
           metadata->crc32;
}

/**
 * @brief Send one boot response and wait until its mailbox is released.
 * @return true after successful transmission, otherwise false.
 */
static bool send_response(const uint8_t *data, uint8_t size)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t padded_data[8] = {0};
    uint32_t mailbox;
    uint32_t started_at;
    uint32_t tx_ok_mask;
    uint8_t index;

    header.ExtId = boot_resp_id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = size;
    header.TransmitGlobalTime = DISABLE;
    if (size > 8U) {
        return false;
    }
    for (index = 0U; index < size; ++index) {
        padded_data[index] = data[index];
    }
    /* HAL_CAN_AddTxMessage reads all eight payload bytes regardless of DLC. */
    WRITE_REG(hcan.Instance->TSR, CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2);
    if (HAL_CAN_AddTxMessage(&hcan, &header, padded_data, &mailbox) != HAL_OK) {
        return false;
    }
    tx_ok_mask = (mailbox == CAN_TX_MAILBOX0) ? CAN_TSR_TXOK0 :
                 (mailbox == CAN_TX_MAILBOX1) ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2;

    started_at = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan, mailbox) != 0U) {
        if ((HAL_GetTick() - started_at) >= CAN_TX_TIMEOUT_MS) {
            (void)HAL_CAN_AbortTxRequest(&hcan, mailbox);
            return false;
        }
    }
    return (READ_REG(hcan.Instance->TSR) & tx_ok_mask) != 0U;
}

/**
 * @brief Send a two-byte status response.
 * @return true on successful transmission.
 */
static bool send_status(uint8_t opcode, uint8_t detail)
{
    const uint8_t response[2] = {opcode, detail};
    return send_response(response, sizeof(response));
}

/**
 * @brief Erase metadata first, then only the application pages required.
 * @return true when every erase operation succeeds.
 */
static bool erase_update_regions(uint32_t image_size)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    HAL_StatusTypeDef result;

    if (image_size == 0U || image_size > APP_MAX_SIZE) {
        return false;
    }
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = META_ADDR;
    erase.NbPages = 1U;
    result = HAL_FLASHEx_Erase(&erase, &page_error);
    if (result == HAL_OK) {
        erase.PageAddress = APP_ADDR;
        erase.NbPages = (image_size + STM32_FLASH_PAGE_SIZE - 1U) /
                        STM32_FLASH_PAGE_SIZE;
        result = HAL_FLASHEx_Erase(&erase, &page_error);
    }
    (void)HAL_FLASH_Lock();
    return result == HAL_OK;
}

/**
 * @brief Program and read back a byte chunk using F1 half-word writes.
 * @return true if programming and verification both succeed.
 */
static bool program_chunk(uint32_t address, const uint8_t *data, uint8_t size)
{
    uint8_t offset;
    bool success = true;

    if ((address & 1U) != 0U || size == 0U || size > 6U ||
        address < APP_ADDR || address + size > APP_ADDR + APP_MAX_SIZE) {
        return false;
    }
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    for (offset = 0U; offset < size; offset = (uint8_t)(offset + 2U)) {
        uint16_t halfword = data[offset];
        if ((uint8_t)(offset + 1U) < size) {
            halfword |= (uint16_t)data[offset + 1U] << 8;
        } else {
            halfword |= UINT16_C(0xFF00);
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              address + offset, halfword) != HAL_OK ||
            *(const uint16_t *)(address + offset) != halfword) {
            success = false;
            break;
        }
    }
    if (HAL_FLASH_Lock() != HAL_OK) {
        success = false;
    }
    return success;
}

/**
 * @brief Program a 32-bit metadata field as two verified half-words.
 * @return true when both half-words were committed correctly.
 */
static bool program_u32(uint32_t address, uint32_t value)
{
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address,
                          (uint16_t)value) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2U,
                          (uint16_t)(value >> 16)) != HAL_OK) {
        return false;
    }
    return *(const uint32_t *)address == value;
}

/**
 * @brief Commit metadata fields, with the validity magic deliberately last.
 * @return true only if the complete record verifies after locking Flash.
 */
static bool commit_metadata(uint32_t size, uint32_t crc32, uint32_t version)
{
    const can_boot_metadata_t *metadata =
        (const can_boot_metadata_t *)META_ADDR;
    bool success;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    success = program_u32(META_ADDR + offsetof(can_boot_metadata_t, image_size), size) &&
              program_u32(META_ADDR + offsetof(can_boot_metadata_t, crc32), crc32) &&
              program_u32(META_ADDR + offsetof(can_boot_metadata_t, version), version) &&
              program_u32(META_ADDR + offsetof(can_boot_metadata_t, magic),
                          CAN_BOOT_METADATA_MAGIC);
    if (HAL_FLASH_Lock() != HAL_OK) {
        success = false;
    }
    return success && metadata->magic == CAN_BOOT_METADATA_MAGIC &&
           metadata->image_size == size && metadata->crc32 == crc32 &&
           metadata->version == version;
}

/**
 * @brief Decode a little-endian uint16.
 * @return Decoded value.
 */
static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief Decode a little-endian uint32.
 * @return Decoded value.
 */
static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

/**
 * @brief Process BEGIN, END, ABORT, or ENTER.
 * @return true unless an essential response transmission failed.
 */
static bool process_control(const uint8_t *data, uint8_t size,
                            bool *stay_in_bootloader)
{
    if (size == CAN_BOOT_ENTER_DLC && data[0] == CAN_BOOT_CTRL_ENTER &&
        data[1] == CAN_BOOT_PROTOCOL_VERSION &&
        data[2] == CAN_BOOT_ENTER_MAGIC0 && data[3] == CAN_BOOT_ENTER_MAGIC1 &&
        data[4] == CAN_BOOT_ENTER_MAGIC2 && data[5] == CAN_BOOT_ENTER_MAGIC3) {
        *stay_in_bootloader = true;
        return true;
    }

    if (size == 8U && data[0] == CAN_BOOT_CTRL_BEGIN) {
        const uint32_t image_size = read_u16(&data[2]);
        const uint32_t expected_crc32 = read_u32(&data[4]);

        *stay_in_bootloader = true;
        if (data[1] != CAN_BOOT_PROTOCOL_VERSION) {
            return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_VERSION);
        }
        if (image_size == 0U || image_size > APP_MAX_SIZE) {
            return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_SIZE);
        }
        if (update_state.active && update_state.received_bytes == 0U &&
            update_state.image_size == image_size &&
            update_state.expected_crc32 == expected_crc32) {
            return send_status(CAN_BOOT_RESP_READY, CAN_BOOT_PROTOCOL_VERSION);
        }
        update_state.active = false;
        if (!erase_update_regions(image_size)) {
            return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_FLASH);
        }
        update_state.image_size = image_size;
        update_state.expected_crc32 = expected_crc32;
        update_state.received_bytes = 0U;
        update_state.expected_sequence = 0U;
        update_state.active = true;
        return send_status(CAN_BOOT_RESP_READY, CAN_BOOT_PROTOCOL_VERSION);
    }

    if (size == 1U && data[0] == CAN_BOOT_CTRL_ABORT) {
        update_state.active = false; /* Metadata was erased by BEGIN and stays invalid. */
        return send_status(CAN_BOOT_RESP_ACK, CAN_BOOT_CTRL_ABORT);
    }

    if (size == 5U && data[0] == CAN_BOOT_CTRL_END) {
        uint32_t actual_crc;
        uint32_t version;
        uint8_t response[5];

        if (!update_state.active ||
            update_state.received_bytes != update_state.image_size) {
            return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_STATE);
        }
        version = read_u32(&data[1]);
        actual_crc = crc32_ieee((const uint8_t *)APP_ADDR,
                                update_state.image_size);
        if (actual_crc != update_state.expected_crc32) {
            response[0] = CAN_BOOT_RESP_CRC_ERROR;
            response[1] = (uint8_t)actual_crc;
            response[2] = (uint8_t)(actual_crc >> 8);
            response[3] = (uint8_t)(actual_crc >> 16);
            response[4] = (uint8_t)(actual_crc >> 24);
            update_state.active = false;
            return send_response(response, sizeof(response));
        }
        if (!commit_metadata(update_state.image_size, actual_crc, version)) {
            update_state.active = false;
            return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_FLASH);
        }
        update_state.active = false;
        if (!send_status(CAN_BOOT_RESP_SUCCESS, CAN_BOOT_ERROR_NONE)) {
            return false;
        }
        HAL_Delay(50U);
        NVIC_SystemReset();
    }

    return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_FORMAT);
}

/**
 * @brief Process one stop-and-wait DATA frame.
 * @return true unless an essential response transmission failed.
 */
static bool process_data(const uint8_t *data, uint8_t size)
{
    uint16_t sequence;
    uint8_t payload_size;
    uint8_t expected_payload_size;
    uint8_t response[4];

    if (!update_state.active || size < 3U || size > 8U) {
        return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_STATE);
    }
    sequence = read_u16(data);
    if (update_state.expected_sequence != 0U &&
        sequence == (uint16_t)(update_state.expected_sequence - 1U)) {
        response[0] = CAN_BOOT_RESP_ACK;
        response[1] = (uint8_t)sequence;
        response[2] = (uint8_t)(sequence >> 8);
        return send_response(response, 3U);
    }
    if (sequence != update_state.expected_sequence) {
        response[0] = CAN_BOOT_RESP_NACK;
        response[1] = CAN_BOOT_ERROR_SEQUENCE;
        response[2] = (uint8_t)update_state.expected_sequence;
        response[3] = (uint8_t)(update_state.expected_sequence >> 8);
        return send_response(response, sizeof(response));
    }

    payload_size = (uint8_t)(size - 2U);
    expected_payload_size =
        (update_state.image_size - update_state.received_bytes > 6U)
            ? 6U
            : (uint8_t)(update_state.image_size - update_state.received_bytes);
    if (payload_size != expected_payload_size || expected_payload_size == 0U) {
        return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_FORMAT);
    }
    if (!program_chunk(APP_ADDR + update_state.received_bytes,
                       &data[2], payload_size)) {
        update_state.active = false;
        return send_status(CAN_BOOT_RESP_ERROR, CAN_BOOT_ERROR_FLASH);
    }

    update_state.received_bytes += payload_size;
    ++update_state.expected_sequence;
    response[0] = CAN_BOOT_RESP_ACK;
    response[1] = (uint8_t)sequence;
    response[2] = (uint8_t)(sequence >> 8);
    return send_response(response, 3U);
}

/**
 * @brief Poll and dispatch at most one accepted CAN frame.
 * @return true on no frame or successful handling, otherwise false.
 */
static bool process_pending_frame(bool *stay_in_bootloader)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0U) {
        return true;
    }
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
        return false;
    }
    if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA) {
        return true;
    }
    if (header.ExtId == boot_ctrl_id) {
        return process_control(data, (uint8_t)header.DLC, stay_in_bootloader);
    }
    if (header.ExtId == boot_data_id) {
        *stay_in_bootloader = true;
        return process_data(data, (uint8_t)header.DLC);
    }
    return true;
}

/** Transfer control without relying on any bootloader interrupt or stack state. */
static void jump_to_app(void)
{
    const uint32_t app_msp = *(const uint32_t *)APP_ADDR;
    const uint32_t app_reset = *(const uint32_t *)(APP_ADDR + 4U);
    uint32_t index;

    if (HAL_CAN_Stop(&hcan) != HAL_OK) {
        fatal_stop();
    }
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    __disable_irq();
    for (index = 0U; index < 8U; ++index) {
        NVIC->ICER[index] = UINT32_C(0xFFFFFFFF);
        NVIC->ICPR[index] = UINT32_C(0xFFFFFFFF);
    }
    SCB->VTOR = APP_ADDR;
    __set_CONTROL(0U);
    __DSB();
    __ISB();
    __set_MSP(app_msp);
    __asm volatile ("bx %0" : : "r" (app_reset) : "memory");
    __builtin_unreachable();
}

/** Stop safely when initialization or CAN response handling cannot continue. */
static void fatal_stop(void)
{
    drive_disable_early();
    __disable_irq();
    for (;;) {
        __WFI();
    }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *handle)
{
    GPIO_InitTypeDef gpio = {0};

    if (handle->Instance != CAN1) {
        return;
    }
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *handle)
{
    if (handle->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
