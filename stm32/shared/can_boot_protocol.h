#ifndef CAN_BOOT_PROTOCOL_H
#define CAN_BOOT_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOOT_ADDR             UINT32_C(0x08000000)
#define BOOT_SIZE             UINT32_C(0x00002000)
#define APP_ADDR              UINT32_C(0x08002000)
#define APP_MAX_SIZE          UINT32_C(0x00005C00)
#define META_ADDR             UINT32_C(0x08007C00)
#define META_SIZE             UINT32_C(0x00000400)
#define STM32_FLASH_PAGE_SIZE UINT32_C(0x00000400)

#define CAN_DEVICE_ID_MASK UINT32_C(0x03FFFFFF)
#define CAN_BOOT_CTRL_BASE UINT32_C(0x0C000000)
#define CAN_BOOT_DATA_BASE UINT32_C(0x10000000)
#define CAN_BOOT_RESP_BASE UINT32_C(0x14000000)

#define CAN_BOOT_PROTOCOL_VERSION UINT8_C(1)
#define CAN_BOOT_METADATA_MAGIC   UINT32_C(0x31505041) /* "APP1" */
#define CAN_BOOT_BKP_DR1_MAGIC    UINT16_C(0xB007)
#define CAN_BOOT_BKP_DR2_MAGIC    UINT16_C(0x10AD)

/* BOOT_ENTER is accepted only with this complete six-byte payload. */
#define CAN_BOOT_ENTER_DLC UINT8_C(6)
#define CAN_BOOT_ENTER_MAGIC0 UINT8_C(0x07)
#define CAN_BOOT_ENTER_MAGIC1 UINT8_C(0xB0)
#define CAN_BOOT_ENTER_MAGIC2 UINT8_C(0xAD)
#define CAN_BOOT_ENTER_MAGIC3 UINT8_C(0x10)

enum can_boot_ctrl_opcode {
    CAN_BOOT_CTRL_ENTER = 0x01,
    CAN_BOOT_CTRL_BEGIN = 0x02,
    CAN_BOOT_CTRL_END = 0x03,
    CAN_BOOT_CTRL_ABORT = 0x04
};

enum can_boot_response_opcode {
    CAN_BOOT_RESP_READY = 0x80,
    CAN_BOOT_RESP_ACK = 0x81,
    CAN_BOOT_RESP_NACK = 0x82,
    CAN_BOOT_RESP_SUCCESS = 0x83,
    CAN_BOOT_RESP_CRC_ERROR = 0x84,
    CAN_BOOT_RESP_ERROR = 0x85
};

enum can_boot_error {
    CAN_BOOT_ERROR_NONE = 0,
    CAN_BOOT_ERROR_FORMAT = 1,
    CAN_BOOT_ERROR_VERSION = 2,
    CAN_BOOT_ERROR_SIZE = 3,
    CAN_BOOT_ERROR_SEQUENCE = 4,
    CAN_BOOT_ERROR_FLASH = 5,
    CAN_BOOT_ERROR_STATE = 6
};

typedef struct {
    uint32_t magic;
    uint32_t image_size;
    uint32_t crc32;
    uint32_t version;
} can_boot_metadata_t;

#ifdef __cplusplus
}
#endif

#endif
