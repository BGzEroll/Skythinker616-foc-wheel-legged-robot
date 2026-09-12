#include "start.h"
#include "task.h"
#include "led_dev.h"
#include "bus/uart_bus.h"
#include "bus/can_bus.h"

#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uart_bus uart0(0);
static can_bus can0(0);
static WebServer web_server(80);

namespace
{
    // 与 STM32 main 分支 can_comm 的扩展 ID 布局保持一致。
    constexpr uint32_t can_discover_id = 0x700;
    constexpr uint32_t can_feedback_base = 0x04000000;
    constexpr uint32_t can_response_base = 0x08000000;
    constexpr uint32_t can_id_type_mask = 0x1C000000;
    constexpr uint32_t can_device_id_mask = 0x03FFFFFF;

    // STM32 CAN Bootloader 协议（与 stm32/shared/can_boot_protocol.h 对齐）。
    constexpr uint32_t can_boot_ctrl_base = 0x0C000000;
    constexpr uint32_t can_boot_data_base = 0x10000000;
    constexpr uint32_t can_boot_response_base = 0x14000000;
    constexpr uint32_t can_boot_type_mask = 0x1C000000;
    constexpr uint32_t can_boot_app_max_size = 0x00005C00;
    constexpr uint8_t can_boot_protocol_version = 1;
    constexpr uint8_t can_boot_ctrl_enter = 0x01;
    constexpr uint8_t can_boot_ctrl_begin = 0x02;
    constexpr uint8_t can_boot_ctrl_end = 0x03;
    constexpr uint8_t can_boot_ctrl_abort = 0x04;
    constexpr uint8_t can_boot_resp_ready = 0x80;
    constexpr uint8_t can_boot_resp_ack = 0x81;
    constexpr uint8_t can_boot_resp_success = 0x83;
    constexpr uint8_t can_boot_resp_crc_error = 0x84;
    constexpr uint8_t can_boot_resp_error = 0x85;
    constexpr uint32_t can_boot_response_timeout_ms = 500;
    constexpr uint8_t can_boot_exchange_retries = 5;

    constexpr uint32_t can_discovery_period_ms = 500;
    constexpr float default_torque_target_nm = 0.0f;
    constexpr float count_to_rad =
        2.0f * 3.14159265358979323846f / 4096.0f;

    // STM32 的 device_id 由 UID 计算，运行时从反馈/发现响应中学习。
    enum class boot_state_t : uint8_t
    {
        idle,
        receiving,
        entering,
        erasing,
        writing,
        finishing,
        success,
        error,
        aborted
    };

    struct can_frame_t
    {
        uint32_t id;
        uint8_t len;
        uint8_t data[8];
    };

    struct encoder_state_t
    {
        bool valid;
        uint16_t sequence;
        uint16_t timestamp_us;
        int32_t full_count;
        float full_angle_rad;
        float speed_rad_s;
        bool speed_valid;
        uint32_t last_receive_ms;
    };

    static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
    static QueueHandle_t boot_response_queue = nullptr;

    static uint32_t foc_target_device_id = 0;
    static float torque_target_nm = default_torque_target_nm;
    static bool torque_enabled = false;
    static bool boot_session_active = false;

    static encoder_state_t latest_encoder = {};

    static constexpr size_t firmware_buffer_size = can_boot_app_max_size;
    static uint8_t firmware_image[firmware_buffer_size];
    static size_t firmware_image_size = 0;
    static uint32_t firmware_version = 1;
    static bool firmware_upload_active = false;
    static bool firmware_upload_rejected = false;
    static bool boot_flash_task_active = false;
    static bool boot_cancel_requested = false;
    static boot_state_t boot_state = boot_state_t::idle;
    static uint32_t boot_progress = 0;
    static uint32_t boot_total = 0;
    static char boot_error[64] = {};

    constexpr char wifi_ssid[] = "test";
    constexpr char wifi_password[] = "123456789";

    static void learn_device_id(uint32_t id)
    {
        const uint32_t type = id & can_id_type_mask;
        if(type != can_feedback_base &&
           type != can_response_base &&
           type != can_boot_response_base)
        {
            return;
        }

        const uint32_t device_id = id & can_device_id_mask;
        if(device_id != 0)
        {
            portENTER_CRITICAL(&state_mux);
            foc_target_device_id = device_id;
            portEXIT_CRITICAL(&state_mux);
        }
    }
}

static bool is_feedback_id(uint32_t id)
{
    return (id & can_id_type_mask) == can_feedback_base;
}

static bool is_boot_response_id(uint32_t id)
{
    return (id & can_boot_type_mask) == can_boot_response_base;
}

static int32_t signed_count_delta(int32_t current, int32_t previous)
{
    const uint32_t delta =
        static_cast<uint32_t>(current) -
        static_cast<uint32_t>(previous);

    if(delta & 0x80000000u)
    {
        return static_cast<int32_t>(
            static_cast<int64_t>(delta) - 0x100000000LL);
    }

    return static_cast<int32_t>(delta);
}

/**
 * @brief 保存最新编码器反馈，并按相邻反馈计算机械速度。
 */
static void process_encoder_frame(
    uint32_t id,
    const uint8_t *data,
    uint8_t len)
{
    if(!is_feedback_id(id) || len != 8)
    {
        return;
    }

    uint16_t sequence;
    uint16_t timestamp_us;
    int32_t full_count;
    memcpy(&sequence, &data[0], sizeof(sequence));
    memcpy(&timestamp_us, &data[2], sizeof(timestamp_us));
    memcpy(&full_count, &data[4], sizeof(full_count));

    portENTER_CRITICAL(&state_mux);

    float speed_rad_s = latest_encoder.speed_rad_s;
    bool speed_valid = false;

    if(latest_encoder.valid)
    {
        const uint16_t sequence_delta =
            static_cast<uint16_t>(sequence - latest_encoder.sequence);
        const uint16_t timestamp_delta =
            static_cast<uint16_t>(timestamp_us - latest_encoder.timestamp_us);

        if(sequence_delta > 0 &&
           sequence_delta <= 1000 &&
           timestamp_delta > 0 &&
           timestamp_delta <= 0x8000)
        {
            speed_rad_s =
                static_cast<float>(
                    signed_count_delta(full_count, latest_encoder.full_count)) *
                count_to_rad *
                1000000.0f /
                static_cast<float>(timestamp_delta);
            speed_valid = true;
        }
        else if(timestamp_delta == 0)
        {
            // 同一个编码器采样可能被重复反馈，保持上一速度。
            speed_valid = latest_encoder.speed_valid;
        }
    }

    latest_encoder.valid = true;
    latest_encoder.sequence = sequence;
    latest_encoder.timestamp_us = timestamp_us;
    latest_encoder.full_count = full_count;
    latest_encoder.full_angle_rad =
        static_cast<float>(full_count) * count_to_rad;
    latest_encoder.speed_rad_s = speed_rad_s;
    latest_encoder.speed_valid = speed_valid;
    latest_encoder.last_receive_ms = millis();

    portEXIT_CRITICAL(&state_mux);
}

static void enqueue_boot_response(
    uint32_t id,
    const uint8_t *data,
    uint8_t len)
{
    if(!is_boot_response_id(id) ||
       !boot_response_queue ||
       len == 0)
    {
        return;
    }

    can_frame_t frame = {};
    frame.id = id;
    frame.len = len;
    memcpy(frame.data, data, len);
    xQueueSend(boot_response_queue, &frame, 0);
}

static void process_can_msg(uint32_t id, const uint8_t *data, uint8_t len)
{
    constexpr uint8_t uart_can_sof_0 = 0xA5;
    constexpr uint8_t uart_can_sof_1 = 0x5A;
    constexpr uint8_t max_data_len = 8;

    if(len > max_data_len || (!data && len > 0))
    {
        return;
    }

    learn_device_id(id);
    process_encoder_frame(id, data, len);
    enqueue_boot_response(id, data, len);

    // UART frame: A5 5A | CAN ID (little-endian) | DLC | data | XOR checksum.
    uint8_t frame[2 + sizeof(id) + 1 + max_data_len + 1];
    uint8_t frame_len = 0;
    frame[frame_len++] = uart_can_sof_0;
    frame[frame_len++] = uart_can_sof_1;
    memcpy(&frame[frame_len], &id, sizeof(id));
    frame_len += sizeof(id);
    frame[frame_len++] = len;

    uint8_t checksum = 0;
    for(uint8_t i = 2; i < frame_len; i++)
    {
        checksum ^= frame[i];
    }

    memcpy(&frame[frame_len], data, len);
    for(uint8_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    frame_len += len;
    frame[frame_len++] = checksum;

    uart0.write_bytes(frame, frame_len);
}

static void send_torque_frame(uint32_t device_id, float torque_nm)
{
    uint8_t data[sizeof(torque_nm)];
    memcpy(data, &torque_nm, sizeof(data));
    can0.send(device_id, data, sizeof(data), true);
}

/**
 * @brief 自动发现 STM32，并按网页设置持续发送目标扭矩。
 *
 * STM32 FOC 在 100 ms 内没有新指令会将目标清零，因此这里 20 ms 发送一次。
 */
static void can_target_proc(void)
{
    constexpr uint32_t send_period_ms = 20;
    static uint32_t discovery_elapsed_ms = can_discovery_period_ms;

    uint32_t device_id;
    float torque_nm;
    bool enabled;
    bool boot_active;

    portENTER_CRITICAL(&state_mux);
    device_id = foc_target_device_id;
    torque_nm = torque_target_nm;
    enabled = torque_enabled;
    boot_active = boot_session_active || boot_flash_task_active;
    portEXIT_CRITICAL(&state_mux);

    if(boot_active)
    {
        return;
    }

    if(device_id == 0)
    {
        // 第一次立即发起发现，之后每 500 ms 重试一次。
        if(discovery_elapsed_ms >= can_discovery_period_ms)
        {
            uint8_t dummy = 0;
            can0.send(can_discover_id, &dummy, 0, false);
            discovery_elapsed_ms = 0;
        }
        else
        {
            discovery_elapsed_ms += send_period_ms;
        }
        return;
    }

    discovery_elapsed_ms = 0;

    if(enabled)
    {
        send_torque_frame(device_id, torque_nm);
    }
}

static uint32_t crc32_ieee(const uint8_t *data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFu;

    for(size_t i = 0; i < size; i++)
    {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; bit++)
        {
            crc = (crc >> 1) ^
                ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

static const char *boot_state_name(boot_state_t state)
{
    switch(state)
    {
        case boot_state_t::idle: return "idle";
        case boot_state_t::receiving: return "receiving";
        case boot_state_t::entering: return "entering";
        case boot_state_t::erasing: return "erasing";
        case boot_state_t::writing: return "writing";
        case boot_state_t::finishing: return "finishing";
        case boot_state_t::success: return "success";
        case boot_state_t::error: return "error";
        case boot_state_t::aborted: return "aborted";
    }

    return "unknown";
}

static void set_boot_state(
    boot_state_t state,
    uint32_t progress,
    uint32_t total)
{
    portENTER_CRITICAL(&state_mux);
    boot_state = state;
    boot_progress = progress;
    boot_total = total;
    boot_error[0] = '\0';
    portEXIT_CRITICAL(&state_mux);
}

static void set_boot_error(const char *message)
{
    portENTER_CRITICAL(&state_mux);
    boot_state = boot_state_t::error;
    boot_error[0] = '\0';
    if(message)
    {
        strncpy(boot_error, message, sizeof(boot_error) - 1);
        boot_error[sizeof(boot_error) - 1] = '\0';
    }
    portEXIT_CRITICAL(&state_mux);
}

static bool boot_cancel_requested_now(void)
{
    bool requested;

    portENTER_CRITICAL(&state_mux);
    requested = boot_cancel_requested;
    portEXIT_CRITICAL(&state_mux);

    return requested;
}

static uint32_t get_target_device_id(void)
{
    uint32_t device_id;

    portENTER_CRITICAL(&state_mux);
    device_id = foc_target_device_id;
    portEXIT_CRITICAL(&state_mux);

    return device_id;
}

static bool set_target_device_id(uint32_t device_id)
{
    if(device_id == 0 || device_id > can_device_id_mask)
    {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    foc_target_device_id = device_id;
    portEXIT_CRITICAL(&state_mux);
    return true;
}

static uint16_t read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1] << 8);
}

static bool wait_boot_response(
    uint32_t device_id,
    uint8_t expected_opcode,
    int32_t expected_sequence)
{
    if(!boot_response_queue)
    {
        return false;
    }

    const uint32_t response_id =
        can_boot_response_base | device_id;
    const uint32_t started_ms = millis();

    while(millis() - started_ms < can_boot_response_timeout_ms)
    {
        if(boot_cancel_requested_now())
        {
            return false;
        }

        can_frame_t response = {};
        if(xQueueReceive(
               boot_response_queue,
               &response,
               pdMS_TO_TICKS(20)) != pdTRUE)
        {
            continue;
        }

        if(response.id != response_id || response.len == 0)
        {
            continue;
        }

        if(response.data[0] == can_boot_resp_error ||
           response.data[0] == can_boot_resp_crc_error)
        {
            return false;
        }

        if(response.data[0] != expected_opcode)
        {
            continue;
        }

        if(expected_sequence >= 0 &&
           (response.len < 3 ||
            read_u16(&response.data[1]) !=
                static_cast<uint16_t>(expected_sequence)))
        {
            continue;
        }

        return true;
    }

    return false;
}

static bool boot_exchange(
    uint32_t id,
    const uint8_t *data,
    uint8_t len,
    uint8_t expected_opcode,
    int32_t expected_sequence)
{
    for(uint8_t attempt = 0;
        attempt < can_boot_exchange_retries;
        attempt++)
    {
        if(boot_cancel_requested_now())
        {
            return false;
        }

        can0.send(id, data, len, true);
        if(wait_boot_response(
               id & can_device_id_mask,
               expected_opcode,
               expected_sequence))
        {
            return true;
        }
    }

    return false;
}

static void send_boot_abort(uint32_t device_id)
{
    const uint8_t abort_command = can_boot_ctrl_abort;
    can0.send(
        can_boot_ctrl_base | device_id,
        &abort_command,
        sizeof(abort_command),
        true);
}

static void finish_boot_flash(
    boot_state_t state,
    const char *message)
{
    portENTER_CRITICAL(&state_mux);
    boot_flash_task_active = false;
    boot_state = state;
    if(state == boot_state_t::success)
    {
        boot_session_active = false;
    }
    if(message)
    {
        strncpy(boot_error, message, sizeof(boot_error) - 1);
        boot_error[sizeof(boot_error) - 1] = '\0';
    }
    else
    {
        boot_error[0] = '\0';
    }
    portEXIT_CRITICAL(&state_mux);
}

static void boot_flash_task_proc(void *arg)
{
    (void)arg;

    uint32_t device_id;
    size_t image_size;
    uint32_t version;

    portENTER_CRITICAL(&state_mux);
    device_id = foc_target_device_id;
    image_size = firmware_image_size;
    version = firmware_version;
    portEXIT_CRITICAL(&state_mux);

    if(device_id == 0)
    {
        finish_boot_flash(boot_state_t::error, "device id is not set");
        vTaskDelete(nullptr);
        return;
    }

    set_boot_state(
        boot_state_t::entering,
        0,
        static_cast<uint32_t>(image_size));

    if(!boot_response_queue)
    {
        finish_boot_flash(
            boot_state_t::error,
            "boot response queue is unavailable");
        vTaskDelete(nullptr);
        return;
    }
    xQueueReset(boot_response_queue);

    const uint8_t enter_command[6] = {
        can_boot_ctrl_enter,
        can_boot_protocol_version,
        0x07,
        0xB0,
        0xAD,
        0x10
    };
    can0.send(
        can_boot_ctrl_base | device_id,
        enter_command,
        sizeof(enter_command),
        true);
    vTaskDelay(pdMS_TO_TICKS(250));

    if(boot_cancel_requested_now())
    {
        send_boot_abort(device_id);
        finish_boot_flash(boot_state_t::aborted, "cancelled");
        vTaskDelete(nullptr);
        return;
    }

    set_boot_state(
        boot_state_t::erasing,
        0,
        static_cast<uint32_t>(image_size));

    uint8_t begin_command[8] = {};
    const uint32_t image_crc = crc32_ieee(firmware_image, image_size);
    begin_command[0] = can_boot_ctrl_begin;
    begin_command[1] = can_boot_protocol_version;
    begin_command[2] = static_cast<uint8_t>(image_size);
    begin_command[3] = static_cast<uint8_t>(image_size >> 8);
    begin_command[4] = static_cast<uint8_t>(image_crc);
    begin_command[5] = static_cast<uint8_t>(image_crc >> 8);
    begin_command[6] = static_cast<uint8_t>(image_crc >> 16);
    begin_command[7] = static_cast<uint8_t>(image_crc >> 24);

    if(!boot_exchange(
           can_boot_ctrl_base | device_id,
           begin_command,
           sizeof(begin_command),
           can_boot_resp_ready,
           -1))
    {
        if(boot_cancel_requested_now())
        {
            send_boot_abort(device_id);
            finish_boot_flash(boot_state_t::aborted, "cancelled");
        }
        else
        {
            finish_boot_flash(boot_state_t::error, "BEGIN timeout or rejection");
        }
        vTaskDelete(nullptr);
        return;
    }

    set_boot_state(
        boot_state_t::writing,
        0,
        static_cast<uint32_t>(image_size));

    uint16_t sequence = 0;
    for(size_t offset = 0; offset < image_size; offset += 6)
    {
        if(boot_cancel_requested_now())
        {
            send_boot_abort(device_id);
            finish_boot_flash(boot_state_t::aborted, "cancelled");
            vTaskDelete(nullptr);
            return;
        }

        const size_t remaining = image_size - offset;
        const uint8_t chunk_size =
            static_cast<uint8_t>(remaining > 6 ? 6 : remaining);
        uint8_t data_command[8] = {};
        data_command[0] = static_cast<uint8_t>(sequence);
        data_command[1] = static_cast<uint8_t>(sequence >> 8);
        memcpy(&data_command[2], &firmware_image[offset], chunk_size);

        if(!boot_exchange(
               can_boot_data_base | device_id,
               data_command,
               static_cast<uint8_t>(chunk_size + 2),
               can_boot_resp_ack,
               sequence))
        {
            if(boot_cancel_requested_now())
            {
                send_boot_abort(device_id);
                finish_boot_flash(boot_state_t::aborted, "cancelled");
            }
            else
            {
                finish_boot_flash(
                    boot_state_t::error,
                    "DATA timeout or sequence rejection");
            }
            vTaskDelete(nullptr);
            return;
        }

        ++sequence;
        set_boot_state(
            boot_state_t::writing,
            static_cast<uint32_t>(offset + chunk_size),
            static_cast<uint32_t>(image_size));
    }

    set_boot_state(
        boot_state_t::finishing,
        static_cast<uint32_t>(image_size),
        static_cast<uint32_t>(image_size));

    uint8_t end_command[5] = {};
    end_command[0] = can_boot_ctrl_end;
    end_command[1] = static_cast<uint8_t>(version);
    end_command[2] = static_cast<uint8_t>(version >> 8);
    end_command[3] = static_cast<uint8_t>(version >> 16);
    end_command[4] = static_cast<uint8_t>(version >> 24);

    if(!boot_exchange(
           can_boot_ctrl_base | device_id,
           end_command,
           sizeof(end_command),
           can_boot_resp_success,
           -1))
    {
        if(boot_cancel_requested_now())
        {
            send_boot_abort(device_id);
            finish_boot_flash(boot_state_t::aborted, "cancelled");
        }
        else
        {
            finish_boot_flash(
                boot_state_t::error,
                "END timeout, CRC failure, or rejection");
        }
        vTaskDelete(nullptr);
        return;
    }

    finish_boot_flash(boot_state_t::success, nullptr);
    vTaskDelete(nullptr);
}

static bool parse_uint32(const String &text, uint32_t &value)
{
    String trimmed = text;
    trimmed.trim();
    if(trimmed.length() == 0)
    {
        return false;
    }

    char *end = nullptr;
    const unsigned long parsed =
        strtoul(trimmed.c_str(), &end, 0);
    if(end == trimmed.c_str() ||
       *end != 0 ||
       parsed > 0xFFFFFFFFUL)
    {
        return false;
    }

    value = static_cast<uint32_t>(parsed);
    return true;
}

static bool parse_float32(const String &text, float &value)
{
    String trimmed = text;
    trimmed.trim();
    if(trimmed.length() == 0)
    {
        return false;
    }

    char *end = nullptr;
    value = strtof(trimmed.c_str(), &end);
    return end != trimmed.c_str() &&
           *end == 0 &&
           isfinite(value);
}

static void send_api_error(
    int status_code,
    const char *message)
{
    char body[192];
    snprintf(
        body,
        sizeof(body),
        R"JSON({"ok":false,"error":"%s"})JSON",
        message ? message : "request failed");
    web_server.send(
        status_code,
        "application/json; charset=utf-8",
        body);
}

static void handle_api_device(void)
{
    String text = web_server.hasArg("device_id")
        ? web_server.arg("device_id")
        : web_server.arg("id");
    uint32_t device_id;

    if(!parse_uint32(text, device_id) ||
       !set_target_device_id(device_id))
    {
        send_api_error(
            400,
            "device_id must be 1..0x03FFFFFF");
        return;
    }

    char body[96];
    snprintf(
        body,
        sizeof(body),
        R"JSON({"ok":true,"device_id":"0x%06lX"})JSON",
        static_cast<unsigned long>(device_id));
    web_server.send(
        200,
        "application/json; charset=utf-8",
        body);
}

static bool boot_operation_active_now(void)
{
    bool active;

    portENTER_CRITICAL(&state_mux);
    active = boot_session_active ||
             firmware_upload_active ||
             boot_flash_task_active;
    portEXIT_CRITICAL(&state_mux);

    return active;
}

static void handle_api_torque(void)
{
    uint32_t requested_device_id = get_target_device_id();
    float requested_torque;

    if(web_server.hasArg("device_id"))
    {
        if(!parse_uint32(
               web_server.arg("device_id"),
               requested_device_id) ||
           !set_target_device_id(requested_device_id))
        {
            send_api_error(
                400,
                "device_id must be 1..0x03FFFFFF");
            return;
        }
    }

    if(requested_device_id == 0)
    {
        send_api_error(
            400,
            "device_id is unknown; wait for feedback or set it manually");
        return;
    }

    if(!web_server.hasArg("value") ||
       !parse_float32(web_server.arg("value"), requested_torque))
    {
        send_api_error(
            400,
            "value must be a finite float in N*m");
        return;
    }

    if(boot_operation_active_now())
    {
        send_api_error(
            409,
            "bootloader operation is active");
        return;
    }

    portENTER_CRITICAL(&state_mux);
    torque_target_nm = requested_torque;
    torque_enabled = true;
    boot_session_active = false;
    portEXIT_CRITICAL(&state_mux);

    // 立即发一帧，随后由 20 ms 周期任务继续刷新。
    send_torque_frame(requested_device_id, requested_torque);

    char body[128];
    snprintf(
        body,
        sizeof(body),
        R"JSON({"ok":true,"device_id":"0x%06lX","torque_nm":%.6f})JSON",
        static_cast<unsigned long>(requested_device_id),
        static_cast<double>(requested_torque));
    web_server.send(
        200,
        "application/json; charset=utf-8",
        body);
}

static void handle_api_torque_stop(void)
{
    uint32_t device_id = get_target_device_id();
    bool boot_active;

    portENTER_CRITICAL(&state_mux);
    torque_target_nm = 0.0f;
    torque_enabled = false;
    boot_active = boot_session_active ||
                  firmware_upload_active ||
                  boot_flash_task_active;
    portEXIT_CRITICAL(&state_mux);

    if(device_id != 0 && !boot_active)
    {
        send_torque_frame(device_id, 0.0f);
    }

    web_server.send(
        200,
        "application/json; charset=utf-8",
        R"JSON({"ok":true,"torque_enabled":false})JSON");
}

static void handle_api_boot_enter(void)
{
    uint32_t device_id = get_target_device_id();
    if(web_server.hasArg("device_id"))
    {
        if(!parse_uint32(
               web_server.arg("device_id"),
               device_id) ||
           !set_target_device_id(device_id))
        {
            send_api_error(
                400,
                "device_id must be 1..0x03FFFFFF");
            return;
        }
    }

    if(device_id == 0)
    {
        send_api_error(
            400,
            "device_id is unknown; wait for feedback or set it manually");
        return;
    }

    bool busy;
    portENTER_CRITICAL(&state_mux);
    busy = firmware_upload_active || boot_flash_task_active;
    portEXIT_CRITICAL(&state_mux);

    if(busy)
    {
        send_api_error(409, "another firmware operation is active");
        return;
    }

    portENTER_CRITICAL(&state_mux);
    torque_target_nm = 0.0f;
    torque_enabled = false;
    boot_session_active = true;
    boot_cancel_requested = false;
    portEXIT_CRITICAL(&state_mux);

    set_boot_state(boot_state_t::entering, 0, 0);

    // 先发零扭矩，再给应用留一个调度窗口处理进入引导命令。
    send_torque_frame(device_id, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));

    const uint8_t enter_command[6] = {
        can_boot_ctrl_enter,
        can_boot_protocol_version,
        0x07,
        0xB0,
        0xAD,
        0x10
    };
    can0.send(
        can_boot_ctrl_base | device_id,
        enter_command,
        sizeof(enter_command),
        true);

    char body[128];
    snprintf(
        body,
        sizeof(body),
        R"JSON({"ok":true,"state":"entering","device_id":"0x%06lX"})JSON",
        static_cast<unsigned long>(device_id));
    web_server.send(
        200,
        "application/json; charset=utf-8",
        body);
}

static void handle_api_boot_abort(void)
{
    const uint32_t device_id = get_target_device_id();
    bool flash_active;

    portENTER_CRITICAL(&state_mux);
    flash_active = boot_flash_task_active;
    if(flash_active)
    {
        boot_cancel_requested = true;
    }
    portEXIT_CRITICAL(&state_mux);

    if(device_id == 0)
    {
        send_api_error(400, "device_id is unknown");
        return;
    }

    if(flash_active)
    {
        web_server.send(
            202,
            "application/json; charset=utf-8",
            R"JSON({"ok":true,"state":"cancelling"})JSON");
        return;
    }

    send_boot_abort(device_id);
    set_boot_state(boot_state_t::aborted, 0, 0);
    web_server.send(
        200,
        "application/json; charset=utf-8",
        R"JSON({"ok":true,"state":"aborted"})JSON");
}

static void handle_firmware_upload(void)
{
    HTTPUpload &upload = web_server.upload();

    switch(upload.status)
    {
        case UPLOAD_FILE_START:
        {
            bool busy;
            uint32_t stop_device_id = 0;
            portENTER_CRITICAL(&state_mux);
            busy = boot_flash_task_active ||
                   firmware_upload_active;
            if(!busy)
            {
                stop_device_id = foc_target_device_id;
                firmware_upload_active = true;
                firmware_upload_rejected = false;
                firmware_image_size = 0;
                firmware_version = 1;
                boot_cancel_requested = false;
                torque_target_nm = 0.0f;
                torque_enabled = false;
                boot_session_active = true;
                boot_state = boot_state_t::receiving;
                boot_progress = 0;
                boot_total = 0;
                boot_error[0] = 0;
            }
            else
            {
                firmware_upload_rejected = true;
            }
            portEXIT_CRITICAL(&state_mux);

            if(!busy && stop_device_id != 0)
            {
                // 上传一开始就发零目标，缩短电机停止等待时间。
                send_torque_frame(stop_device_id, 0.0f);
            }
            break;
        }

        case UPLOAD_FILE_WRITE:
            if(!firmware_upload_rejected)
            {
                if(upload.currentSize >
                   firmware_buffer_size - firmware_image_size)
                {
                    firmware_upload_rejected = true;
                }
                else
                {
                    memcpy(
                        &firmware_image[firmware_image_size],
                        upload.buf,
                        upload.currentSize);
                    firmware_image_size += upload.currentSize;

                    portENTER_CRITICAL(&state_mux);
                    boot_progress =
                        static_cast<uint32_t>(firmware_image_size);
                    portEXIT_CRITICAL(&state_mux);
                }
            }
            break;

        case UPLOAD_FILE_END:
            if(firmware_image_size == 0)
            {
                firmware_upload_rejected = true;
            }
            portENTER_CRITICAL(&state_mux);
            firmware_upload_active = false;
            boot_total = static_cast<uint32_t>(firmware_image_size);
            portEXIT_CRITICAL(&state_mux);
            break;

        case UPLOAD_FILE_ABORTED:
            firmware_upload_rejected = true;
            portENTER_CRITICAL(&state_mux);
            firmware_upload_active = false;
            boot_session_active = false;
            boot_state = boot_state_t::error;
            strncpy(
                boot_error,
                "browser upload aborted",
                sizeof(boot_error) - 1);
            boot_error[sizeof(boot_error) - 1] = 0;
            portEXIT_CRITICAL(&state_mux);
            break;

        default:
            break;
    }
}

static void handle_api_boot_upload_complete(void)
{
    bool rejected;
    size_t image_size;
    uint32_t version = 1;
    uint32_t device_id = get_target_device_id();

    portENTER_CRITICAL(&state_mux);
    rejected = firmware_upload_rejected;
    image_size = firmware_image_size;
    portEXIT_CRITICAL(&state_mux);

    if(rejected || image_size == 0 || image_size > firmware_buffer_size)
    {
        portENTER_CRITICAL(&state_mux);
        firmware_upload_active = false;
        boot_session_active = false;
        portEXIT_CRITICAL(&state_mux);
        set_boot_error(
            image_size > firmware_buffer_size
                ? "bin is larger than STM32 application area"
                : "invalid or incomplete bin upload");
        send_api_error(
            image_size > firmware_buffer_size ? 413 : 400,
            image_size > firmware_buffer_size
                ? "bin is larger than 23552 bytes"
                : "invalid or incomplete bin upload");
        return;
    }

    if(web_server.hasArg("device_id"))
    {
        if(!parse_uint32(
               web_server.arg("device_id"),
               device_id) ||
               !set_target_device_id(device_id))
        {
            portENTER_CRITICAL(&state_mux);
            firmware_upload_active = false;
            boot_session_active = false;
            portEXIT_CRITICAL(&state_mux);
            set_boot_error("invalid device id");
            send_api_error(
                400,
                "device_id must be 1..0x03FFFFFF");
            return;
        }
    }

    if(web_server.hasArg("version") &&
       !parse_uint32(web_server.arg("version"), version))
    {
        portENTER_CRITICAL(&state_mux);
        firmware_upload_active = false;
        boot_session_active = false;
        portEXIT_CRITICAL(&state_mux);
        set_boot_error("invalid firmware version");
        send_api_error(400, "version must be an unsigned 32-bit integer");
        return;
    }

    if(device_id == 0)
    {
        portENTER_CRITICAL(&state_mux);
        firmware_upload_active = false;
        boot_session_active = false;
        portEXIT_CRITICAL(&state_mux);
        set_boot_error("device id is not set");
        send_api_error(
            400,
            "device_id is unknown; wait for feedback or set it manually");
        return;
    }

    portENTER_CRITICAL(&state_mux);
    firmware_version = version;
    boot_flash_task_active = true;
    boot_cancel_requested = false;
    boot_session_active = true;
    boot_state = boot_state_t::entering;
    boot_progress = 0;
    boot_total = static_cast<uint32_t>(image_size);
    boot_error[0] = 0;
    portEXIT_CRITICAL(&state_mux);

    const BaseType_t result = xTaskCreatePinnedToCore(
        boot_flash_task_proc,
        "can_flash",
        4096,
        nullptr,
        3,
        nullptr,
        1);

    if(result != pdPASS)
    {
        portENTER_CRITICAL(&state_mux);
        boot_flash_task_active = false;
        boot_session_active = false;
        portEXIT_CRITICAL(&state_mux);
        set_boot_error("cannot create flash task");
        send_api_error(500, "cannot create flash task");
        return;
    }

    char body[160];
    const uint32_t crc = crc32_ieee(firmware_image, image_size);
    snprintf(
        body,
        sizeof(body),
        R"JSON({"ok":true,"state":"queued","size":%lu,"crc32":"0x%08lX"})JSON",
        static_cast<unsigned long>(image_size),
        static_cast<unsigned long>(crc));
    web_server.send(
        202,
        "application/json; charset=utf-8",
        body);
}

static void handle_api_status(void)
{
    uint32_t device_id;
    float target_nm;
    bool target_enabled;
    encoder_state_t encoder;
    boot_state_t state;
    uint32_t progress;
    uint32_t total;
    char error[sizeof(boot_error)];

    portENTER_CRITICAL(&state_mux);
    device_id = foc_target_device_id;
    target_nm = torque_target_nm;
    target_enabled = torque_enabled;
    encoder = latest_encoder;
    state = boot_state;
    progress = boot_progress;
    total = boot_total;
    strncpy(error, boot_error, sizeof(error) - 1);
    error[sizeof(error) - 1] = 0;
    portEXIT_CRITICAL(&state_mux);

    const bool wifi_connected =
        WiFi.status() == WL_CONNECTED;
    const String ip = WiFi.localIP().toString();
    const uint32_t age_ms = encoder.valid
        ? millis() - encoder.last_receive_ms
        : 0;

    char speed[32];
    if(encoder.speed_valid)
    {
        snprintf(speed, sizeof(speed), "%.6f", static_cast<double>(
            encoder.speed_rad_s));
    }
    else
    {
        strcpy(speed, "null");
    }

    char body[1400];
    snprintf(
        body,
        sizeof(body),
        R"JSON({"ok":true,"wifi":{"connected":%s,"ssid":"test","ip":"%s"},"device_id":"0x%06lX","torque":{"target_nm":%.6f,"enabled":%s},"encoder":{"valid":%s,"sequence":%u,"timestamp_us":%u,"full_count":%ld,"full_angle_rad":%.6f,"speed_rad_s":%s,"age_ms":%lu},"boot":{"state":"%s","progress":%lu,"total":%lu,"error":"%s"}})JSON",
        wifi_connected ? "true" : "false",
        ip.c_str(),
        static_cast<unsigned long>(device_id),
        static_cast<double>(target_nm),
        target_enabled ? "true" : "false",
        encoder.valid ? "true" : "false",
        static_cast<unsigned int>(encoder.sequence),
        static_cast<unsigned int>(encoder.timestamp_us),
        static_cast<long>(encoder.full_count),
        static_cast<double>(encoder.full_angle_rad),
        speed,
        static_cast<unsigned long>(age_ms),
        boot_state_name(state),
        static_cast<unsigned long>(progress),
        static_cast<unsigned long>(total),
        error);

    web_server.send(
        200,
        "application/json; charset=utf-8",
        body);
}

static const char index_html[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP CAN Monitor</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}
body{max-width:960px;margin:0 auto;padding:20px;background:#111827;color:#e5e7eb}
section{background:#1f2937;border-radius:12px;padding:16px;margin:12px 0}
h1{font-size:22px;margin:0 0 16px}
h2{font-size:17px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}
.item{background:#111827;border-radius:8px;padding:10px}
.label{color:#9ca3af;font-size:12px}
.value{font-family:ui-monospace,monospace;font-size:16px;margin-top:4px}
label{display:block;color:#d1d5db;margin:8px 0 4px}
input,button{box-sizing:border-box;border:1px solid #4b5563;border-radius:7px;
  padding:9px;background:#111827;color:#f9fafb;font-size:15px}
input{width:100%}
button{cursor:pointer;background:#2563eb;border-color:#3b82f6;margin:8px 6px 0 0}
button.stop{background:#b91c1c;border-color:#ef4444}
button.secondary{background:#4b5563;border-color:#6b7280}
.row{display:flex;gap:10px;align-items:end;flex-wrap:wrap}
.field{flex:1;min-width:180px}
#message{white-space:pre-wrap;color:#93c5fd;min-height:22px}
small{color:#9ca3af}
</style>
</head>
<body>
<h1>ESP CAN / STM32 控制台</h1>
<section>
<h2>连接与编码器</h2>
<div class="grid">
<div class="item"><div class="label">WiFi</div><div id="wifi" class="value">...</div></div>
<div class="item"><div class="label">STM32 device_id</div><div id="statusDevice" class="value">...</div></div>
<div class="item"><div class="label">sequence</div><div id="sequence" class="value">...</div></div>
<div class="item"><div class="label">timestamp_us</div><div id="timestamp" class="value">...</div></div>
<div class="item"><div class="label">full_count</div><div id="count" class="value">...</div></div>
<div class="item"><div class="label">full_angle_rad</div><div id="angle" class="value">...</div></div>
<div class="item"><div class="label">speed_rad_s</div><div id="speed" class="value">...</div></div>
<div class="item"><div class="label">数据年龄</div><div id="age" class="value">...</div></div>
</div>
</section>
<section>
<h2>目标扭矩</h2>
<div class="row">
<div class="field"><label for="deviceId">device_id（可留空，自动发现）</label>
<input id="deviceId" placeholder="例如 0x123456"></div>
<div class="field"><label for="torque">目标扭矩（N·m）</label>
<input id="torque" type="number" step="0.001" value="0.000"></div>
</div>
<button class="secondary" onclick="setDevice()">设置 device_id</button>
<button onclick="sendTorque()">发送并持续保持</button>
<button class="stop" onclick="stopTorque()">停止发送 / 目标置零</button>
<div id="torqueState"><small>默认不发送扭矩命令。</small></div>
</section>
<section>
<h2>STM32 CAN Bootloader</h2>
<small>bin 上限 23552 字节。上传后自动进入引导、擦除、分包写入、校验并重启。</small>
<div class="row">
<div class="field"><label for="version">固件版本 uint32</label>
<input id="version" value="1" inputmode="numeric"></div>
<div class="field"><label for="firmware">Application .bin</label>
<input id="firmware" type="file" accept=".bin,application/octet-stream"></div>
</div>
<button onclick="enterBoot()">发送进入 Bootloader</button>
<button onclick="uploadFirmware()">上传并刷写 bin</button>
<button class="stop secondary" onclick="abortBoot()">中止当前更新</button>
<div id="bootState">状态：...</div>
</section>
<section><div id="message"></div></section>
<script>
const byId=id=>document.getElementById(id);
function form(data){return new URLSearchParams(data);}
async function request(url,options){
  const response=await fetch(url,options||{});
  let body={};
  try{body=await response.json();}catch(e){}
  if(!response.ok)throw new Error(body.error||("HTTP "+response.status));
  return body;
}
function deviceValue(){return byId("deviceId").value.trim();}
function showMessage(value){byId("message").textContent=value;}
async function setDevice(){
  try{
    const body=await request("/api/device",{method:"POST",
      headers:{"Content-Type":"application/x-www-form-urlencoded"},
      body:form({device_id:deviceValue()})});
    byId("deviceId").value=body.device_id;
    showMessage("device_id 已设置为 "+body.device_id);
  }catch(error){showMessage("设置失败："+error.message);}
}
async function sendTorque(){
  try{
    const body=await request("/api/torque",{method:"POST",
      headers:{"Content-Type":"application/x-www-form-urlencoded"},
      body:form({device_id:deviceValue(),value:byId("torque").value})});
    showMessage("已发送目标扭矩 "+Number(body.torque_nm).toFixed(6)+" N·m，并每 20 ms 刷新");
  }catch(error){showMessage("扭矩发送失败："+error.message);}
}
async function stopTorque(){
  try{await request("/api/torque/stop",{method:"POST"});
    showMessage("已停止发送扭矩目标，STM32 将在超时后归零");
  }catch(error){showMessage("停止失败："+error.message);}
}
async function enterBoot(){
  try{await request("/api/boot/enter",{method:"POST",
      headers:{"Content-Type":"application/x-www-form-urlencoded"},
      body:form({device_id:deviceValue()})});
    showMessage("已发送进入 Bootloader 命令");
  }catch(error){showMessage("进入 Bootloader 失败："+error.message);}
}
async function uploadFirmware(){
  const input=byId("firmware");
  if(!input.files.length){showMessage("请先选择 .bin 文件");return;}
  const file=input.files[0];
  if(file.size<1||file.size>23552){
    showMessage("bin 大小必须为 1..23552 字节");return;
  }
  const formData=new FormData();
  formData.append("file",file,file.name);
  const params=new URLSearchParams();
  if(deviceValue())params.set("device_id",deviceValue());
  params.set("version",byId("version").value.trim()||"1");
  try{
    await request("/api/boot/upload?"+params.toString(),
      {method:"POST",body:formData});
    showMessage("上传完成，正在通过 CAN 刷写；请观察下方进度");
  }catch(error){showMessage("上传/刷写启动失败："+error.message);}
}
async function abortBoot(){
  try{await request("/api/boot/abort",{method:"POST"});
    showMessage("已请求中止更新；若 STM32 已在 Bootloader 中，需要复位才能回到应用");
  }catch(error){showMessage("中止失败："+error.message);}
}
async function refresh(){
  try{
    const s=await request("/api/status");
    byId("wifi").textContent=s.wifi.connected?"已连接 "+s.wifi.ip:"未连接";
    byId("statusDevice").textContent=s.device_id;
    if(!byId("deviceId").value&&s.device_id!=="0x000000")
      byId("deviceId").value=s.device_id;
    const e=s.encoder;
    byId("sequence").textContent=e.valid?e.sequence:"--";
    byId("timestamp").textContent=e.valid?e.timestamp_us+" us":"--";
    byId("count").textContent=e.valid?e.full_count:"--";
    byId("angle").textContent=e.valid?Number(e.full_angle_rad).toFixed(6):"--";
    byId("speed").textContent=e.valid&&e.speed_rad_s!==null?
      Number(e.speed_rad_s).toFixed(6):"--";
    byId("age").textContent=e.valid?e.age_ms+" ms":"--";
    byId("torqueState").textContent="目标 "+
      Number(s.torque.target_nm).toFixed(6)+" N·m，"+
      (s.torque.enabled?"持续发送中":"未发送");
    const b=s.boot;
    let progress="";
    if(b.total)progress=" "+b.progress+"/"+b.total+" bytes";
    byId("bootState").textContent="状态："+b.state+progress+
      (b.error?"："+b.error:"");
  }catch(error){byId("wifi").textContent="状态获取失败";}
}
setInterval(refresh,300);
refresh();
</script>
</body>
</html>
)HTML";

static void setup_web_routes(void)
{
    web_server.on(
        "/",
        HTTP_GET,
        []()
        {
            web_server.send_P(
                200,
                "text/html; charset=utf-8",
                index_html);
        });
    web_server.on(
        "/api/status",
        HTTP_GET,
        handle_api_status);
    web_server.on(
        "/api/device",
        HTTP_POST,
        handle_api_device);
    web_server.on(
        "/api/torque",
        HTTP_POST,
        handle_api_torque);
    web_server.on(
        "/api/torque/stop",
        HTTP_POST,
        handle_api_torque_stop);
    web_server.on(
        "/api/boot/enter",
        HTTP_POST,
        handle_api_boot_enter);
    web_server.on(
        "/api/boot/abort",
        HTTP_POST,
        handle_api_boot_abort);
    web_server.on(
        "/api/boot/upload",
        HTTP_POST,
        handle_api_boot_upload_complete,
        handle_firmware_upload);
    web_server.onNotFound(
        []()
        {
            send_api_error(404, "not found");
        });
}

static void web_server_proc(void)
{
    static bool connect_started = false;
    static bool server_started = false;
    static uint32_t last_connect_attempt_ms = 0;
    const uint32_t now = millis();

    if(WiFi.status() != WL_CONNECTED)
    {
        if(!connect_started ||
           now - last_connect_attempt_ms >= 5000)
        {
            WiFi.begin(wifi_ssid, wifi_password);
            connect_started = true;
            last_connect_attempt_ms = now;
        }
        return;
    }

    if(!server_started)
    {
        web_server.begin();
        server_started = true;
    }

    web_server.handleClient();
}

static void can_recv_proc(void)
{
    can0.receive(process_can_msg);
}

/**
 * @brief 任务列表
 */
static void task_list(void)
{
    static task can_recv_task(1, can_recv_proc, 4096, 4, 1);
    can_recv_task.start();

    static task can_target_task(20, can_target_proc, 2048, 3, 1);
    can_target_task.start();

    static task web_task(5, web_server_proc, 8192, 2, 0);
    web_task.start();

    static task board_led_dev_task(50, board_led_dev_proc, 2048, 2, 0);
    board_led_dev_task.start();
}

/**
 * @brief 初始化 CAN 到 UART 透传
 */
void start_init_all(void)
{
    uart0.init();
    can0.init();
    boot_response_queue = xQueueCreate(16, sizeof(can_frame_t));
    setup_web_routes();
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(wifi_ssid, wifi_password);
    board_led.init();
    task_list();
}
