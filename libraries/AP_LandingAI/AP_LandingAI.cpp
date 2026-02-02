#include "AP_LandingAI.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_LandingAI::init() {
    _healthy = false;
    _awaiting_ack = false;
    _last_msg_ms = 0;
    _roll_err = 0.0f;
    _pitch_err = 0.0f;
}

// 處理 GCS 指令
bool AP_LandingAI::handle_start_command(const mavlink_command_int_t &packet) {
    gcs().send_text(MAV_SEVERITY_INFO, "AI Landing: Start Cmd Received");
    
    // 轉發給 AI 電腦
    send_start_landing_cmd((uint8_t)packet.param1, packet.param2, (uint8_t)packet.param3);
    
    return true;
}

// 轉發指令給 AI 電腦 (Target ID 211)
void AP_LandingAI::send_start_landing_cmd(uint8_t stream_type, float freq_hz, uint8_t frame) {
    const uint8_t target_system = mavlink_system.sysid;
    const uint8_t target_component = 211; 

    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_channel_t chan = (mavlink_channel_t)i;
        if (HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
            mavlink_msg_command_long_send(
                chan, target_system, target_component,
                31020, 0, (float)stream_type, freq_hz, (float)frame, 0, 0, 0, 0
            );
        }
    }

    _last_cmd_sent_ms = AP_HAL::millis();
    _awaiting_ack = true;
}

void AP_LandingAI::update() {
    uint32_t now = AP_HAL::millis();

    // Story 1.2: 超時監控 (1秒)
    if (_awaiting_ack && (now - _last_cmd_sent_ms > 1000)) {
        _awaiting_ack = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "AI Landing: Command ACK Timeout!");
    }

    // Story 1.3: 數據過期檢查 (2秒)
    if (_healthy && (now - _last_msg_ms > 2000)) {
        _healthy = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "AI Landing: Data Stream Lost!");
    }

    // Story 1.4: 1Hz 狀態回報
    static uint32_t last_report_ms = 0;
    if (now - last_report_ms >= 1000) {
        gcs().send_text(MAV_SEVERITY_INFO, "AI Status: Healthy=%d, Conf=%.2f", _healthy, _healthy ? 1.0f : 0.0f);        
        send_status_to_gcs();
        last_report_ms = now;
    }
}

void AP_LandingAI::handle_msg(const mavlink_message_t &msg) {
    if (msg.msgid == 180) { // 假設 180 是 AI_LANDING_CORRECTION
        mavlink_ai_landing_correction_t packet;
        mavlink_msg_ai_landing_correction_decode(&msg, &packet);

        if (packet.flags & 0x01) {
            _roll_err = packet.roll_err;
            _pitch_err = packet.pitch_err;
            _last_msg_ms = AP_HAL::millis();
            _healthy = true;
        }
    }
}

void AP_LandingAI::send_status_to_gcs() {
    // 1. 發送 Named Float (Mission Planner Status 頁面可見)
    gcs().send_named_float("AI_CONF", _healthy ? 1.0f : 0.0f);

    // 2. 發送自定義健康包 (ID 188)
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_channel_t chan = (mavlink_channel_t)i;
        if (HAVE_PAYLOAD_SPACE(chan, AI_LANDING_STATUS)) {
            mavlink_msg_ai_landing_status_send(
                chan, AP_HAL::millis(), 
                _healthy ? 1.0f : 0.0f, // visual_confidence
                _healthy ? 0 : 1,       // target_lost
                0.0f, 0.0f              // 其他預留
            );
        }
    }
}