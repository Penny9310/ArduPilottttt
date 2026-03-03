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
    _yaw_err = 0.0f;       // 補全偏航角修正初始化
    _distance = 0.0f;      // 補全距離初始化
    _confidence = 0.0f;
}

// 處理 GCS 指令 (Story 1.2)
bool AP_LandingAI::handle_start_command(const mavlink_command_int_t &packet) {
    gcs().send_text(MAV_SEVERITY_INFO, "AI Landing: Start Cmd Received");
    
    // 轉發給 AI 電腦 (將 ID 統一為您測試用的 31010 或維持專案定義)
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
                31010, 0, (float)stream_type, freq_hz, (float)frame, 0, 0, 0, 0
            ); // 將指令 ID 修正為與測試一致的 31010
        }
    }

    _last_cmd_sent_ms = AP_HAL::millis();
    _awaiting_ack = true;
}

void AP_LandingAI::update() {
    uint32_t now = AP_HAL::millis();

    // 1. Story 1.2: 指令超時監控 (維持現狀)
    if (_awaiting_ack && (now - _last_cmd_sent_ms > 1000)) {
        _awaiting_ack = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "AI Landing: Command ACK Timeout!");
    }

    // --- 1. 暴力驗證觸發段 ---
    // 門檻設為 1800，確保只有在您明確輸入 rc 8 2000 時才觸發
    if (hal.rcin->read(7) > 1800) { 
        _healthy = true;
        _confidence = 0.85f;
        _last_msg_ms = now; // 只有高位才刷新時間
    }

    // --- 2. Story 1.3: 逾時判定 (核心驗證點) ---
    // 當您輸入 rc 8 1000 時，上方判斷失效，時間停止刷新
    // 1 秒後 (now - _last_msg_ms > 1000) 成立，狀態必須切回 0
    if (_healthy && (now - _last_msg_ms > 1000)) {
        _healthy = false;
        _confidence = 0.0f; 
        gcs().send_text(MAV_SEVERITY_WARNING, "AI Landing: Data Stream Lost!");
    }

    // --- 3. Story 1.4: 每秒狀態回報 ---
    static uint32_t last_report_ms = 0;
    if (now - last_report_ms >= 1000) {
        gcs().send_text(MAV_SEVERITY_INFO, "AI Status: Healthy=%d, Conf=%.2f", _healthy, _confidence);
        last_report_ms = now;
    }
}

void AP_LandingAI::handle_msg(const mavlink_message_t &msg) {
    // 檢查訊息 ID (需與 flightstack.xml 定義一致)
    if (msg.msgid == 187) { 
        hal.console->printf("RECV MSG RECV: ID=%u", msg.msgid);
        
        mavlink_ai_landing_correction_t packet;
        mavlink_msg_ai_landing_correction_decode(&msg, &packet);

        // 修正：放寬 Flag 判定條件。只要 flags > 0 (包含 0x01, 0x02, 0x03) 即可
        if (packet.flags > 0) { 
            _roll_err = packet.roll_err;
            _pitch_err = packet.pitch_err;
            _yaw_err = packet.yaw_err;     // 儲存偏航角修正
            _distance = packet.distance;   // 儲存目標距離
            _confidence = packet.confidence;
            _last_msg_ms = AP_HAL::millis();
            _healthy = true;
        }
    }
}

void AP_LandingAI::send_status_to_gcs() {
    // 1. 發送 Named Float (Mission Planner Status 頁面可見)
    gcs().send_named_float("AI_CONF", _confidence);

    // 2. 發送自定義健康包 (ID 188)
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_channel_t chan = (mavlink_channel_t)i;
        if (HAVE_PAYLOAD_SPACE(chan, AI_LANDING_STATUS)) {
            mavlink_msg_ai_landing_status_send(
                chan, AP_HAL::millis(), 
                _confidence,             // 使用實際信心值
                _healthy ? 0 : 1,        // target_lost 狀態
                0.0f, 0.0f               // 預留欄位
            );
        }
    }
}