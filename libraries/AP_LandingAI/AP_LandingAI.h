#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_LandingAI {
public:
    // 初始化變數
    void init();

    // 宣告 Story 1.2 的核心函式
    void send_start_landing_cmd(uint8_t stream_type, float freq_hz, uint8_t frame);

    // 核心邏輯：每秒執行多次，檢查是否超時 (符合 Story 1.2 逾時要求)
    void update();

    // 解析來自 AI 模組的訊息 (Story 1.3)
    void handle_msg(const mavlink_message_t &msg);

    void send_status_to_gcs();

    // 回報數據是否可用
    bool is_healthy() const { return _healthy; }

    bool handle_start_command(const mavlink_command_int_t &packet);

private:
    // 數據快取與健康狀態
    uint32_t _last_msg_ms;       // 最後一次收到訊息的時間
    
    // --- 誤差量與狀態變數 ---
    float _roll_err;             // 儲存來自 AI 的橫滾誤差
    float _pitch_err;            // 儲存來自 AI 的俯仰誤差
    float _yaw_err;              // 儲存來自 AI 的偏航誤差 (解決宣告錯誤)
    float _distance;             // 儲存目標物水平距離
    float _confidence;           // 儲存 AI 視覺信心值
    
    bool _healthy;               // 數據有效性標籤

    // --- Story 1.2 狀態監控變數 ---
    uint32_t _last_cmd_sent_ms;    // 紀錄指令發送時間
    bool _awaiting_ack;            // 是否正在等待 ACK
    bool _last_command_success;    // 紀錄最後一次指令執行結果
};