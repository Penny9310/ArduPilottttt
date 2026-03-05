#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_LandingAI {
public:
    void init();
    void update();

    bool handle_start_command(const mavlink_command_int_t &packet);
    bool handle_stop_command(const mavlink_command_int_t &packet);

    void handle_msg(const mavlink_message_t &msg);

    void set_ai_target(uint8_t sysid, uint8_t compid);

private:
    void send_start_landing_cmd(uint8_t stream_type, float freq_hz, uint8_t frame);
    void send_stop_landing_cmd();

    void handle_command_ack(const mavlink_message_t &msg);
    void handle_correction_187(const mavlink_message_t &msg);

    void send_status_to_gcs();

private:
    uint8_t _ai_sysid = 42;
    uint8_t _ai_compid = 211;

    bool _healthy = false;
    bool _awaiting_ack = false;

    uint32_t _last_cmd_sent_ms = 0;
    uint32_t _last_msg_ms = 0;

    float _roll_err = 0.0f;
    float _pitch_err = 0.0f;
    float _yaw_err = 0.0f;
    float _distance = 0.0f;
    float _confidence = 0.0f;

    static constexpr uint32_t ACK_TIMEOUT_MS = 1000;
    static constexpr uint32_t STREAM_TIMEOUT_MS = 1000;
};