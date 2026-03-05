#include "AP_LandingAI.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_LandingAI::init()
{
    _healthy = false;
    _awaiting_ack = false;
    _last_cmd_sent_ms = 0;
    _last_msg_ms = 0;

    _roll_err = 0.0f;
    _pitch_err = 0.0f;
    _yaw_err = 0.0f;
    _distance = 0.0f;
    _confidence = 0.0f;
}

void AP_LandingAI::set_ai_target(uint8_t sysid, uint8_t compid)
{
    _ai_sysid = sysid;
    _ai_compid = compid;
}

bool AP_LandingAI::handle_start_command(const mavlink_command_int_t &packet)
{
    gcs().send_text(MAV_SEVERITY_INFO,
                    "AI Landing: Start Cmd Received (stream=%u,freq=%.1f,frame=%u)",
                    (unsigned)packet.param1,
                    (double)packet.param2,
                    (unsigned)packet.param3);

    send_start_landing_cmd((uint8_t)packet.param1, packet.param2, (uint8_t)packet.param3);
    return true;
}

bool AP_LandingAI::handle_stop_command(const mavlink_command_int_t &packet)
{
    (void)packet;
    gcs().send_text(MAV_SEVERITY_INFO, "AI Landing: Stop Cmd Received");
    send_stop_landing_cmd();
    return true;
}

void AP_LandingAI::send_start_landing_cmd(uint8_t stream_type, float freq_hz, uint8_t frame)
{
    const uint8_t target_system = _ai_sysid;
    const uint8_t target_component = _ai_compid;

    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        const mavlink_channel_t chan = (mavlink_channel_t)i;
        if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
            continue;
        }

        mavlink_msg_command_long_send(
            chan,
            target_system,
            target_component,
            31020,
            0,
            (float)stream_type,
            freq_hz,
            (float)frame,
            0.0f, 0.0f, 0.0f, 0.0f
        );
    }

    _last_cmd_sent_ms = AP_HAL::millis();
    _awaiting_ack = true;
}

void AP_LandingAI::send_stop_landing_cmd()
{
    const uint8_t target_system = _ai_sysid;
    const uint8_t target_component = _ai_compid;

    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        const mavlink_channel_t chan = (mavlink_channel_t)i;
        if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
            continue;
        }

        mavlink_msg_command_long_send(
            chan,
            target_system,
            target_component,
            31021,
            0,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
        );
    }

    _last_cmd_sent_ms = AP_HAL::millis();
    _awaiting_ack = true;
}

void AP_LandingAI::update()
{
    const uint32_t now = AP_HAL::millis();

    if (_awaiting_ack && (now - _last_cmd_sent_ms > ACK_TIMEOUT_MS)) {
        _awaiting_ack = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "AI Landing: Command ACK Timeout!");
    }

    if (_healthy && (now - _last_msg_ms > STREAM_TIMEOUT_MS)) {
        _healthy = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "AI Landing: Data Stream Lost!");
    }

    static uint32_t last_report_ms = 0;
    if (now - last_report_ms >= 1000) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "AI Status: Healthy=%d, Conf=%.2f",
                        (int)_healthy, (double)_confidence);
        send_status_to_gcs();
        last_report_ms = now;
    }
}

void AP_LandingAI::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        handle_command_ack(msg);
        return;
    }

    if (msg.msgid == 187) {
        handle_correction_187(msg);
        return;
    }
}

void AP_LandingAI::handle_command_ack(const mavlink_message_t &msg)
{
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);

    if (ack.command != 31020 && ack.command != 31021) {
        return;
    }

    _awaiting_ack = false;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "AI Landing: ACK cmd=%u result=%u err=%u",
                    (unsigned)ack.command,
                    (unsigned)ack.result,
                    (unsigned)ack.result_param2);
}

void AP_LandingAI::handle_correction_187(const mavlink_message_t &msg)
{
    mavlink_ai_landing_correction_t packet{};
    mavlink_msg_ai_landing_correction_decode(&msg, &packet);

    if (packet.flags == 0) {
        return;
    }

    _roll_err = packet.roll_err;
    _pitch_err = packet.pitch_err;
    _yaw_err = packet.yaw_err;
    _distance = packet.distance;
    _confidence = packet.confidence;

    _last_msg_ms = AP_HAL::millis();
    _healthy = true;
}

void AP_LandingAI::send_status_to_gcs()
{
    gcs().send_named_float("AI_CONF", _confidence);

    const uint8_t target_lost = _healthy ? 0 : 1;

    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        const mavlink_channel_t chan = (mavlink_channel_t)i;
        if (!HAVE_PAYLOAD_SPACE(chan, AI_LANDING_STATUS)) {
            continue;
        }

        // This uses ardupilotmega.xml's AI_LANDING_STATUS definition (id=188)
        mavlink_msg_ai_landing_status_send(
            chan,
            AP_HAL::millis(),
            _confidence,
            target_lost,
            0.0f,
            0.0f
        );
    }
}