#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "../sensors/sensors.h"

#define TELEM_MAX_FRAME 128

// Packet types
#define TELEM_PKT_STATE     0x01
#define TELEM_PKT_GAINS     0x02
#define TELEM_PKT_CMD       0x03
#define TELEM_PKT_LOG       0x04

#pragma pack(push, 1)
typedef struct {
    uint8_t  packet_type;
    uint32_t timestamp_ms;
    float    pressure_bar;
    float    flow_ml_s;
    float    temperature_c;
    float    pump_duty;
    float    heater_duty;
    float    setpoint_pressure;
    float    est_resistance;
    uint8_t  phase;
    uint8_t  machine_state;
} telem_state_packet_t;

typedef struct {
    uint8_t  packet_type;
    float    kp;
    float    ki;
    float    kd;
    float    setpoint;
} telem_gains_cmd_t;
#pragma pack(pop)

typedef struct {
    uint8_t frame_buf[TELEM_MAX_FRAME];
    uint16_t frame_len;
    void (*send_bytes)(const uint8_t *data, uint16_t len);
} telemetry_ctx_t;

void telemetry_init(telemetry_ctx_t *ctx, void (*send_fn)(const uint8_t *, uint16_t));
void telemetry_send_state(telemetry_ctx_t *ctx, const telem_state_packet_t *pkt);
bool telemetry_decode_frame(const uint8_t *frame, uint16_t frame_len,
                            uint8_t *out, uint16_t out_max, uint16_t *out_len);

// COBS encoding/decoding — output_max prevents buffer overflows
uint16_t cobs_encode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t output_max);
uint16_t cobs_decode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t output_max);

#endif
