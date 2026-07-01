#include "telemetry.h"
#include <string.h>

uint16_t cobs_encode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t output_max)
{
    if (output_max < 2)
        return 0;

    uint16_t read_idx = 0;
    uint16_t write_idx = 1;
    uint16_t code_idx = 0;
    uint8_t code = 1;

    while (read_idx < len) {
        if (input[read_idx] == 0x00) {
            if (code_idx >= output_max) return 0;
            output[code_idx] = code;
            if (write_idx >= output_max) return 0;
            code_idx = write_idx++;
            code = 1;
        } else {
            if (write_idx >= output_max) return 0;
            output[write_idx++] = input[read_idx];
            code++;
            if (code == 0xFF) {
                if (code_idx >= output_max) return 0;
                output[code_idx] = code;
                if (write_idx >= output_max) return 0;
                code_idx = write_idx++;
                code = 1;
            }
        }
        read_idx++;
    }
    if (code_idx >= output_max) return 0;
    output[code_idx] = code;
    return write_idx;
}

uint16_t cobs_decode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t output_max)
{
    uint16_t read_idx = 0;
    uint16_t write_idx = 0;

    while (read_idx < len) {
        uint8_t code = input[read_idx++];
        if (code == 0)
            break;

        for (uint8_t i = 1; i < code && read_idx < len; i++) {
            if (write_idx >= output_max) return 0;
            output[write_idx++] = input[read_idx++];
        }

        if (code < 0xFF && read_idx < len) {
            if (write_idx >= output_max) return 0;
            output[write_idx++] = 0x00;
        }
    }

    if (write_idx > 0 && output[write_idx - 1] == 0x00)
        write_idx--;

    return write_idx;
}

void telemetry_init(telemetry_ctx_t *ctx, void (*send_fn)(const uint8_t *, uint16_t))
{
    memset(ctx, 0, sizeof(telemetry_ctx_t));
    ctx->send_bytes = send_fn;
}

void telemetry_send_state(telemetry_ctx_t *ctx, const telem_state_packet_t *pkt)
{
    if (!ctx->send_bytes)
        return;

    uint8_t raw[sizeof(telem_state_packet_t)];
    memcpy(raw, pkt, sizeof(raw));

    uint16_t encoded_len = cobs_encode(raw, sizeof(raw), ctx->frame_buf,
                                        TELEM_MAX_FRAME - 1);
    if (encoded_len == 0 || encoded_len >= TELEM_MAX_FRAME)
        return;

    ctx->frame_buf[encoded_len] = 0x00;
    encoded_len++;

    ctx->frame_len = encoded_len;
    ctx->send_bytes(ctx->frame_buf, encoded_len);
}

bool telemetry_decode_frame(const uint8_t *frame, uint16_t frame_len,
                            uint8_t *out, uint16_t out_max, uint16_t *out_len)
{
    if (frame_len < 2)
        return false;

    uint16_t len = frame_len;
    if (frame[len - 1] == 0x00)
        len--;

    *out_len = cobs_decode(frame, len, out, out_max);
    return *out_len > 0;
}
