/*
 * Unit tests for COBS encoding/decoding (telemetry framing).
 */

#include <stdio.h>
#include <string.h>

#define UNIT_TEST
#include "../App/telemetry/telemetry.h"
#include "../App/telemetry/telemetry.c"

static int test_roundtrip_no_zeros(void)
{
    uint8_t input[] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t encoded[16], decoded[16];

    uint16_t enc_len = cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
    uint16_t dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));

    if (dec_len != sizeof(input) || memcmp(input, decoded, sizeof(input)) != 0) {
        printf("FAIL: roundtrip without zeros\n");
        return 1;
    }
    return 0;
}

static int test_roundtrip_with_zeros(void)
{
    uint8_t input[] = { 0x00, 0x01, 0x00, 0x02, 0x00 };
    uint8_t encoded[16], decoded[16];

    uint16_t enc_len = cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
    uint16_t dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));

    if (dec_len != sizeof(input) || memcmp(input, decoded, sizeof(input)) != 0) {
        printf("FAIL: roundtrip with zeros\n");
        return 1;
    }
    return 0;
}

static int test_roundtrip_all_zeros(void)
{
    uint8_t input[] = { 0x00, 0x00, 0x00, 0x00 };
    uint8_t encoded[16], decoded[16];

    uint16_t enc_len = cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
    uint16_t dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));

    if (dec_len != sizeof(input) || memcmp(input, decoded, sizeof(input)) != 0) {
        printf("FAIL: roundtrip all zeros\n");
        return 1;
    }
    return 0;
}

static int test_empty_input(void)
{
    uint8_t encoded[4], decoded[4];
    uint16_t enc_len = cobs_encode(NULL, 0, encoded, sizeof(encoded));
    uint16_t dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));

    if (dec_len != 0) {
        printf("FAIL: empty input should decode to empty\n");
        return 1;
    }
    return 0;
}

static int test_single_byte(void)
{
    uint8_t input[] = { 0x42 };
    uint8_t encoded[4], decoded[4];

    uint16_t enc_len = cobs_encode(input, 1, encoded, sizeof(encoded));
    uint16_t dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));

    if (dec_len != 1 || decoded[0] != 0x42) {
        printf("FAIL: single byte roundtrip\n");
        return 1;
    }
    return 0;
}

static int test_encoded_has_no_zeros(void)
{
    uint8_t input[] = { 0x00, 0x11, 0x00, 0x22, 0x00, 0x33 };
    uint8_t encoded[16];

    uint16_t enc_len = cobs_encode(input, sizeof(input), encoded, sizeof(encoded));

    for (uint16_t i = 0; i < enc_len; i++) {
        if (encoded[i] == 0x00) {
            printf("FAIL: encoded stream contains zero at index %d\n", i);
            return 1;
        }
    }
    return 0;
}

static int test_overflow_protection(void)
{
    uint8_t input[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
    uint8_t encoded[3]; // too small for 5-byte input

    uint16_t enc_len = cobs_encode(input, sizeof(input), encoded, sizeof(encoded));
    if (enc_len != 0) {
        printf("FAIL: should return 0 on output overflow\n");
        return 1;
    }

    // Decode overflow
    uint8_t valid_encoded[16];
    uint16_t valid_len = cobs_encode(input, sizeof(input), valid_encoded, sizeof(valid_encoded));
    uint8_t tiny_out[2];
    uint16_t dec_len = cobs_decode(valid_encoded, valid_len, tiny_out, sizeof(tiny_out));
    if (dec_len != 0) {
        printf("FAIL: decode should return 0 on output overflow\n");
        return 1;
    }
    return 0;
}

int main(void)
{
    int failures = 0;
    struct { const char *name; int (*fn)(void); } tests[] = {
        { "roundtrip_no_zeros", test_roundtrip_no_zeros },
        { "roundtrip_with_zeros", test_roundtrip_with_zeros },
        { "roundtrip_all_zeros", test_roundtrip_all_zeros },
        { "empty_input", test_empty_input },
        { "single_byte", test_single_byte },
        { "no_zeros_in_encoded", test_encoded_has_no_zeros },
        { "overflow_protection", test_overflow_protection },
    };

    int n = sizeof(tests) / sizeof(tests[0]);
    for (int i = 0; i < n; i++) {
        int result = tests[i].fn();
        printf("[%s] %s\n", result == 0 ? "PASS" : "FAIL", tests[i].name);
        failures += result;
    }

    printf("\n%d/%d tests passed\n", n - failures, n);
    return failures;
}
