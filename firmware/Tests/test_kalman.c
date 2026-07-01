/*
 * Unit tests for Kalman filter — validates against known trajectories.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#define UNIT_TEST
#include "../App/control/kalman.h"
#include "../App/control/kalman.c"

#define ASSERT_NEAR(a, b, tol) \
    do { \
        float _a = (a), _b = (b); \
        if (fabsf(_a - _b) > (tol)) { \
            printf("FAIL: %s:%d: %.6f != %.6f (tol %.6f)\n", \
                   __FILE__, __LINE__, _a, _b, (tol)); \
            return 1; \
        } \
    } while (0)

static int test_init_state(void)
{
    kalman_filter_t kf;
    kalman_init(&kf, 0.001f);
    ASSERT_NEAR(kf.x[0], 0.0f, 0.01f);
    ASSERT_NEAR(kf.x[1], 0.0f, 0.01f);
    ASSERT_NEAR(kf.x[2], 10.0f, 0.01f); // default resistance
    return 0;
}

static int test_predict_increases_pressure_with_pump(void)
{
    kalman_filter_t kf;
    kalman_init(&kf, 0.001f);

    float p0 = kf.x[0];
    kalman_predict(&kf, 0.8f); // 80% pump duty
    float p1 = kf.x[0];

    if (p1 <= p0) {
        printf("FAIL: pressure should increase with pump on: %.4f -> %.4f\n", p0, p1);
        return 1;
    }
    return 0;
}

static int test_update_corrects_toward_measurement(void)
{
    kalman_filter_t kf;
    kalman_init(&kf, 0.001f);

    // Set state far from measurement
    kf.x[0] = 0.0f;
    kf.x[1] = 0.0f;

    // Measure 5 bar, 2 mL/s
    for (int i = 0; i < 100; i++) {
        kalman_predict(&kf, 0.0f);
        kalman_update(&kf, 5.0f, 2.0f);
    }

    // Should converge toward measurements
    if (fabsf(kf.x[0] - 5.0f) > 1.0f) {
        printf("FAIL: pressure should converge to ~5.0, got %.4f\n", kf.x[0]);
        return 1;
    }
    if (fabsf(kf.x[1] - 2.0f) > 1.0f) {
        printf("FAIL: flow should converge to ~2.0, got %.4f\n", kf.x[1]);
        return 1;
    }
    return 0;
}

static int test_resistance_estimate_converges(void)
{
    kalman_filter_t kf;
    kalman_init(&kf, 0.001f);
    kf.x[2] = 5.0f; // start with wrong resistance

    // Simulate steady state: pressure=9, flow=1.5 => R_true = 6.0
    float true_r = 6.0f;
    for (int i = 0; i < 2000; i++) {
        kalman_predict(&kf, 0.5f);
        kalman_update(&kf, 1.5f * true_r, 1.5f);
    }

    float est_r = kalman_get_resistance(&kf);
    if (fabsf(est_r - true_r) > 3.0f) {
        printf("FAIL: resistance should converge toward %.1f, got %.4f\n", true_r, est_r);
        return 1;
    }
    return 0;
}

static int test_reset(void)
{
    kalman_filter_t kf;
    kalman_init(&kf, 0.001f);
    kf.x[0] = 9.0f;
    kf.x[1] = 3.0f;
    kf.x[2] = 15.0f;

    kalman_reset(&kf);
    ASSERT_NEAR(kf.x[0], 0.0f, 0.01f);
    ASSERT_NEAR(kf.x[2], 10.0f, 0.01f);
    return 0;
}

int main(void)
{
    int failures = 0;
    struct { const char *name; int (*fn)(void); } tests[] = {
        { "init_state", test_init_state },
        { "predict_pressure_increase", test_predict_increases_pressure_with_pump },
        { "update_corrects_state", test_update_corrects_toward_measurement },
        { "resistance_converges", test_resistance_estimate_converges },
        { "reset", test_reset },
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
