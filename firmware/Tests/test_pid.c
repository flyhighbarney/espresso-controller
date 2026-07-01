/*
 * Unit tests for PID controller — host-runnable (compile with gcc, no HAL).
 * Uses a minimal test harness (no external framework dependency).
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

#define UNIT_TEST
#include "../App/control/pid.h"
#include "../App/control/pid.c"

#define ASSERT_NEAR(a, b, tol) \
    do { \
        float _a = (a), _b = (b); \
        if (fabsf(_a - _b) > (tol)) { \
            printf("FAIL: %s:%d: %.6f != %.6f (tol %.6f)\n", \
                   __FILE__, __LINE__, _a, _b, (tol)); \
            return 1; \
        } \
    } while (0)

static int test_proportional_only(void)
{
    pid_controller_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.001f);
    pid_set_limits(&pid, -10.0f, 10.0f);

    float out = pid_update(&pid, 5.0f, 3.0f); // error = 2
    ASSERT_NEAR(out, 4.0f, 0.001f); // kp * error = 2 * 2
    return 0;
}

static int test_integral_accumulation(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 10.0f, 0.0f, 0.01f);
    pid_set_limits(&pid, -100.0f, 100.0f);

    // Apply constant error=1.0 for 100 steps
    float out = 0.0f;
    for (int i = 0; i < 100; i++)
        out = pid_update(&pid, 1.0f, 0.0f);

    // integral = 100 * 1.0 * 0.01 = 1.0, output = ki * integral = 10
    ASSERT_NEAR(out, 10.0f, 0.1f);
    return 0;
}

static int test_output_clamping(void)
{
    pid_controller_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.001f);
    pid_set_limits(&pid, 0.0f, 1.0f);

    float out = pid_update(&pid, 10.0f, 0.0f);
    ASSERT_NEAR(out, 1.0f, 0.001f); // clamped to max

    out = pid_update(&pid, 0.0f, 10.0f);
    ASSERT_NEAR(out, 0.0f, 0.001f); // clamped to min
    return 0;
}

static int test_reset(void)
{
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 0.01f);
    pid_set_limits(&pid, -100.0f, 100.0f);

    for (int i = 0; i < 50; i++)
        pid_update(&pid, 5.0f, 0.0f);

    pid_reset(&pid);
    ASSERT_NEAR(pid.integral, 0.0f, 0.001f);
    ASSERT_NEAR(pid.prev_error, 0.0f, 0.001f);
    return 0;
}

static int test_derivative_kick(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.01f);
    pid_set_limits(&pid, -1000.0f, 1000.0f);

    pid_update(&pid, 0.0f, 0.0f); // first call, derivative = 0
    float out = pid_update(&pid, 5.0f, 0.0f); // error jumps to 5

    // derivative = (5 - 0) / 0.01 = 500, output = kd * deriv = 500
    ASSERT_NEAR(out, 500.0f, 0.1f);
    return 0;
}

static int test_integral_windup_limit(void)
{
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 100.0f, 0.0f, 0.01f);
    pid_set_limits(&pid, -1000.0f, 1000.0f);
    pid_set_integral_limit(&pid, 5.0f);

    for (int i = 0; i < 1000; i++)
        pid_update(&pid, 10.0f, 0.0f);

    // integral should be clamped to 5.0
    ASSERT_NEAR(pid.integral, 5.0f, 0.001f);
    return 0;
}

int main(void)
{
    int failures = 0;
    struct { const char *name; int (*fn)(void); } tests[] = {
        { "proportional_only", test_proportional_only },
        { "integral_accumulation", test_integral_accumulation },
        { "output_clamping", test_output_clamping },
        { "reset", test_reset },
        { "derivative_kick", test_derivative_kick },
        { "integral_windup_limit", test_integral_windup_limit },
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
