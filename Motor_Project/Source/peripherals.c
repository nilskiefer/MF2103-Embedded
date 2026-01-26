// peripherals.c
#include "peripherals.h"
#include "main.h"
#include <stdint.h>

// This file provides hardware access for:
//  - GPIO motor enable pins
//  - PWM outputs (Timer 3)
//  - Encoder counter and velocity estimation (Timer 1)
// Everything is done with integer math (no floating point).

/* ----------------- Units & scaling ----------------- */

// Control input uses signed Q30: full scale = [-2^30, 2^30-1]
// Fixed-point is used here because the assignment forbids float usage.
#define CTRL_Q 30
#define CTRL_MAX ((int32_t)0x3FFFFFFF)
#define CTRL_MIN ((int32_t)0xC0000000)
#define CTRL_MAG_MAX ((uint32_t)(1UL << CTRL_Q))

/* ----------------- Config (tune in Watch) ----------------- */

// Encoder resolution (quadrature decoding => 4x)
#define ENCODER_PPR 512
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * 4)

// Rolling window target (ms) for velocity estimation.
volatile int32_t g_vel_window_ms = 40U;

// Raw (unaveraged) velocity in RPM for debugging/Watch.
volatile int32_t g_vel_raw_rpm = 0;


/* ----------------- Aliases ----------------- */

// Aliases make the intent clearer at call sites.
#define ENC_TIMER htim1
#define PWM_TIMER htim3

/* ----------------- Helpers ----------------- */

// Set a GPIO pin using the atomic BSRR register.
static inline void gpio_set(GPIO_TypeDef *port, uint16_t pin) {
    port->BSRR = pin;
}

// Clear a GPIO pin using the upper half of BSRR.
static inline void gpio_clear(GPIO_TypeDef *port, uint16_t pin) {
    port->BSRR = (uint32_t)pin << 16U;
}

// Saturate controller input to the allowed Q30 range.
static inline int32_t clamp_ctrl(int32_t x) {
    if (x > CTRL_MAX)
        return CTRL_MAX;
    if (x < CTRL_MIN)
        return CTRL_MIN;
    return x;
}

// Convert Q30 control value to timer counts in range [0, ARR].
static inline uint32_t ctrl_to_counts(int32_t ctrl, uint32_t top) {
    const int32_t sat = clamp_ctrl(ctrl);
    // Handle CTRL_MIN specially to avoid overflow when negating.
    uint32_t mag = 0U;
    if (sat == CTRL_MIN) {
        mag = CTRL_MAG_MAX;
    } else {
        if (sat < 0) {
            mag = (uint32_t)(-sat);
        } else {
            mag = (uint32_t)sat;
        }
    }
    uint32_t duty = (uint32_t)(((uint64_t)mag * (uint64_t)top) >> CTRL_Q);
    if (duty > (top - 1U))
        duty = top - 1U;
    return duty;
}

/* ----------------- GPIO ----------------- */
void Peripheral_GPIO_EnableMotor(void) {
    // Enable both half-bridges on the motor driver.
    gpio_set(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin);
    gpio_set(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin);
}

void Peripheral_GPIO_DisableMotor(void) {
    // Disable both half-bridges (motor coasts).
    gpio_clear(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin);
    gpio_clear(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin);
}

/* ----------------- PWM ----------------- */
void Peripheral_PWM_ActuateMotor(int32_t control) {
    // ARR is the timer period, so top = ARR + 1 counts.
    const uint32_t pwm_arr = (uint32_t)PWM_TIMER.Instance->ARR;
    const uint32_t pwm_top = pwm_arr + 1U;
    const uint32_t duty_counts = ctrl_to_counts(control, pwm_top);

    // Direction is set by choosing which PWM channel is active.
    if (control > 0) {
        // Clockwise: use CCR2, keep CCR1 low.
        PWM_TIMER.Instance->CCR1 = 0U;
        PWM_TIMER.Instance->CCR2 = duty_counts;
    } else if (control < 0) {
        // Counter-clockwise: use CCR1, keep CCR2 low.
        PWM_TIMER.Instance->CCR1 = duty_counts;
        PWM_TIMER.Instance->CCR2 = 0U;
    } else {
        // Zero -> motor off.
        PWM_TIMER.Instance->CCR1 = 0U;
        PWM_TIMER.Instance->CCR2 = 0U;
    }
}

/* ----------------- Encoder velocity ----------------- */
int32_t Peripheral_Encoder_CalculateVelocity(uint32_t ms) {
    enum { BUF_N = 32 };

    // Previous raw encoder count (16-bit hardware counter).
    static int16_t prev_count = 0;
    // Previous time (ms).
    static uint32_t prev_ms = 0;

    // Circular buffers for delta counts and delta time.
    static int16_t delta_count_buf[BUF_N];
    static uint16_t delta_ms_buf[BUF_N];
    static uint8_t buf_index = 0;
    static uint8_t buf_count = 0;

    // Rolling sums for the active window.
    static int32_t sum_delta_count = 0;
    static uint32_t sum_delta_ms = 0;

    // Last calculated velocity (RPM).
    static int32_t vel_rpm = 0;

    // Encoder counter is 16-bit; cast preserves wrap-around behavior.
    const int16_t count = (int16_t)ENC_TIMER.Instance->CNT;

    if (prev_ms == 0U) {
        // First call initialization: zero history and return 0.
        prev_count = count;
        prev_ms = ms;
        for (uint32_t i = 0; i < BUF_N; i++) {
            delta_count_buf[i] = 0;
            delta_ms_buf[i] = 0;
        }
        buf_index = 0;
        buf_count = 0;
        sum_delta_count = 0;
        sum_delta_ms = 0;
        vel_rpm = 0;
        return 0;
    }

    // Time delta; unsigned subtraction handles wrap-around of ms counter.
    const uint32_t delta_ms = ms - prev_ms;
    prev_ms = ms;
    if (delta_ms == 0U)
        return vel_rpm;

    // Signed subtraction handles counter wrap-around correctly.
    const int16_t delta_count = (int16_t)(count - prev_count);
    prev_count = count;

    // Remove old sample
    sum_delta_count -= (int32_t)delta_count_buf[buf_index];
    sum_delta_ms -= (uint32_t)delta_ms_buf[buf_index];

    // Add new sample
    delta_count_buf[buf_index] = delta_count;
    if (delta_ms > 65535U) {
        delta_ms_buf[buf_index] = 65535U;
    } else {
        delta_ms_buf[buf_index] = (uint16_t)delta_ms;
    }
    sum_delta_count += (int32_t)delta_count_buf[buf_index];
    sum_delta_ms += (uint32_t)delta_ms_buf[buf_index];

    buf_index++;
    if (buf_index >= BUF_N)
        buf_index = 0;
    if (buf_count < BUF_N)
        buf_count++;

    // Trim to approx g_vel_window_ms by removing oldest samples.
    while (sum_delta_ms > (uint32_t)g_vel_window_ms && buf_count > 1) {
        sum_delta_count -= (int32_t)delta_count_buf[buf_index];
        sum_delta_ms -= (uint32_t)delta_ms_buf[buf_index];
        delta_count_buf[buf_index] = 0;
        delta_ms_buf[buf_index] = 0;

        buf_index++;
        if (buf_index >= BUF_N)
            buf_index = 0;
        buf_count--;
    }

    if (sum_delta_ms == 0U)
        return vel_rpm;

    // RPM estimate:
    //   counts per window -> revolutions per minute
    const int64_t rpm_num = (int64_t)sum_delta_count * 60000LL;
    const int64_t rpm_den = (int64_t)ENCODER_COUNTS_PER_REV * (int64_t)sum_delta_ms;
    if (rpm_den == 0)
        return vel_rpm;

    const int32_t rpm_est = (int32_t)(rpm_num / rpm_den);

    // Raw (unaveraged) velocity for debugging/Watch.
    g_vel_raw_rpm = (int32_t)((int64_t)delta_count * 60000LL /
                              ((int64_t)ENCODER_COUNTS_PER_REV * (int64_t)delta_ms));

    // Rolling average output (no extra IIR smoothing).
    vel_rpm = rpm_est;
    return vel_rpm;
}
