// peripherals.c
#include "peripherals.h"
#include "main.h"
#include <stdint.h>

/* ----------------- Config (tune in Watch) ----------------- */
#define ENCODER_PPR             512
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4)

volatile int32_t g_vel_window_ms = 40U;   // rolling window target (ms)

// Extra smoothing after the rolling window:
// 3 => 1/8 smoothing, 4 => 1/16 smoothing (more smooth, more delay)
volatile int32_t g_vel_iir_shift = 3;

/* ----------------- Constants ----------------- */
#define CTRL_MAX                ((int32_t)0x3FFFFFFF)
#define CTRL_MIN                ((int32_t)0xC0000000)

/* ----------------- GPIO ----------------- */
void Peripheral_GPIO_EnableMotor(void)
{
    MOTOR_EN1_GPIO_Port->BSRR = MOTOR_EN1_Pin;
    MOTOR_EN2_GPIO_Port->BSRR = MOTOR_EN2_Pin;
}

void Peripheral_GPIO_DisableMotor(void)
{
    MOTOR_EN1_GPIO_Port->BSRR = (uint32_t)MOTOR_EN1_Pin << 16U;
    MOTOR_EN2_GPIO_Port->BSRR = (uint32_t)MOTOR_EN2_Pin << 16U;
}

/* ----------------- PWM ----------------- */
void Peripheral_PWM_ActuateMotor(int32_t control)
{
    uint32_t pwm_arr = (uint32_t)htim3.Instance->ARR;
    uint32_t pwm_top = pwm_arr + 1U;

    int32_t control_q30 = control;
    if (control_q30 > CTRL_MAX) control_q30 = CTRL_MAX;
    if (control_q30 < CTRL_MIN) control_q30 = CTRL_MIN;

    uint32_t control_mag;
    if (control_q30 == CTRL_MIN) control_mag = (uint32_t)1U << 30U;
    else                         control_mag = (uint32_t)((control_q30 < 0) ? -control_q30 : control_q30);

    uint32_t duty_counts = (uint32_t)(((uint64_t)control_mag * (uint64_t)pwm_top) >> 30U);
    if (duty_counts > pwm_arr) duty_counts = pwm_arr;

    if (control_q30 > 0)
    {
        htim3.Instance->CCR1 = 0U;
        htim3.Instance->CCR2 = duty_counts;
    }
    else if (control_q30 < 0)
    {
        htim3.Instance->CCR1 = duty_counts;
        htim3.Instance->CCR2 = 0U;
    }
    else
    {
        htim3.Instance->CCR1 = 0U;
        htim3.Instance->CCR2 = 0U;
    }
}

/* ----------------- Encoder velocity ----------------- */
int32_t Peripheral_Encoder_CalculateVelocity(uint32_t ms)
{
    enum { BUF_N = 32 };

    static int16_t  prev_count = 0;
    static uint32_t prev_ms    = 0;

    static int16_t  delta_count_buf[BUF_N];
    static uint16_t delta_ms_buf[BUF_N];
    static uint8_t  buf_index = 0;
    static uint8_t  buf_count = 0;

    static int32_t  sum_delta_count = 0;
    static uint32_t sum_delta_ms = 0;

    static int32_t  vel_rpm_filt = 0;

    int16_t count = (int16_t)htim1.Instance->CNT;

    if (prev_ms == 0U)
    {
        prev_count = count;
        prev_ms = ms;
        for (uint32_t i = 0; i < BUF_N; i++) { delta_count_buf[i] = 0; delta_ms_buf[i] = 0; }
        buf_index = 0;
        buf_count = 0;
        sum_delta_count = 0;
        sum_delta_ms = 0;
        vel_rpm_filt = 0;
        return 0;
    }

    uint32_t delta_ms = ms - prev_ms;
    prev_ms = ms;
    if (delta_ms == 0U) return vel_rpm_filt;

    int16_t delta_count = (int16_t)(count - prev_count);
    prev_count = count;

    // Remove old sample
    sum_delta_count -= (int32_t)delta_count_buf[buf_index];
    sum_delta_ms -= (uint32_t)delta_ms_buf[buf_index];

    // Add new sample
    delta_count_buf[buf_index] = delta_count;
    delta_ms_buf[buf_index] = (delta_ms > 65535U) ? 65535U : (uint16_t)delta_ms;

    sum_delta_count += (int32_t)delta_count_buf[buf_index];
    sum_delta_ms += (uint32_t)delta_ms_buf[buf_index];

    buf_index++;
    if (buf_index >= BUF_N) buf_index = 0;
    if (buf_count < BUF_N) buf_count++;

    // Trim to approx g_vel_window_ms
    while (sum_delta_ms > (uint32_t)g_vel_window_ms && buf_count > 1)
    {
        sum_delta_count -= (int32_t)delta_count_buf[buf_index];
        sum_delta_ms -= (uint32_t)delta_ms_buf[buf_index];
        delta_count_buf[buf_index] = 0;
        delta_ms_buf[buf_index] = 0;

        buf_index++;
        if (buf_index >= BUF_N) buf_index = 0;
        buf_count--;
    }

    if (sum_delta_ms == 0U) return vel_rpm_filt;

    int64_t rpm_num = (int64_t)sum_delta_count * 60000LL;
    int64_t rpm_den = (int64_t)ENCODER_COUNTS_PER_REV * (int64_t)sum_delta_ms;
    if (rpm_den == 0) return vel_rpm_filt;

    int32_t rpm_est = (int32_t)(rpm_num / rpm_den);

    int32_t iir_shift = g_vel_iir_shift;
    if (iir_shift < 0) iir_shift = 0;
    if (iir_shift > 8) iir_shift = 8;

    vel_rpm_filt += (rpm_est - vel_rpm_filt) >> iir_shift;
    return vel_rpm_filt;
}
