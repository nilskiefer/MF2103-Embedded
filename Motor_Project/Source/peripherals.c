#include "peripherals.h"
#include "main.h"
#include <stdint.h>

// Encoder on the lab kit is configured in TIM1 encoder mode.
// The encoder datasheet specifies PPR/turn; TIM encoder mode counts 4x (both edges on both channels).
#define ENCODER_PPR               512
#define ENCODER_COUNTS_PER_REV    (ENCODER_PPR * 4)

// Rolling window length target in ms. Bigger = less jitter, more delay.
int32_t VEL_WINDOW_MS = 90U;

// Control range specified by the assignment contract (Q30):
//  +1,073,741,823 = +100% duty
//  -1,073,741,824 = -100% duty
#define CTRL_MAX                  ((int32_t)0x3FFFFFFF)
#define CTRL_MIN                  ((int32_t)0xC0000000)

void Peripheral_GPIO_EnableMotor(void)
{
    // Write only to the GPIO data register (BSRR).
    MOTOR_EN1_GPIO_Port->BSRR = MOTOR_EN1_Pin;
    MOTOR_EN2_GPIO_Port->BSRR = MOTOR_EN2_Pin;
}

void Peripheral_GPIO_DisableMotor(void)
{
    MOTOR_EN1_GPIO_Port->BSRR = (uint32_t)MOTOR_EN1_Pin << 16U;
    MOTOR_EN2_GPIO_Port->BSRR = (uint32_t)MOTOR_EN2_Pin << 16U;
}

void Peripheral_PWM_ActuateMotor(int32_t control)
{
    // Map the signed Q30 control input to the PWM duty cycle:
    // duty = |control| / 2^30 * (ARR+1)

    uint32_t arr = (uint32_t)htim3.Instance->ARR;
    uint32_t top = arr + 1U;

    int32_t u = control;
    if (u > CTRL_MAX) u = CTRL_MAX;
    if (u < CTRL_MIN) u = CTRL_MIN;

    uint32_t mag;
    if (u == CTRL_MIN)
    {
        mag = (uint32_t)1U << 30U;   // abs(-2^30)
    }
    else
    {
        mag = (uint32_t)((u < 0) ? -u : u);
    }

    uint32_t duty = (uint32_t)(((uint64_t)mag * (uint64_t)top) >> 30U);
    if (duty > arr) duty = arr;

    if (u > 0)
    {
        // swapped
        htim3.Instance->CCR1 = 0U;
        htim3.Instance->CCR2 = duty;
    }
    else if (u < 0)
    {
        // swapped
        htim3.Instance->CCR1 = duty;
        htim3.Instance->CCR2 = 0U;
    }
    else
    {
        htim3.Instance->CCR1 = 0U;
        htim3.Instance->CCR2 = 0U;
    }
}

int32_t Peripheral_Encoder_CalculateVelocity(uint32_t ms)
{
    // Rolling window buffer size:
    // Must be large enough that BUF_N samples cover at least VEL_WINDOW_MS.
    enum { BUF_N = 32 };

    static int16_t  cnt_prev = 0;
    static uint32_t ms_prev  = 0;

    static int16_t  dx_buf[BUF_N];
    static uint16_t dt_buf[BUF_N];
    static uint8_t  idx = 0;
    static uint8_t  filled = 0;

    static int32_t  sum_dx = 0;
    static uint32_t sum_dt = 0;

    // Extra smoothing (IIR)
    static int32_t  vel_filt = 0;

    int16_t cnt = (int16_t)htim1.Instance->CNT;

    if (ms_prev == 0U)
    {
        cnt_prev = cnt;
        ms_prev  = ms;

        for (uint32_t i = 0; i < BUF_N; i++)
        {
            dx_buf[i] = 0;
            dt_buf[i] = 0;
        }

        idx = 0;
        filled = 0;
        sum_dx = 0;
        sum_dt = 0;
        vel_filt = 0;
        return 0;
    }

    uint32_t dt = ms - ms_prev;
    ms_prev = ms;
    if (dt == 0U) return vel_filt;

    // wraparound-safe 16-bit delta
    int16_t dx16 = (int16_t)(cnt - cnt_prev);
    cnt_prev = cnt;

    // Remove oldest sample at current index from the sums
    sum_dx -= (int32_t)dx_buf[idx];
    sum_dt -= (uint32_t)dt_buf[idx];

    // Insert newest sample
    dx_buf[idx] = dx16;
    dt_buf[idx] = (dt > 65535U) ? 65535U : (uint16_t)dt;

    sum_dx += (int32_t)dx_buf[idx];
    sum_dt += (uint32_t)dt_buf[idx];

    // Advance ring index
    idx++;
    if (idx >= BUF_N) idx = 0;
    if (filled < BUF_N) filled++;

    // Trim window down to about VEL_WINDOW_MS
    while (sum_dt > (uint32_t)VEL_WINDOW_MS && filled > 1)
    {
        // oldest element is at idx (the next one to be overwritten)
        sum_dx -= (int32_t)dx_buf[idx];
        sum_dt -= (uint32_t)dt_buf[idx];

        dx_buf[idx] = 0;
        dt_buf[idx] = 0;

        idx++;
        if (idx >= BUF_N) idx = 0;
        filled--;
    }

    if (sum_dt == 0U) return vel_filt;

    // rpm = sum_dx * 60000 / (counts_per_rev * sum_dt_ms)
    int64_t num = (int64_t)sum_dx * 60000LL;
    int64_t den = (int64_t)ENCODER_COUNTS_PER_REV * (int64_t)sum_dt;

    if (den == 0) return vel_filt;

    int32_t rpm = (int32_t)(num / den);

    // Low-delay smoothing on top of the rolling window
    // >>3 = 1/8 smoothing; if still noisy, try >>4.
    vel_filt += (rpm - vel_filt) >> 3;

    return vel_filt;
}
