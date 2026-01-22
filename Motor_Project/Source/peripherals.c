// peripherals.c
#include "peripherals.h"
#include "main.h"
#include <stdint.h>

/* ----------------- Config (tune in Watch) ----------------- */
#define ENCODER_PPR             512
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4)

volatile int32_t VEL_WINDOW_MS = 40U;   // rolling window target (ms)

// Extra smoothing after the rolling window:
// 3 => 1/8 smoothing, 4 => 1/16 smoothing (more smooth, more delay)
volatile int32_t VEL_IIR_SHIFT = 3;

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
    uint32_t arr = (uint32_t)htim3.Instance->ARR;
    uint32_t top = arr + 1U;

    int32_t u = control;
    if (u > CTRL_MAX) u = CTRL_MAX;
    if (u < CTRL_MIN) u = CTRL_MIN;

    uint32_t mag;
    if (u == CTRL_MIN) mag = (uint32_t)1U << 30U;
    else              mag = (uint32_t)((u < 0) ? -u : u);

    uint32_t duty = (uint32_t)(((uint64_t)mag * (uint64_t)top) >> 30U);
    if (duty > arr) duty = arr;

    if (u > 0)
    {
        htim3.Instance->CCR1 = 0U;
        htim3.Instance->CCR2 = duty;
    }
    else if (u < 0)
    {
        htim3.Instance->CCR1 = duty;
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

    static int16_t  cnt_prev = 0;
    static uint32_t ms_prev  = 0;

    static int16_t  dx_buf[BUF_N];
    static uint16_t dt_buf[BUF_N];
    static uint8_t  idx = 0;
    static uint8_t  filled = 0;

    static int32_t  sum_dx = 0;
    static uint32_t sum_dt = 0;

    static int32_t  vel_filt = 0;

    int16_t cnt = (int16_t)htim1.Instance->CNT;

    if (ms_prev == 0U)
    {
        cnt_prev = cnt;
        ms_prev = ms;
        for (uint32_t i = 0; i < BUF_N; i++) { dx_buf[i] = 0; dt_buf[i] = 0; }
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

    int16_t dx16 = (int16_t)(cnt - cnt_prev);
    cnt_prev = cnt;

    // Remove old sample
    sum_dx -= (int32_t)dx_buf[idx];
    sum_dt -= (uint32_t)dt_buf[idx];

    // Add new sample
    dx_buf[idx] = dx16;
    dt_buf[idx] = (dt > 65535U) ? 65535U : (uint16_t)dt;

    sum_dx += (int32_t)dx_buf[idx];
    sum_dt += (uint32_t)dt_buf[idx];

    idx++;
    if (idx >= BUF_N) idx = 0;
    if (filled < BUF_N) filled++;

    // Trim to approx VEL_WINDOW_MS
    while (sum_dt > (uint32_t)VEL_WINDOW_MS && filled > 1)
    {
        sum_dx -= (int32_t)dx_buf[idx];
        sum_dt -= (uint32_t)dt_buf[idx];
        dx_buf[idx] = 0;
        dt_buf[idx] = 0;

        idx++;
        if (idx >= BUF_N) idx = 0;
        filled--;
    }

    if (sum_dt == 0U) return vel_filt;

    int64_t num = (int64_t)sum_dx * 60000LL;
    int64_t den = (int64_t)ENCODER_COUNTS_PER_REV * (int64_t)sum_dt;
    if (den == 0) return vel_filt;

    int32_t rpm = (int32_t)(num / den);

    int32_t sh = VEL_IIR_SHIFT;
    if (sh < 0) sh = 0;
    if (sh > 8) sh = 8;

    vel_filt += (rpm - vel_filt) >> sh;
    return vel_filt;
}
