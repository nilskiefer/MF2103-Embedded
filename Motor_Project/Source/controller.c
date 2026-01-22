// controller.c
#include "controller.h"
#include <stdint.h>

/* ----------------- Config (tune in Watch) ----------------- */
// Normalization: full-scale error for 1.0 in Q15
#define RPM_SCALE              4000

// Controller gains (Q15)
volatile int32_t Kp_q15        = 1000;
volatile int32_t Ki_q15        = 300;

// Feedforward (control units per rpm)
volatile int32_t U_PER_RPM     = 100000;

// Noise / smoothness knobs
volatile int32_t ERR_DEADBAND_RPM = 0;    // ignore +/- this rpm error

/* ----------------- Constants / state ----------------- */
#define CTRL_MAX   ((int32_t)0x3FFFFFFF)
#define CTRL_MIN   ((int32_t)0xC0000000)

static int32_t  i_q30 = 0;
static int32_t  u_prev_q30 = 0;
static uint32_t last_ms = 0;
static uint8_t  first_call = 1;

/* ----------------- Helpers ----------------- */
static inline int32_t sat_ctrl(int64_t x)
{
    if (x > (int64_t)CTRL_MAX) return CTRL_MAX;
    if (x < (int64_t)CTRL_MIN) return CTRL_MIN;
    return (int32_t)x;
}

static inline int32_t clamp_q15(int64_t x)
{
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return (int32_t)x;
}

static inline int32_t iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

/* ----------------- API ----------------- */
int32_t Controller_PIController(const int32_t* reference,
                                const int32_t* measured,
                                const uint32_t* millisec)
{
    if (first_call)
    {
        first_call = 0;
        last_ms = *millisec;
        i_q30 = 0;
        u_prev_q30 = 0;
        return 0;
    }

    uint32_t dt_ms = *millisec - last_ms;
    last_ms = *millisec;
    if (dt_ms == 0U) return u_prev_q30;

    int32_t r_rpm = *reference;
    int32_t y_rpm = *measured;
    int32_t e_rpm = r_rpm - y_rpm;

    if (iabs32(e_rpm) <= ERR_DEADBAND_RPM) e_rpm = 0;

    // Feedforward (Q30 units)
    int32_t u_ff_q30 = sat_ctrl((int64_t)U_PER_RPM * (int64_t)r_rpm);

    // Normalize error to Q15
    int32_t e_q15 = clamp_q15(((int64_t)e_rpm * 32768LL) / (int64_t)RPM_SCALE);

    // P: Q15*Q15 -> Q30
    int32_t p_q30 = sat_ctrl((int64_t)Kp_q15 * (int64_t)e_q15);

    // I: dt-aware integration in Q30 units
    int64_t di_q30 = ((int64_t)Ki_q15 * (int64_t)e_q15 * (int64_t)dt_ms) / 1000LL;
    int32_t i_candidate = sat_ctrl((int64_t)i_q30 + di_q30);

    // Candidate output (for anti-windup decision)
    int64_t u_cand_ll = (int64_t)u_ff_q30 + (int64_t)p_q30 + (int64_t)i_candidate;
    int32_t u_sat = sat_ctrl(u_cand_ll);

    // Anti-windup: freeze I if saturated and error pushes further into saturation
    if ((int64_t)u_sat != u_cand_ll)
    {
        if (!((u_cand_ll > (int64_t)CTRL_MAX && e_q15 > 0) ||
              (u_cand_ll < (int64_t)CTRL_MIN && e_q15 < 0)))
        {
            i_q30 = i_candidate;
        }
    }
    else
    {
        i_q30 = i_candidate;
    }

    int32_t u_q30 = sat_ctrl((int64_t)u_ff_q30 + (int64_t)p_q30 + (int64_t)i_q30);

    u_prev_q30 = u_q30;
    return u_q30;
}

void Controller_Reset(void)
{
    i_q30 = 0;
    u_prev_q30 = 0;
    last_ms = 0;
    first_call = 1;
}
