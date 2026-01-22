#include "controller.h"
#include <stdint.h>

/* ===================== Config (tune in Watch) ===================== */

// Normalize rpm error into Q15
#define RPM_SCALE  4000

// PI gains in Q15 (0..32767 ~ 0..1.0)
volatile int32_t Kp_q15 = 1300;
volatile int32_t Ki_q15 = 4000;     // start here once P is stable

// If you want PI-only, keep this 0
#define USE_FEEDFORWARD  1
volatile int32_t U_PER_RPM = 99000; // only used if USE_FEEDFORWARD=1

// Noise handling
volatile int32_t ERR_DEADBAND_RPM = 10;   // ignore tiny error (helps jitter)

// Integrate only when close to target:
// window = max(ABS_INT_WINDOW_RPM, |ref| * INT_WINDOW_PCT / 100)
volatile int32_t INT_WINDOW_PCT = 10;     // e.g. 10% of reference
volatile int32_t ABS_INT_WINDOW_RPM = 80; // minimum window so low refs still integrate

// Clamp integrator to prevent overflow / windup (Q30 units)
volatile int32_t I_CLAMP_Q30 = 300000000;

/* ===================== Constants / state ===================== */

#define CTRL_MAX   ((int32_t)0x3FFFFFFF)
#define CTRL_MIN   ((int32_t)0xC0000000)

static int32_t  i_q30 = 0;
static uint32_t last_ms = 0;
static uint8_t  first_call = 1;

/* ===================== Helpers ===================== */

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

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/* ===================== API ===================== */

int32_t Controller_PIController(const int32_t* reference,
                                const int32_t* measured,
                                const uint32_t* millisec)
{
    if (first_call)
    {
        first_call = 0;
        last_ms = *millisec;
        i_q30 = 0;
        return 0;
    }

    uint32_t dt_ms = *millisec - last_ms;
    last_ms = *millisec;
    if (dt_ms == 0U) return 0;

    int32_t r_rpm = *reference;
    int32_t y_rpm = *measured;
    int32_t e_rpm = r_rpm - y_rpm;

    // Deadband for noise
    if (iabs32(e_rpm) <= ERR_DEADBAND_RPM)
        e_rpm = 0;

    // Optional feedforward
    int32_t u_ff_q30 = 0;
#if USE_FEEDFORWARD
    u_ff_q30 = sat_ctrl((int64_t)U_PER_RPM * (int64_t)r_rpm);
#endif

    // Normalize error to Q15
    int32_t e_q15 = clamp_q15(((int64_t)e_rpm * 32768LL) / (int64_t)RPM_SCALE);

    // P term: Q15*Q15 -> Q30 units
    int32_t p_q30 = sat_ctrl((int64_t)Kp_q15 * (int64_t)e_q15);

    // Integration window scales with reference magnitude
    int32_t r_abs = iabs32(r_rpm);
    int32_t win_rpm = (int32_t)(((int64_t)r_abs * (int64_t)INT_WINDOW_PCT) / 100LL);
    if (win_rpm < ABS_INT_WINDOW_RPM) win_rpm = ABS_INT_WINDOW_RPM;

    // I update only when close enough
    int32_t i_candidate = i_q30;
    if (iabs32(e_rpm) <= win_rpm)
    {
        int64_t di_q30 = ((int64_t)Ki_q15 * (int64_t)e_q15 * (int64_t)dt_ms) / 1000LL;
        i_candidate = sat_ctrl((int64_t)i_q30 + di_q30);
        i_candidate = clamp_i32(i_candidate, -I_CLAMP_Q30, I_CLAMP_Q30);
    }

    // Anti-windup using saturation check
    int64_t u_cand_ll = (int64_t)u_ff_q30 + (int64_t)p_q30 + (int64_t)i_candidate;
    int32_t u_sat = sat_ctrl(u_cand_ll);

    if ((int64_t)u_sat == u_cand_ll)
    {
        i_q30 = i_candidate;
    }
    else
    {
        // If saturated and error pushes further into saturation, freeze I
        if (!((u_cand_ll > (int64_t)CTRL_MAX && e_q15 > 0) ||
              (u_cand_ll < (int64_t)CTRL_MIN && e_q15 < 0)))
        {
            i_q30 = i_candidate;
        }
    }

    return sat_ctrl((int64_t)u_ff_q30 + (int64_t)p_q30 + (int64_t)i_q30);
}

void Controller_Reset(void)
{
    i_q30 = 0;
    last_ms = 0;
    first_call = 1;
}
