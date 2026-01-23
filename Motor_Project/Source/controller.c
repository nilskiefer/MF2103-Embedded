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

static int32_t  integrator_q30 = 0;
static uint32_t last_update_ms = 0;
static uint8_t  is_first_call = 1;

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
    if (is_first_call)
    {
        is_first_call = 0;
        last_update_ms = *millisec;
        integrator_q30 = 0;
        return 0;
    }

    uint32_t delta_ms = *millisec - last_update_ms;
    last_update_ms = *millisec;
    if (delta_ms == 0U) return 0;

    int32_t ref_rpm = *reference;
    int32_t meas_rpm = *measured;
    int32_t err_rpm = ref_rpm - meas_rpm;

    // Deadband for noise
    if (iabs32(err_rpm) <= ERR_DEADBAND_RPM)
        err_rpm = 0;

    // Optional feedforward
    int32_t ff_q30 = 0;
#if USE_FEEDFORWARD
    ff_q30 = sat_ctrl((int64_t)U_PER_RPM * (int64_t)ref_rpm);
#endif

    // Normalize error to Q15
    int32_t err_q15 = clamp_q15(((int64_t)err_rpm * 32768LL) / (int64_t)RPM_SCALE);

    // P term: Q15*Q15 -> Q30 units
    int32_t p_term_q30 = sat_ctrl((int64_t)Kp_q15 * (int64_t)err_q15);

    // Integration window scales with reference magnitude
    int32_t ref_abs = iabs32(ref_rpm);
    int32_t int_window_rpm = (int32_t)(((int64_t)ref_abs * (int64_t)INT_WINDOW_PCT) / 100LL);
    if (int_window_rpm < ABS_INT_WINDOW_RPM) int_window_rpm = ABS_INT_WINDOW_RPM;

    // I update only when close enough
    int32_t integrator_candidate = integrator_q30;
    if (iabs32(err_rpm) <= int_window_rpm)
    {
        int64_t di_q30 = ((int64_t)Ki_q15 * (int64_t)err_q15 * (int64_t)delta_ms) / 1000LL;
        integrator_candidate = sat_ctrl((int64_t)integrator_q30 + di_q30);
        integrator_candidate = clamp_i32(integrator_candidate, -I_CLAMP_Q30, I_CLAMP_Q30);
    }

    // Anti-windup using saturation check
    int64_t control_candidate_ll = (int64_t)ff_q30 + (int64_t)p_term_q30 + (int64_t)integrator_candidate;
    int32_t control_sat = sat_ctrl(control_candidate_ll);

    if ((int64_t)control_sat == control_candidate_ll)
    {
        integrator_q30 = integrator_candidate;
    }
    else
    {
        // If saturated and error pushes further into saturation, freeze I
        if (!((control_candidate_ll > (int64_t)CTRL_MAX && err_q15 > 0) ||
              (control_candidate_ll < (int64_t)CTRL_MIN && err_q15 < 0)))
        {
            integrator_q30 = integrator_candidate;
        }
    }

    return sat_ctrl((int64_t)ff_q30 + (int64_t)p_term_q30 + (int64_t)integrator_q30);
}

void Controller_Reset(void)
{
    integrator_q30 = 0;
    last_update_ms = 0;
    is_first_call = 1;
}
