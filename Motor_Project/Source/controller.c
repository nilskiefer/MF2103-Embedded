#include "controller.h"
#include <stdint.h>

// This file implements a PI controller using ONLY integer math.
// The controller output is in Q30 fixed-point format:
//   +2^30-1  => +100% duty (full clockwise)
//   -2^30    => -100% duty (full counter-clockwise)
// The application calls Controller_PIController() periodically and provides time.

/* ===================== Units & scaling ===================== */

// Internal control uses signed Q30: full scale = [-2^30, 2^30-1]
// We use fixed-point (Q30/Q15) because the task forbids floating point,
// and fixed-point gives predictable, efficient math on the MCU.
#define CTRL_Q          30
#define CTRL_MAX        ((int32_t)0x3FFFFFFF)
#define CTRL_MIN        ((int32_t)0xC0000000)
#define Q15_ONE         32768

/* ===================== Config (tune in Watch) ===================== */

// Normalize RPM error into Q15 before applying gains.
// Example: RPM_SCALE = 4000 means 4000 RPM maps to ~1.0 in Q15.
#define RPM_SCALE  4000

// PI gains in Q15 (0..32767 ~ 0..1.0)
volatile int32_t Kp = 1300;
volatile int32_t Ki = 4000;       // start here once P is stable

// Feedforward: set to 0 to disable. Units: Q30 per RPM.
volatile int32_t U_PER_RPM = 99000;

// Noise handling
volatile int32_t ERR_DEADBAND_RPM = 10;   // ignore tiny error (helps jitter)

// Integrate only when close to target:
// if |error| <= INT_WINDOW_RPM then integrator updates
volatile int32_t INT_WINDOW_RPM = 200;

// Clamp integrator to prevent overflow / windup (Q30 units)
volatile int32_t I_CLAMP = 300000000;

/* ===================== Controller state ===================== */

// Integrator state in Q30
static int32_t  integrator = 0;
// Time of previous control update (ms)
static uint32_t last_update_ms = 0;
// Used to force "first call after reset returns 0"
static uint8_t  first_call = 1;

/* ===================== Helpers ===================== */

// Saturate to the valid controller output range (Q30).
// We use 64-bit inputs to avoid overflow during intermediate math.
static inline int32_t sat_ctrl(int64_t x)
{
    if (x > (int64_t)CTRL_MAX) return CTRL_MAX;
    if (x < (int64_t)CTRL_MIN) return CTRL_MIN;
    return (int32_t)x;
}

// Clamp to signed 16-bit range used by Q15.
static inline int32_t clamp_q15(int64_t x)
{
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return (int32_t)x;
}

// Integer absolute value (32-bit).
static inline int32_t iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

// Clamp to [lo, hi].
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
    // First call after reset must return zero and initialize state.
    if (first_call)
    {
        first_call = 0;
        last_update_ms = *millisec;
        integrator = 0;
        return 0;
    }

    // Compute elapsed time (ms) since last controller update.
    // Unsigned subtraction handles timer wrap-around correctly.
    const uint32_t now_ms = *millisec;
    const uint32_t delta_ms = now_ms - last_update_ms;
    last_update_ms = now_ms;
    if (delta_ms == 0U) return 0;  // avoid divide-by-zero and double-update

    // Read inputs once (pass-by-reference in API).
    const int32_t ref_rpm = *reference;
    const int32_t meas_rpm = *measured;
    int32_t err_rpm = ref_rpm - meas_rpm;

    // Deadband for noise
    if (iabs32(err_rpm) <= ERR_DEADBAND_RPM) err_rpm = 0;

    // Normalize error to Q15
    // err_q15 ~= err_rpm / RPM_SCALE, scaled by 2^15
    const int32_t err_q15 = clamp_q15(((int64_t)err_rpm * (int64_t)Q15_ONE) / (int64_t)RPM_SCALE);

    // Feedforward (set U_PER_RPM = 0 to disable)
    // Units: (Q30 per RPM) * RPM = Q30
    const int32_t ff = sat_ctrl((int64_t)U_PER_RPM * (int64_t)ref_rpm);

    // P term: Q15 * Q15 -> Q30
    const int32_t p_term = sat_ctrl((int64_t)Kp * (int64_t)err_q15);

    // I update only when close enough (reduces windup on large steps)
    int32_t integrator_candidate = integrator;
    if (iabs32(err_rpm) <= INT_WINDOW_RPM)
    {
        // Integrate with respect to time (ms -> seconds via /1000).
        // di is in Q30 because Ki(Q15) * err(Q15) => Q30.
        const int64_t di = ((int64_t)Ki * (int64_t)err_q15 * (int64_t)delta_ms) / 1000LL;
        integrator_candidate = sat_ctrl((int64_t)integrator + di);
        integrator_candidate = clamp_i32(integrator_candidate, -I_CLAMP, I_CLAMP);
    }

    // Anti-windup: only commit I when output does not saturate further
    const int64_t ctrl_candidate = (int64_t)ff + (int64_t)p_term + (int64_t)integrator_candidate;
    const int32_t ctrl_sat = sat_ctrl(ctrl_candidate);
    if ((int64_t)ctrl_sat == ctrl_candidate)
    {
        // Not saturated -> accept integrator update.
        integrator = integrator_candidate;
    }
    else
    {
        // Saturated: only accept I if it moves away from saturation.
        const uint8_t pushes_further =
            (ctrl_candidate > (int64_t)CTRL_MAX && err_q15 > 0) ||
            (ctrl_candidate < (int64_t)CTRL_MIN && err_q15 < 0);
        if (!pushes_further) integrator = integrator_candidate;
    }

    // Final control output (Q30).
    return sat_ctrl((int64_t)ff + (int64_t)p_term + (int64_t)integrator);
}

void Controller_Reset(void)
{
    // Reset internal state so the next PI call returns 0 once.
    integrator = 0;
    last_update_ms = 0;
    first_call = 1;
}
