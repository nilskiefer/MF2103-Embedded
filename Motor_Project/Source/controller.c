#include "controller.h"
#include <stdint.h>

#define CTRL_MAX   ((int32_t)0x3FFFFFFF)
#define CTRL_MIN   ((int32_t)0xC0000000)

#define RPM_SCALE  4000

// Gains in Q15 (0..32767 ~ 0..1.0)
volatile int32_t Kp_q15 = 4000;     // small, because feedforward does the heavy lifting
volatile int32_t Ki_q15 = 300;      // start very small

// Feedforward: control_units per rpm.
// From your measurement: ~200,000,000 at ~2000 rpm => ~100,000 per rpm.
volatile int32_t U_PER_RPM = 100000;

// Noise/actuation helpers
volatile int32_t ERR_DEADBAND_RPM = 30;     // ignore +/- this many rpm
volatile int32_t DU_MAX = 0;                // 0 disables slew limit; try 15000000 if needed

static int32_t  i_state_q30 = 0;
static uint32_t last_ms = 0;
static uint8_t  first_call = 1;
static int32_t  u_prev_q30 = 0;

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

static inline int32_t abs_i32(int32_t x)
{
  return (x < 0) ? -x : x;
}

int32_t Controller_PIController(const int32_t* reference,
                                const int32_t* measured,
                                const uint32_t* millisec)
{
  if (first_call)
  {
    first_call = 0;
    last_ms = *millisec;
    i_state_q30 = 0;
    u_prev_q30 = 0;
    return 0;
  }

  uint32_t dt_ms = *millisec - last_ms;
  last_ms = *millisec;
  if (dt_ms == 0U) return u_prev_q30;

  int32_t r_rpm = *reference;
  int32_t y_rpm = *measured;
  int32_t e_rpm = r_rpm - y_rpm;

  // Deadband against encoder ripple
  if (abs_i32(e_rpm) <= ERR_DEADBAND_RPM)
    e_rpm = 0;

  // Feedforward in Q30 control units
  int32_t u_ff_q30 = sat_ctrl((int64_t)U_PER_RPM * (int64_t)r_rpm);

  // Normalize error to Q15
  int32_t e_q15 = clamp_q15(((int64_t)e_rpm * 32768LL) / (int64_t)RPM_SCALE);

  // P term: Q15*Q15 = Q30
  int32_t p_q30 = sat_ctrl((int64_t)Kp_q15 * (int64_t)e_q15);

  // I increment: (Ki*e) is Q30/s, multiply by dt/1000 -> Q30
  int64_t di_q30 = ((int64_t)Ki_q15 * (int64_t)e_q15 * (int64_t)dt_ms) / 1000LL;
  int32_t i_candidate = sat_ctrl((int64_t)i_state_q30 + di_q30);

  int64_t u_candidate_ll = (int64_t)u_ff_q30 + (int64_t)p_q30 + (int64_t)i_candidate;
  int32_t u_sat = sat_ctrl(u_candidate_ll);

  // Anti-windup
  if ((int64_t)u_sat != u_candidate_ll)
  {
    if ((u_candidate_ll > (int64_t)CTRL_MAX && e_q15 > 0) ||
        (u_candidate_ll < (int64_t)CTRL_MIN && e_q15 < 0))
    {
      // freeze integrator
    }
    else
    {
      i_state_q30 = i_candidate;
    }
  }
  else
  {
    i_state_q30 = i_candidate;
  }

  int32_t u_q30 = sat_ctrl((int64_t)u_ff_q30 + (int64_t)p_q30 + (int64_t)i_state_q30);

  // Optional slew limit (helps if you still see buzzing)
  if (DU_MAX > 0)
  {
    int32_t du = u_q30 - u_prev_q30;
    if (du > DU_MAX) du = DU_MAX;
    if (du < -DU_MAX) du = -DU_MAX;
    u_q30 = u_prev_q30 + du;
  }

  u_prev_q30 = u_q30;
  return u_q30;
}

void Controller_Reset(void)
{
  i_state_q30 = 0;
  last_ms = 0;
  first_call = 1;
  u_prev_q30 = 0;
}
