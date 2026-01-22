#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6100100)
#include <arm_acle.h>
#endif

/**
 * @brief Apply a PI-control law to calculate the control signal for the motor.
 *
 * This function applies a Proportional-Integral (PI) control law to calculate
 * the control signal for the motor based on the reference value, measured value,
 * and the current time in milliseconds.
 *
 * @param reference Pointer to the reference value.
 * @param measured Pointer to the measured value.
 * @param millisec Pointer to the timestamp in milliseconds.
 * @return The calculated control signal for the motor.
 */
int32_t Controller_PIController(const int32_t* reference, const int32_t* measured, const uint32_t* millisec);

/**
 * @brief Reset internal state variables, such as the integrator.
 *
 * This function triggers a reset of the internal state variables of the controller,
 * including the integrator, to their initial values.
 * It doesn't take any arguments and doesn't return any value.
 */
void Controller_Reset(void);

#ifdef __cplusplus
}
#endif

#endif   // _CONTROLLER_H_
