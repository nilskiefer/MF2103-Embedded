#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_
#ifdef __cplusplus
extern "C" {
#endif

#ifdef STM32F103xB
#include "stm32f1xx.h"
#endif
#ifdef STM32L476xx
#include "stm32l4xx.h"
#endif

/**
 * @brief Enable both half-bridges to drive the motor.
 *
 * This function enables both half-bridges to drive the motor.
 * It doesn't take any arguments and doesn't return any value.
 */
void Peripheral_GPIO_EnableMotor(void);

/**
 * @brief Disable both half-bridges to stop the motor.
 *
 * This function disables both half-bridges to stop the motor.
 * It doesn't take any arguments and doesn't return any value.
 */
void Peripheral_GPIO_DisableMotor(void);

/**
 * @brief Drive the motor in both directions.
 *
 * This function drives the motor in both directions using a PWM signal.
 * The input specifies the duty cycle [-1,073,741,824 to +1,073,741,823],
 * which corresponds to a percentage of the maximum duty cycle (-100% to +100%).
 *
 * @param control The control signal for driving the motor.
 */
void Peripheral_PWM_ActuateMotor(int32_t control);

/**
 * @brief Read the encoder value and calculate the current velocity in RPM.
 *
 * This function reads the relevant register(s) and calculates the current velocity
 * of the motor in revolutions per minute, based on the elapsed time milliseconds.
 *
 * The velocity is calculated using the formula:
 * v[k] = K * (x[k] - x[k-1]) / (t[k] - t[k-1])
 *
 * where,
 * v[k] is the calculated velocity in RPM,
 * x[k] is the current encoder reading,
 * x[k-1] is the previous encoder reading,
 * t[k] is the current time in milliseconds,
 * t[k-1] is the previous time in milliseconds,
 * and K corresponds to any necessary scaling factor.
 *
 * This formula estimates the motor velocity by calculating the change in position
 * divided by the change in time between two consecutive readings. To obtain an accurate 
 * velocity estimation, this function should be called repeatedly with updated elapsed 
 * time values. The first time this function is called, it should return zero.
 * 
 * This function must be READ ONLY on the encoder register!
 *
 * @param millisec The time elapsed in milliseconds.
 * @return The calculated motor velocity in RPM.
 */
int32_t Peripheral_Encoder_CalculateVelocity(uint32_t millisec);

#ifdef __cplusplus
}
#endif

#endif   // _PERIPHERALS_H_
