#include "main.h" 

#include "application.h" 
#include "controller.h"
#include "peripherals.h"

/* Global variables ----------------------------------------------------------*/
int32_t reference, velocity, control;
uint32_t millisec;

/* Functions -----------------------------------------------------------------*/

/* Run setup needed for all periodic tasks */
void Application_Setup()
{
  // Reset global variables
  reference = 2000;
  velocity = 0;
  control = 0;
  millisec = 0;

  // Initialise hardware
  Peripheral_GPIO_EnableMotor();

  // Initialize controller
  Controller_Reset();
}

/* Define what to do in the infinite loop */
void Application_Loop()
{
  // Wait for next sample -- also prevent from re-entering again
  while((Main_GetTickMillisec() % PERIOD_CTRL > 0) || (Main_GetTickMillisec() == millisec))
  {
    // Do nothing while waiting
  }

  // Get time
  millisec = Main_GetTickMillisec();

  // Every 4 sec ...
  if (millisec % PERIOD_REF == 0)
  {
    // Flip the direction of the reference
    reference = -reference;
  }

  // Every 10 msec ...
  if (millisec % PERIOD_CTRL == 0)
  {
    // Calculate motor velocity
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);

    // Calculate control signal
    control = Controller_PIController(&reference, &velocity, &millisec);

    // Apply control signal to motor
    Peripheral_PWM_ActuateMotor(control);
  }
}
