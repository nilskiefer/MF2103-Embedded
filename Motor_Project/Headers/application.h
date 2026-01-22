#ifndef _Application_H_
#define _Application_H_
#ifdef __cplusplus
extern "C" {
#endif

#define PERIOD_CTRL 10		//!< Period of the control loop in milliseconds.
#define PERIOD_REF 4000		//!< Period of the reference switch in milliseconds.

/**
 * @brief Initializes the application.
 *
 * This function is responsible for initializing the application.
 * It should set up peripherals, variables, libraries, and any other initial 
 * configurations necessary for the application.
 * It doesn't take any arguments and doesn't return any value.
 */
void Application_Setup(void);

/**
 * @brief Main application loop.
 *
 * This function contains the main loop that runs continuously after initialization.
 * It handles the core functionality of the application.
 * It doesn't take any arguments and doesn't return any value.
 */
void Application_Loop(void);

#ifdef __cplusplus
}
#endif

#endif   // _Application_H_
