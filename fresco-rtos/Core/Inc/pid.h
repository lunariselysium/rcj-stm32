/**
 * @file pid.h
 * @brief Header for a PID controller.
 *
 * @author pms67 (https://github.com/pms67/PID/)
 * @date Nov 25, 2025
 */

#ifndef APP_UTILS_PID_H
#define APP_UTILS_PID_H

typedef struct {
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;
} PIDController;

/**
 * @brief Initializes the PID controller structure.
 * @param pid Pointer to the PIDController structure.
 */
void PIDController_Init(PIDController *pid);

/**
 * @brief Updates the PID controller and returns the control output.
 * @param pid Pointer to the PIDController structure.
 * @param setpoint The desired value.
 * @param measurement The actual measured value.
 * @return The calculated control output.
 */
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif // APP_UTILS_PID_H
