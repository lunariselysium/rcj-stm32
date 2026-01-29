/* kinematics.h */
#pragma once
#include <math.h>

// --- Robot Physical Constants (MEASURE THESE!) ---

// Radius of the wheel in Meters (e.g., 50mm wheel = 0.05f)
#define WHEEL_RADIUS_M  0.05f

// Distance from robot center to wheel center in Meters
// (Measure diagonal distance from center of chassis to center of a wheel)
#define CHASSIS_RADIUS_M 0.20f

// Motor Gear Ratio (if 19:1 gearbox, enter 19.0f)
// If the RPM you read from CAN is already "Output Shaft RPM", set to 1.0f
#define GEAR_RATIO 19.0f

// --- Math Constants ---
#define RPM_TO_RADS (2.0f * 3.14159f / 60.0f)
#define RADS_TO_RPM (60.0f / (2.0f * 3.14159f))

typedef struct {
    float v_x;      // m/s (Forward)
    float v_y;      // m/s (Left/Right Strafe)
    float omega;    // rad/s (Rotation CCW)
} Chassis_Velocity_t;

typedef struct {
    float fl;
    float fr;
    float bl;
    float br;
} Wheel_Speeds_t;

// Function to calculate wheel RPMs from chassis velocity
Wheel_Speeds_t kinematics_inverse(Chassis_Velocity_t cmd);
