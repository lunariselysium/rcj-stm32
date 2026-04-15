/* kinematics.c */
#include "kinematics.h"

Wheel_Speeds_t kinematics_inverse(Chassis_Velocity_t cmd) {
    Wheel_Speeds_t wheel_rpm;

    // 1. Linear velocity contribution (m/s)
    // For an X-Drive (wheels at 45 deg):
    // FL = +Vx + Vy
    // FR = +Vx - Vy
    // BL = +Vx - Vy
    // BR = +Vx + Vy

    // 2. Rotation contribution (rad/s * radius = m/s)
    float v_rot = cmd.omega * CHASSIS_RADIUS_M;

    // 3. Combine
    // We mix x, y and rotation.
    float linear_fl = cmd.v_x - cmd.v_y - v_rot;
    float linear_fr = cmd.v_x - cmd.v_y + v_rot;
    float linear_bl = cmd.v_x + cmd.v_y - v_rot;
    float linear_br = cmd.v_x + cmd.v_y + v_rot;

    // 4. Convert linear m/s to Wheel RPM
    // RPM = (Linear Speed / Wheel Circumference) * 60 * GearRatio
    float conversion_factor = (1.0f / WHEEL_RADIUS_M) * RADS_TO_RPM * GEAR_RATIO;

    wheel_rpm.fl = linear_fl * conversion_factor;
    wheel_rpm.fr = linear_fr * conversion_factor;
    wheel_rpm.bl = linear_bl * conversion_factor;
    wheel_rpm.br = linear_br * conversion_factor;

    return wheel_rpm;
}

Chassis_Velocity_t kinematics_forward(Wheel_Speeds_t wheel_rpm) {
    Chassis_Velocity_t chassis;
    
    // Convert wheel RPM to linear speeds (m/s)
    float conversion_factor = (1.0f / WHEEL_RADIUS_M) * RPM_TO_RADS * GEAR_RATIO;
    float inv_conversion = 1.0f / conversion_factor;
    
    float linear_fl = wheel_rpm.fl * inv_conversion;
    float linear_fr = wheel_rpm.fr * inv_conversion;
    float linear_bl = wheel_rpm.bl * inv_conversion;
    float linear_br = wheel_rpm.br * inv_conversion;
    
    // Solve for v_x, v_y, omega using derived equations
    chassis.v_x = (linear_fl + linear_fr + linear_bl + linear_br) * 0.25f;
    chassis.v_y = ((linear_fl + linear_br) - (linear_fr + linear_bl)) * 0.25f;
    chassis.omega = ((linear_fl + linear_bl) - (linear_fr + linear_br)) * 0.25f / CHASSIS_RADIUS_M;
    
    return chassis;
}
