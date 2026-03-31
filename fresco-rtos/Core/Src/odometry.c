/* odometry.c */
#include "odometry.h"
#include "motor_driver.h"
#include "cmsis_os.h"
#include "ros_bridge.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static Pose2D_t current_pose = {0};
static uint32_t last_update_tick = 0;

// Helper to normalize angle to -PI..+PI
static float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void odometry_init(void) {
    current_pose.x = 0.0f;
    current_pose.y = 0.0f;
    current_pose.theta = 0.0f;
    last_update_tick = osKernelGetTickCount();
}

void odometry_update(float dt) {
    if (dt <= 0.0f) return;
    
    // Read current wheel RPMs
    Wheel_Speeds_t wheel_rpm;
    wheel_rpm.fl = (float)motors_get_rpm(3); // Motor 3 is FL
    wheel_rpm.fr = -(float)motors_get_rpm(0); // Motor 0 is FR
    wheel_rpm.bl = (float)motors_get_rpm(2); // Motor 2 is BL
    wheel_rpm.br = -(float)motors_get_rpm(1); // Motor 1 is BR
    
    // Note: Verify motor mapping! The motor_driver.c mapping may differ.
    // In motor_driver.c lines 89-92:
    // motors[0].target_rpm = -(int16_t)CLAMP_RPM(target_speeds.fr);
    // motors[1].target_rpm = -(int16_t)CLAMP_RPM(target_speeds.br);
    // motors[2].target_rpm = (int16_t)CLAMP_RPM(target_speeds.bl);
    // motors[3].target_rpm = (int16_t)CLAMP_RPM(target_speeds.fl);
    // So motor index 0 = FR, 1 = BR, 2 = BL, 3 = FL.
    // The above assignment matches that mapping.
    
    // Compute chassis velocity from wheel RPMs
    Chassis_Velocity_t chassis = kinematics_forward(wheel_rpm);
    
    // Integrate using simple Euler method
    // Rotate chassis velocities into world frame
    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float vx_world = chassis.v_x * cos_theta - chassis.v_y * sin_theta;
    float vy_world = chassis.v_x * sin_theta + chassis.v_y * cos_theta;
    
    current_pose.x += vx_world * dt;
    current_pose.y += vy_world * dt;
    current_pose.theta = normalize_angle(current_pose.theta + chassis.omega * dt);
}

Pose2D_t odometry_get_pose(void) {
    Pose2D_t pose;
    taskENTER_CRITICAL();
    pose = current_pose;
    taskEXIT_CRITICAL();
    return pose;
}

void odometry_set_pose(float x, float y, float theta) {
    taskENTER_CRITICAL();
    current_pose.x = x;
    current_pose.y = y;
    current_pose.theta = normalize_angle(theta);
    taskEXIT_CRITICAL();
}

// Debug function: print pose via UART (requires printf redirection)
void odometry_print_pose(void) {
#ifdef DEBUG_ODOM
    Pose2D_t p = odometry_get_pose();
    printf("[ODOM] x=%.3f y=%.3f theta=%.3f\n", p.x, p.y, p.theta);
#endif
}

// Send odometry as debug string via ROS bridge (packet ID 0x4F)
void odometry_send_debug(void) {
    if (g_ros_bridge_ptr == NULL) return;
    Pose2D_t p = odometry_get_pose();
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "ODOM x=%.3f y=%.3f th=%.3f", p.x, p.y, p.theta);
    if (len > 0) {
        ROS_Bridge_Send_Packet(g_ros_bridge_ptr, 0x4F, buf, len);
    }
}
