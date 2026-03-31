/* odometry.h */
#pragma once

#include <stdint.h>
#include "kinematics.h"

typedef struct {
    float x;        // meters, forward
    float y;        // meters, left
    float theta;    // radians, CCW positive (wrapped to -PI..+PI)
} Pose2D_t;

void odometry_init(void);
void odometry_update(float dt); // dt in seconds
Pose2D_t odometry_get_pose(void);
void odometry_set_pose(float x, float y, float theta);

// Debug function to print pose via UART (if DEBUG_ODOM defined)
void odometry_print_pose(void);