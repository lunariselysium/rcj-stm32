/* motor_driver.h */
#pragma once

#include "main.h"
#include "can.h"
#include "pid.h"
#include "kinematics.h"


#define MAX_CURRENT 10000

void motors_init(CAN_HandleTypeDef* hcan);

// The FreeRTOS Task Function
void motor_control_task_entry(void);

// User Application Functions
void motors_set_rpm_manual(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
int16_t motors_get_rpm(uint8_t motor_index); // 0-3

void chassis_set_velocity(float vx, float vy, float omega);
