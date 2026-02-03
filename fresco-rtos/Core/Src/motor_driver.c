/* motor_driver.c */
#include "motor_driver.h"
#include "cmsis_os.h" // For osDelay
#include <stdbool.h>

// --- PID Configuration ---
#define PID_KP  10.0f
#define PID_KI  0.5f
#define PID_KD  0.0f
#define PID_TAU 0.02f
#define SAMPLE_TIME_S 0.001f // 1ms loop

// --- Private Structures ---
typedef struct {
    PIDController pid;
    volatile int16_t current_rpm; // Updated by ISR
    volatile int16_t target_rpm;  // Updated by App
} Motor_t;

// --- Private Variables ---
static CAN_HandleTypeDef* phcan;
static Motor_t motors[4]; // 4 Motors

static CAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[8];

static Chassis_Velocity_t target_vel = {0}; // Holds global target
static bool manual_mode = false;

// --- Internal Helpers ---
static void can_filter_config(void);
static void send_can_command(void);

// --- Initialization ---
void motors_init(CAN_HandleTypeDef* hcan) {
    phcan = hcan;

    // Initialize PIDs for all 4 motors
    for(int i=0; i<4; i++) {
        motors[i].current_rpm = 0;
        motors[i].target_rpm = 0;

        motors[i].pid.Kp = PID_KP;
        motors[i].pid.Ki = PID_KI;
        motors[i].pid.Kd = PID_KD;
        motors[i].pid.tau = PID_TAU;
        motors[i].pid.T = SAMPLE_TIME_S;

        motors[i].pid.limMin = -MAX_CURRENT;
        motors[i].pid.limMax = MAX_CURRENT;
        motors[i].pid.limMinInt = -5000.0f; // Anti-windup limit
        motors[i].pid.limMaxInt = 5000.0f;

        PIDController_Init(&motors[i].pid);
    }

    // Configure Hardware
    can_filter_config();
    HAL_CAN_Start(phcan);
    HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// --- The Real-Time Task ---
void motor_control_task_entry(void) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Calculate Kinematics (If not in manual testing mode)
        if (!manual_mode) {
            Chassis_Velocity_t current_cmd;

            // Copy local to avoid tearing
            // (Strictly speaking, float reads are atomic on ARM Cortex-M4, but good practice)
            current_cmd = target_vel;

            // Field Centric Placeholder (Optional for later)
            // if (field_centric) { apply_rotation_matrix(&current_cmd, robot_yaw); }

            // Calculate Target RPMs
            Wheel_Speeds_t target_speeds = kinematics_inverse(current_cmd);

			#define CLAMP_RPM(val) ((val) > MOTOR_MAX_RPM_LIMIT ? MOTOR_MAX_RPM_LIMIT : \
                                   ((val) < -MOTOR_MAX_RPM_LIMIT ? -MOTOR_MAX_RPM_LIMIT : (val)))

            motors[0].target_rpm = -(int16_t)CLAMP_RPM(target_speeds.fr);
            motors[1].target_rpm = -(int16_t)CLAMP_RPM(target_speeds.br);
            motors[2].target_rpm = (int16_t)CLAMP_RPM(target_speeds.bl);
            motors[3].target_rpm = (int16_t)CLAMP_RPM(target_speeds.fl);
        }

        // 2. Update PIDs (Standard logic)
        for(int i=0; i<4; i++) {
            PIDController_Update(&motors[i].pid,
                                 (float)motors[i].target_rpm,
                                 (float)motors[i].current_rpm);
        }

        // 3. Send CAN
        send_can_command();
    }
}


// --- Public Setters/Getters ---
void motors_set_rpm_manual(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    taskENTER_CRITICAL();
    motors[0].target_rpm = m1;
    motors[1].target_rpm = m2;
    motors[2].target_rpm = m3;
    motors[3].target_rpm = m4;
    manual_mode = true;
    taskEXIT_CRITICAL();
}

int16_t motors_get_rpm(uint8_t motor_index) {
    if (motor_index < 4) return motors[motor_index].current_rpm;
    return 0;
}

void chassis_set_velocity(float vx, float vy, float omega) {
    // Basic critical section to ensure we don't read half-updated structs
    taskENTER_CRITICAL();
    target_vel.v_x = vx;
    target_vel.v_y = vy;
    target_vel.omega = omega;
    manual_mode = false;
    taskEXIT_CRITICAL();
}

// --- CAN Low Level ---

static void send_can_command(void) {
    uint32_t send_mail_box;

    // Standard ID for DJI GM6020/M3508 (0x200 or 0x1FF depending on motor ID)
    // Assuming motors 1-4 use ID 0x200
    tx_header.StdId = 0x200;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // Map PID output to Current
    for(int i=0; i<4; i++) {
        int16_t current = (int16_t)motors[i].pid.out;
        tx_data[i*2]     = (current >> 8);
        tx_data[i*2 + 1] = current;
    }

    HAL_CAN_AddTxMessage(phcan, &tx_header, tx_data, &send_mail_box);
}

// ISR: Updates RPM values when CAN message arrives
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // Check Motor IDs (Usually 0x201, 0x202, 0x203, 0x204 for GM6020/M3508)
        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204) {
            uint8_t index = rx_header.StdId - 0x201;

            // Speed is usually bytes [2] and [3] (High byte first)
            int16_t speed = (rx_data[2] << 8) | rx_data[3];

            if (index < 4) {
                motors[index].current_rpm = speed;
            }
        }
    }
}

static void can_filter_config(void) {
    CAN_FilterTypeDef can_filter;
    can_filter.FilterActivation = ENABLE;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(phcan, &can_filter);
}
