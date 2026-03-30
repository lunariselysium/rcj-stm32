#ifndef BNO085_APP_H
#define BNO085_APP_H

#include <stdint.h>

#define BNO_INT_Pin              GPIO_PIN_1
#define BNO_INT_GPIO_Port        GPIOB

extern float g_latest_yaw_deg;

// Drop-in replacement for your WitMotion getter
uint16_t BNO085_GetYaw_Degrees(void);

// Task entry function
void bno085TaskEntry(void *argument);

#endif
