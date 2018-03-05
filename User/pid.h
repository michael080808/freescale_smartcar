#ifndef __PID_H__
#define __PID_H__

#include "oled.h"
#include "camera.h"
#include "chlib_k.h"
#include "encoder.h"

#define SERVO_CENTER 778                            // 舵机直行中心值
#define SERVO_LIMITS 180                            // 舵机行程限制值

#define SERVO_Kp 9                                  // 舵机比例
#define SERVO_Ki 0                                  // 舵机积分
#define SERVO_Kd -2.5                               // 舵机微分

#define MOTOR_Kp 1                                  // 电机比例
#define MOTOR_Ki 0                                  // 电机积分
#define MOTOR_Kd 0                                  // 电机微分

#define IMAGE_ROW_MIN 16                            // 图像处理行最小
#define IMAGE_ROW_MAX 50                            // 图像处理行最大

#define SPEED_MAX 2000                              // 最大编码器脉冲
#define SPEED_MID 1000                              // 居中编码器脉冲
#define SPEED_MIN 800                               // 最小编码器脉冲

#define MAX_PWM 1000                                // 限制最大PWM脉冲宽度

extern uint32_t w[50];                              // 计算各个中线偏差的权重

void PID_Controller(void);                          // PID控制函数
void PID_UART_Tx_Oscillometer(uint32_t instance);   // PID结果显示

#endif