#ifndef __PID_H__
#define __PID_H__

#include "oled.h"
#include "camera.h"
#include "chlib_k.h"
#include "encoder.h"

#define SERVO_CENTER 778 // 舵机直行中心值
#define SERVO_LIMITS 180 // 舵机行程限制值

#define SERVO_Kp 6     // 舵机比例
#define SERVO_Ki 0.005 // 舵机积分
#define SERVO_Kd 10    // 舵机微分

#define MOTOR_MAX_SPEED 1000 // 电机最大速度
#define MOTOR_LMT_SPEED 400  // 电机行程速度

#define MOTOR_Kp 30    // 电机比例
#define MOTOR_Ki 0     // 电机积分
#define MOTOR_Kd 0     // 电机微分

typedef union PID_PWM_UNION{
    float   float32bits;
    uint8_t int8bits[4];
} PID_PWM_UNION;

extern float cur_error; // 当前误差
extern float pre_error; // 上次误差
extern float sum_error; // 累加误差

void PID_Controller(void); // PID控制函数

#endif