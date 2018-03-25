#ifndef __PID_H__
#define __PID_H__

#include "oled.h"
#include "camera.h"
#include "chlib_k.h"
#include "encoder.h"

#define SERVO_CENTER     768			// 舵机直行中心值
#define SERVO_LIMITS     200			// 舵机行程限制值

#define MOTOR_DELTA      0.003623		// 公差

#define MOTOR_SPEED_STR  250			// 直道理想速度
#define MOTOR_SPEED_CUR  130			// 大弯道理想速度
#define MOTOR_SPEED_SCUR 160			// 小弯道理想速度

typedef union PID_PWM_UNION {
	float   float32bits;
	uint8_t int8bits[4];
} PID_PWM_UNION;

extern uint32_t pic_count;                              // 图像计数器
extern int8_t standard[];				// 矫正数组

void PID_Start_Detect();				// 启动停止线查找
void PID_Stop_Detect();					// 关闭停止线查找5秒
void PID_Controller();					// PID控制函数

#endif