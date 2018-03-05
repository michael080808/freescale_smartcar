#include "pid.h"

float servo_cur_error = 0; // 当前误差
float servo_pre_error = 0; // 上次误差
float servo_sum_error = 0; // 累加误差

float motor_cur_error = 0; // 当前误差
float motor_pre_error = 0; // 上次误差
float motor_sum_error = 0; // 累加误差

uint8_t sample[3] = {40, 27, 13};
int16_t deltas[3] = {0};

void PID_Controller()
{
    uint8_t i, j;
    float result;

    if(l_line_index[48]==152 && r_line_index[48]==0 &&
       l_line_index[45]==152 && r_line_index[45]==0 &&
       l_line_index[43]==152 && r_line_index[43]==0 &&
       l_line_index[40]==152 && r_line_index[40]==0)
    {
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, SERVO_CENTER);
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 700);
    }
    else
    {
        for(i = 0; i < 3; i++)
            deltas[i] = (l_line_index[sample[i]] + r_line_index[sample[i]]) / 2 - CAMERA_CENTER + 3;

        servo_cur_error = deltas[0];
        servo_sum_error += servo_cur_error;

        // 计算舵机PID
        result = SERVO_Kp * servo_cur_error + SERVO_Ki * servo_sum_error + SERVO_Kd * (servo_cur_error - servo_pre_error);

        // 限制舵机PID控制范围, PID运算结果不应超过舵机范围
        if(result >=  SERVO_LIMITS)
            result =  SERVO_LIMITS;
        if(result <= -SERVO_LIMITS)
            result = -SERVO_LIMITS;

        // 根据PID计算舵机角度
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, SERVO_CENTER + result);

        //记录当前误差为上一次的误差
        servo_pre_error = servo_cur_error;

        // 计算车辆速度
        encoder_speed.float32bits = (-encoder_val) * 3.14 * 5.5 / 2000 * 0.4 * (1000000 / encoder_period);

        if(deltas[2] >= 0)
            motor_cur_error = deltas[2];
        else
            motor_cur_error = -deltas[2];
        motor_sum_error += motor_cur_error;

        // 计算电机PID
        result = MOTOR_Kp * motor_cur_error + MOTOR_Ki * motor_sum_error + MOTOR_Kd * (motor_cur_error - motor_pre_error);

        // 限制电机PID控制范围, PID运算结果不应超过电机最大速度
        if(result >= MOTOR_LMT_SPEED)
            result = MOTOR_LMT_SPEED; // 电机最大速度;
        if(result <= 0)
            result = 0; // 电机最小速度;

        // 根据PID计算电机速度
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, MOTOR_MAX_SPEED - result);

        // 记录当前误差为上一次的误差
        motor_pre_error = motor_cur_error; // 之前没有加，待测试
    }
}
