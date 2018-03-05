#include "pid.h"

int32_t servo_res = 0;     // 舵机结果
int32_t motor_res = 0;     // 电机结果

float servo_cur_error = 0;  // 舵机当前误差
float servo_pre_error = 0;  // 舵机上次误差
float servo_sum_error = 0;  // 舵机累加误差

float motor_cur_error = 0;  // 电机当前误差
float motor_pre_error = 0;  // 电机上次误差
float motor_lst_error = 0;  // 电机更早误差
float motor_sum_error = 0;  // 电机累加误差

uint32_t w[50] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 3, 3,
    3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
};

void PID_Controller()
{
    uint8_t i;                  // 循环计数下标
    
    uint8_t empty_l = 0;        // 左边界为空的行数
    uint8_t empty_r = 0;        // 右边界为空的行数
    uint8_t empty_mid = 0;      // 中线为空的行数
    
    int32_t w_sum = 0;          // 权重加和
    int32_t d_sum = 0;          // 偏移加和

    // 根据权重计算误差
    for(i = IMAGE_ROW_MIN; i < IMAGE_ROW_MAX; i++)
        if(l_line_index[i] != CAMERA_ROW && r_line_index[i] != 0)
            w_sum += w[i], d_sum += ((l_line_index[i] + r_line_index[i]) / 2 - CAMERA_CENTER + 4) * w[i];
        else if(l_line_index[i] != CAMERA_ROW && r_line_index[i] == 0)
            w_sum += w[i], d_sum += ((l_line_index[i] + r_line_index[i]) / 2 - CAMERA_CENTER + 4) * w[i], empty_r++;
        else if(l_line_index[i] != CAMERA_ROW && r_line_index[i] == 0)
            w_sum += w[i], d_sum += ((l_line_index[i] + r_line_index[i]) / 2 - CAMERA_CENTER + 4) * w[i], empty_l++;
        else
            empty_mid++, empty_l++, empty_r++; 
    servo_cur_error = d_sum / w_sum;
    
    // 累加误差
    servo_sum_error += servo_cur_error;

    // 计算舵机PID
    servo_res = SERVO_Kp * servo_cur_error + SERVO_Ki * servo_sum_error + SERVO_Kd * (servo_cur_error - servo_pre_error);

    // 限制舵机PID控制范围, PID运算结果不应超过舵机范围
    if(servo_res >= +SERVO_LIMITS) servo_res = +SERVO_LIMITS;
    if(servo_res <= -SERVO_LIMITS) servo_res = -SERVO_LIMITS;

    // 根据PID计算舵机角度
    servo_res = SERVO_CENTER + servo_res;
    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, servo_res);

    //记录当前误差为上一次的误差
    servo_pre_error = servo_cur_error;

    // 根据空行数计算当前速度误差
    if(empty_mid < 10)
        motor_cur_error = SPEED_MAX + encoder_val;
    else if(empty_l - empty_r > 10 || empty_r - empty_l > 10)
        motor_cur_error = SPEED_MIN + encoder_val;
    else
        motor_cur_error = SPEED_MID + encoder_val;

    // 累加误差
    motor_sum_error += motor_cur_error - motor_pre_error;

    // 计算电机PID
    motor_res = motor_cur_error + MOTOR_Kp * (motor_cur_error - motor_pre_error) + MOTOR_Ki * motor_sum_error + MOTOR_Kd * (motor_cur_error + motor_lst_error - 2 * motor_pre_error);

    // 限制PWM不能无限增高
    if(motor_res >= MAX_PWM)
        motor_res = MAX_PWM;
    if(motor_res <= 0)
        motor_res = 0;

    // 根据PID计算电机速度
    FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, motor_res);

    //记录当前误差为上一次的误差
    motor_lst_error = motor_pre_error;
    motor_pre_error = motor_cur_error;
}

void PID_UART_Tx_Oscillometer(uint32_t instance)
{
    int i;

    // 符合山外多功能调试助手，起始帧为0x03 0xFC
    UART_WriteByte(instance, 0x03);
    UART_WriteByte(instance, 0xFC);

    // 逐个字节连续发送
    for(i = 0; i < 4; i++)
        UART_WriteByte(instance, servo_res >> i * 8 & 0xFF);
    for(i = 0; i < 4; i++)
        UART_WriteByte(instance, motor_res >> i * 8 & 0xFF);

    // 符合山外多功能调试助手，结束帧为0xFC 0x03
    UART_WriteByte(instance, 0xFC);
    UART_WriteByte(instance, 0x03);
}