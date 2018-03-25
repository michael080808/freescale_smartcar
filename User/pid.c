#include "pid.h"

// 舵机位置式PID使用参数
float SERVO_Kp = 4;          // 舵机比例
float SERVO_Ki = 0;          // 舵机积分
float SERVO_Kd = 0;          // 舵机微分

float servo_cur_error = 0;   // 当前加权误差
float servo_pre_error = 0;   // 上次加权误差
float servo_sum_error = 0;	 // 累加误差

// 电机增量式PD使用参数
float MOTOR_Kp = 30;         // 电机比例
float MOTOR_Kd = 0;          // 电机微分
float motor_pwmDuty = 0;     // 电机PID输出结果：电机PWM值

float motor_cur_error = 0;   // 当前误差
float motor_lst_error = 0;   // 上次误差
float motor_pre_error = 0;   // 前次误差
float motor_sum_error = 0;   // 累加误差

// 中心检查参数
float center_sum = 0;        // 总误差
float center_check = 0;      // 判断赛道类型阈值
int16_t center[15] = { 0 };  // 判断赛道类型数据

// 相邻差值参数
int8_t standard[50] = { 0 }; // 矫正数组
int16_t deltas[24] = { 0 };  // 相邻中线值偏差

// 赛道元素状态
uint16_t miss_lines = 0;     // 丢失的线数
uint8_t  str_count = 0;      // 直线判断次数
uint8_t  cur_count = 0;      // 弯道判断次数
uint32_t pic_count = 0;      // 图像计数器
uint8_t  detect_sign = 0;    // 停车线检测标志
uint8_t  stop_sign1 = 0;     // 停车标志1
uint8_t  stop_sign2 = 0;     // 停车标志2
uint32_t stop_delay = 0;     // 停车延迟

void PID_Start_Detect()
{
        stop_delay = 0;
	detect_sign = 1;
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, DISABLE); // 关闭定时器中断
}

void PID_Stop_Detect()
{
        stop_delay = 0;
	detect_sign = 0;
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, ENABLE);  // 开启定时器中断
}

void PID_Controller()
{
	uint8_t i;
	float result;

        // 图像计数
        if(pic_count < 10)
            pic_count++;
        
	// 初始化舵机PID为初始值
	SERVO_Kp = 4; // 舵机比例
	SERVO_Ki = 0; // 舵机积分
	SERVO_Kd = 0; // 舵机微分

	// 丢失线数初始化
	miss_lines = 0;

	if (l_line_index[48] == 152 && r_line_index[48] == 0 &&
		l_line_index[45] == 152 && r_line_index[45] == 0 &&
		l_line_index[43] == 152 && r_line_index[43] == 0 &&
		l_line_index[40] == 152 && r_line_index[40] == 0)
	{
		// 此处是十字路口的识别，保持直线行驶即可
		FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, SERVO_CENTER);
		FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1500);
	}
	else
	{
		// 直线、小弯道、大弯道的识别与执行控制

		servo_cur_error = 0; // 初始化当前加权误差为0， 方便累加
		
		// 停车线检测标志
		if (detect_sign)
		{
			stop_sign1 = 0;
                        stop_sign2 = 0;
			for (i = 1; i < 152; i++)
			{
				if ((image ? img2[35][i - 1] : img1[35][i - 1]) <= THRESHOLD && (image ? img2[35][i] : img1[35][i]) > THRESHOLD)
					stop_sign1++;
                                if ((image ? img2[40][i - 1] : img1[40][i - 1]) <= THRESHOLD && (image ? img2[40][i] : img1[40][i]) > THRESHOLD)
					stop_sign2++;
			}
		}

		// 对采样区(44至21行)进行相邻偏差值的调整
		for (i = 0; i < 24; i++)
		{
			// 对于左侧边界，不存在时进行调整
			if (l_line_index[44 - i] == CAMERA_CENTER)
				l_line_index[44 - i] = r_line_index[44 - i] + standard[44 - i];
			// 对于右侧边界，不存在时进行调整
			if (r_line_index[44 - i] == CAMERA_CENTER)
				r_line_index[44 - i] = l_line_index[44 - i] - standard[44 - i];
			// 计算相邻中点差值
			if (i > 0 && (l_line_index[44 - i] == 152 || r_line_index[44 - i] == 0))
			{
				deltas[i] = deltas[i - 1];
				miss_lines++;
			}
			else
				deltas[i] = (l_line_index[44 - i] + r_line_index[44 - i]) / 2 - CAMERA_CENTER;
			// 累加计算当前加权误差
			servo_cur_error += deltas[i] * (23 - i) * MOTOR_DELTA;
		}

		// 对采样区(38至24行)进行中线统计
		for (i = 0; i < 15; i++)
		{
			// 对于左侧边界，不存在时进行调整
			if (l_line_index[38 - i] == CAMERA_CENTER)
				l_line_index[38 - i] = r_line_index[38 - i] + standard[38 - i];
			// 对于右侧边界，不存在时进行调整
			if (r_line_index[38 - i] == CAMERA_CENTER)
				r_line_index[38 - i] = l_line_index[38 - i] - standard[38 - i];
			// 获取当前行中心线
			if (i > 0 && (l_line_index[38 - i] == 152 && r_line_index[38 - i] == 0))
				center[i] = center[i - 1];
			else
				center[i] = (l_line_index[38 - i] + r_line_index[38 - i]) / 2;
			// 累加中心线
			center_sum += center[i];
		}
		// 计算累加平均并进行中心检查
		center_sum = center_sum / 15;
		// 初始化中心检查值
		center_check = 0;
		for (i = 0; i < 15; i++)
		{
			if (center[i] - center_sum < 0)
			{
				if (i >= 8)
					center_check = center_check - 1 * (center[i] - center_sum);
				else
					center_check = center_check - (center[i] - center_sum);
			}
			else
			{
				if (i >= 8)
					center_check = center_check + 1 * (center[i] - center_sum);
				else
					center_check = center_check + (center[i] - center_sum);
			}
		}

		// 计算车辆速度
		encoder_speed.float32bits = (-encoder_val) * 3.14 * 5.5 / 2000 * 0.4 * (1000000 / encoder_period);

		// 电机PID控制
		if (stop_sign1 < 6 || stop_sign2 < 6)
		{
			if (miss_lines >= 5)
			{
				cur_count++;
				if (cur_count <= 3)
				{
					// 对于大弯道情况1
					motor_cur_error = MOTOR_SPEED_CUR - encoder_speed.float32bits;
					motor_pwmDuty = MOTOR_Kp * (motor_cur_error) + MOTOR_Kd * (motor_cur_error - motor_lst_error);
					motor_lst_error = motor_cur_error;
					FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 400 + motor_pwmDuty);
					SERVO_Kp = 5;
					SERVO_Kd = 20;
				}
				else
				{
					if (miss_lines >= 5)
					{
						// 对于大弯道情况2
						str_count = 0;
						motor_cur_error = MOTOR_SPEED_CUR - encoder_speed.float32bits;
						motor_pwmDuty = MOTOR_Kp * (motor_cur_error) + MOTOR_Kd * (motor_cur_error - motor_lst_error);
						motor_lst_error = motor_cur_error;
						FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1200 + motor_pwmDuty);
						SERVO_Kp = 5;
						SERVO_Kd = 20;
					}
					else
					{
						// 对于小弯道情况1
						motor_cur_error = MOTOR_SPEED_SCUR - encoder_speed.float32bits;
						motor_pwmDuty = MOTOR_Kp * (motor_cur_error)+MOTOR_Kd * (motor_cur_error - motor_lst_error);
						motor_lst_error = motor_cur_error;
						FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1100 + motor_pwmDuty);
						SERVO_Kp = 3;
						SERVO_Kd = 17;
					}

				}
			}
			else
			{
				str_count++;
				if (str_count <= 3)
				{
					// 对于大弯道情况3
					motor_cur_error = MOTOR_SPEED_CUR - encoder_speed.float32bits;
					motor_pwmDuty = MOTOR_Kp * (motor_cur_error)+MOTOR_Kd * (motor_cur_error - motor_lst_error);
					motor_lst_error = motor_cur_error;
					FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 400 + motor_pwmDuty);
					SERVO_Kp = 5;
					SERVO_Kd = 20;
				}
				else
				{
					cur_count = 0;
					if (center_check >= 95)
					{
						// 对于小弯道情况2
						motor_cur_error = MOTOR_SPEED_SCUR - encoder_speed.float32bits;
						motor_pwmDuty = MOTOR_Kp * (motor_cur_error)+MOTOR_Kd * (motor_cur_error - motor_lst_error);
						motor_lst_error = motor_cur_error;
						FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1100 + motor_pwmDuty);
						SERVO_Kp = 3;
						SERVO_Kd = 17;
					}
					else
					{
						// 对于直道情况
						motor_cur_error = MOTOR_SPEED_STR - encoder_speed.float32bits;
						motor_pwmDuty = MOTOR_Kp * (motor_cur_error)+MOTOR_Kd * (motor_cur_error - motor_lst_error);
						motor_lst_error = motor_cur_error;
						FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1600 + motor_pwmDuty);
						SERVO_Kp = 1;
						SERVO_Kd = 15;
					}
				}
			}
		}
		else
		{
			detect_sign = 0;
                        if(stop_delay < 0x10)
                            stop_delay++;
                        else
                            FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 0);
			SERVO_Kp = 5;
		}

		// 舵机PID控制
		// 舵机误差累加
		servo_sum_error += servo_cur_error;
		// 计算舵机PID
		result = SERVO_Kp * servo_cur_error + SERVO_Ki * servo_sum_error + SERVO_Kd * (servo_cur_error - servo_pre_error);

		// 限制舵机PID控制范围, PID运算结果不应超过舵机范围
		if (result >= SERVO_LIMITS)
			result = SERVO_LIMITS;
		if (result <= -SERVO_LIMITS)
			result = -SERVO_LIMITS;

		// 根据PID计算舵机角度
		FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, SERVO_CENTER + result);

		//记录当前误差为上一次的误差
		servo_pre_error = servo_cur_error;
	}
}