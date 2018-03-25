/*********************************************************
  @2017CameraDemo
  @固件库：超核V2.4
  @author：wgq & lq
  @2017.11.27
  @for seu2017 摄像头组
*********************************************************/

#include "pid.h"
#include "oled.h"
#include "camera.h"
#include "chlib_k.h"
#include "encoder.h"

int main()
{
	uint8_t k;

	DisableInterrupts; // 初始化之前屏蔽全局中断

	UART_QuickInit(UART3_RX_PC16_TX_PC17, 115200);							// 蓝牙串口初始化，UART3，115200bps
	OLED_Init();															// OLED初始化
	OLED_Clear();															// OLED清空
	CAMERA_Init();															// 摄像头初始化 
	ENCODER_Init(kQD_PHABEncoding, 400);									// 编码器初始化
	FTM_PWM_QuickInit(FTM2_CH0_PB18, kPWM_EdgeAligned, 50, SERVO_CENTER);	// 舵机初始化 
	FTM_PWM_QuickInit(FTM0_CH7_PD07, kPWM_EdgeAligned, 500, 0);				// 电机初始化
	FTM_PWM_QuickInit(FTM0_CH5_PD05, kPWM_EdgeAligned, 500, 0);				// 电机初始化

	GPIO_QuickInit(HW_GPIOB, 11, kGPIO_Mode_IPD);							// B1按钮初始化
	GPIO_QuickInit(HW_GPIOB, 17, kGPIO_Mode_IPD);							// B2按钮初始化

	PIT_QuickInit(HW_PIT_CH1, 5000000);										// 设置定时器周期
	PIT_CallbackInstall(HW_PIT_CH1, PID_Start_Detect);						// 注册定时器中断函数
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, ENABLE);						// 启动定时器中断

	EnableInterrupts; // 初始化完成开启全局中断

	while (true)
	{
		if (PBin(11))
			PID_Stop_Detect();  // 关闭停车线检查5s
		if (PBin(17))
			PID_Start_Detect();	// 启动停车线检查

		CAMERA_Processing();
		CAMERA_Display_Full();
		// 使用第3帧图像进行矫正
		if (pic_count == 3)
                    for (k = 0; k < 50; k++)
                        standard[k] = l_line_index[50] - r_line_index[k];
		PID_Controller();
		ENCODER_Display_Speed();
		ENCODER_UART_TX_Speed(HW_UART3);
	}
}