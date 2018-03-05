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
    DisableInterrupts; // 初始化之前屏蔽全局中断
    
    UART_QuickInit(UART3_RX_PC16_TX_PC17, 115200); // 蓝牙串口初始化，UART3，115200bps
    OLED_Init(); // OLED初始化
    OLED_Clear(); // OLED清空
    CAMERA_Init(); // 摄像头初始化 
    ENCODER_Init(kQD_PHABEncoding, 10000); // 编码器初始化
    FTM_PWM_QuickInit(FTM2_CH0_PB18, kPWM_EdgeAligned, 50, 750); // 舵机初始化 
    FTM_PWM_QuickInit(FTM0_CH7_PD07, kPWM_EdgeAligned, 100,  0); // 电机初始化
    EnableInterrupts; // 初始化完成开启全局中断

    while(true)
    {
        CAMERA_Processing();
        CAMERA_Display_Full();
        PID_Controller();
        ENCODER_Display_Speed();
        PID_UART_Tx_Oscillometer(HW_UART3);
    }
}