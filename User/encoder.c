#include "encoder.h"

uint8_t  encoder_dir;               // 编码盘方向
int16_t  encoder_val;               // 编码盘脉冲
uint32_t encoder_period;            // 编码盘检测周期
ENCODER_SPEED_UNION encoder_speed;  // 编码盘速度计算

void ENCODER_Init(FTM_QD_Mode_Type mode, uint32_t timeInUs)
{
    encoder_period = timeInUs; // 记录编码盘的检测周期

    FTM_QD_QuickInit(FTM1_QD_PHA_PB00_PHB_PB01, kFTM_QD_NormalPolarity, mode); // 初始化编码盘FTM设置 
    
    PIT_QuickInit(HW_PIT_CH0, timeInUs); // 设置定时器周期
    PIT_CallbackInstall(HW_PIT_CH0, ENCODER_Interrupt_Handler); // 注册定时器中断函数
    PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF, ENABLE); // 启动定时器中断
}

void ENCODER_Interrupt_Handler(void)
{
    FTM_QD_GetData(HW_FTM1, &encoder_val, &encoder_dir); // 获取编码盘方向和脉冲值
    FTM_QD_ClearCount(HW_FTM1); // 清除脉冲计数
}

void ENCODER_Display_Speed(void)
{
    uint8_t p[7] = {0};

    // 输出不超过6位的整型数据
    sprintf(p, "%06d", -encoder_val); 
    OLED_ShowString_1206(0, 51, p, 1);

    // 输出当前脉冲数得到的速度
    encoder_speed.float32bits = (-encoder_val) * 3.14 * 5.5 / 2000 * 0.4 * (1000000 / encoder_period); 
    OLED_ShowNum_1206(68, 51, encoder_speed.float32bits, 1);

    // 输出速度单位
    sprintf(p, "cm/s");
    OLED_ShowString_1206(104, 51, p, 1);
    
    // 刷新OLED以显示图像
    OLED_Refresh_Gram();
}

void ENCODER_UART_TX_Speed(const uint32_t instance)
{
    uint8_t i;

    // 输出当前脉冲数得到的速度
    encoder_speed.float32bits = (-encoder_val) * 3.14 * 5.5 / 2000 * 0.4 * (1000000 / encoder_period);
    
    // 符合山外多功能调试助手，起始帧为0x03 0xFC
    UART_WriteByte(instance, 0x03);
    UART_WriteByte(instance, 0xFC);

    // 从高位向低位依次输出float数据一个
    for(i = 0; i < 4; i++)
        UART_WriteByte(instance, encoder_speed.int8bits[i]);
    
    // 符合山外多功能调试助手，结束帧为0xFC 0x03
    UART_WriteByte(instance, 0xFC);
    UART_WriteByte(instance, 0x03);
}