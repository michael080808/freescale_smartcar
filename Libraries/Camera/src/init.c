/*********************************************************/
//@demo
//@固件库：超核V2.4
//@author：th
//@2016.11.30
//@for seu2016 摄像头四轮组
/*********************************************************/

#include "init.h"
#include "varieble.h"
#include "isr.h"
#include "sysinit.h"
#include "DEV_SCCB.h"
#include "oled.h"
//初始化
void init(void){

    init_steer();//舵机初始化

}

void init_ov7620(void){
        //DMA初始化
        DMA_InitTypeDef DMA_InitStruct1 = {0};//定义初始化结构体
        DMA_InitStruct1.chl = HW_DMA_CH0;                                           
        DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ;                             
        DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;             
        DMA_InitStruct1.minorLoopByteCnt = 1;
        DMA_InitStruct1.majorLoopCnt = col_num;//主循环采集数，即一次主循环采集一行
    
        DMA_InitStruct1.sAddr = (uint32_t)&(PTC->PDIR)+1;//dma源地址：ptc8~15                           
        DMA_InitStruct1.sLastAddrAdj = 0;                                          
        DMA_InitStruct1.sAddrOffset = 0;                                           
        DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;//数据宽度                           
        DMA_InitStruct1.sMod = kDMA_ModuloDisable;                                 
    
        DMA_InitStruct1.dLastAddrAdj = 0;
        DMA_InitStruct1.dAddrOffset = 1;
        DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
        DMA_InitStruct1.dMod = kDMA_ModuloDisable;
        
        DMA_Init(&DMA_InitStruct1);
        DMA_DisableRequest(HW_DMA_CH0);//先关闭DMA传输
        
        //配置摄像头寄存器
        uint8_t i=0;
        //初始化SCCB所需引脚，SCCB是一种简化的类似于I2C的通信协议。
        //初始化PTC3和PTC0，作为SCCB中的SDA和SCL。
        GPIO_QuickInit(HW_GPIOC, 0, kGPIO_Mode_OPP);
        GPIO_QuickInit(HW_GPIOC, 3, kGPIO_Mode_OPP);
        while(i==0)
          i += LPLD_SCCB_WriteReg(0x14,0x24);		//QVGA(320*120)
        while(i==1)
          i += LPLD_SCCB_WriteReg(0x70, 0xc1);		//驱动电流增加一倍
        while(i==2)
          i += LPLD_SCCB_WriteReg(0x24, 0x20);		//连续采集模式(320*240)
        while(i==3)
          i += LPLD_SCCB_WriteReg(0x06, 0xa0);		//亮度控制

              

}


void init_steer(void){
  //占空比 = pwmDuty/10000*100%
  //理论中值占空比=1.5/20=7.5%,实际需根据机械结构和舵机本身进行调整
  FTM_PWM_QuickInit(FTM2_CH0_PB18,kPWM_EdgeAligned,50,0);
  //FTM_PWM_InvertPolarity(HW_FTM2,HW_FTM_CH0,kFTM_PWM_LowTrue);
  
}