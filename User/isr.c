/*********************************************************
  @2017CameraDemo
  @固件库：超核V2.4
  @author：wgq & lq
  @2017.11.27
  @for seu2017 摄像头组
*********************************************************/

#include "isr.h"

//中断服务函数，需要采集数据量(行，场)可自行修改
//可以在函数中加入led的控制来检测是否进入中断
//不建议在中断服务函数中执行延时或数据处理/串口收发
void CAMERA_Interrupt_Handler(uint32_t mask)
{
    if(mask & (1 << 7)) //行中断
    {
        if(H_Cnt % 2 && H_Cnt < 100)
            DMA_EnableRequest(HW_DMA_CH0); //使能DMA通道0
        H_Cnt++; 
    }

    if(mask & (1 << 6)) //场中断
    {
        H_Cnt = 0;
        //20场之后开始采集
        if(V_Cnt < 20)
            V_Cnt++;
        else
        {
            image = 1 - image;  //奇偶场切换
            PEout(25) = image;  //PORT_E25显示当前是奇数场还是偶数场
            DMA_SetDestAddress(HW_DMA_CH0, image ? (uint32_t) img1[0] : (uint32_t) img2[0]); //设置DMA通道地址
            imgaddr = image ? img2[0] : img1[0]; //imgaddr获取当前没有被DMA占用的通道进行处理
        }
    }
}
