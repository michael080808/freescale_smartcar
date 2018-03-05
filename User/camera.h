#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "isr.h"
#include "oled.h"
#include "sccbext.h"
#include "chlib_k.h"

#define CAMERA_ROW    50                            //摄像头采集行数
#define CAMERA_COL    152                           //摄像头采集列数
#define CAMERA_CENTER 80                            //摄像头采集中心值

#define P_WIDTH       8                             //黑线宽度
#define BW_DELTA      50                            //白色宽度
#define THRESHOLD     130                           //摄像头阈值
#define LINE_EDGE     2                             //线边界
#define BLOCK_LEN     20                            //块长度

extern uint8_t  image;  
extern uint16_t H_Cnt;                              //记录行中断数
extern uint32_t V_Cnt;                              //记录场中断次数

extern uint8_t  image;                              //标定奇偶场
//image为0, 表示正在处理img1, DMA接收存储在img2
//image为1, 表示正在处理img2, DMA接收存储在img1
extern uint8_t  img1[CAMERA_ROW][CAMERA_COL];       //奇数场存储位置
extern uint8_t  img2[CAMERA_ROW][CAMERA_COL];       //偶数场存储位置
extern uint8_t *imgaddr;                            //当前待处理场起始地址

extern uint8_t  l_line_index[CAMERA_ROW];           //左引导线列号
extern uint8_t  r_line_index[CAMERA_ROW];           //右引导线列号

extern const uint8_t offset[];                      //每一行的lp1,lp2扫描偏移量

void CAMERA_Init(void);                             //图像初始化
void CAMERA_Processing(void);                       //图像识别处理函数
void CAMERA_Display_Full(void);                     //显示完整图像到OLED
void CAMERA_Display_Edge(void);                     //显示边界图像到OLED
void CAMERA_UART_TX_Full(const uint32_t instance);  //完整图像灰度串口发s送
void CAMERA_UART_TX_Edge(const uint32_t instance);  //边界图像灰度串口发送

#endif
