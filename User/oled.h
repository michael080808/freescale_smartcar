/*********************************************************
  @2017CameraDemo
  @固件库：超核V2.4
  @author：wgq & lq
  @2017.11.27
  @for seu2017 摄像头组
*********************************************************/
#include "chlib_k.h"

#ifndef OLED_H
#define OLED_H


#define uint8_t  unsigned char
#define uint32_t unsigned long 			  	 
//-------------------------------OLED--------------------------------------//

#define OLED_ROW (64)
#define OLED_COL (128)

#define OLED_RST  PAout(14)
#define OLED_RS   PAout(13)
#define OLED_CS   PAout(12)
#define OLED_SDIN PAout(16)
#define OLED_SCLK PAout(15)	
		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
extern uint8_t OLED_GRAM[128][8];
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowString_1608(uint8_t x,uint8_t y, uint8_t *p,uint8_t mode);
void OLED_ShowString_1206(uint8_t x,uint8_t y, uint8_t *p,uint8_t mode);	 
void OLED_ShowNum_1206(uint8_t x, uint8_t y, float num, uint8_t mode);

#endif  
	 
