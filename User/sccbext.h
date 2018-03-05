#ifndef __DEV_SCCB_H__
#define __DEV_SCCB_H__
#include "chlib_k.h"

#define ADR_OV7670      0x42

//定义SCCB设备地址
#define SCCB_DEV_ADR    ADR_OV7670
//定义SCL、SDA的引脚
#define SCCB_SCL        PCout(0)
#define SCCB_SDA_O      PCout(3)
#define SCCB_SDA_I      PCin(3)
//定义SDA输入输出
#define SCCB_SDA_OUT()  BITBAND_REG(PTC->PDDR, 3)=1
#define SCCB_SDA_IN()   BITBAND_REG(PTC->PDDR, 3)=0

#define SCCB_DELAY()	LPLD_SCCB_Delay(5000)

uint8_t LPLD_SCCB_WriteReg(uint16_t, uint8_t);
uint8_t LPLD_SCCB_ReadReg(uint8_t, uint8_t*, uint16_t);

#endif
