#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "pid.h"
#include "oled.h"
#include "sccbext.h"
#include "chlib_k.h"

typedef union ENCODER_SPEED_UNION{
    float   float32bits;
    uint8_t int8bits[4];
} ENCODER_SPEED_UNION;

extern uint8_t  encoder_dir;
extern int16_t  encoder_val;
extern uint32_t encoder_period;
extern ENCODER_SPEED_UNION encoder_speed; 

void ENCODER_Init(FTM_QD_Mode_Type mode, uint32_t timeInUs);
void ENCODER_Interrupt_Handler(void);
void ENCODER_Display_Speed(void);
void ENCODER_UART_TX_Speed(const uint32_t instance);

#endif