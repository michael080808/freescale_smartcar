#include "SCCB.h"

#define SDA_DDR_OUT()       do {GPIO_PinConfig(HW_GPIOC, 3, kOutput);}while(0)
#define SDA_DDR_IN()        do {GPIO_PinConfig(HW_GPIOC, 3, kInput);}while(0)
#define SDA_H()             do {GPIO_WriteBit(HW_GPIOC, 3, 1);}while(0)
#define SDA_L()             do {GPIO_WriteBit(HW_GPIOC, 3, 0);}while(0)
#define SCL_H()             do {GPIO_WriteBit(HW_GPIOC, 0, 1);}while(0)
#define SCL_L()             do {GPIO_WriteBit(HW_GPIOC, 0, 0);}while(0)
#define I2C_DELAY()         DelayUs(1)

////////////////////////////////SCCB内部函数////////////////////////////////////
static inline uint8_t SDA_IN(void)
{
    return GPIO_ReadBit(HW_GPIOC, 3);
}

static bool I2C_Start(void)
{
    SDA_DDR_OUT();
    SDA_H();
    SCL_H();
    I2C_DELAY();
    SDA_L();
    I2C_DELAY();
    SCL_L();
    return true;
}

static void I2C_Stop(void)
{
    SCL_L();
    SDA_L();
    I2C_DELAY();
    SCL_H();
    SDA_H();
    I2C_DELAY();
}

static void I2C_Ack(void)
{
    SCL_L();
    SDA_L();
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    SCL_L();
    I2C_DELAY();
}

static void I2C_NAck(void)
{
    SCL_L();
    I2C_DELAY();
    SDA_H();
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    SCL_L();
    I2C_DELAY();
}

static bool I2C_WaitAck(void)
{
    uint8_t ack;
    SDA_DDR_IN();
    SCL_L();
    
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    ack = SDA_IN();
    SCL_L();
    SDA_DDR_OUT();
    
    return ack;
}

static void I2C_SendByte(uint8_t data)
{
    volatile uint8_t i;
    
    i = 8;
    while(i--)
    {
        if(data & 0x80) SDA_H();
        else SDA_L();
        data <<= 1;
        I2C_DELAY();
        SCL_H();
        I2C_DELAY();
        SCL_L();
    }

}

static uint8_t I2C_GetByte(void)
{
    uint8_t i,byte;
    
    i = 8;
    byte = 0;

    SDA_DDR_IN();
    while(i--)
    {
        SCL_L();
        I2C_DELAY();
        SCL_H();
        I2C_DELAY();
        byte = (byte<<1)|(SDA_IN() & 1);
    }
    SCL_L();
    SDA_DDR_OUT();
    return byte;
}

int SCCB_BurstWrite(uint8_t chipAddr, uint32_t addr, uint32_t addrLen, uint8_t *buf, uint32_t len)
{
    uint8_t *p;
    uint8_t err;
    
    p = (uint8_t*)&addr;
    err = 0;
    chipAddr <<= 1;
    
    I2C_Start();
    I2C_SendByte(chipAddr);
    err += I2C_WaitAck();

    while(addrLen--)
    {
        I2C_SendByte(*p++);
        err += I2C_WaitAck();
    }
    
    while(len--)
    {
        I2C_SendByte(*buf++);
        err += I2C_WaitAck();  
    }

    I2C_Stop();
    return err;
}

int SCCB_WriteSingleReg(uint8_t chipAddr, uint8_t addr, uint8_t data)
{
    return SCCB_BurstWrite(chipAddr, addr, 1, &data, 1);
}
////////////////////////////////////内部函数定义结束///////////////////////////////////

//SCCB读寄存器
int SCCB_ReadReg(uint8_t chipAddr, uint8_t addr, uint8_t* data){
      uint8_t err;
    uint8_t retry;
    
    retry = 10;
    chipAddr <<= 1;
    
    while(retry--)
    {
        err = 0;
        I2C_Start();
        I2C_SendByte(chipAddr);
        err += I2C_WaitAck();
        
        I2C_SendByte(addr);
        err += I2C_WaitAck();
        
        I2C_Stop();
        I2C_Start();
        I2C_SendByte(chipAddr+1);
        err += I2C_WaitAck();
        
        *data = I2C_GetByte();
       // err += I2C_WaitAck();
        
        I2C_NAck();
        I2C_Stop();
        if(!err)
        {
            break;
        }
    }

    return err;
}

int SCCB_WriteReg(uint8_t chipAddr, uint8_t addr, uint8_t data){
      uint8_t err;
    uint8_t retry;
    
    retry = 10;
    
    while(retry--)
    {
        err = SCCB_WriteSingleReg(chipAddr, addr, data);
        if(!err)
        {
            break;
        }
    }
    return err;
}
