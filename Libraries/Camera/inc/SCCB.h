#include "chlib_k.h"
int SCCB_ReadReg(uint8_t chipAddr, uint8_t addr, uint8_t* data);
int SCCB_WriteReg(uint8_t chipAddr, uint8_t addr, uint8_t data);