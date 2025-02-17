#ifndef __BK4819_H__
#define __BK4819_H__

#include "Spi.h"

void Hal_Bk4819_Write(uint8_t reg, uint16_t data);
uint16_t Hal_Bk4819_Read(uint8_t reg);

#endif
