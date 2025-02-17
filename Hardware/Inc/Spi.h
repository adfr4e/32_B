#ifndef __SPI_H__
#define __SPI_H__

#include "HBM32G003.h"
#include "delay.h"

#define SPI_PORT GPIOA
#define SDA_PIN GPIO_PIN2
#define CS_PIN GPIO_PIN3
#define SCK_PIN GPIO_PIN4

#define SDA_HIGH GPIO_SetBit(SPI_PORT, SDA_PIN)
#define SDA_LOW GPIO_ClrBit(SPI_PORT, SDA_PIN)

#define CS_HIGH GPIO_SetBit(SPI_PORT, CS_PIN)
#define CS_LOW GPIO_ClrBit(SPI_PORT, CS_PIN)

#define SCK_HIGH GPIO_SetBit(SPI_PORT, SCK_PIN)
#define SCK_LOW GPIO_ClrBit(SPI_PORT, SCK_PIN)

void Hal_SpiInit(void);
void Hal_SPI_WriteByte(uint8_t data);
uint8_t Hal_SPI_ReadByte(void);

#endif
