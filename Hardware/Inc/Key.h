#ifndef __KEY_H__
#define __KEY_H__

#include "HBM32G003.h"
#include "delay.h"

#define KEY_PORT GPIOA
#define KEY_PTT_PIN GPIO_PIN9
#define KEY_UP_PIN GPIO_PIN11
#define KEY_DOWN_PIN GPIO_PIN12

void Hal_KeyInit(void);
uint8_t Hal_KeyGetVal(void);

#endif
