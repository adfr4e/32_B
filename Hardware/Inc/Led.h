#ifndef __LED_H__
#define __LED_H__

#include "HBM32G003.h"

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN10

void Hal_LedInit(void);
void Hal_LedProc(void);
void Hal_LedOn(void);
void Hal_LedOff(void);
void Hal_LedToggle(void);

#endif
