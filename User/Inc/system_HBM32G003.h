#ifndef __SYSTEM_HBM32G003_H__
#define __SYSTEM_HBM32G003_H__

#include "core_cm0.h"

#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************************************************************************
 * 系统时钟设定
 *****************************************************************************************************************************************/
#define SYS_CLK_48MHz		  0	 	//0 内部高频48MHz RC振荡器
#define SYS_CLK_48MHz_DIV2	  1		//1 内部高频48MHz RC振荡器  2分频
#define SYS_CLK_48MHz_DIV4	  2		//2 内部高频48MHz RC振荡器  4分频
#define SYS_CLK_XTAL		  3		//3 外部晶体振荡器（4-32MHz）
#define SYS_CLK_XTAL_DIV2	  4		//4 外部晶体振荡器（4-32MHz） 2分频
#define SYS_CLK_XTAL_DIV4	  5		//5 外部晶体振荡器（4-32MHz） 4分频
#define SYS_CLK_32KHz		  6		//6 内部低频32KHz RC振荡器

#define __HSI		(48000000UL)		//高速内部时钟
#define __LSI		(   32000UL)		//低速内部时钟
#define __HSE		(24000000UL)		//高速外部时钟	 
	 
	 
extern uint32_t SystemCoreClock;		// System Clock Frequency (Core Clock)
extern uint32_t CyclesPerUs;			// Cycles per micro second

extern void SystemInit(void);
void SystemClkSel(uint8_t clk);
void SystemCoreClockUpdate(void);


#ifdef __cplusplus
}
#endif

#endif //__SYSTEM_HBM32G003_H__
