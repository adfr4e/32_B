/**********************************主函数入口**********************************/
/* include--------------------------------------------------------------------*/
#include "main.h"

int main(void)
{

	SystemClkSel(SYS_CLK_48MHz); // 配置系统时钟为48MHz
	SystemCoreClockUpdate();	 // 计算配置的系统时钟

	Hal_CPUInit();
	OS_TaskInit();

	Hal_LedInit();
	OS_CreatTask(OS_TASK1, Hal_LedProc, 1, OS_RUN); // 10ms进入SysTick_Handler中断

	OS_Start();
}
