#include "Cpu.h"

static void Hal_CoreCoclkConfig(void);
static void hal_CPU_Critical_Control(CPU_EA_TYPEDEF cmd, unsigned char *pSta);

/*******************************************************************************
 * Function Name  :
 * Description    : 滴答定时器初始化设置
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_CPUInit(void)
{
    Hal_CoreCoclkConfig();
    OS_CPUInterruptCBSRegister(hal_CPU_Critical_Control);
}

/*******************************************************************************
 * Function Name  :
 * Description    : 设定滴答定时器1us定时
 * Return         : None
 * Attention      : None
 *******************************************************************************/
static void Hal_CoreCoclkConfig(void)
{
    SysTick_Config(SystemCoreClock / 100);
}

/********************************************************************************************************
 *  @函数名   hal_getprimask
 *  @描述     获取CPU总中断状态
 *  @参数     无
 *  @返回值   0-总中断关闭 1-总中断打开
 *  @注意     无
 ********************************************************************************************************/
static unsigned char hal_getprimask(void)
{
    return (!__get_PRIMASK()); // 0是中断打开，1是中断关闭，所以要取反
}

/********************************************************************************************************
 *  @函数名   hal_CPU_Critical_Control
 *  @描述     CPU临界处理控制
 *  @参数     cmd-控制命令  *pSta-总中断状态
 *  @返回值   无
 *  @注意     无
 ********************************************************************************************************/
static void hal_CPU_Critical_Control(CPU_EA_TYPEDEF cmd, unsigned char *pSta)
{
    if (cmd == CPU_ENTER_CRITICAL)
    {
        *pSta = hal_getprimask(); // 保存中断状态
        __disable_irq();          // 关CPU总中断
    }
    else if (cmd == CPU_EXIT_CRITICAL)
    {
        if (*pSta)
        {
            __enable_irq(); // 打开中断
        }
        else
        {
            __disable_irq(); // 关闭中断
        }
    }
}

void SysTick_Handler(void)
{
    OS_ClockInterruptHandle();
    // 定时中断处理
}
