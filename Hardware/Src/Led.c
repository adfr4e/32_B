#include "Led.h"
/* Function Declaration ------------------------------------------------------*/
void Hal_LedToggle(void);
static void Hal_LedConfig(void);

/*******************************************************************************
 * Function Name  :
 * Description    : Led初始化函数
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_LedInit(void)
{
    Hal_LedConfig();
}

/*******************************************************************************
 * Function Name  :
 * Description    : Led任务函数
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_LedProc(void)
{
    static uint32_t i = 0;
    i++;
    if (i > 50)
    {
        i = 0;
        Hal_LedToggle();
    }
}

/*******************************************************************************
 * Function Name  :
 * Description    : Led点亮功能
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_LedOn(void)
{
    GPIO_ClrBit(LED_PORT, LED_PIN);
}

/*******************************************************************************
 * Function Name  :
 * Description    : Led熄灭功能
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_LedOff(void)
{
    GPIO_SetBit(LED_PORT, LED_PIN);
}

/*******************************************************************************
 * Function Name  :
 * Description    : Led翻转功能
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_LedToggle(void)
{
    GPIO_InvBit(LED_PORT, LED_PIN);
}

/*******************************************************************************
 * Function Name  : void LedConfig(void)
 * Description    : LED静态初始化函数
 * Return         : None
 * Attention      : 低电平点亮,初始化为高电平
 *******************************************************************************/
static void Hal_LedConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = MODE_PP_OUT; // 推挽输出
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    GPIO_SetBit(LED_PORT, LED_PIN); // 设置为高电平
}
