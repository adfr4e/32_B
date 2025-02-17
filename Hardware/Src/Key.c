#include "Key.h"

/* Function Declaration ------------------------------------------------------*/
static void Hal_Keyconfig(void);

/*******************************************************************************
 * Function Name  :
 * Description    : 按键初始化调用函数
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_KeyInit(void)
{
    Hal_Keyconfig();
}

/*******************************************************************************
 * Function Name  :
 * Description    : 按键检测
 * Return         : None
 * Attention      : None
 *******************************************************************************/
uint8_t Hal_KeyGetVal(void)
{
    uint8_t KeyVal = 0;
    if (GPIO_GetBit(KEY_PORT, KEY_PTT_PIN) == 0)
    {
        delay_ms(20);
        while (GPIO_GetBit(KEY_PORT, KEY_PTT_PIN) == 0)
            ;
        delay_ms(20);
        KeyVal = 1;
    }

    if (GPIO_GetBit(KEY_PORT, KEY_UP_PIN) == 0)
    {
        delay_ms(20);
        while (GPIO_GetBit(KEY_PORT, KEY_UP_PIN) == 0)
            ;
        delay_ms(20);
        KeyVal = 2;
    }

    if (GPIO_GetBit(KEY_PORT, KEY_DOWN_PIN) == 0)
    {
        delay_ms(20);
        while (GPIO_GetBit(KEY_PORT, KEY_DOWN_PIN) == 0)
            ;
        delay_ms(20);
        KeyVal = 3;
    }
    return KeyVal;
}

/*******************************************************************************
 * Function Name  :
 * Description    : 按键静态初始化文件
 * Return         : None
 * Attention      : None
 *******************************************************************************/
static void Hal_Keyconfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PTT
    GPIO_InitStruct.Pin = KEY_PTT_PIN;
    GPIO_InitStruct.Mode = MODE_PU_IN; // 上拉输入
    GPIO_Init(KEY_PORT, &GPIO_InitStruct);

    // UP
    GPIO_InitStruct.Pin = KEY_UP_PIN;
    GPIO_InitStruct.Mode = MODE_PU_IN; // 上拉输入
    GPIO_Init(KEY_PORT, &GPIO_InitStruct);

    // DOWN
    GPIO_InitStruct.Pin = KEY_DOWN_PIN;
    GPIO_InitStruct.Mode = MODE_PU_IN; // 上拉输入
    GPIO_Init(KEY_PORT, &GPIO_InitStruct);
}
