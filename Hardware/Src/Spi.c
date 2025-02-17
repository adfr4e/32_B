#include "Spi.h"

/* Function Declaration ------------------------------------------------------*/
static void Hal_SpiConfig(void);
static void SDA_SwitchModeInput(void);
static void SDA_SwitchModeOutput(void);

/*******************************************************************************
 * Function Name  :
 * Description    : SPI外部调用函数
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_SpiInit(void)
{
    Hal_SpiConfig();
}

/*******************************************************************************
 * Function Name  :
 * Description    : 写字节
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_SPI_WriteByte(uint8_t data)
{
    for (int i = 7; i >= 0; i--)
    {
        if (data & (1 << i))
        {
            SDA_HIGH;
        }
        else
        {
            SDA_LOW;
        }

        // 产生SCK上升沿
        SCK_HIGH;
        delay_ns(25);
        SCK_LOW;
        delay_ns(25);
    }
}

/*******************************************************************************
 * Function Name  :
 * Description    : 读字节
 * Return         : None
 * Attention      : None
 *******************************************************************************/
uint8_t Hal_SPI_ReadByte(void)
{
    uint8_t data = 0;

    SDA_SwitchModeInput();
    for (int i = 7; i >= 0; i--)
    {
        // 产生SCK上升沿
        SCK_HIGH;
        delay_ns(25);

        // 读取SDATA
        if (GPIO_GetBit(SPI_PORT, SDA_PIN))
        {
            data |= (1 << i);
        }

        // 产生SCK下降沿
        SCK_LOW;
        delay_ns(25);
    }

    // 恢复SDA为输出模式
    SDA_SwitchModeOutput();

    return data;
}

/*******************************************************************************
 * Function Name  :
 * Description    : SDA设置为输入模式
 * Return         : None
 * Attention      : None
 *******************************************************************************/
static void SDA_SwitchModeInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // SDA
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = MODE_FLOATING_IN; // 浮空输入
    GPIO_Init(SPI_PORT, &GPIO_InitStruct);
}

/*******************************************************************************
 * Function Name  :
 * Description    : SDA设置为输出模式
 * Return         : None
 * Attention      : None
 *******************************************************************************/
static void SDA_SwitchModeOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // SDA
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = MODE_PP_OUT; // 推挽输出
    GPIO_Init(SPI_PORT, &GPIO_InitStruct);
}

/*******************************************************************************
 * Function Name  :
 * Description    : SPI静态初始化函数
 * Return         : None
 * Attention      : None
 *******************************************************************************/
static void Hal_SpiConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // SDA
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = MODE_PP_OUT; // 推挽输出
    GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    // CS
    GPIO_InitStruct.Pin = CS_PIN;
    GPIO_InitStruct.Mode = MODE_PP_OUT; // 推挽输出
    GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    // SCK
    GPIO_InitStruct.Pin = SCK_PIN;
    GPIO_InitStruct.Mode = MODE_PP_OUT; // 推挽输出
    GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    // 初始化状态:CS高电平, SCK低电平
    CS_HIGH;
    SCK_LOW;
}
