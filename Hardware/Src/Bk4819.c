#include "Bk4819.h"

/*******************************************************************************
 * Function Name  :
 * Description    : 写数据到BK4819寄存器
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void Hal_Bk4819_Write(uint8_t reg, uint16_t data)
{
    CS_LOW;

    // 发送寄存器地址
    Hal_SPI_WriteByte((reg << 1) | 0x00);

    // 发送数据(高位在前)
    Hal_SPI_WriteByte((data >> 8) & 0xFF);
    Hal_SPI_WriteByte(data & 0xFF);

    CS_HIGH;
}

/*******************************************************************************
 * Function Name  :
 * Description    : 从BK4819寄存器读取数据
 * Return         : None
 * Attention      : None
 *******************************************************************************/
uint16_t Hal_Bk4819_Read(uint8_t reg)
{
    uint16_t data = 0;

    CS_LOW;

    // 发送寄存器地址
    Hal_SPI_WriteByte((reg << 1) | 0x01);

    // 读取数据(高位在前)
    data = Hal_SPI_ReadByte() << 8;
    data |= Hal_SPI_ReadByte();

    SCK_HIGH;

    return data;
}
