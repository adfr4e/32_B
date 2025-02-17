#include <stdint.h>
#include "HBM32G003.h"

uint32_t SystemCoreClock  = __HSI;   				      //System Clock Frequency (Core Clock)
uint32_t CyclesPerUs      = (__HSI / 1000000); 		      //Cycles per micro second


/****************************************************************************************************************************************** 
* 函数名称: 
* 功能说明: This function is used to update the variable SystemCoreClock and must be called whenever the core clock is changed
* 输    入: 
* 输    出: 
* 注意事项: 
******************************************************************************************************************************************/
void SystemCoreClockUpdate(void)    
{
	uint8_t Clk_Div = 1;
	uint32_t delay = 0;
	
	if((SYS->CLKSEL & SYS_CLKSEL_SYS_MSK) == 0)
	{
		if(SYS->RCCR & SYS_RCCR_RCHF_SEL_MSK)
		{
			SystemCoreClock  = __HSI >> 1; 
		}
		else
		{
			SystemCoreClock  = __HSI; 
		}
	}
	else
	{
		Clk_Div = 0x01 << ((SYS->CLKSEL & SYS_CLKSEL_DIV_MSK) >> SYS_CLKSEL_DIV_POS);
		
		if((SYS->RCCR & SYS_RCCR_RCHF_SEL_MSK)&&((SYS->CLKSEL&SYS_CLKSEL_SRC_MSK)==0))
		{
			Clk_Div  *= 2;      //如果RCHF是24M，则分频系数乘以2
		}
		
		if(((SYS->CLKSEL & SYS_CLKSEL_SRC_MSK)>> SYS_CLKSEL_SRC_POS) == 0x00)
		{
			SystemCoreClock  = __HSI / Clk_Div; 
		}
		else if(((SYS->CLKSEL & SYS_CLKSEL_SRC_MSK) >> SYS_CLKSEL_SRC_POS) == 0x01)
		{
			SystemCoreClock  = __LSI / Clk_Div; 
		}
		else if(((SYS->CLKSEL & SYS_CLKSEL_SRC_MSK) >> SYS_CLKSEL_SRC_POS) == 0x02)
		{
			SystemCoreClock  = __HSE / Clk_Div; 
		}	
	}
	
	CyclesPerUs = SystemCoreClock/1000000;
	
	for(delay = 0;delay < 100000;delay++);
}


void Switch_CLK_48MHZ(void)
{
	uint32_t temp = 0;
	
	temp = SYS->RCCR;
	
	temp |= 0x01 << SYS_RCCR_RCHF_EN_POS;             //RCHF使能 
	temp &= ~(0x01 << SYS_RCCR_RCHF_SEL_POS);         //RCHF选择48MHZ
	
	SYS->RCCR = temp;
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x03 << SYS_CLKSEL_SRC_POS);          //时钟源选择RCHF
	temp &= ~(0x01 << SYS_CLKSEL_SYS_POS);          //系统时钟选择为RCHF
	
	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN &= ~(0x01 << SYS_CLKDIV_EN_DIV_POS);    //分频时钟禁止输出
}


void Switch_CLK_48MHZ_DIV2(void)
{
	uint32_t temp = 0;
	
	temp = SYS->RCCR;
	
	temp |= 0x01 << SYS_RCCR_RCHF_EN_POS;             //RCHF使能 
	temp |= (0x01 << SYS_RCCR_RCHF_SEL_POS);          //RCHF选择24MHZ
	
	SYS->RCCR = temp;
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x03 << SYS_CLKSEL_SRC_POS);          //时钟源选择RCHF
	temp &= ~(0x01 << SYS_CLKSEL_SYS_POS);          //系统时钟选择为RCHF
	
	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN &= ~(0x01 << SYS_CLKDIV_EN_DIV_POS);    //分频时钟禁止输出
}


void Switch_CLK_48MHZ_DIV4(void)
{
	uint32_t temp = 0;
	
	Switch_CLK_48MHZ_DIV2();                            //切换时钟时首先切回到内部24MHZ
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x07 << SYS_CLKSEL_DIV_POS);       
	temp |= 0x01 << SYS_CLKSEL_DIV_POS;                 //分频时钟选择2分频

	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN |= 0x01 << SYS_CLKDIV_EN_DIV_POS;    //分频时钟输出 
	
	SYS->CLKSEL |= 0x01 << SYS_CLKSEL_SYS_POS;          //系统时钟选择分频时钟
}


void Switch_CLK_XTAL(void)
{
	uint32_t temp = 0, k = 0;
	
	Switch_CLK_48MHZ_DIV2();                            //切换时钟时首先切回到内部24MHZ
	
	SYS->XTACR |= 0x01 << SYS_XTACR_XTAH_EN_POS;        //XTAH使能
	
	for(k = 0; k < 100000;k++);                         //XTAH使能后，至少等待2ms再使用
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x07 << SYS_CLKSEL_DIV_POS);              //分频时钟选择1分频
	
	temp &= ~(0x03 << SYS_CLKSEL_SRC_POS);       
	temp |= 0x02 << SYS_CLKSEL_SRC_POS;                 //时钟源选择XTAH
	
	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN |= 0x01 << SYS_CLKDIV_EN_DIV_POS;    //分频时钟输出 
	
	SYS->CLKSEL |= 0x01 << SYS_CLKSEL_SYS_POS;          //系统时钟选择分频时钟
}


void Switch_CLK_XTAL_DIV2(void)
{
	uint32_t temp = 0, k = 0;
	
	Switch_CLK_48MHZ_DIV2();                            //切换时钟时首先切回到内部24MHZ
	
	SYS->XTACR |= 0x01 << SYS_XTACR_XTAH_EN_POS;        //XTAH使能
	
	for(k = 0; k < 100000;k++);                         //XTAL使能后，至少等待2ms再使用
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x07 << SYS_CLKSEL_DIV_POS);       
	temp |= 0x01 << SYS_CLKSEL_DIV_POS;                 //分频时钟选择2分频
	
	temp &= ~(0x03 << SYS_CLKSEL_SRC_POS);       
	temp |= 0x02 << SYS_CLKSEL_SRC_POS;                 //时钟源选择XTAL
	
	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN |= 0x01 << SYS_CLKDIV_EN_DIV_POS;    //分频时钟输出 
	
	SYS->CLKSEL |= 0x01 << SYS_CLKSEL_SYS_POS;          //系统时钟选择分频时钟
}


void Switch_CLK_XTAL_DIV4(void)
{
	uint32_t temp = 0, k = 0;
	
	Switch_CLK_48MHZ_DIV2();                            //切换时钟时首先切回到内部24MHZ
	
	SYS->XTACR |= 0x01 << SYS_XTACR_XTAH_EN_POS;        //XTAH使能
	
	for(k = 0; k < 100000;k++);                         //XTAL使能后，至少等待2ms再使用
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x07 << SYS_CLKSEL_DIV_POS);       
	temp |= 0x02 << SYS_CLKSEL_DIV_POS;          //分频时钟选择4分频
	
	temp &= ~(0x03 << SYS_CLKSEL_SRC_POS);       
	temp |= 0x02 << SYS_CLKSEL_SRC_POS;          //时钟源选择XTAL
	
	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN |= 0x01 << SYS_CLKDIV_EN_DIV_POS;    //分频时钟输出 
	
	SYS->CLKSEL |= 0x01 << SYS_CLKSEL_SYS_POS;          //系统时钟选择分频时钟
}


void Switch_CLK_32KHZ(void)
{
	uint32_t temp = 0;
	
	Switch_CLK_48MHZ_DIV2();                            //切换时钟时首先切回到内部24MHZ
	
	temp = SYS->CLKSEL;
	
	temp &= ~(0x07 << SYS_CLKSEL_DIV_POS);       //分频时钟选择1分频
	
	temp &= ~(0x03 << SYS_CLKSEL_SRC_POS);       
	temp |= 0x01 << SYS_CLKSEL_SRC_POS;          //时钟源选择内部32KHZ
	
	SYS->CLKSEL = temp;
	
	SYS->CLKDIV_EN |= 0x01 << SYS_CLKDIV_EN_DIV_POS;    //分频时钟输出 
	
	SYS->CLKSEL |= 0x01 << SYS_CLKSEL_SYS_POS;          //系统时钟选择分频时钟            
}


/****************************************************************************************************************************************** 
* 函数名称: 
* 功能说明: The necessary initializaiton of systerm
* 输    入: 
* 输    出: 
* 注意事项: 
******************************************************************************************************************************************/
void SystemClkSel(uint8_t clk)
{		
	uint32_t TRIM_POW_Addr = 0x3FFE4;
	uint32_t TRIM_RC_Addr = 0x3FFE0;
	
	PMU->TRIM_POW = FLASH_Read_Word_2(TRIM_POW_Addr);
	
	PMU->TRIM_RC = FLASH_Read_Word_2(TRIM_RC_Addr);
	
	
	switch(clk)
	{
		case SYS_CLK_48MHz:			        //0 内部高频48MHz RC振荡器
			
			Switch_CLK_48MHZ();
		
		break;
		
		case SYS_CLK_48MHz_DIV2:			//1 内部高频48MHz RC振荡器  2分频
			
			Switch_CLK_48MHZ_DIV2();
		
		break;
		
		case SYS_CLK_48MHz_DIV4:			//2 内部高频48MHz RC振荡器  4分频
			
			Switch_CLK_48MHZ_DIV4();
		
		break;
		
		case SYS_CLK_XTAL:			        //3 外部晶体振荡器（4-32MHz）
			
			Switch_CLK_XTAL();
		
		break;
		
		case SYS_CLK_XTAL_DIV2:			    //4 外部晶体振荡器（4-32MHz）  2分频
			
			Switch_CLK_XTAL_DIV2();
		
		break;
		
		case SYS_CLK_XTAL_DIV4:			    //5 外部晶体振荡器（4-32MHz）  4分频
			
			Switch_CLK_XTAL_DIV4();
		
		break;
		
		case SYS_CLK_32KHz:			        //6 内部低频32KHz RC振荡器
			
			Switch_CLK_32KHZ();
		
		break;	
	}	
}


/****************************************************************************************************************************************** 
* 函数名称: 
* 功能说明: The necessary initializaiton of systerm
* 输    入: 
* 输    出: 
* 注意事项: 
******************************************************************************************************************************************/
void SystemInit(void)
{		
	FLASH_Init(FLASH_QuadReadData,FLASH_CLKDIV_1);     //配置FLASH控制器为4线读模式，时钟1分频
	
	SystemClkSel(SYS_CLK_48MHz);                       //配置系统时钟为48MHz
	
	SystemCoreClockUpdate();	                       //计算配置的系统时钟
}


