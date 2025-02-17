#ifndef __HBM32G003_H__
#define __HBM32G003_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers **********************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                        */
  HardFault_IRQn	          = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt				 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                  */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                  */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt              */
  
/******  Cortex-M0 specific Interrupt Numbers ************************************************/
  UART0_IRQn          = 0,
  TIMPLUS0_IRQn       = 1,
  PWMBASE0_IRQn       = 2,
  PWMPLUS0_IRQn       = 3,
  IIC0_IRQn           = 4,
  ADC_IRQn            = 5,
  SPI0_IRQn           = 6,
  IWDT_IRQn           = 7,
  GPIOA0_IRQn         = 8,
  GPIOA1_IRQn         = 9,
  GPIOA2_IRQn         = 10,
  GPIOA3_IRQn         = 11,
  GPIOA4_IRQn         = 12,
  GPIOA5_IRQn         = 13,
  GPIOA6_IRQn         = 14,
  GPIOA7_IRQn         = 15,
  GPIOA8_IRQn         = 16,
  GPIOA9_IRQn         = 17,
  GPIOA10_IRQn        = 18,	
  GPIOA11_IRQn        = 19,
  GPIOA12_IRQn        = 20,
  GPIOA13_IRQn        = 21,
  GPIOA14_IRQn        = 22,
  GPIOA15_IRQn        = 23,
  IRQ24_IRQn          = 24,
  IRQ25_IRQn          = 25,
  IRQ26_IRQn          = 26,
  IRQ27_IRQn          = 27,
  IRQ28_IRQn          = 28,
  IRQ29_IRQn          = 29,
  IRQ30_IRQn          = 30,
  IRQ31_IRQn          = 31,
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT		    0	   /*!< UART does not provide a MPU present or not	     */
#define __NVIC_PRIO_BITS		2	   /*!< UART Supports 2 Bits for the Priority Levels	 */
#define __Vendor_SysTickConfig  0	   /*!< Set to 1 if different SysTick Config is used	 */

#if   defined ( __CC_ARM )
  #pragma anon_unions
#endif

#include <stdio.h>
#include "core_cm0.h"				   /* Cortex-M0 processor and core peripherals		     */

typedef enum {RESET = 0, SET = 1} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = 1} FunctionalState;

#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = 1} ErrorStatus;


/******************************************************************************/
/*				Device Specific Peripheral registers structures			 */
/******************************************************************************/

typedef struct {
	__IO uint32_t CLKSEL;				//Clock Select

	__IO uint32_t CLKDIV_EN;			//CLK_DIV时钟源开关

	__IO uint32_t CLKEN;			    //外设Clock Enable

	uint32_t RESERVED1[1];
	
	__IO uint32_t IWDTRST_ST;           //独立看门狗复位状态             
	
	uint32_t RESERVED2[(0x100 - 0x14)/4];
	
	__IO uint32_t RCCR;					//RC Control Register
	
	__IO uint32_t XTACR;			    //XTA Control Register
} SYS_TypeDef;

#define SYS_CLKSEL_SYS_POS		      0		//系统时钟选择	0 RCHF  1 CLK_DIVx
#define SYS_CLKSEL_SYS_MSK		      (0x01 << SYS_CLKSEL_SYS_POS)

#define SYS_CLKSEL_DIV_POS		      1		//选择CLK_DIVx  0 DIV1  1 DIV2  2 DIV4   3 DIV8  4  DIV16   5  DIV32
#define SYS_CLKSEL_DIV_MSK		      (0x07 << SYS_CLKSEL_DIV_POS)

#define SYS_CLKSEL_SRC_POS		      4		//选择SRC  0 RCHF   1 RCLF   2 XTAL   3 RESERVED
#define SYS_CLKSEL_SRC_MSK		      (0x07 << SYS_CLKSEL_SRC_POS)

#define SYS_CLKSEL_ADC_POS		      10	//ADC采样时钟选择  0 1分频   1 2分频   2 4分频   3 8分频
#define SYS_CLKSEL_ADC_MSK		      (0x03 << SYS_CLKSEL_ADC_POS)


#define SYS_CLKDIV_EN_DIV_POS	      0		//分频时钟选择	0 使能分频时钟输出  1 禁止分频时钟输出	
#define SYS_CLKDIV_EN_DIV_MSK         (0x01 << SYS_CLKDIV_EN_DIV_POS)


#define SYS_CLKEN_GPIOA_POS		      0		//GPIOA时钟门控   
#define SYS_CLKEN_GPIOA_MSK		      (0x01 << SYS_CLKEN_GPIOA_POS)

#define SYS_CLKEN_IIC0_POS		      4		//IIC0时钟门控   
#define SYS_CLKEN_IIC0_MSK		      (0x01 << SYS_CLKEN_IIC0_POS)

#define SYS_CLKEN_UART0_POS		      6     //UART0时钟门控   
#define SYS_CLKEN_UART0_MSK		      (0x01 << SYS_CLKEN_UART0_POS)

#define SYS_CLKEN_SPI0_POS		      10    //SPI0时钟门控   
#define SYS_CLKEN_SPI0_MSK		      (0x01 << SYS_CLKEN_SPI0_POS)

#define SYS_CLKEN_TIMPLUS0_POS        15    //TIMPLUS0时钟门控   
#define SYS_CLKEN_TIMPLUS0_MSK        (0x01 << SYS_CLKEN_TIMPLUS0_POS)

#define SYS_CLKEN_PWMBASE0_POS		  17	//PWMBASE0时钟门控   
#define SYS_CLKEN_PWMBASE0_MSK		  (0x01 << SYS_CLKEN_PWMBASE0_POS)

#define SYS_CLKEN_PWMPLUS0_POS		  20	//PWMPLUS0时钟门控   
#define SYS_CLKEN_PWMPLUS0_MSK		  (0x01 << SYS_CLKEN_PWMPLUS0_POS)

#define SYS_CLKEN_IWDT_POS		      23    //IWDT时钟门控   
#define SYS_CLKEN_IWDT_MSK		      (0x01 << SYS_CLKEN_IWDT_POS)

#define SYS_CLKEN_ADC_POS		      25    //ADC时钟门控   
#define SYS_CLKEN_ADC_MSK		      (0x01 << SYS_CLKEN_ADC_POS)


#define SYS_IWDTRST_ST_POS		      0	    //独立看门狗复位状态
#define SYS_IWDTRST_ST_MSK		      (0x01 << SYS_IWDTRST_ST_POS)
 
 
#define SYS_RCCR_RCHF_EN_POS          0     //RCHF使能控制位
#define SYS_RCCR_RCHF_EN_MSK          (0x01 << SYS_RCCR_RCHF_EN_POS)

#define SYS_RCCR_RCLF_EN_POS          1     //RCLF使能控制位
#define SYS_RCCR_RCLF_EN_MSK          (0x01 << SYS_RCCR_RCLF_EN_POS)

#define SYS_RCCR_RCHF_SEL_POS         4     //RCHF频率选择控制位   0: 48MHZ  1: 24MHZ
#define SYS_RCCR_RCHF_SEL_MSK         (0x01 <<SYS_RCCR_RCHF_SEL_POS)

#define SYS_RCCR_XTAH_RCHF_POS        8
#define SYS_RCCR_XTAH_RCHF_MSK        (0x01 <<SYS_RCCR_XTAH_RCHF_POS) 

#define SYS_RCCR_XTAH_RCLF_POS        9
#define SYS_RCCR_XTAH_RCLF_MSK        (0x01 <<SYS_RCCR_XTAH_RCLF_POS) 


#define SYS_XTACR_XTAH_EN_POS		  0	//XTAH使能
#define SYS_XTACR_XTAH_EN_MSK		  (0x01 << SYS_XTACR_XTAH_EN_POS)



typedef struct {
	__IO uint32_t PORTA_SEL0;                //功能选择
	
	__IO uint32_t PORTA_SEL1;
	
	uint32_t RESERVED1[(0x100 - 0x08)/4];
	
	__IO uint32_t PORTA_INEN;               //输入使能
	
	uint32_t RESERVED2[(0x200 - 0x104)/4];
	
	__IO uint32_t PORTA_PU_IN;              //上拉使能
	
	uint32_t RESERVED3[(0x300 - 0x204)/4];
	
    __IO uint32_t PORTA_PD_IN;              //下拉使能
  
    uint32_t RESERVED4[(0x400 - 0x304)/4];     
			 
	__IO uint32_t PORTA_OD_OUT;              //开漏使能
    
    uint32_t RESERVED5[(0x500 - 0x404)/4]; 
	
	__IO uint32_t PORTA_WAKEUP;             //唤醒使能
    
    uint32_t RESERVED6[(0x600 - 0x504)/4]; 
	
	__IO uint32_t PORT_CFG;                 //端口配置
} PORT_TypeDef;


#define PORT_CFG_DS_POS		            0		//驱动能力选择  00 5ma  01 14ma  10 22ma  11 30ma 
#define PORT_CFG_DS_MSK		            (0x03 << PORT_CFG_DS_POS)

#define PORT_CFG_SPIFLASH_DS_POS		2		//驱动能力选择  00 5ma  01 14ma  10 22ma  11 30ma 
#define PORT_CFG_SPIFLASH_DS_MSK	    (0x03 << PORT_CFG_SPIFLASH_DS_POS)

#define PORT_CFG_PUR_POS		        8		//上拉电阻选择  00 保留  01  150K   10 40K    11  32K  
#define PORT_CFG_PUR_MSK		        (0x03 << PORT_CFG_PUR_POS)

#define PORT_CFG_HYS_POS		        10		//输入迟滞等级选择  0  低输入 >0.7VDD  <0.3VDD   1 高输入  >0.85VDD <0.15VDD 
#define PORT_CFG_HYS_MSK		        (0x01 << PORT_CFG_HYS_POS)

#define PORT_CFG_WKRF_POS		        11		//唤醒功能沿选择  0  下降沿  1  上升沿
#define PORT_CFG_WKRF_MSK		        (0x01 << PORT_CFG_WKRF_POS)


typedef struct {
	__IO uint32_t DATA;                 //数据寄存器

	__IO uint32_t DIR;					//0 输入	1 输出

	__IO uint32_t INTLVLTRG;			//Interrupt Level Trigger  1 电平触发中断	0 边沿触发中断

	__IO uint32_t INTBE;			    //Both Edge，当INTLVLTRG设为边沿触发中断时，此位置1表示上升沿和下降沿都触发中断，置0时触发边沿由INTRISEEN选择

	__IO uint32_t INTRISEEN;			//Interrupt Rise Edge Enable   1 上升沿/高电平触发中断	0 下降沿/低电平触发中断

	__IO uint32_t INTEN;			    //1 中断使能	0 中断禁止

	__IO uint32_t INTRAWSTAT;			//中断检测单元是否检测到了触发中断的条件 1 检测到了中断触发条件	0 没有检测到中断触发条件

	__IO uint32_t INTSTAT;				//INTSTAT.PIN0 = INTRAWSTAT.PIN0 & INTEN.PIN0

	__IO uint32_t INTCLR;			    //写1清除中断标志，只对边沿触发中断有用

} GPIO_TypeDef;


typedef struct {
	__IO uint32_t LOAD;					    //IWDT计数器初始值
	
	__IO uint32_t VALUE;					//IWDT当前计数值
	
	__IO uint32_t CTRL;                     //IWDT控制寄存器
	
	__IO uint32_t IF;						//[0] 中断标志，写0清除 
	
	__IO uint32_t FEED;                     //喂狗寄存器  写入0x55即可喂狗
	
} IWDT_TypeDef;


#define IWDT_CTRL_EN_POS      0   //IWDT启动使能
#define IWDT_CTRL_EN_MSK      (0x01 << IWDT_CTRL_EN_POS)

#define IWDT_CTRL_RSTEN_POS   1   //IWDT复位输出使能
#define IWDT_CTRL_RSTEN_MSK   (0x01 << IWDT_CTRL_RSTEN_POS)


typedef struct {
	__IO uint32_t EN;             //定时器使能
	
	__IO uint32_t DIV;            //定时器分频值
	
	__IO uint32_t CTR;            //定时器控制寄存器
	
	uint32_t RESERVED1[1];
	
	__IO uint32_t IE;             //中断使能寄存器
		
	__IO uint32_t IF;             //状态寄存器
	
	uint32_t RESERVED2[2];
	
	__IO uint32_t HIGH_PERIOD;    //高16位周期配置寄存器
	
	__IO uint32_t HIGH_CNT;       //高16位当前计数值寄存器

	__IO uint32_t HIGH_CVAL;      //高16位捕获值寄存器	
	
	uint32_t RESERVED3[1];
	
	__IO uint32_t LOW_PERIOD;     //低16位周期配置寄存器
	
	__IO uint32_t LOW_CNT;        //低16位当前计数值寄存器

	__IO uint32_t LOW_CVAL;       //低16位捕获值寄存器	
	
	uint32_t RESERVED4[1];
	
	__IO uint32_t HALL_VAL;       //HALL的原始信号电平
	
}TIMPLUS_TypeDef;

#define TIMPLUS_EN_LOW_POS            0        //低16位定时器使能寄存器    
#define TIMPLUS_EN_LOW_MSK            (0x01 << TIMPLUS_EN_LOW_POS)

#define TIMPLUS_EN_HIGH_POS           1        //高16位定时器使能寄存器    
#define TIMPLUS_EN_HIGH_MSK           (0x01 << TIMPLUS_EN_HIGH_POS)


#define TIMPLUS_DIV_POS               0        //定时器分频寄存器  0 1分频   1  2分频    FF 256分频
#define TIMPLUS_DIV_MSK               (0xFF << TIMPLUS_DIV_POS)  


#define TIMPLUS_CTR_LOW_MODE_POS       0        //低16位定时器模式寄存器  
#define TIMPLUS_CTR_LOW_MODE_MSK       (0x03 << TIMPLUS_CTR_LOW_MODE_POS)

#define TIMPLUS_CTR_LOW_CLKSEL_POS     2        //低16位定时器计数时钟源选择
#define TIMPLUS_CTR_LOW_CLKSEL_MSK     (0x03 << TIMPLUS_CTR_LOW_CLKSEL_POS)

#define TIMPLUS_CTR_LOW_EXTSEL_POS     4        //低16位定时器计数模式或输入捕获模式输入信号选择
#define TIMPLUS_CTR_LOW_EXTSEL_MSK     (0x01 << TIMPLUS_CTR_LOW_EXTSEL_POS)

#define TIMPLUS_CTR_LOW_EXTLEVEL_POS   5        //低16位定时器计数模式或输入捕获模式输入信号有效边沿选择
#define TIMPLUS_CTR_LOW_EXTLEVEL_MSK   (0x03 << TIMPLUS_CTR_LOW_EXTV_POS)

#define TIMPLUS_CTR_LOW_OUTEN_POS      7        //低16位定时器周期脉冲输出
#define TIMPLUS_CTR_LOW_OUTEN_MSK      (0x01 << TIMPLUS_CTR_LOW_OUTEN_POS)

#define TIMPLUS_CTR_HIGH_MODE_POS      16        //高16位定时器模式寄存器  
#define TIMPLUS_CTR_HIGH_MODE_MSK      (0x03 << TIMPLUS_CTR_HIGH_MODE_POS)

#define TIMPLUS_CTR_HIGH_CLKSEL_POS    18        //高16位定时器计数时钟源选择
#define TIMPLUS_CTR_HIGH_CLKSEL_MSK    (0x03 << TIMPLUS_CTR_HIGH_CLKSEL_POS)

#define TIMPLUS_CTR_HIGH_EXTSEL_POS    20        //高16位定时器计数模式或输入捕获模式输入信号选择
#define TIMPLUS_CTR_HIGH_EXTSEL_MSK    (0x01 << TIMPLUS_CTR_HIGH_EXTSEL_POS)

#define TIMPLUS_CTR_HIGH_EXTLEVEL_POS  21        //高16位定时器计数模式或输入捕获模式输入信号有效边沿选择
#define TIMPLUS_CTR_HIGH_EXTLEVEL_MSK  (0x03 << TIMPLUS_CTR_HIGH_EXTLEVEL_POS)

#define TIMPLUS_CTR_HIGH_OUTEN_POS     23        //高16位定时器周期脉冲输出
#define TIMPLUS_CTR_HIGH_OUTEN_MSK     (0x01 << TIMPLUS_CTR_HIGH_OUTEN_POS)


#define TIMPLUS_IE_LOW_OVF_POS         0        //低16位定时器溢出中断使能
#define TIMPLUS_IE_LOW_OVF_MSK         (0x01 << TIMPLUS_IE_LOW_OVF_POS)

#define TIMPLUS_IE_LOW_RISE_POS        1        //低16位定时器输入脉冲上升沿中断使能
#define TIMPLUS_IE_LOW_RISE_MSK        (0x01 << TIMPLUS_IE_LOW_RISE_POS)

#define TIMPLUS_IE_LOW_FALL_POS        2        //低16位定时器输入脉冲下降沿中断使能
#define TIMPLUS_IE_LOW_FALL_MSK        (0x01 << TIMPLUS_IE_LOW_FALL_POS)

#define TIMPLUS_IE_HIGH_OVF_POS        8        //高16位定时器溢出中断使能
#define TIMPLUS_IE_HIGH_OVF_MSK        (0x01 << TIMPLUS_IE_HIGH_OVF_POS)

#define TIMPLUS_IE_HIGH_RISE_POS       9        //高16位定时器输入脉冲上升沿中断使能
#define TIMPLUS_IE_HIGH_RISE_MSK       (0x01 << TIMPLUS_IE_HIGH_RISE_POS)

#define TIMPLUS_IE_HIGH_FALL_POS       10       //高16位定时器输入脉冲下降沿中断使能
#define TIMPLUS_IE_HIGH_FALL_MSK       (0x01 << TIMPLUS_IE_HIGH_FALL_POS)

#define TIMPLUS_IE_HALL0_RISE_POS      16       //HALL0上升沿中断使能
#define TIMPLUS_IE_HALL0_RISE_MSK      (0x01 << TIMPLUS_IE_HALL0_RISE_POS)

#define TIMPLUS_IE_HALL0_FALL_POS      17       //HALL0下降沿中断使能
#define TIMPLUS_IE_HALL0_FALL_MSK      (0x01 << TIMPLUS_IE_HALL0_FALL_POS)

#define TIMPLUS_IE_HALL1_RISE_POS      18       //HALL1上升沿中断使能
#define TIMPLUS_IE_HALL1_RISE_MSK      (0x01 << TIMPLUS_IE_HALL1_RISE_POS)

#define TIMPLUS_IE_HALL1_FALL_POS      19       //HALL1下降沿中断使能
#define TIMPLUS_IE_HALL1_FALL_MSK      (0x01 << TIMPLUS_IE_HALL1_FALL_POS)

#define TIMPLUS_IE_HALL2_RISE_POS      20       //HALL2上升沿中断使能
#define TIMPLUS_IE_HALL2_RISE_MSK      (0x01 << TIMPLUS_IE_HALL2_RISE_POS)

#define TIMPLUS_IE_HALL2_FALL_POS      21       //HALL2下降沿中断使能
#define TIMPLUS_IE_HALL2_FALL_MSK      (0x01 << TIMPLUS_IE_HALL2_FALL_POS)


#define TIMPLUS_IF_LOW_OVF_POS         0        //低16位定时器溢出中断状态
#define TIMPLUS_IF_LOW_OVF_MSK         (0x01 << TIMPLUS_IF_LOW_OVF_POS)

#define TIMPLUS_IF_LOW_RISE_POS        1        //低16位定时器输入脉冲上升沿中断状态
#define TIMPLUS_IF_LOW_RISE_MSK        (0x01 << TIMPLUS_IF_LOW_RISE_POS)

#define TIMPLUS_IF_LOW_FALL_POS        2        //低16位定时器输入脉冲下降沿中断状态
#define TIMPLUS_IF_LOW_FALL_MSK        (0x01 << TIMPLUS_IF_LOW_FALL_POS)

#define TIMPLUS_IF_HIGH_OVF_POS        8        //高16位定时器溢出中断状态
#define TIMPLUS_IF_HIGH_OVF_MSK        (0x01 << TIMPLUS_IF_HIGH_OVF_POS)

#define TIMPLUS_IF_HIGH_RISE_POS       9        //高16位定时器输入脉冲上升沿中断状态
#define TIMPLUS_IF_HIGH_RISE_MSK       (0x01 << TIMPLUS_IF_HIGH_RISE_POS)

#define TIMPLUS_IF_HIGH_FALL_POS       10       //高16位定时器输入脉冲下降沿中断状态
#define TIMPLUS_IF_HIGH_FALL_MSK       (0x01 << TIMPLUS_IF_HIGH_FALL_POS)

#define TIMPLUS_IF_HALL0_RISE_POS      16       //HALL0上升沿中断状态
#define TIMPLUS_IF_HALL0_RISE_MSK      (0x01 << TIMPLUS_IF_HALL0_RISE_POS)

#define TIMPLUS_IF_HALL0_FALL_POS      17       //HALL0下降沿中断状态
#define TIMPLUS_IF_HALL0_FALL_MSK      (0x01 << TIMPLUS_IF_HALL0_FALL_POS)

#define TIMPLUS_IF_HALL1_RISE_POS      18       //HALL1上升沿中断状态
#define TIMPLUS_IF_HALL1_RISE_MSK      (0x01 << TIMPLUS_IF_HALL1_RISE_POS)

#define TIMPLUS_IF_HALL1_FALL_POS      19       //HALL1下降沿中断状态
#define TIMPLUS_IF_HALL1_FALL_MSK      (0x01 << TIMPLUS_IF_HALL1_FALL_POS)

#define TIMPLUS_IF_HALL2_RISE_POS      20       //HALL2上升沿中断状态
#define TIMPLUS_IF_HALL2_RISE_MSK      (0x01 << TIMPLUS_IF_HALL2_RISE_POS)

#define TIMPLUS_IF_HALL2_FALL_POS      21       //HALL2下降沿中断状态
#define TIMPLUS_IF_HALL2_FALL_MSK      (0x01 << TIMPLUS_IF_HALL2_FALL_POS)


#define TIMPLUS_HALL_VAL_HALL0_POS    0        //HALL0的原始信号电平
#define TIMPLUS_HALL_VAL_HALL0_MSK    (0x01 << TIMPLUS_HALL_VAL_HALL0_POS)

#define TIMPLUS_HALL_VAL_HALL1_POS    1        //HALL1的原始信号电平
#define TIMPLUS_HALL_VAL_HALL1_MSK    (0x01 << TIMPLUS_HALL_VAL_HALL1_POS)

#define TIMPLUS_HALL_VAL_HALL2_POS    2        //HALL2的原始信号电平
#define TIMPLUS_HALL_VAL_HALL2_MSK    (0x01 << TIMPLUS_HALL_VAL_HALL2_POS)



typedef struct {
	
	__IO uint32_t EN;                     //PWMBASE使能寄存器
											
	__IO uint32_t DIV;                    //PWMBASE分频寄存器
	
	__IO uint32_t OUTCTR;                 //PWMBASE输出控制寄存器
	
	__IO uint32_t PERIOD;                 //PWMBASE周期配置寄存器
	
	__IO uint32_t IE;                     //PWMBASE中断使能寄存器
	
	__IO uint32_t IF;                     //PWMBASE中断状态寄存器
	
	__IO uint32_t CNT;                    //PWMBASE当前计数值寄存器  
	
	uint32_t RESERVED1[1];
	
	__IO uint32_t CH0_COMP;               //PWMBASE通道0的比较点配置寄存器
	
	uint32_t RESERVED2[3];
	
	__IO uint32_t CH1_COMP;               //PWMBASE通道1的比较点配置寄存器
	
	uint32_t RESERVED3[3];
	
	__IO uint32_t CH2_COMP;               //PWMBASE通道2的比较点配置寄存器
	
	
} PWMBASE_TypeDef;


#define PWMBASE_EN_POS                0      //PWMBASE计数器使能
#define PWMBASE_EN_MSK                (0x01 << PWMBASE_EN_POS)

#define PWMBASE_DIV_POS               0      //PWMBASE计数时钟分频系数    0 1分频   1  2分频    FF 256分频
#define PWMBASE_DIV_MSK               (0xFF << PWMBASE_DIV_POS)

#define PWMBASE_OUTCTR_CH0_INV_POS    0      //PWMBASE通道0输出极性翻转
#define PWMBASE_OUTCTR_CH0_INV_MSK    (0x01 << PWMBASE_OUTCTR_CH0_INV_POS)

#define PWMBASE_OUTCTR_CH1_INV_POS    1      //PWMBASE通道1输出极性翻转
#define PWMBASE_OUTCTR_CH1_INV_MSK    (0x01 << PWMBASE_OUTCTR_CH1_INV_POS)

#define PWMBASE_OUTCTR_CH2_INV_POS    2      //PWMBASE通道2输出极性翻转
#define PWMBASE_OUTCTR_CH2_INV_MSK    (0x01 << PWMBASE_OUTCTR_CH2_INV_POS)

#define PWMBASE_OUTCTR_CH0_EN_POS     4      //PWMBASE通道0输出使能
#define PWMBASE_OUTCTR_CH0_EN_MSK     (0x01 << PWMBASE_OUTCTR_CH0_EN_POS)

#define PWMBASE_OUTCTR_CH1_EN_POS     5      //PWMBASE通道1输出使能
#define PWMBASE_OUTCTR_CH1_EN_MSK     (0x01 << PWMBASE_OUTCTR_CH1_EN_POS)

#define PWMBASE_OUTCTR_CH2_EN_POS     6      //PWMBASE通道2输出使能
#define PWMBASE_OUTCTR_CH2_EN_MSK     (0x01 << PWMBASE_OUTCTR_CH2_EN_POS)

#define PWMBASE_IE_CH0_COMP_POS       0      //PWMBASE通道0到达比较点中断使能
#define PWMBASE_IE_CH0_COMP_MSK       (0x01 << PWMBASE_IE_CH0_COMP_POS)

#define PWMBASE_IE_CH1_COMP_POS       1      //PWMBASE通道1到达比较点中断使能
#define PWMBASE_IE_CH1_COMP_MSK       (0x01 << PWMBASE_IE_CH1_COMP_POS)

#define PWMBASE_IE_CH2_COMP_POS       2      //PWMBASE通道2到达比较点中断使能
#define PWMBASE_IE_CH2_COMP_MSK       (0x01 << PWMBASE_IE_CH2_COMP_POS)

#define PWMBASE_IE_OVF_POS            3      //PWMBASE周期溢出中断使能
#define PWMBASE_IE_OVF_MSK            (0x01 << PWMBASE_IE_OVF_POS)

#define PWMBASE_IF_CH0_COMP_POS       0      //PWMBASE通道0到达比较点中断状态
#define PWMBASE_IF_CH0_COMP_MSK       (0x01 << PWMBASE_IF_CH0_COMP_POS)

#define PWMBASE_IF_CH1_COMP_POS       1      //PWMBASE通道1到达比较点中断状态
#define PWMBASE_IF_CH1_COMP_MSK       (0x01 << PWMBASE_IF_CH1_COMP_POS)

#define PWMBASE_IF_CH2_COMP_POS       2      //PWMBASE通道2到达比较点中断状态
#define PWMBASE_IF_CH2_COMP_MSK       (0x01 << PWMBASE_IF_CH2_COMP_POS)

#define PWMBASE_IF_OVF_POS            3      //PWMBASE周期溢出中断状态
#define PWMBASE_IF_OVF_MSK            (0x01 << PWMBASE_IF_OVF_POS)


typedef struct {
	
	__IO uint32_t CTR;                    //PWMPLUS配置寄存器
											
	__IO uint32_t OUTCTR;                 //PWMPLUS输出控制寄存器
	
	__IO uint32_t CLK;                    //PWMPLUS时钟源选择及分频寄存器
	
	__IO uint32_t BRK_CTR;                //PWMPLUS_BRAKE配置寄存器
	
	__IO uint32_t MSK_LEVEL;              //PWMPLUS强制输出电平选择寄存器
	
	uint32_t RESERVED1[2];
	
	__IO uint32_t PERIOD;                 //PWMPLUS周期配置寄存器
	
	__IO uint32_t CH0_COMP;               //PWMPLUS通道0的比较点配置寄存器
	
	__IO uint32_t CH1_COMP;               //PWMPLUS通道1的比较点配置寄存器
	
	__IO uint32_t CH2_COMP;               //PWMPLUS通道2的比较点配置寄存器
	
	uint32_t RESERVED2[1];
	
	__IO uint32_t CH0_DEADZONE;           //PWMPLUS通道0死区长度配置寄存器
	
	__IO uint32_t CH1_DEADZONE;           //PWMPLUS通道1死区长度配置寄存器
	
	__IO uint32_t CH2_DEADZONE;           //PWMPLUS通道2死区长度配置寄存器
	
	uint32_t RESERVED3[1];
	
	__IO uint32_t TRIG_COMP;              //PWMPLUS内部触发比较点寄存器
	
	__IO uint32_t TRIG_SEL;               //PWMPLUS内部触发配置寄存器
	
	uint32_t RESERVED4[6];
	
	__IO uint32_t IE;                     //PWMPLUS中断使能寄存器
	
	__IO uint32_t IF;                     //PWMPLUS中断状态寄存器
	
	uint32_t RESERVED5[7];

	__IO uint32_t SWLOAD;                 //PWMPLUS  SWLOAD控制寄存器
	
	__IO uint32_t MSK_EN;                 //PWMPLUS屏蔽使能控制寄存器
	
	uint32_t RESERVED6[21];
	
	__IO uint32_t CNT_STATE;              //PWMPLUS当前计数值及状态寄存器  
	
	__IO uint32_t BRK_STATE;              //PWMPLUS刹车状态寄存器  
	
}PWMPLUS_TypeDef;


#define PWMPLUS_CTR_EN_POS             0   //PWMPLUS使能
#define PWMPLUS_CTR_EN_MSK             (0x01 << PWMPLUS_CTR_EN_POS)

#define PWMPLUS_CTR_DIR_POS            1   //PWMPLUS计数器计数方向配置   0  向上计数   1  向下计数
#define PWMPLUS_CTR_DIR_MSK            (0x01 << PWMPLUS_CTR_DIR_POS)

#define PWMPLUS_CTR_OPM_POS            2   //PWMPLUS单次计数模式         0  单次计数   1  重复计数
#define PWMPLUS_CTR_OPM_MSK            (0x01 << PWMPLUS_CTR_OPM_POS)

#define PWMPLUS_CTR_MODE_POS           3   //PWMPLUS输出模式             0  边沿对齐   1  中心对齐
#define PWMPLUS_CTR_MODE_MSK           (0x01 << PWMPLUS_CTR_MODE_POS)

#define PWMPLUS_CTR_RELOAD_POS         8   //PWMPLUS自动装载寄存器
#define PWMPLUS_CTR_RELOAD_MSK         (0xFF << PWMPLUS_CTR_RELOAD_POS)


#define PWMPLUS_OUTCTR_CH0_IDLE_POS    0   //PWMPLUS原始通道0空闲时输出电平
#define PWMPLUS_OUTCTR_CH0_IDLE_MSK    (0x01 << PWMPLUS_OUTCTR_CH0_IDLE_POS)

#define PWMPLUS_OUTCTR_CH0N_IDLE_POS   1   //PWMPLUS原始通道0N空闲时输出电平
#define PWMPLUS_OUTCTR_CH0N_IDLE_MSK   (0x01 << PWMPLUS_OUTCTR_CH0N_IDLE_POS)

#define PWMPLUS_OUTCTR_CH1_IDLE_POS    2   //PWMPLUS原始通道1空闲时输出电平
#define PWMPLUS_OUTCTR_CH1_IDLE_MSK    (0x01 << PWMPLUS_OUTCTR_CH1_IDLE_POS)

#define PWMPLUS_OUTCTR_CH1N_IDLE_POS   3   //PWMPLUS原始通道1N空闲时输出电平
#define PWMPLUS_OUTCTR_CH1N_IDLE_MSK   (0x01 << PWMPLUS_OUTCTR_CH1N_IDLE_POS)

#define PWMPLUS_OUTCTR_CH2_IDLE_POS    4   //PWMPLUS原始通道2空闲时输出电平
#define PWMPLUS_OUTCTR_CH2_IDLE_MSK    (0x01 << PWMPLUS_OUTCTR_CH2_IDLE_POS)

#define PWMPLUS_OUTCTR_CH2N_IDLE_POS   5   //PWMPLUS原始通道2N空闲时输出电平
#define PWMPLUS_OUTCTR_CH2N_IDLE_MSK   (0x01 << PWMPLUS_OUTCTR_CH2N_IDLE_POS)

#define PWMPLUS_OUTCTR_CH0_START_POS   8   //PWMPLUS原始通道0开始计数时输出电平
#define PWMPLUS_OUTCTR_CH0_START_MSK   (0x01 << PWMPLUS_OUTCTR_CH0_START_POS)

#define PWMPLUS_OUTCTR_CH1_START_POS   9   //PWMPLUS原始通道1开始计数时输出电平
#define PWMPLUS_OUTCTR_CH1_START_MSK   (0x01 << PWMPLUS_OUTCTR_CH1_START_POS)

#define PWMPLUS_OUTCTR_CH2_START_POS   10   //PWMPLUS原始通道2开始计数时输出电平
#define PWMPLUS_OUTCTR_CH2_START_MSK   (0x01 << PWMPLUS_OUTCTR_CH2_START_POS)

#define PWMPLUS_OUTCTR_CH0_INV_POS     16   //PWMPLUS原始通道0输出电平反转
#define PWMPLUS_OUTCTR_CH0_INV_MSK     (0x01 << PWMPLUS_OUTCTR_CH0_INV_POS)

#define PWMPLUS_OUTCTR_CH0N_INV_POS    17   //PWMPLUS原始通道0N输出电平反转
#define PWMPLUS_OUTCTR_CH0N_INV_MSK    (0x01 << PWMPLUS_OUTCTR_CH0N_INV_POS)

#define PWMPLUS_OUTCTR_CH1_INV_POS     18   //PWMPLUS原始通道1输出电平反转
#define PWMPLUS_OUTCTR_CH1_INV_MSK     (0x01 << PWMPLUS_OUTCTR_CH1_INV_POS)

#define PWMPLUS_OUTCTR_CH1N_INV_POS    19   //PWMPLUS原始通道1N输出电平反转
#define PWMPLUS_OUTCTR_CH1N_INV_MSK    (0x01 << PWMPLUS_OUTCTR_CH1N_INV_POS)

#define PWMPLUS_OUTCTR_CH2_INV_POS     20   //PWMPLUS原始通道2输出电平反转
#define PWMPLUS_OUTCTR_CH2_INV_MSK     (0x01 << PWMPLUS_OUTCTR_CH2_INV_POS)

#define PWMPLUS_OUTCTR_CH2N_INV_POS    21   //PWMPLUS原始通道2N输出电平反转
#define PWMPLUS_OUTCTR_CH2N_INV_MSK    (0x01 << PWMPLUS_OUTCTR_CH2N_INV_POS)

#define PWMPLUS_OUTCTR_CH0_OUTEN_POS   24   //PWMPLUS原始通道0输出使能
#define PWMPLUS_OUTCTR_CH0_OUTEN_MSK   (0x01 << PWMPLUS_OUTCTR_CH0_OUTEN_POS)

#define PWMPLUS_OUTCTR_CH0N_OUTEN_POS  25   //PWMPLUS原始通道0N输出使能
#define PWMPLUS_OUTCTR_CH0N_OUTEN_MSK  (0x01 << PWMPLUS_OUTCTR_CH0N_OUTEN_POS)

#define PWMPLUS_OUTCTR_CH1_OUTEN_POS   26   //PWMPLUS原始通道1输出使能
#define PWMPLUS_OUTCTR_CH1_OUTEN_MSK   (0x01 << PWMPLUS_OUTCTR_CH1_OUTEN_POS)

#define PWMPLUS_OUTCTR_CH1N_OUTEN_POS  27   //PWMPLUS原始通道1N输出使能
#define PWMPLUS_OUTCTR_CH1N_OUTEN_MSK  (0x01 << PWMPLUS_OUTCTR_CH1N_OUTEN_POS)

#define PWMPLUS_OUTCTR_CH2_OUTEN_POS   28   //PWMPLUS原始通道2输出使能
#define PWMPLUS_OUTCTR_CH2_OUTEN_MSK   (0x01 << PWMPLUS_OUTCTR_CH2_OUTEN_POS)

#define PWMPLUS_OUTCTR_CH2N_OUTEN_POS  29   //PWMPLUS原始通道2N输出使能
#define PWMPLUS_OUTCTR_CH2N_OUTEN_MSK  (0x01 << PWMPLUS_OUTCTR_CH2N_OUTEN_POS)


#define PWMPLUS_CLK_SRC_POS            0    //计数时钟选择   000 内部预分频时钟作为计数时钟    011  选择低16位定时器作为计数时钟   100  选择高16位定时器作为计数时钟
#define PWMPLUS_CLK_SRC_MSK            (0x07 << PWMPLUS_CLK_SRC_POS)

#define PWMPLUS_CLK_PREDIV_POS         8    //PWMPLUS计数时钟选择
#define PWMPLUS_CLK_PREDIV_MSK         (0xFF << PWMPLUS_CLK_PREDIV_POS)


#define PWMPLUS_BRK_CTR_CH0EN_POS      0    //PWMPLUS通道0和0N刹车控制使能
#define PWMPLUS_BRK_CTR_CH0EN_MSK      (0x03 << PWMPLUS_BRK_CTR_CH0EN_POS)

#define PWMPLUS_BRK_CTR_CH1EN_POS      3    //PWMPLUS通道1和1N刹车控制使能
#define PWMPLUS_BRK_CTR_CH1EN_MSK      (0x03 << PWMPLUS_BRK_CTR_CH1EN_POS)

#define PWMPLUS_BRK_CTR_CH2EN_POS      6    //PWMPLUS通道2和2N刹车控制使能
#define PWMPLUS_BRK_CTR_CH2EN_MSK      (0x03 << PWMPLUS_BRK_CTR_CH2EN_POS)

#define PWMPLUS_BRK_CTR_INLEV_POS      12    //PWMPLUS刹车输入有效电平选择
#define PWMPLUS_BRK_CTR_INLEV_MSK      (0x03 << PWMPLUS_BRK_CTR_INLEV_POS)

#define PWMPLUS_BRK_CTR_CH0_POL_POS    16    //PWMPLUS刹车时通道0输出电平选择
#define PWMPLUS_BRK_CTR_CH0_POL_MSK    (0x01 << PWMPLUS_BRK_CTR_CH0_POL_POS)

#define PWMPLUS_BRK_CTR_CH0N_POL_POS   17    //PWMPLUS刹车时通道0N输出电平选择
#define PWMPLUS_BRK_CTR_CH0N_POL_MSK   (0x01 << PWMPLUS_BRK_CTR_CH0N_POL_POS)

#define PWMPLUS_BRK_CTR_CH1_POL_POS    18    //PWMPLUS刹车时通道1输出电平选择
#define PWMPLUS_BRK_CTR_CH1_POL_MSK    (0x01 << PWMPLUS_BRK_CTR_CH1_POL_POS)

#define PWMPLUS_BRK_CTR_CH1N_POL_POS   19    //PWMPLUS刹车时通道1N输出电平选择
#define PWMPLUS_BRK_CTR_CH1N_POL_MSK   (0x01 << PWMPLUS_BRK_CTR_CH1N_POL_POS)

#define PWMPLUS_BRK_CTR_CH2_POL_POS    20    //PWMPLUS刹车时通道2输出电平选择
#define PWMPLUS_BRK_CTR_CH2_POL_MSK    (0x01 << PWMPLUS_BRK_CTR_CH2_POL_POS)

#define PWMPLUS_BRK_CTR_CH2N_POL_POS   21    //PWMPLUS刹车时通道2N输出电平选择
#define PWMPLUS_BRK_CTR_CH2N_POL_MSK   (0x01 << PWMPLUS_BRK_CTR_CH2N_POL_POS)

#define PWMPLUS_BRK_CTR_FILTER_POS     24    //PWMPLUS数字滤波
#define PWMPLUS_BRK_CTR_FILTER_MSK     (0x03 << PWMPLUS_BRK_CTR_FILTER_POS)


#define PWMPLUS_MSK_LEVEL_CH0_POS      0    //PWMPLUS通道0屏蔽电平选择
#define PWMPLUS_MSK_LEVEL_CH0_MSK      (0x01 << PWMPLUS_MSK_LEVEL_CH0_POS)

#define PWMPLUS_MSK_LEVEL_CH0N_POS     1    //PWMPLUS通道0N屏蔽电平选择
#define PWMPLUS_MSK_LEVEL_CH0N_MSK     (0x01 << PWMPLUS_MSK_LEVEL_CH0N_POS)

#define PWMPLUS_MSK_LEVEL_CH1_POS      2    //PWMPLUS通道1屏蔽电平选择
#define PWMPLUS_MSK_LEVEL_CH1_MSK      (0x01 << PWMPLUS_MSK_LEVEL_CH1_POS)

#define PWMPLUS_MSK_LEVEL_CH1N_POS     3    //PWMPLUS通道1N屏蔽电平选择
#define PWMPLUS_MSK_LEVEL_CH1N_MSK     (0x01 << PWMPLUS_MSK_LEVEL_CH1N_POS)

#define PWMPLUS_MSK_LEVEL_CH2_POS      4    //PWMPLUS通道2屏蔽电平选择
#define PWMPLUS_MSK_LEVEL_CH2_MSK      (0x01 << PWMPLUS_MSK_LEVEL_CH2_POS)

#define PWMPLUS_MSK_LEVEL_CH2N_POS     5    //PWMPLUS通道2N屏蔽电平选择
#define PWMPLUS_MSK_LEVEL_CH2N_MSK     (0x01 << PWMPLUS_MSK_LEVEL_CH2N_POS)


#define PWMPLUS_TRIG_CTR_SEL0_POS      0    //PWMPLUS输出的触发信号0功能选择
#define PWMPLUS_TRIG_CTR_SEL0_MSK      (0x0F << PWMPLUS_TRIG_CTR_SEL0_POS)

#define PWMPLUS_TRIG_CTR_SEL1_POS      4    //PWMPLUS输出的触发信号1功能选择
#define PWMPLUS_TRIG_CTR_SEL1_MSK      (0x0F << PWMPLUS_TRIG_CTR_SEL1_POS)

#define PWMPLUS_TRIG_CTR_SEL2_POS      8    //PWMPLUS输出的触发信号2功能选择
#define PWMPLUS_TRIG_CTR_SEL2_MSK      (0x0F << PWMPLUS_TRIG_CTR_SEL2_POS)

#define PWMPLUS_TRIG_CTR_SEL3_POS      12   //PWMPLUS输出的触发信号3功能选择
#define PWMPLUS_TRIG_CTR_SEL3_MSK      (0x0F << PWMPLUS_TRIG_CTR_SEL3_POS)


#define PWMPLUS_IE_CH0_UP_COMP_POS      0    //PWMPLUS通道0向上计数到达比较点中断使能
#define PWMPLUS_IE_CH0_UP_COMP_MSK      (0x01 << PWMPLUS_IE_CH0_UP_COMP_POS)

#define PWMPLUS_IE_CH1_UP_COMP_POS      1    //PWMPLUS通道1向上计数到达比较点中断使能
#define PWMPLUS_IE_CH1_UP_COMP_MSK      (0x01 << PWMPLUS_IE_CH1_UP_COMP_POS)

#define PWMPLUS_IE_CH2_UP_COMP_POS      2    //PWMPLUS通道2向上计数到达比较点中断使能
#define PWMPLUS_IE_CH2_UP_COMP_MSK      (0x01 << PWMPLUS_IE_CH2_UP_COMP_POS)

#define PWMPLUS_IE_UP_OVF_POS          3    //PWMPLUS向上计数周期溢出中断使能
#define PWMPLUS_IE_UP_OVF_MSK          (0x01 << PWMPLUS_IE_UP_OVF_POS)

#define PWMPLUS_IE_UP_TRIG_POS         4    //PWMPLUS向上计数到达触发点中断使能
#define PWMPLUS_IE_UP_TRIG_MSK         (0x01 << PWMPLUS_IE_UP_TRIG_POS)

#define PWMPLUS_IE_CH0_DOWN_COMP_POS    8    //PWMPLUS通道0向下计数到达比较点中断使能
#define PWMPLUS_IE_CH0_DOWN_COMP_MSK    (0x01 << PWMPLUS_IE_CH0_DOWN_COMP_POS)

#define PWMPLUS_IE_CH1_DOWN_COMP_POS    9    //PWMPLUS通道1向下计数到达比较点中断使能
#define PWMPLUS_IE_CH1_DOWN_COMP_MSK    (0x01 << PWMPLUS_IE_CH1_DOWN_COMP_POS)

#define PWMPLUS_IE_CH2_DOWN_COMP_POS    10   //PWMPLUS通道2向下计数到达比较点中断使能
#define PWMPLUS_IE_CH2_DOWN_COMP_MSK    (0x01 << PWMPLUS_IE_CH2_DOWN_COMP_POS)

#define PWMPLUS_IE_DOWN_OVF_POS        11   //PWMPLUS向下计数周期溢出中断使能
#define PWMPLUS_IE_DOWN_OVF_MSK        (0x01 << PWMPLUS_IE_DOWN_OVF_POS)

#define PWMPLUS_IE_DOWN_TRIG_POS       12   //PWMPLUS向下计数到达触发点中断使能
#define PWMPLUS_IE_DOWN_TRIG_MSK       (0x01 << PWMPLUS_IE_DOWN_TRIG_POS)

#define PWMPLUS_IE_BRK0_POS            16   //PWMPLUS刹车0中断使能
#define PWMPLUS_IE_BRK0_MSK            (0x01 << PWMPLUS_IE_BRK0_POS)

#define PWMPLUS_IE_BRK1_POS            17   //PWMPLUS刹车1中断使能
#define PWMPLUS_IE_BRK1_MSK            (0x01 << PWMPLUS_IE_BRK1_POS)

#define PWMPLUS_IE_RELOAD_POS          19   //PWMPLUS自动装载中断使能
#define PWMPLUS_IE_RELOAD_MSK          (0x01 << PWMPLUS_IE_RELOAD_POS)


#define PWMPLUS_IF_CH0_UP_COMP_POS      0    //PWMPLUS通道0向上计数到达比较点中断状态
#define PWMPLUS_IF_CH0_UP_COMP_MSK      (0x01 << PWMPLUS_IF_CH0_UP_COMP_POS)

#define PWMPLUS_IF_CH1_UP_COMP_POS      1    //PWMPLUS通道1向上计数到达比较点中断状态
#define PWMPLUS_IF_CH1_UP_COMP_MSK      (0x01 << PWMPLUS_IF_CH1_UP_COMP_POS)

#define PWMPLUS_IF_CH2_UP_COMP_POS      2    //PWMPLUS通道2向上计数到达比较点中断状态
#define PWMPLUS_IF_CH2_UP_COMP_MSK      (0x01 << PWMPLUS_IF_CH2_UP_COMP_POS)

#define PWMPLUS_IF_UP_OVF_POS           3    //PWMPLUS向上计数周期溢出中断状态
#define PWMPLUS_IF_UP_OVF_MSK           (0x01 << PWMPLUS_IF_UP_OVF_POS)

#define PWMPLUS_IF_UP_TRIG_POS          4    //PWMPLUS向上计数到达触发点中断状态
#define PWMPLUS_IF_UP_TRIG_MSK          (0x01 << PWMPLUS_IF_UP_TRIG_POS)

#define PWMPLUS_IF_CH0_DOWN_COMP_POS    8    //PWMPLUS通道0向下计数到达比较点中断状态
#define PWMPLUS_IF_CH0_DOWN_COMP_MSK    (0x01 << PWMPLUS_IF_CH0_DOWN_COMP_POS)

#define PWMPLUS_IF_CH1_DOWN_COMP_POS    9    //PWMPLUS通道1向下计数到达比较点中断状态
#define PWMPLUS_IF_CH1_DOWN_COMP_MSK    (0x01 << PWMPLUS_IF_CH1_DOWN_COMP_POS)

#define PWMPLUS_IF_CH2_DOWN_COMP_POS    10   //PWMPLUS通道2向下计数到达比较点中断状态
#define PWMPLUS_IF_CH2_DOWN_COMP_MSK    (0x01 << PWMPLUS_IF_CH2_DOWN_COMP_POS)

#define PWMPLUS_IF_DOWN_OVF_POS         11   //PWMPLUS向下计数周期溢出中断状态
#define PWMPLUS_IF_DOWN_OVF_MSK         (0x01 << PWMPLUS_IF_DOWN_OVF_POS)

#define PWMPLUS_IF_DOWN_TRIG_POS        12   //PWMPLUS向下计数到达触发点中断状态
#define PWMPLUS_IF_DOWN_TRIG_MSK        (0x01 << PWMPLUS_IF_DOWN_TRIG_POS)

#define PWMPLUS_IF_BRK0_POS             16   //PWMPLUS刹车0中断状态
#define PWMPLUS_IF_BRK0_MSK             (0x01 << PWMPLUS_IF_BRK0_POS)

#define PWMPLUS_IF_BRK1_POS             17   //PWMPLUS刹车1中断状态
#define PWMPLUS_IF_BRK1_MSK             (0x01 << PWMPLUS_IF_BRK1_POS)

#define PWMPLUS_IF_RELOAD_POS           19   //PWMPLUS自动装载中断状态
#define PWMPLUS_IF_RELOAD_MSK           (0x01 << PWMPLUS_IF_RELOAD_POS)


#define PWMPLUS_SWLOAD_POS              0    //PWMPLUS软件LOAD
#define PWMPLUS_SWLOAD_MSK              (0x01 << PWMPLUS_SWLOAD_POS)


#define PWMPLUS_MSK_EN_CH0_POS         0    //PWMPLUS通道0屏蔽使能
#define PWMPLUS_MSK_EN_CH0_MSK         (0x01 << PWMPLUS_MSK_EN_CH0_POS)

#define PWMPLUS_MSK_EN_CH0N_POS        1    //PWMPLUS通道0N屏蔽使能
#define PWMPLUS_MSK_EN_CH0N_MSK        (0x01 << PWMPLUS_MSK_EN_CH0N_POS)

#define PWMPLUS_MSK_EN_CH1_POS         2    //PWMPLUS通道1屏蔽使能
#define PWMPLUS_MSK_EN_CH1_MSK         (0x01 << PWMPLUS_MSK_EN_CH1_POS)

#define PWMPLUS_MSK_EN_CH1N_POS        3    //PWMPLUS通道1N屏蔽使能
#define PWMPLUS_MSK_EN_CH1N_MSK        (0x01 << PWMPLUS_MSK_EN_CH1N_POS)

#define PWMPLUS_MSK_EN_CH2_POS         4    //PWMPLUS通道2屏蔽使能
#define PWMPLUS_MSK_EN_CH2_MSK         (0x01 << PWMPLUS_MSK_EN_CH2_POS)

#define PWMPLUS_MSK_EN_CH2N_POS        5    //PWMPLUS通道2N屏蔽使能
#define PWMPLUS_MSK_EN_CH2N_MSK        (0x01 << PWMPLUS_MSK_EN_CH2N_POS)


#define PWMPLUS_CNT_STATE_CNT_POS      0    //PWMPLUS当前计数值
#define PWMPLUS_CNT_STATE_CNT_MSK      (0xFFFF << PWMPLUS_CNT_STATE_CNT_POS)

#define PWMPLUS_CNT_STATE_DIR_POS      16    //PWMPLUS当前计数方向
#define PWMPLUS_CNT_STATE_DIR_MSK      (0x01 << PWMPLUS_CNT_STATE_DIR_POS)

#define PWMPLUS_CNT_STATE_STATE_POS    17    //PWMPLUS计数器工作状态
#define PWMPLUS_CNT_STATE_STATE_MSK    (0x01 << PWMPLUS_CNT_STATE_STATE_POS)

#define PWMPLUS_BRK0_STATE_POS         0     //PWMPLUS刹车0输入信号当前状态
#define PWMPLUS_BRK0_STATE_MSK         (0x01 << PWMPLUS_BRK0_STATE_POS)

#define PWMPLUS_BRK1_STATE_POS         1     //PWMPLUS刹车1输入信号当前状态
#define PWMPLUS_BRK1_STATE_MSK         (0x01 << PWMPLUS_BRK1_STATE_POS)



typedef struct {
	__IO uint32_t CTRL;       //控制寄存器

	__IO uint32_t WDATA;      //写数据寄存器

	__IO uint32_t RDATA;      //读数据寄存器

	uint32_t RESERVED1[1];
	
	__IO uint32_t IE;         //中断使能

	__IO uint32_t IF;         //中断状态
	
} SPI_TypeDef;


#define SPI_CTRL_CLKDIV_POS     0           //Clock Divider, SPI工作时钟 = SYS_Freq/pow(2, CLKDIV+2)
#define SPI_CTRL_CLKDIV_MSK     (0x07 << SPI_CTRL_CLKDIV_POS)

#define SPI_CTRL_EN_POS         3           //SPI系统使能
#define SPI_CTRL_EN_MSK         (0x01 << SPI_CTRL_EN_POS)

#define SPI_CTRL_CPHA_POS		4		    //0 在SCLK的第一个跳变沿采样数据	1 在SCLK的第二个跳变沿采样数据
#define SPI_CTRL_CPHA_MSK		(0x01 << SPI_CTRL_CPHA_POS)

#define SPI_CTRL_CPOL_POS		5		    //0 空闲状态下SCLK为低电平		  1 空闲状态下SCLK为高电平
#define SPI_CTRL_CPOL_MSK		(0x01 << SPI_CTRL_CPOL_POS)

#define SPI_CTRL_MSTR_POS		6		    //Master, 1 主模式	0 从模式
#define SPI_CTRL_MSTR_MSK		(0x01 << SPI_CTRL_MSTR_POS)

#define SPI_CTRL_LSBF_POS		7		    //数据传输顺序选择  0 MSB Fisrt  1 LSB Fisrt
#define SPI_CTRL_LSBF_MSK		(0x01 << SPI_CTRL_LSBF_POS)

#define SPI_CTRL_DATAHOLD_POS   8           //从模式下CPHA为1时，数据保持时间配置寄存器
#define SPI_CTRL_DATAHOLD_MSK   (0x0F << SPI_CTRL_DATAHOLD_POS)

#define SPI_CTRL_MST_SSN_POS    12          //主机模式下，SSN输出   默认输出高电平
#define SPI_CTRL_MST_SSN_MSK    (0x01 << SPI_CTRL_MST_SSN_POS)

#define SPI_SPIF_IE_POS         1           //传输结束中断使能
#define SPI_SPIF_IE_MSK         (0x01 << SPI_SPIF_IE_POS)

#define SPI_SPIF_IF_POS         1           //传输结束中断状态
#define SPI_SPIF_IF_MSK         (0x01 << SPI_SPIF_IF_POS)


typedef struct {
	union {
		__IO uint32_t RDR;    //接收数据寄存器
		
		__IO uint32_t SDR;    //发送数据寄存器
		
		__IO uint32_t DLL;    //波特率低八位配置寄存器
	};
	union {
		__IO uint32_t DHL;    //波特率高八位配置寄存器
		
		__IO uint32_t IE;     //中断使能寄存器   
	};
	
	uint32_t RESERVED1[1];
	
	__IO uint32_t CTRL;       //控制寄存器
	
	__IO uint32_t MCR;        //LOOPBACK使能 
	
	__IO uint32_t LSR;       //状态寄存器
	
} UART_TypeDef;



#define UART_IE_RXIEN_POS        0      //接收数据有效中断使能
#define UART_IE_RXIEN_MSK        (0x01 << UART_IE_RXIEN_POS)

#define UART_IE_TXE_POS          1      //发送数据寄存器空中断使能
#define UART_IE_TXE_MSK          (0x01 << UART_IE_TXE_POS)


#define UART_CTRL_DATALEN_POS    0      //数据位宽  00 5bit   01  6bit  10  7bit  11 8bit
#define UART_CTRL_DATALEN_MSK    (0x03 << UART_CTRL_DATALEN_POS)

#define UART_CTRL_STOP2BIT_POS   2      //停止位   0  1stop    1   2stop
#define UART_CTRL_STOP2BIT_MSK   (0x01 << UART_CTRL_STOP2BIT_POS)

#define UART_CTRL_PARITY_POS	 3		//000 无校验    001 奇校验   011 偶校验   101 固定为1    111 固定为0
#define UART_CTRL_PARITY_MSK	 (0x07 << UART_CTRL_PARITY_POS)

#define UART_CTRL_BRKEN_POS		 6		//Break控制使能
#define UART_CTRL_BRKEN_MSK		 (0x01 << UART_CTRL_BRKEN_POS)

#define UART_CTRL_BAUDEN_POS	 7		//波特率设置开启位
#define UART_CTRL_BAUDEN_MSK	 (0x01 << UART_CTRL_BAUDEN_POS)

#define UART_MCR_LOOPBACKEN_POS	 4      //回环模式使能位
#define UART_MCR_LOOPBACKEN_MSK	 (0x01 << UART_MCR_LOOPBACKEN_POS)
 
#define UART_LSR_REC_POS        0       //数据接收状态
#define UART_LSR_REC_MSK        (0x01 << UART_LSR_REC_POS)

#define UART_LSR_OVF_POS        1       //数据溢出状态
#define UART_LSR_OVF_MSK        (0x01 << UART_LSR_OVF_POS)

#define UART_LSR_PARITY_POS     2       //奇偶校验状态
#define UART_LSR_PARITY_MSK     (0x01 << UART_LSR_PARITY_POS)

#define UART_LSR_STOP_POS       3       //STOP状态
#define UART_LSR_STOP_MSK       (0x01 << UART_LSR_STOP_POS)

#define UART_LSR_BREAK_POS      4       //BREAK状态
#define UART_LSR_BREAK_MSK      (0x01 << UART_LSR_BREAK_POS)

#define UART_LSR_THRE_POS       5       //发送数据寄存器空状态
#define UART_LSR_THRE_MSK       (0x01 << UART_LSR_THRE_POS)

#define UART_LSR_TEMT_POS       6       //无发送数据状态
#define UART_LSR_TEMT_MSK       (0x01 << UART_LSR_TEMT_POS)


typedef struct {
	__IO uint32_t CLKDIV;       //分频寄存器 
	
	__IO uint32_t CTRL;         //控制寄存器  
	
	__IO uint32_t TXR;          //发送寄存器
	
	__IO uint32_t RXR;          //接收寄存器
	
	__IO uint32_t CR;           //命令寄存器
	
	__IO uint32_t SR;           //状态寄存器
	
} IIC_TypeDef;

#define IIC_CTRL_EN_POS    0    //模块使能
#define IIC_CTRL_EN_MSK    (0x01 << IIC_CTRL_EN_POS)

#define IIC_CTRL_IE_POS    1    //中断使能
#define IIC_CTRL_IE_MSK    (0x01 << IIC_CTRL_IE_POS)


#define IIC_CR_IF_POS      0    //清中断
#define IIC_CR_IF_MSK      (0x01 << IIC_CR_IF_POS)

#define IIC_CR_ACK_POS     1    //接收模式下，向总线反馈ACK
#define IIC_CR_ACK_MSK     (0x01 << IIC_CR_ACK_POS)

#define IIC_CR_WR_POS      2    //向从机写数据
#define IIC_CR_WR_MSK      (0x01 << IIC_CR_WR_POS)

#define IIC_CR_RD_POS      3    //从从机读数据
#define IIC_CR_RD_MSK      (0x01 << IIC_CR_RD_POS)

#define IIC_CR_STOP_POS    4    //产生STOP信号
#define IIC_CR_STOP_MSK    (0x01 << IIC_CR_STOP_POS)

#define IIC_CR_START_POS   5    //产生START信号
#define IIC_CR_START_MSK   (0x01 << IIC_CR_START_POS)


#define IIC_SR_IF_POS      0    //清中断
#define IIC_SR_IF_MSK      (0x01 << IIC_SR_IF_POS)

#define IIC_SR_TIP_POS     1    //传输进行中
#define IIC_SR_TIP_MSK     (0x01 << IIC_SR_TIP_POS)

#define IIC_SR_ARB_POS     2    //仲裁丢失
#define IIC_SR_ARB_MSK     (0x01 << IIC_SR_ARB_POS)

#define IIC_SR_BUSY_POS    3    //总线忙
#define IIC_SR_BUSY_MSK    (0x01 << IIC_SR_BUSY_POS)

#define IIC_SR_RACK_POS    4    //接收到从机发送过来的ACK
#define IIC_SR_RACK_MSK    (0x01 << IIC_SR_RACK_POS)


typedef struct {
	__IO uint32_t LPOW_MD;      //低功耗模式选择
	
	__IO uint32_t LPMD_WKEN;    //低功耗唤醒源使能
	
	__IO uint32_t LPMD_WKST;    //低功耗唤醒源状态
	
	uint32_t RESERVED1[5];
	
	__IO uint32_t TRIM_POW;     //POWER TRIM 
	
	__IO uint32_t TRIM_RC;      //RCHF RCLF TRIM
	
	__IO uint32_t TRIM_LOCK;    //TRIM LOCK
	
	uint32_t RESERVED2[1];
	
} PMU_TypeDef;


#define PMU_LPOW_MD_STANDBY_POS         0     //STANDBY模式
#define PMU_LPOW_MD_STANDBY_MSK         (0x01 << PMU_LPOW_MD_STANDBY_POS)

#define PMU_LPOW_MD_SLEEP_POS           1     //SLEEP模式
#define PMU_LPOW_MD_SLEEP_MSK           (0x01 << PMU_LPOW_MD_SLEEP_POS)

#define PMU_LPOW_MD_STOP_POS            3     //STOP模式
#define PMU_LPOW_MD_STOP_MSK            (0x01 << PMU_LPOW_MD_STOP_POS)


#define PMU_LPMD_WKEN_IO_POS            2     //低功耗模式下  IO信号唤醒使能
#define PMU_LPMD_WKEN_IO_MSK            (0x01 << PMU_LPMD_WKEN_IO_POS)


#define PMU_LPMD_WKST_IO_POS            2     //低功耗模式下  IO信号唤醒标志
#define PMU_LPMD_WKST_IO_MSK            (0x01 << PMU_LPMD_WKST_IO_POS)


#define PMU_TRIM_LOCK_POS		        0	//TRIM_LOCK       写入0x55锁定TRIM寄存器
#define PMU_TRIM_LOCK_MSK		        (0xFF << PMU_TRIM_LOCK_POS)


typedef struct {
	__IO uint32_t CFG;          //CACHE配置寄存器      
	
	__IO uint32_t PF_CTRL;      //预取控制寄存器   	
} CACHE_TypeDef;


#define CACHE_CFG_RESET_POS       0              //CACHE复位信号
#define CACHE_CFG_RESET_MSK       (0x01 << CACHE_CFG_RESET_POS)

#define CACHE_CFG_IDLE_POS        1              //CACHE空闲状态
#define CACHE_CFG_IDLE_MSK        (0x01 << CACHE_CFG_IDLE_POS)


#define CACHE_PF_CTRL_START_POS   0              //预取启动位
#define CACHE_PF_CTRL_START_MSK   (0x01 << CACHE_PF_CTRL_START_POS)

#define CACHE_PF_CTRL_UNLOCK_POS  1              //预取解锁配置寄存器
#define CACHE_PF_CTRL_UNLOCK_MSK  (0x01 << CACHE_PF_CTRL_UNLOCK_POS)

#define CACHE_PF_CTRL_ADDR_POS    4             //预取空间的基地址
#define CACHE_PF_CTRL_ADDR_MSK    (0x7FF << CACHE_PF_CTRL_ADDR_POS)


typedef struct {
	__IO uint32_t CFG;               //ADC配置寄存器      
	
	__IO uint32_t START;             //ADC启动寄存器  

	__IO uint32_t IE;                //ADC中断使能寄存器  
	
	__IO uint32_t IF;                //ADC中断状态寄存器  
	
 	struct {
		__IO uint32_t STAT;          //通道状态寄存器
		
		__IO uint32_t DATA;          //通道数据寄存器
		
	    uint32_t RESERVED1[2];
	} CH[9];
	
	__IO uint32_t FIFO_STAT;         //FIFO状态寄存器
	
	__IO uint32_t FIFO_DATA;         //FIFO数据寄存器
	
	uint32_t RESERVED2[2];
	
	__IO uint32_t EXTTRIG_SEL;       //外部触发信号选择
	
	uint32_t RESERVED3[11];
	
	__IO uint32_t CTRL;              //ADC控制寄存器 
	
	uint32_t RESERVED4[3];
	
	__IO uint32_t CALIB_OFFSET;     //ADC校准OFFSET寄存器
	
	__IO uint32_t CALIB_KD;         //ADC校准KD寄存器
	
} ADC_TypeDef;


#define ADC_CFG_CH0_POS			    0		//通道0选中
#define ADC_CFG_CH0_MSK			    (0x01 << ADC_CFG_CH0_POS)

#define ADC_CFG_CH1_POS			    1		//通道1选中
#define ADC_CFG_CH1_MSK			    (0x01 << ADC_CFG_CH1_POS)

#define ADC_CFG_CH2_POS			    2		//通道2选中
#define ADC_CFG_CH2_MSK			    (0x01 << ADC_CFG_CH2_POS)

#define ADC_CFG_CH3_POS			    3		//通道3选中
#define ADC_CFG_CH3_MSK			    (0x01 << ADC_CFG_CH3_POS)

#define ADC_CFG_CH4_POS			    4		//通道4选中
#define ADC_CFG_CH4_MSK			    (0x01 << ADC_CFG_CH4_POS)

#define ADC_CFG_CH5_POS			    5		//通道5选中
#define ADC_CFG_CH5_MSK			    (0x01 << ADC_CFG_CH5_POS)

#define ADC_CFG_CH6_POS			    6		//通道6选中
#define ADC_CFG_CH6_MSK			    (0x01 << ADC_CFG_CH6_POS)
 
#define ADC_CFG_CH7_POS			    7		//通道7选中
#define ADC_CFG_CH7_MSK			    (0x01 << ADC_CFG_CH7_POS)

#define ADC_CFG_CH8_POS			    8		//通道8选中
#define ADC_CFG_CH8_MSK			    (0x01 << ADC_CFG_CH8_POS)

#define ADC_CFG_AVG_POS		        9		//0 1次采样	  1 2次采样取平均值	  2 4次采样取平均值	  3 8次采样取平均值	
#define ADC_CFG_AVG_MSK		        (0x03 << ADC_CFG_AVG_POS)

#define ADC_CFG_CONT_POS		    11		//Continuous conversion，只在软件启动模式下有效，0 单次转换，转换完成后START位自动清除停止转换
#define ADC_CFG_CONT_MSK		    (0x01 << ADC_CFG_CONT_POS)							//   1 连续转换，启动后一直采样、转换，直到软件清除START位

#define ADC_CFG_SMPL_SETUP_POS      12      //ADC外部采样时钟方式下采样建立时间   0:1  1:2  2:4  3:8  4:16  5:32  6:64  7:128
#define ADC_CFG_SMPL_SETUP_MSK      (0x07 << ADC_CFG_SPL_SETUP_POS)

#define ADC_CFG_MEM_MODE_POS        15      //ADC数据存储方式选择   0:FIFO模式   1:通道模式
#define ADC_CFG_MEM_MODE_MSK        (0x01 << ADC_CFG_MEM_MODE_POS)

#define ADC_CFG_SMPL_CLK_POS        16      //ADC采样模式选择    0:内部采样时钟  1:外部采样时钟
#define ADC_CFG_SMPL_CLK_MSK        (0x01 << ADC_CFG_SMPL_CLK_POS)

#define ADC_CFG_IN_SMPL_WIN_POS     17      //ADC内部采样时钟方式采样窗口设置   0:1Tclk   1:3Tclk   2:5Tclk   3:7Tclk  4:9Tclk  5:11Tclk  6:13Tclk  7:15Tclk
#define ADC_CFG_IN_SMPL_WIN_MSK     (0x07 << ADC_CFG_IN_SMPL_WIN_POS)

#define ADC_CFG_EN_POS              20      //ADC使能控制位   0:不使能  1:使能
#define ADC_CFG_EN_MSK              (0x01 << ADC_CFG_EN_POS)

#define ADC_CFG_EN_AVDDSNS_POS      21      //ADC VDD检测使能常开控制位   0:只有通道8有效时,VDD检测使能才打开   1:VDD检测使能常开
#define ADC_CFG_EN_AVDDSNS_MSK      (0x01 << ADC_CFG_EN_AVDDSNS_POS)

#define ADC_CFG_TRIG_POS            22      //ADC触发源信号选择    0  CPU触发   1  选择外部信号触发采样
#define ADC_CFG_TRIG_MSK            (0x01 << ADC_CFG_TRIG_POS)


#define ADC_START_START_POS         0      //ADC启动信号   0:不启动  1:启动
#define ADC_START_START_MSK         (0x01 << ADC_START_START_POS) 

#define ADC_START_BUSY_POS          1      //ADC忙状态   0:空闲  1:忙
#define ADC_START_BUSY_MSK          (0x01 << ADC_START_BUSY_POS) 

#define ADC_START_SOFT_RESET_POS    2      //ADC软复位使能位
#define ADC_START_SOFT_RESET_MSK    (0x01 << ADC_START_SOFT_RESET_POS) 

#define ADC_START_FIFOCLR_POS       3      //ADC FIFO清除使能
#define ADC_START_FIFOCLR_MSK       (0x01 << ADC_START_FIFOCLR_POS) 


#define ADC_IE_CH0EOC_POS			0		//CH0转换完成中断使能
#define ADC_IE_CH0EOC_MSK			(0x01 << ADC_IE_CH0EOC_POS)

#define ADC_IE_CH1EOC_POS			1		//CH1转换完成中断使能
#define ADC_IE_CH1EOC_MSK			(0x01 << ADC_IE_CH1EOC_POS)

#define ADC_IE_CH2EOC_POS			2		//CH2转换完成中断使能
#define ADC_IE_CH2EOC_MSK			(0x01 << ADC_IE_CH2EOC_POS)

#define ADC_IE_CH3EOC_POS			3		//CH3转换完成中断使能
#define ADC_IE_CH3EOC_MSK			(0x01 << ADC_IE_CH3EOC_POS)

#define ADC_IE_CH4EOC_POS			4		//CH4转换完成中断使能
#define ADC_IE_CH4EOC_MSK			(0x01 << ADC_IE_CH4EOC_POS)

#define ADC_IE_CH5EOC_POS			5		//CH5转换完成中断使能
#define ADC_IE_CH5EOC_MSK			(0x01 << ADC_IE_CH5EOC_POS)

#define ADC_IE_CH6EOC_POS			6		//CH6转换完成中断使能
#define ADC_IE_CH6EOC_MSK			(0x01 << ADC_IE_CH6EOC_POS)

#define ADC_IE_CH7EOC_POS			7		//CH7转换完成中断使能
#define ADC_IE_CH7EOC_MSK			(0x01 << ADC_IE_CH7EOC_POS)

#define ADC_IE_CH8EOC_POS			8		//CH8转换完成中断使能
#define ADC_IE_CH8EOC_MSK			(0x01 << ADC_IE_CH8EOC_POS)

#define ADC_IE_FIFO_FULL_POS	    9       //FIFO满中断使能
#define ADC_IE_FIFO_FULL_MSK	    (0x01 << ADC_IE_FIFO_FULL_POS)

#define ADC_IE_FIFO_HFULL_POS	    10       //FIFO半满中断使能
#define ADC_IE_FIFO_HFULL_MSK	    (0x01 << ADC_IE_FIFO_HFULL_POS)

#define ADC_IE_FIFO_OVF_POS			11		//FIFO溢出中断使能
#define ADC_IE_FIFO_OVF_MSK			(0x01 << ADC_IE_FIFO_OVF_POS)


#define ADC_IF_CH0EOC_POS			0		//CH0转换完成中断状态
#define ADC_IF_CH0EOC_MSK			(0x01 << ADC_IF_CH0EOC_POS)

#define ADC_IF_CH1EOC_POS			1		//CH1转换完成中断状态
#define ADC_IF_CH1EOC_MSK			(0x01 << ADC_IF_CH1EOC_POS)

#define ADC_IF_CH2EOC_POS			2		//CH2转换完成中断状态
#define ADC_IF_CH2EOC_MSK			(0x01 << ADC_IF_CH2EOC_POS)

#define ADC_IF_CH3EOC_POS			3		//CH3转换完成中断状态
#define ADC_IF_CH3EOC_MSK			(0x01 << ADC_IF_CH3EOC_POS)

#define ADC_IF_CH4EOC_POS			4		//CH4转换完成中断状态
#define ADC_IF_CH4EOC_MSK			(0x01 << ADC_IF_CH4EOC_POS)

#define ADC_IF_CH5EOC_POS			5		//CH5转换完成中断状态
#define ADC_IF_CH5EOC_MSK			(0x01 << ADC_IF_CH5EOC_POS)

#define ADC_IF_CH6EOC_POS			6		//CH6转换完成中断状态
#define ADC_IF_CH6EOC_MSK			(0x01 << ADC_IF_CH6EOC_POS)

#define ADC_IF_CH7EOC_POS			7		//CH7转换完成中断状态
#define ADC_IF_CH7EOC_MSK			(0x01 << ADC_IF_CH7EOC_POS)

#define ADC_IF_CH8EOC_POS			8		//CH8转换完成中断状态
#define ADC_IF_CH8EOC_MSK			(0x01 << ADC_IF_CH8EOC_POS)

#define ADC_IF_FIFO_FULL_POS	    9       //FIFO满中断状态
#define ADC_IF_FIFO_FULL_MSK	    (0x01 << ADC_IF_FIFO_FULL_POS)

#define ADC_IF_FIFO_HFULL_POS	    10       //FIFO半满中断状态
#define ADC_IF_FIFO_HFULL_MSK	    (0x01 << ADC_IF_FIFO_HFULL_POS)

#define ADC_IF_FIFO_OVF_POS			11		//FIFO溢出中断状态
#define ADC_IF_FIFO_OVF_MSK			(0x01 << ADC_IF_FIFO_OVF_POS)


#define ADC_STAT_EOC_POS			0		//向ADC_IF相应通道对应位写1可清零
#define ADC_STAT_EOC_MSK			(0x01 << ADC_STAT_EOC_POS)
#define ADC_STAT_OVF_POS			1		//读数据寄存器清除
#define ADC_STAT_OVF_MSK			(0x01 << ADC_STAT_OVF_POS)


#define ADC_DATA_DATA_POS           0       //ADC通道数据寄存器
#define ADC_DATA_DATA_MSK           (0xFFF << ADC_DATA_DATA_POS)

#define ADC_DATA_NUM_POS            12      //ADC数据对应的通道编号
#define ADC_DATA_NUM_MSK            (0x0F << ADC_DATA_NUM_POS)


#define ADC_FIFO_STAT_FULL_POS      0       //ADC数据FIFO满标志位
#define ADC_FIFO_STAT_FULL_MSK      (0x01 << ADC_FIFO_STAT_FULL_POS)

#define ADC_FIFO_STAT_HFULL_POS     1       //ADC数据FIFO半满标志位
#define ADC_FIFO_STAT_HFULL_MSK     (0x01 << ADC_FIFO_STAT_HFULL_POS)

#define ADC_FIFO_STAT_EMPTY_POS     2       //ADC数据FIFO空标志位
#define ADC_FIFO_STAT_EMPTY_MSK     (0x01 << ADC_FIFO_STAT_EMPTY_POS)

#define ADC_FIFO_STAT_OVF_POS       3       //ADC数据FIFO溢出标志位
#define ADC_FIFO_STAT_OVF_MSK       (0x01 << ADC_FIFO_STAT_OVF_POS)

#define ADC_FIFO_STAT_LEVEL_POS     4       //ADC数据FIFO水线设置
#define ADC_FIFO_STAT_LEVEL_MSK     (0x0F << ADC_FIFO_STAT_LEVEL_POS)


#define ADC_FIFO_DATA_POS           0       //ADC数据FIFO寄存器
#define ADC_FIFO_DATA_MSK           (0xFFF << ADC_FIFO_DATA_POS)

#define ADC_FIFO_NUM_POS            12      //ADC数据FIFO对应的通道编号
#define ADC_FIFO_NUM_MSK            (0x0F << ADC_FIFO_NUM_POS)


#define ADC_EXTTRIG_SEL_TRIG0_POS   0       //选择PWMPLUS_TRIG0触发ADC采样
#define ADC_EXTTRIG_SEL_TRIG0_MSK   (0x01 << ADC_EXTTRIG_SEL_TRIG0_POS)

#define ADC_EXTTRIG_SEL_TRIG1_POS   1       //选择PWMPLUS_TRIG1触发ADC采样
#define ADC_EXTTRIG_SEL_TRIG1_MSK   (0x01 << ADC_EXTTRIG_SEL_TRIG1_POS)

#define ADC_EXTTRIG_SEL_TRIG2_POS   2       //选择PWMPLUS_TRIG2触发ADC采样 
#define ADC_EXTTRIG_SEL_TRIG2_MSK   (0x01 << ADC_EXTTRIG_SEL_TRIG2_POS)

#define ADC_EXTTRIG_SEL_TRIG3_POS   3       //选择PWMPLUS_TRIG3触发ADC采样
#define ADC_EXTTRIG_SEL_TRIG3_MSK   (0x01 << ADC_EXTTRIG_SEL_TRIG3_POS)

#define ADC_EXTTRIG_SEL_LOW_POS     4       //选择TIMERPLUS_LOW触发ADC采样
#define ADC_EXTTRIG_SEL_LOW_MSK     (0x01 << ADC_EXTTRIG_SEL_LOW_POS)

#define ADC_EXTTRIG_SEL_HIGH_POS    5       //选择TIMERPLUS_HIGH触发ADC采样
#define ADC_EXTTRIG_SEL_HIGH_MSK    (0x01 << ADC_EXTTRIG_SEL_HIGH_POS)


#define ADC_CTRL_VREF_POS           0       //ADC内部VREF选择   0:内部  1:外部
#define ADC_CTRL_VREF_MSK           (0x01 << ADC_CTRL_VREF_POS)


#define ADC_CALIB_OFFSET_POS        0       //ADC数据校准OFFSET
#define ADC_CALIB_OFFSET_MSK        (0xFF << ADC_CALIB_OFFSET_POS)

#define ADC_CALIB_OFFSET_VALID_POS  16      //ADC数据校准OFFSET数据是否有效
#define ADC_CALIB_OFFSET_VALID_MSK  (0xFF << ADC_CALIB_OFFSET_VALID_POS)


#define ADC_CALIB_KD_POS            0       //ADC数据校准KD
#define ADC_CALIB_KD_MSK            (0x3FF << ADC_CALIB_KD_POS)

#define ADC_CALIB_KD_VALID_POS      16      //ADC数据校准KD数据是否有效
#define ADC_CALIB_KD_VALID_MSK      (0xFF << ADC_CALIB_KD_VALID_POS)



typedef struct {
	__IO uint32_t CFG;          //FLASH配置寄存器      
	
	__IO uint32_t ADDR;         //FLASH地址寄存器   

	__IO uint32_t WDATA;        //FLASH写数据寄存器
	
	__IO uint32_t RDATA;        //FLASH读数据寄存器 
	
	__IO uint32_t START;        //FLASH启动寄存器
	
	__IO uint32_t STATE;        //FLASH状态寄存器  
	
	uint32_t RESERVED1[(0x68 - 0x18)/4];
	
	__IO uint32_t MASK4K;        
	
} FLASH_TypeDef;


#define FLASH_CFG_CLKDIV_POS          0    //FLASH时钟分频系数   0: 1/8   1:  1/4   2:  1/2   3:  1/1
#define FLASH_CFG_CLKDIV_MSK          (0x03 << FLASH_CFG_CLKDIV_POS)

#define FLASH_CFG_READMODE_POS        2    //FLASH读模式配置    0: 单线读   1:  2线读   2:  4线读      
#define FLASH_CFG_READMODE_MSK        (0x03 << FLASH_CFG_READMODE_POS)

#define FLASH_CFG_CMD_POS             4    //FLASH命令码配置
#define FLASH_CFG_CMD_MSK             (0x0F << FLASH_CFG_CMD_POS)

#define FLASH_START_EN_POS            0    //FLASH启动控制位
#define FLASH_START_EN_MSK            (0x01 << FLASH_START_EN_POS)

#define FLASH_START_PROG_END_POS      1    //FLASH页编程操作预置数结束控制位
#define FLASH_START_PROG_END_MSK      (0x01 << FLASH_START_PROG_END_POS)

#define FLASH_STATE_BUSY_POS          0    //FLASH控制器忙标志
#define FLASH_STATE_BUSY_MSK          (0x01 << FLASH_STATE_BUSY_POS)

#define FLASH_STATE_PROG_VALID_POS    1    //FLASH编程命令操作时，数据寄存器有效标志位
#define FLASH_STATE_PROG_VALID_MSK    (0x01 << FLASH_STATE_PROG_VALID_POS)


/******************************************************************************/
/*						 Peripheral memory map							  */
/******************************************************************************/
#define RAM_BASE		    ((uint32_t)0x20000000)
#define AHB_BASE			((uint32_t)0x40000000)
#define APB1_BASE		 	((uint32_t)0x40060000)
#define APB2_BASE			((uint32_t)0x400B0000)
#define FLASH_DATA          ((uint32_t)0x41000000)

/* AHB Peripheral memory map */
#define SYS_BASE			(AHB_BASE + 0x00000)
#define PMU_BASE			(AHB_BASE + 0x00800)
#define CACHE_BASE	        (AHB_BASE + 0x02000)


/* APB1 Peripheral memory map */

#define GPIOA_BASE		    (APB1_BASE + 0x0000)
#define TIMPLUS0_BASE       (APB1_BASE + 0x7000)
#define IWDT_BASE		    (APB1_BASE + 0xA000)
#define UART0_BASE		    (APB1_BASE + 0xB000)
#define FLASH_CTRL_BASE		(APB1_BASE + 0xF000)

/* APB2 Peripheral memory map */
#define PORT_BASE			(APB2_BASE + 0x0000)
#define PWMBASE0_BASE		(APB2_BASE + 0x1000)
#define PWMPLUS0_BASE		(APB2_BASE + 0x4000)
#define SPI0_BASE			(APB2_BASE + 0x8000)
#define IIC0_BASE			(APB2_BASE + 0x9000)
#define ADC0_BASE			(APB2_BASE + 0xA000)

/******************************************************************************/
/*						 Peripheral declaration							 */
/******************************************************************************/
#define SYS					((SYS_TypeDef  *) SYS_BASE)

#define PMU 				((PMU_TypeDef  *) PMU_BASE)

#define CACHE				((CACHE_TypeDef*) CACHE_BASE)


#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE)

#define TIMPLUS0			((TIMPLUS_TypeDef *) TIMPLUS0_BASE)

#define IWDT				((IWDT_TypeDef  *) IWDT_BASE)

#define FLASH               ((FLASH_TypeDef  *) FLASH_CTRL_BASE)


#define PORT				((PORT_TypeDef *) PORT_BASE)

#define PWMBASE0			((PWMBASE_TypeDef  *) PWMBASE0_BASE)

#define PWMPLUS0			((PWMPLUS_TypeDef  *) PWMPLUS0_BASE)

#define SPI0				((SPI_TypeDef  *) SPI0_BASE)

#define IIC0                ((IIC_TypeDef  *) IIC0_BASE)

#define UART0				((UART_TypeDef *) UART0_BASE)

#define ADC				    ((ADC_TypeDef  *) ADC0_BASE)


#include "system_HBM32G003.h"
#include "HBM32G003_port.h"
#include "HBM32G003_gpio.h"
#include "HBM32G003_flash.h"
#include "HBM32G003_conf.h"
#include "HBM32G003_uart.h"
#include "HBM32G003_iwdt.h"
#include "HBM32G003_exti.h"
#include "HBM32G003_spi.h"
#include "HBM32G003_iic.h"
#include "HBM32G003_pwmbase.h"
#include "HBM32G003_pwmplus.h"
#include "HBM32G003_timerplus.h"
#include "HBM32G003_power.h"
#include "HBM32G003_adc.h"

#endif //__HBM32G003_H__
