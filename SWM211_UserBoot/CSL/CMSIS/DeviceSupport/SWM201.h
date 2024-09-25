#ifndef __SWM201_H__
#define __SWM201_H__

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
  UART0_IRQn               = 0,
  TIMR0_IRQn               = 1,
  CORDIC_IRQn              = 2,
  UART1_IRQn               = 3,
  PWM1_IRQn                = 4,
  TIMR1_IRQn               = 5,
  HALL_IRQn                = 6,
  PWM0_IRQn                = 7,
  BOD_IRQn                 = 8,
  PWMBRK_IRQn              = 9,
  RTC_IRQn                 = 10,
  WDT_IRQn                 = 11,
  I2C0_IRQn                = 12,
  XTALSTOP_IRQn            = 13,
  ADC0_IRQn                = 14,
  ACMP_IRQn                = 15,
  BTIMR0_IRQn              = 16,
  BTIMR1_IRQn              = 17,
  BTIMR2_IRQn              = 18,
  BTIMR3_IRQn              = 19,
  GPIOA_IRQn               = 20,
  GPIOB_IRQn               = 21,
  GPIOM_IRQn               = 22,
  GPIOA0_GPIOM0_IRQn       = 23,
  GPIOA1_GPIOM1_IRQn       = 24,
  GPIOA2_GPIOM2_IRQn       = 25,
  GPIOA3_GPIOM3_IRQn       = 26,
  GPIOB0_GPIOA8_IRQn       = 27,
  GPIOB1_GPIOA9_IRQn       = 28,
  GPIOB2_GPIOA10_IRQn      = 29,
  GPIOB3_GPIOA11_SPI0_IRQn = 30,
  GPIOB4_GPIOB10_QEI_IRQn  = 31,
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
#include <stdbool.h>
#include "core_cm0.h"				   /* Cortex-M0 processor and core peripherals		     */
#include "system_SWM201.h"


/******************************************************************************/
/*				Device Specific Peripheral registers structures			 */
/******************************************************************************/
typedef struct {
	__IO uint32_t CLKSEL;				    //Clock Select

	__IO uint32_t CLKDIVx_ON;				//[0] CLK_DIVx时钟源开关

	__IO uint32_t CLKEN0;					//Clock Enable
	
	__IO uint32_t CLKEN1;

	__IO uint32_t SLEEP;
	
		 uint32_t RESERVED[4];
	
	__IO uint32_t RSTSR;					//Reset Status
	
		 uint32_t RESERVED2[2];
	
	__IO uint32_t RTCWKCR;					//RTC Wakeup Control
	
	__IO uint32_t RTCWKSR;
	
		 uint32_t RESERVED3[18];
	
	__I  uint32_t CHIPID[4];
	
	__IO uint32_t BACKUP[4];				//Data Backup Register
	
		 uint32_t RESERVED4[12];
	
	__IO uint32_t PRNGCR;					//伪随机数控制寄存器
	
	__IO uint32_t PRNGDL;					//伪随机数数据寄存器低32位
	
	__IO uint32_t PRNGDH;					//伪随机数数据寄存器高32位
	
		 uint32_t RESERVED5[9];
		 
	__IO uint32_t PAWKEN;				    //PORTA Wakeup Enable
	__IO uint32_t PBWKEN;
	
         uint32_t RESERVED6[6];
	
	__IO uint32_t PMWKEN;
	
		 uint32_t RESERVED7[3];

	__IO uint32_t PAWKSR;				    //PORTA Wakeup Status，写1清零
	__IO uint32_t PBWKSR;
	
		 uint32_t RESERVED8[6];
	
	__IO uint32_t PMWKSR;
	
		 uint32_t RESERVED9[(0x400-0x150)/4-1];
	
	__IO uint32_t IOFILT0;					//IO Filter 0
	__IO uint32_t IOFILT1;
	
		 uint32_t RESERVED10[(0x720-0x404)/4-1];
	
	__IO uint32_t PRSTEN;					//外设复位使能，只有当PRSTEN的值为0x55时，才能写PRSTR0、PRSTR1
	__IO uint32_t PRSTR0;
	__IO uint32_t PRSTR1;

    //Analog Control: 0x400AA000
         uint32_t RESERVED11[(0x400AA000-0x40000728)/4-1];
	
	__IO uint32_t HRCCR;					//High speed RC Control Register
		 uint32_t RESERVED12[3];
    
    __IO uint32_t BODCR;
	__IO uint32_t BODSR;
	
		 uint32_t RESERVED13[2];
	
	__IO uint32_t XTALCR;
	__IO uint32_t XTALSR;
	
		 uint32_t RESERVED14[10];
	
    __IO uint32_t LRCCR;					//Low speed RC Control Register
    
         uint32_t RESERVED15[7];
	
	__IO uint32_t OPACR;					//OPA Control Register
	__IO uint32_t PGACR;					//PGA Control Register
	
		 uint32_t RESERVED16[2];
	
	__IO uint32_t ACMPCR;					//Analog Comparator Control Register
	__IO uint32_t ACMPSR;					//Analog Comparator Status Register
	__IO uint32_t ACMPCR2;
} SYS_TypeDef;


#define SYS_CLKSEL_SYS_Pos			0		//系统时钟选择	1 HRC	0 CLK_DIVx
#define SYS_CLKSEL_SYS_Msk			(0x01 << SYS_CLKSEL_SYS_Pos)
#define SYS_CLKSEL_CLK_DIVx_Pos		1		//选择CLK_DIVx  0 CLK_DIV1   1 CLK_DIV8
#define SYS_CLKSEL_CLK_DIVx_Msk		(0x01 << SYS_CLKSEL_CLK_DIVx_Pos)
#define SYS_CLKSEL_CLK_Pos			2		//Clock Source	0 LRC   2 XTAL_32K   3 XTAL   4 HRC
#define SYS_CLKSEL_CLK_Msk			(0x07 << SYS_CLKSEL_CLK_Pos)
#define SYS_CLKSEL_RTC_Pos			5		//RTC时钟源选择  0 LRC   1 XTAL_32K
#define SYS_CLKSEL_RTC_Msk			(0x01 << SYS_CLKSEL_RTC_Pos)
#define SYS_CLKSEL_IOFILT_Pos		6		//IO Filter时钟选择，0 HRC   2 XTAL   3 LRC
#define SYS_CLKSEL_IOFILT_Msk		(0x03 << SYS_CLKSEL_IOFILT_Pos)
#define SYS_CLKSEL_WDT_Pos			12		//看门狗时钟选择  0 HRC   1 XTAL   2 LRC   3 XTAL_32K
#define SYS_CLKSEL_WDT_Msk			(0x03 << SYS_CLKSEL_WDT_Pos)
#define SYS_CLKSEL_ADC_Pos			16		//ADC时钟选择  0b0000 HRC   0b0001 XTAL   0b1000 HRC/4   0b1001 XTAL/4   0b1100 HRC/8   0b1101 XTAL/8
#define SYS_CLKSEL_ADC_Msk			(0x0F << SYS_CLKSEL_ADC_Pos)
#define SYS_CLKSEL_WKUP_Pos			24		//SLEEP唤醒时钟选择  0 LRC  1 XTAL_32k
#define SYS_CLKSEL_WKUP_Msk			(0x01 << SYS_CLKSEL_WKUP_Pos)

#define SYS_CLKDIV_ON_Pos           0
#define SYS_CLKDIV_ON_Msk           (0x01 << SYS_CLKDIV_ON_Pos)

#define SYS_CLKEN0_GPIOA_Pos		0
#define SYS_CLKEN0_GPIOA_Msk		(0x01 << SYS_CLKEN0_GPIOA_Pos)
#define SYS_CLKEN0_GPIOB_Pos		1
#define SYS_CLKEN0_GPIOB_Msk		(0x01 << SYS_CLKEN0_GPIOB_Pos)
#define SYS_CLKEN0_GPIOM_Pos		4
#define SYS_CLKEN0_GPIOM_Msk		(0x01 << SYS_CLKEN0_GPIOM_Pos)
#define SYS_CLKEN0_UART0_Pos		6
#define SYS_CLKEN0_UART0_Msk		(0x01 << SYS_CLKEN0_UART0_Pos)
#define SYS_CLKEN0_UART1_Pos		7
#define SYS_CLKEN0_UART1_Msk		(0x01 << SYS_CLKEN0_UART1_Pos)
#define SYS_CLKEN0_WDT_Pos			10
#define SYS_CLKEN0_WDT_Msk			(0x01 << SYS_CLKEN0_WDT_Pos)
#define SYS_CLKEN0_TIMR_Pos			11
#define SYS_CLKEN0_TIMR_Msk			(0x01 << SYS_CLKEN0_TIMR_Pos)
#define SYS_CLKEN0_PWM_Pos			12
#define SYS_CLKEN0_PWM_Msk			(0x01 << SYS_CLKEN0_PWM_Pos)
#define SYS_CLKEN0_SPI0_Pos			13
#define SYS_CLKEN0_SPI0_Msk			(0x01 << SYS_CLKEN0_SPI0_Pos)
#define SYS_CLKEN0_I2C0_Pos			15
#define SYS_CLKEN0_I2C0_Msk			(0x01 << SYS_CLKEN0_I2C0_Pos)
#define SYS_CLKEN0_CORDIC_Pos		20
#define SYS_CLKEN0_CORDIC_Msk		(0x01 << SYS_CLKEN0_CORDIC_Pos)
#define SYS_CLKEN0_DIV_Pos			21
#define SYS_CLKEN0_DIV_Msk			(0x01 << SYS_CLKEN0_DIV_Pos)
#define SYS_CLKEN0_ANAC_Pos			25		//模拟控制单元时钟使能
#define SYS_CLKEN0_ANAC_Msk			(0x01 << SYS_CLKEN0_ANAC_Pos)
#define SYS_CLKEN0_ADC0_Pos			26
#define SYS_CLKEN0_ADC0_Msk			(0x01 << SYS_CLKEN0_ADC0_Pos)

#define SYS_CLKEN1_RTC_Pos			19
#define SYS_CLKEN1_RTC_Msk			(0x01 << SYS_CLKEN1_RTC_Pos)
#define SYS_CLKEN1_IOFILT_Pos		20		//IO Filter
#define SYS_CLKEN1_IOFILT_Msk		(0x01 << SYS_CLKEN1_IOFILT_Pos)
#define SYS_CLKEN1_BTIMR_Pos		22
#define SYS_CLKEN1_BTIMR_Msk		(0x01 << SYS_CLKEN1_BTIMR_Pos)
#define SYS_CLKEN1_QEI_Pos			26
#define SYS_CLKEN1_QEI_Msk			(0x01 << SYS_CLKEN1_QEI_Pos)

#define SYS_SLEEP_SLEEP_Pos			0		//将该位置1后，系统将进入SLEEP模式
#define SYS_SLEEP_SLEEP_Msk			(0x01 << SYS_SLEEP_SLEEP_Pos)
#define SYS_SLEEP_STOP_Pos			1		//将该位置1后，系统将进入STOP SLEEP模式
#define SYS_SLEEP_STOP_Msk			(0x01 << SYS_SLEEP_STOP_Pos)

#define SYS_RSTSR_POR_Pos			0		//1 出现过POR复位，写1清零
#define SYS_RSTSR_POR_Msk			(0x01 << SYS_RSTSR_POR_Pos)
#define SYS_RSTSR_WDT_Pos			1		//1 出现过WDT复位，写1清零
#define SYS_RSTSR_WDT_Msk			(0x01 << SYS_RSTSR_WDT_Pos)

#define SYS_RTCWKCR_EN_Pos			0		//RTC唤醒使能
#define SYS_RTCWKCR_EN_Msk			(0x01 << SYS_RTCWKCR_EN_Pos)

#define SYS_RTCWKSR_FLAG_Pos		0		//RTC唤醒标志，写1清零
#define SYS_RTCWKSR_FLAG_Msk		(0x01 << SYS_RTCWKSR_FLAG_Pos)

#define SYS_PRNGCR_CLR_Pos			0		//种子清零，至少保持一个LRC时钟周期
#define SYS_PRNGCR_CLR_Msk			(0x01 << SYS_PRNGCR_CLR_Pos)
#define SYS_PRNGCR_MODE_Pos			1		//0 关闭   2 必须开启XTAL才能工作   3 无需开启XTAL（只需开启HRC和LRC）
#define SYS_PRNGCR_MODE_Msk			(0x03 << SYS_PRNGCR_MODE_Pos)
#define SYS_PRNGCR_RDY_Pos			8		//1 可以从PRNGDL和PRNGDH读取数据
#define SYS_PRNGCR_RDY_Msk			(0x01 << SYS_PRNGCR_RDY_Pos)

#define SYS_IOFILT_TIM_Pos			0		//滤波窗口时间 = Tfilter_clk * 时钟分频 * 2^TIM
#define SYS_IOFILT_TIM_Msk			(0x0F << SYS_IOFILT_TIM_Pos)
#define SYS_IOFILT_CLKDIV_Pos		4		//0 时钟不分频   1 时钟32分频
#define SYS_IOFILT_CLKDIV_Msk		(0x01 << SYS_IOFILT_CLKDIV_Pos)
#define SYS_IOFILT_IOSEL_Pos		5		//被滤波IO选择，每个IOFILT可为四个IO中的一个进行滤波
#define SYS_IOFILT_IOSEL_Msk		(0x03 << SYS_IOFILT_IOSEL_Pos)

#define SYS_PRSTR0_GPIOA_Pos		0		//1 复位GPIOA    0 不复位
#define SYS_PRSTR0_GPIOA_Msk		(0x01 <<SYS_PRSTR0_GPIOA_Pos)
#define SYS_PRSTR0_GPIOB_Pos		1
#define SYS_PRSTR0_GPIOB_Msk		(0x01 <<SYS_PRSTR0_GPIOB_Pos)
#define SYS_PRSTR0_GPIOM_Pos		4
#define SYS_PRSTR0_GPIOM_Msk		(0x01 <<SYS_PRSTR0_GPIOM_Pos)
#define SYS_PRSTR0_UART0_Pos		6
#define SYS_PRSTR0_UART0_Msk		(0x01 <<SYS_PRSTR0_UART0_Pos)
#define SYS_PRSTR0_UART1_Pos		7
#define SYS_PRSTR0_UART1_Msk		(0x01 <<SYS_PRSTR0_UART1_Pos)
#define SYS_PRSTR0_WDT_Pos			10
#define SYS_PRSTR0_WDT_Msk			(0x01 <<SYS_PRSTR0_WDT_Pos)
#define SYS_PRSTR0_TIMR_Pos			11
#define SYS_PRSTR0_TIMR_Msk			(0x01 <<SYS_PRSTR0_TIMR_Pos)
#define SYS_PRSTR0_PWM_Pos			12
#define SYS_PRSTR0_PWM_Msk			(0x01 <<SYS_PRSTR0_PWM_Pos)
#define SYS_PRSTR0_SPI0_Pos			13
#define SYS_PRSTR0_SPI0_Msk			(0x01 <<SYS_PRSTR0_SPI0_Pos)
#define SYS_PRSTR0_I2C0_Pos			15
#define SYS_PRSTR0_I2C0_Msk			(0x01 <<SYS_PRSTR0_I2C0_Pos)
#define SYS_PRSTR0_CORDIC_Pos		20
#define SYS_PRSTR0_CORDIC_Msk		(0x01 <<SYS_PRSTR0_CORDIC_Pos)
#define SYS_PRSTR0_DIV_Pos			21
#define SYS_PRSTR0_DIV_Msk			(0x01 <<SYS_PRSTR0_DIV_Pos)
#define SYS_PRSTR0_ANAC_Pos			25
#define SYS_PRSTR0_ANAC_Msk			(0x01 <<SYS_PRSTR0_ANAC_Pos)
#define SYS_PRSTR0_ADC0_Pos			26
#define SYS_PRSTR0_ADC0_Msk			(0x01 <<SYS_PRSTR0_ADC0_Pos)

#define SYS_PRSTR1_RTC_Pos			19
#define SYS_PRSTR1_RTC_Msk			(0x01 <<SYS_PRSTR1_RTC_Pos)
#define SYS_PRSTR1_IOFILT_Pos		20
#define SYS_PRSTR1_IOFILT_Msk		(0x01 <<SYS_PRSTR1_IOFILT_Pos)
#define SYS_PRSTR1_BTIMR_Pos		22
#define SYS_PRSTR1_BTIMR_Msk		(0x01 <<SYS_PRSTR1_BTIMR_Pos)

#define SYS_HRCCR_ON_Pos			0		//High speed RC ON
#define SYS_HRCCR_ON_Msk			(0x01 << SYS_HRCCR_ON_Pos)
#define SYS_HRCCR_DBL_Pos		    1		//Double Frequency	 0 30MHz   1 60MHz
#define SYS_HRCCR_DBL_Msk		    (0x03 << SYS_HRCCR_DBL_Pos)

#define SYS_BODCR_IE_Pos		    1		//Interrupt Enable
#define SYS_BODCR_IE_Msk		    (0x01 << SYS_BODCR_IE_Pos)
#define SYS_BODCR_INTLVL_Pos		4		//BOD中断触发电平，0 1.9v   1 2.1v   2 2.3v   3 2.5v   4 2.7v   5 3.5v   6 4.1v
#define SYS_BODCR_INTLVL_Msk		(0x07 << SYS_BODCR_INTLVL_Pos)
#define SYS_BODCR_RSTLVL_Pos		7		//BOD复位电平，0 1.7v   1 1.9v   2 2.1v   3 2.7v   4 3.5v
#define SYS_BODCR_RSTLVL_Msk		(0x07 << SYS_BODCR_RSTLVL_Pos)

#define SYS_BODSR_IF_Pos			0		//中断标志，写1清零
#define SYS_BODSR_IF_Msk			(0x01 << SYS_BODSR_IF_Pos)
#define SYS_BODSR_ST_Pos			1		//BOD Status
#define SYS_BODSR_ST_Msk			(0x01 << SYS_BODSR_ST_Pos)

#define SYS_XTALCR_32KON_Pos		0		//XTAL_32K On
#define SYS_XTALCR_32KON_Msk		(0x01 << SYS_XTALCR_32KON_Pos)
#define SYS_XTALCR_ON_Pos			1		//XTAL On
#define SYS_XTALCR_ON_Msk			(0x01 << SYS_XTALCR_ON_Pos)
#define SYS_XTALCR_32KDET_Pos		4		//XTAL_32K Stop Detect
#define SYS_XTALCR_32KDET_Msk		(0x01 << SYS_XTALCR_32KDET_Pos)
#define SYS_XTALCR_DET_Pos			5		//XTAL Stop Detect
#define SYS_XTALCR_DET_Msk			(0x01 << SYS_XTALCR_DET_Pos)
#define SYS_XTALCR_32KDRV_Pos		8		//XTAL_32K 驱动能力，可微调频率
#define SYS_XTALCR_32KDRV_Msk		(0x0F << SYS_XTALCR_32KDRV_Pos)
#define SYS_XTALCR_DRV_Pos			16		//XTAL 驱动能力，可微调频率
#define SYS_XTALCR_DRV_Msk			(0x1F << SYS_XTALCR_DRV_Pos)

#define SYS_XTALSR_32KSTOP_Pos		0		//XTAL_32K Stop，写1清零
#define SYS_XTALSR_32KSTOP_Msk		(0x01 << SYS_XTALSR_32KSTOP_Pos)
#define SYS_XTALSR_STOP_Pos			1		//XTAL Stop，写1清零
#define SYS_XTALSR_STOP_Msk			(0x01 << SYS_XTALSR_STOP_Pos)

#define SYS_LRCCR_ON_Pos			0		//Low Speed RC On
#define SYS_LRCCR_ON_Msk			(0x01 << SYS_LRCCR_ON_Pos)

#define SYS_OPACR_OPA0ON_Pos		0		//OPA0 开启
#define SYS_OPACR_OPA0ON_Msk		(0x01 << SYS_OPACR_OPA0ON_Pos)
#define SYS_OPACR_OPA1ON_Pos		1
#define SYS_OPACR_OPA1ON_Msk		(0x01 << SYS_OPACR_OPA1ON_Pos)
#define SYS_OPACR_OPA2ON_Pos		2
#define SYS_OPACR_OPA2ON_Msk		(0x01 << SYS_OPACR_OPA2ON_Pos)
#define SYS_OPACR_VREFON_Pos		4		//VREF On, 1 PGA1/2的正输入端接内部VREF
#define SYS_OPACR_VREFON_Msk		(0x01 << SYS_OPACR_VREFON_Pos)
#define SYS_OPACR_OPA1MD_Pos		5		//OPA1 Mode, 0 OPA   1 PGA
#define SYS_OPACR_OPA1MD_Msk		(0x01 << SYS_OPACR_OPA1MD_Pos)
#define SYS_OPACR_OPA2MD_Pos		6		//OPA2 Mode, 0 OPA   1 PGA
#define SYS_OPACR_OPA2MD_Msk		(0x01 << SYS_OPACR_OPA2MD_Pos)
#define SYS_OPACR_OPA0DV_Pos		16		//OPA0 Drive strength, 0 X1   1 X2
#define SYS_OPACR_OPA0DV_Msk		(0x03 << SYS_OPACR_OPA0DV_Pos)
#define SYS_OPACR_OPA1DV_Pos		18
#define SYS_OPACR_OPA1DV_Msk		(0x03 << SYS_OPACR_OPA1DV_Pos)
#define SYS_OPACR_OPA2DV_Pos		20
#define SYS_OPACR_OPA2DV_Msk		(0x03 << SYS_OPACR_OPA2DV_Pos)
#define SYS_OPACR_OPA0IB_Pos		24		//OPA0 bias current, 0 5uA   1 10uA（更高压摆率）
#define SYS_OPACR_OPA0IB_Msk		(0x01 << SYS_OPACR_OPA0IB_Pos)
#define SYS_OPACR_OPA1IB_Pos		25
#define SYS_OPACR_OPA1IB_Msk		(0x01 << SYS_OPACR_OPA1IB_Pos)
#define SYS_OPACR_OPA2IB_Pos		26
#define SYS_OPACR_OPA2IB_Msk		(0x01 << SYS_OPACR_OPA2IB_Pos)

#define SYS_PGACR_OPA1GN_Pos		4		//PGA模式下，OPA1增益反馈电阻选择，0 10K  1 15K  2 20K
#define SYS_PGACR_OPA1GN_Msk		(0x03 << SYS_PGACR_OPA1GN_Pos)
#define SYS_PGACR_OPA2GN_Pos		6
#define SYS_PGACR_OPA2GN_Msk		(0x03 << SYS_PGACR_OPA2GN_Pos)
#define SYS_PGACR_OPA1SW_Pos		16		//PGA模式下，OPA1输出滤波电阻选择，0 1K  1 10K
#define SYS_PGACR_OPA1SW_Msk		(0x03 << SYS_PGACR_OPA1SW_Pos)
#define SYS_PGACR_OPA2SW_Pos		18
#define SYS_PGACR_OPA2SW_Msk		(0x03 << SYS_PGACR_OPA2SW_Pos)

#define SYS_ACMPCR_CMP0ON_Pos		0		//ACMP0 开启
#define SYS_ACMPCR_CMP0ON_Msk		(0x01 << SYS_ACMPCR_CMP0ON_Pos)
#define SYS_ACMPCR_CMP1ON_Pos		1
#define SYS_ACMPCR_CMP1ON_Msk		(0x01 << SYS_ACMPCR_CMP1ON_Pos)
#define SYS_ACMPCR_CMP2ON_Pos		2
#define SYS_ACMPCR_CMP2ON_Msk		(0x01 << SYS_ACMPCR_CMP2ON_Pos)
#define SYS_ACMPCR_CMP3ON_Pos		3
#define SYS_ACMPCR_CMP3ON_Msk		(0x01 << SYS_ACMPCR_CMP3ON_Pos)
#define SYS_ACMPCR_CMP0HYS_Pos		8		//ACMP0 迟滞开启
#define SYS_ACMPCR_CMP0HYS_Msk		(0x01 << SYS_ACMPCR_CMP0HYS_Pos)
#define SYS_ACMPCR_CMP1HYS_Pos		9
#define SYS_ACMPCR_CMP1HYS_Msk		(0x01 << SYS_ACMPCR_CMP1HYS_Pos)
#define SYS_ACMPCR_CMP2HYS_Pos		10
#define SYS_ACMPCR_CMP2HYS_Msk		(0x01 << SYS_ACMPCR_CMP2HYS_Pos)
#define SYS_ACMPCR_CMP3HYS_Pos		11
#define SYS_ACMPCR_CMP3HYS_Msk		(0x01 << SYS_ACMPCR_CMP3HYS_Pos)
#define SYS_ACMPCR_CMP0IE_Pos		16		//ACMP0 中断使能
#define SYS_ACMPCR_CMP0IE_Msk		(0x01 << SYS_ACMPCR_CMP0IE_Pos)
#define SYS_ACMPCR_CMP1IE_Pos		17
#define SYS_ACMPCR_CMP1IE_Msk		(0x01 << SYS_ACMPCR_CMP1IE_Pos)
#define SYS_ACMPCR_CMP2IE_Pos		18
#define SYS_ACMPCR_CMP2IE_Msk		(0x01 << SYS_ACMPCR_CMP2IE_Pos)
#define SYS_ACMPCR_CMP3IE_Pos		19
#define SYS_ACMPCR_CMP3IE_Msk		(0x01 << SYS_ACMPCR_CMP3IE_Pos)

#define SYS_ACMPSR_CMP0OUT_Pos		0		//0 N > P   1 P > N
#define SYS_ACMPSR_CMP0OUT_Msk		(0x01 << SYS_ACMPSR_CMP0OUT_Pos)
#define SYS_ACMPSR_CMP1OUT_Pos		1
#define SYS_ACMPSR_CMP1OUT_Msk		(0x01 << SYS_ACMPSR_CMP1OUT_Pos)
#define SYS_ACMPSR_CMP2OUT_Pos		2
#define SYS_ACMPSR_CMP2OUT_Msk		(0x01 << SYS_ACMPSR_CMP2OUT_Pos)
#define SYS_ACMPSR_CMP3OUT_Pos		3
#define SYS_ACMPSR_CMP3OUT_Msk		(0x01 << SYS_ACMPSR_CMP3OUT_Pos)
#define SYS_ACMPSR_CMP0IF_Pos		8		//中断标志，写1清零
#define SYS_ACMPSR_CMP0IF_Msk		(0x01 << SYS_ACMPSR_CMP0IF_Pos)
#define SYS_ACMPSR_CMP1IF_Pos		9
#define SYS_ACMPSR_CMP1IF_Msk		(0x01 << SYS_ACMPSR_CMP1IF_Pos)
#define SYS_ACMPSR_CMP2IF_Pos		10
#define SYS_ACMPSR_CMP2IF_Msk		(0x01 << SYS_ACMPSR_CMP2IF_Pos)
#define SYS_ACMPSR_CMP3IF_Pos		11
#define SYS_ACMPSR_CMP3IF_Msk		(0x01 << SYS_ACMPSR_CMP3IF_Pos)

#define SYS_ACMPCR2_HALL0_Pos		0		//1 ACMP0输出连接HALL0输入
#define SYS_ACMPCR2_HALL0_Msk		(0x01 << SYS_ACMPCR2_HALL0_Pos)
#define SYS_ACMPCR2_HALL1_Pos		1		//1 ACMP1输出连接HALL1输入
#define SYS_ACMPCR2_HALL1_Msk		(0x01 << SYS_ACMPCR2_HALL1_Pos)
#define SYS_ACMPCR2_HALL2_Pos		2		//1 ACMP2输出连接HALL2输入
#define SYS_ACMPCR2_HALL2_Msk		(0x01 << SYS_ACMPCR2_HALL2_Pos)
#define SYS_ACMPCR2_VREF_Pos		3		//ACMP内部基准电压VREF，电压值为 0.25 + 0.05 * VREF
#define SYS_ACMPCR2_VREF_Msk		(0x0F << SYS_ACMPCR2_VREF_Pos)
#define SYS_ACMPCR2_P8K2_Pos		7		//1 ACMP0、ACMP1、ACMP2的P端经8.2K电阻相连，相连点电平替代VREF电平作为它们的N端输入
#define SYS_ACMPCR2_P8K2_Msk		(0x01 << SYS_ACMPCR2_P8K2_Pos)
#define SYS_ACMPCR2_0NVR_Pos		8		//1 ACMP0 N输入端接内部VREF
#define SYS_ACMPCR2_0NVR_Msk		(0x01 << SYS_ACMPCR2_0NVR_Pos)
#define SYS_ACMPCR2_1NVR_Pos		9
#define SYS_ACMPCR2_1NVR_Msk		(0x01 << SYS_ACMPCR2_1NVR_Pos)
#define SYS_ACMPCR2_2NVR_Pos		10
#define SYS_ACMPCR2_2NVR_Msk		(0x01 << SYS_ACMPCR2_2NVR_Pos)
#define SYS_ACMPCR2_3NVR_Pos		11
#define SYS_ACMPCR2_3NVR_Msk		(0x01 << SYS_ACMPCR2_3NVR_Pos)




typedef struct {
	__IO uint32_t FUNC0;					//引脚功能选择
	
	__IO uint32_t FUNC1;
	
		 uint32_t RESERVED[62];
	
    __IO uint32_t PULLU;              		//上拉使能
    
         uint32_t RESERVED2[63];
    
    __IO uint32_t PULLD;	              	//下拉使能
    
         uint32_t RESERVED3[63];
    
    __IO uint32_t INEN;               		//输入使能
    
         uint32_t RESERVED4[63];

	__IO uint32_t OPEND;              		//开漏使能      
} PORT_TypeDef;




typedef struct {
	__IO uint32_t ODR;
#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7
#define PIN8    8
#define PIN9    9
#define PIN10   10
#define PIN11   11
#define PIN12   12
#define PIN13   13
#define PIN14   14
#define PIN15   15

	__IO uint32_t DIR;					    //0 输入	1 输出

	__IO uint32_t INTLVLTRG;				//Interrupt Level Trigger  1 电平触发中断	0 边沿触发中断

	__IO uint32_t INTBE;					//Both Edge，当INTLVLTRG设为边沿触发中断时，此位置1表示上升沿和下降沿都触发中断，置0时触发边沿由INTRISEEN选择

	__IO uint32_t INTRISEEN;				//Interrupt Rise Edge Enable   1 上升沿/高电平触发中断	0 下降沿/低电平触发中断

	__IO uint32_t INTEN;					//1 中断使能	0 中断禁止

	__IO uint32_t INTRAWSTAT;			    //中断检测单元是否检测到了触发中断的条件 1 检测到了中断触发条件	0 没有检测到中断触发条件

	__IO uint32_t INTSTAT;				    //INTSTAT.PIN0 = INTRAWSTAT.PIN0 & INTEN.PIN0

	__IO uint32_t INTCLR;				    //写1清除中断标志，只对边沿触发中断有用
	
		 uint32_t RESERVED[3];
	
	__IO uint32_t IDR;
	
		 uint32_t RESERVED2[3];
	
	__IO uint32_t DATAPIN0;					//PIN0引脚的DATA寄存器，单个引脚对应整个32位寄存器，方便实现原子写操作
	__IO uint32_t DATAPIN1;
	__IO uint32_t DATAPIN2;
	__IO uint32_t DATAPIN3;
	__IO uint32_t DATAPIN4;
	__IO uint32_t DATAPIN5;
	__IO uint32_t DATAPIN6;
	__IO uint32_t DATAPIN7;
	__IO uint32_t DATAPIN8;
	__IO uint32_t DATAPIN9;
	__IO uint32_t DATAPIN10;
	__IO uint32_t DATAPIN11;
	__IO uint32_t DATAPIN12;
	__IO uint32_t DATAPIN13;
	__IO uint32_t DATAPIN14;
	__IO uint32_t DATAPIN15;
} GPIO_TypeDef;




typedef struct {
	__IO uint32_t LOAD;						//定时器加载值，使能后定时器从此数值开始向下递减计数

	__I  uint32_t VALUE;					//定时器当前值，LDVAL-CVAL 可计算出计时时长

	__IO uint32_t CR;
	
		 uint32_t RESERVED;
	
	__IO uint32_t IE;
		
	__IO uint32_t IF;
	
	__IO uint32_t HALT;						//[0] 1 暂停计数    0 恢复计数
	
	__IO uint32_t OCCR;
	
	__IO uint32_t OCMAT0;
	__IO uint32_t OCMAT1;
	
	__IO uint32_t ICLOW;
	__IO uint32_t ICHIGH;
	
	__IO uint32_t PREDIV;					//预分频，8位
} TIMR_TypeDef;


#define TIMR_LOAD_VALUE_Pos			0
#define TIMR_LOAD_VALUE_Msk			(0xFFFFFF << TIMR_LOAD_VALUE_Pos)
#define TIMR_LOAD_RELOAD_Pos		24		//1 计数器立即以新写入的LOAD值开始计数，只有BTIMR有此功能
#define TIMR_LOAD_RELOAD_Msk		(0x01 << TIMR_LOAD_RELOAD_Pos)

#define TIMR_CR_CLKSRC_Pos			0		//时钟源：  0 内部系统时钟	2 外部引脚脉冲计数
#define TIMR_CR_CLKSRC_Msk			(0x03 << TIMR_CR_CLKSRC_Pos)
#define TIMR_CR_MODE_Pos			2		//工作模式：0 定时器    1 输入捕获    2 输出比较
#define TIMR_CR_MODE_Msk			(0x03 << TIMR_CR_MODE_Pos)
#define TIMR_CR_ICEDGE_Pos			4		//输入捕获模式下计数启动边沿：0 双边沿   1 上升沿   2 下降沿
#define TIMR_CR_ICEDGE_Msk			(0x03 << TIMR_CR_ICEDGE_Pos)

#define TIMR_IE_TO_Pos				0		//Time out
#define TIMR_IE_TO_Msk				(0x01 << TIMR_IE_TO_Pos)
#define TIMR_IE_OC0_Pos				1		//输出比较，第一个反转点
#define TIMR_IE_OC0_Msk				(0x01 << TIMR_IE_OC0_Pos)
#define TIMR_IE_OC1_Pos				2		//输出比较，第二个反转点
#define TIMR_IE_OC1_Msk				(0x01 << TIMR_IE_OC1_Pos)
#define TIMR_IE_ICR_Pos				3		//输入捕获，上升沿中断
#define TIMR_IE_ICR_Msk				(0x01 << TIMR_IE_ICR_Pos)
#define TIMR_IE_ICF_Pos				4		//输入捕获，下降沿中断
#define TIMR_IE_ICF_Msk				(0x01 << TIMR_IE_ICF_Pos)

#define TIMR_IF_TO_Pos				0		//超时中断标志，写1清零
#define TIMR_IF_TO_Msk				(0x01 << TIMR_IF_TO_Pos)
#define TIMR_IF_OC0_Pos				1
#define TIMR_IF_OC0_Msk				(0x01 << TIMR_IF_OC0_Pos)
#define TIMR_IF_OC1_Pos				2
#define TIMR_IF_OC1_Msk				(0x01 << TIMR_IF_OC1_Pos)
#define TIMR_IF_ICR_Pos				3
#define TIMR_IF_ICR_Msk				(0x01 << TIMR_IF_ICR_Pos)
#define TIMR_IF_ICF_Pos				4
#define TIMR_IF_ICF_Msk				(0x01 << TIMR_IF_ICF_Pos)

#define TIMR_OCCR_FORCELVL_Pos		0		//Force Levle，强制输出电平
#define TIMR_OCCR_FORCELVL_Msk		(0x01 << TIMR_OCCR_FORCELVL_Pos)
#define TIMR_OCCR_INITLVL_Pos		1		//Initial Level, 初始输出电平
#define TIMR_OCCR_INITLVL_Msk		(0x01 << TIMR_OCCR_INITLVL_Pos)
#define TIMR_OCCR_FORCEEN_Pos		2		//Force Enable, 强制输出使能
#define TIMR_OCCR_FORCEEN_Msk		(0x01 << TIMR_OCCR_FORCEEN_Pos)


typedef struct {
	__IO uint32_t HALLIE;					//[0] HALL中断使能
	
		 uint32_t RESERVED;
	
	__IO uint32_t HALLIF;
	
	__IO uint32_t HALLEN;					//[0] HALL功能开关
	
	__IO uint32_t HALLDR;					//HALL输入跳变沿将计数器（加载值 - 当前值）存入此寄存器
	
		 uint32_t RESERVED2[2];
	
	__IO uint32_t HALLSR;
	
		 uint32_t RESERVED3[8];
	
	__IO uint32_t EN;
} TIMRG_TypeDef;


#define TIMRG_HALLIF_IN0_Pos		0		//HALL输入信号0触发中断标志，写1清零
#define TIMRG_HALLIF_IN0_Msk		(0x01 << TIMRG_HALLIF_IN0_Pos)
#define TIMRG_HALLIF_IN1_Pos		1
#define TIMRG_HALLIF_IN1_Msk		(0x01 << TIMRG_HALLIF_IN1_Pos)
#define TIMRG_HALLIF_IN2_Pos		2
#define TIMRG_HALLIF_IN2_Msk		(0x01 << TIMRG_HALLIF_IN2_Pos)

#define TIMRG_HALLSR_IN0_Pos		0		//HALL输入信号当前电平
#define TIMRG_HALLSR_IN0_Msk		(0x01 << TIMRG_HALLSR_IN0_Pos)
#define TIMRG_HALLSR_IN1_Pos		1
#define TIMRG_HALLSR_IN1_Msk		(0x01 << TIMRG_HALLSR_IN1_Pos)
#define TIMRG_HALLSR_IN2_Pos		2
#define TIMRG_HALLSR_IN2_Msk		(0x01 << TIMRG_HALLSR_IN2_Pos)

#define TIMRG_EN_TIMR0_Pos			0
#define TIMRG_EN_TIMR0_Msk			(0x01 << TIMRG_EN_TIMR0_Pos)
#define TIMRG_EN_TIMR1_Pos			1
#define TIMRG_EN_TIMR1_Msk			(0x01 << TIMRG_EN_TIMR1_Pos)
#define TIMRG_EN_TIMR2_Pos			2
#define TIMRG_EN_TIMR2_Msk			(0x01 << TIMRG_EN_TIMR2_Pos)
#define TIMRG_EN_TIMR3_Pos			3
#define TIMRG_EN_TIMR3_Msk			(0x01 << TIMRG_EN_TIMR3_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t CTRL;
	
	__IO uint32_t BAUD;
	
	__IO uint32_t FIFO;
	
	__IO uint32_t LINCR;
	
	union {
		__IO uint32_t CTSCR;
		
		__IO uint32_t RTSCR;
	};
	
	__IO uint32_t CFG;
	
	__IO uint32_t TOCR;						//Timeout Control Register
} UART_TypeDef;


#define UART_DATA_DATA_Pos			0
#define UART_DATA_DATA_Msk			(0x1FF << UART_DATA_DATA_Pos)
#define UART_DATA_VALID_Pos			9		//当DATA字段有有效的接收数据时，该位硬件置1，读取数据后自动清零
#define UART_DATA_VALID_Msk			(0x01 << UART_DATA_VALID_Pos)
#define UART_DATA_PAERR_Pos			10		//Parity Error
#define UART_DATA_PAERR_Msk			(0x01 << UART_DATA_PAERR_Pos)

#define UART_CTRL_TXIDLE_Pos		0		//TX IDLE: 0 正在发送数据	1 空闲状态，没有数据发送
#define UART_CTRL_TXIDLE_Msk		(0x01 << UART_CTRL_TXIDLE_Pos)
#define UART_CTRL_TXFF_Pos		    1		//TX FIFO Full
#define UART_CTRL_TXFF_Msk		    (0x01 << UART_CTRL_TXFF_Pos)
#define UART_CTRL_TXIE_Pos			2		//TX 中断使能: 1 TX FF 中数据少于设定个数时产生中断
#define UART_CTRL_TXIE_Msk			(0x01 << UART_CTRL_TXIE_Pos)
#define UART_CTRL_RXNE_Pos			3		//RX FIFO Not Empty
#define UART_CTRL_RXNE_Msk			(0x01 << UART_CTRL_RXNE_Pos)
#define UART_CTRL_RXIE_Pos			4		//RX 中断使能: 1 RX FF 中数据达到设定个数时产生中断
#define UART_CTRL_RXIE_Msk			(0x01 << UART_CTRL_RXIE_Pos)
#define UART_CTRL_RXOV_Pos			5		//RX FIFO Overflow，写1清零
#define UART_CTRL_RXOV_Msk			(0x01 << UART_CTRL_RXOV_Pos)
#define UART_CTRL_TXDOIE_Pos		6		//TX Done 中断使能，发送FIFO空且发送发送移位寄存器已将最后一位发送出去
#define UART_CTRL_TXDOIE_Msk		(0x01 << UART_CTRL_TXDOIE_Pos)
#define UART_CTRL_EN_Pos			9
#define UART_CTRL_EN_Msk			(0x01 << UART_CTRL_EN_Pos)
#define UART_CTRL_LOOP_Pos			10
#define UART_CTRL_LOOP_Msk			(0x01 << UART_CTRL_LOOP_Pos)
#define UART_CTRL_TOIE_Pos			14		//TimeOut 中断使能，接收到上个字符后，超过 TOTIME/BAUDRAUD 秒没有接收到新的数据
#define UART_CTRL_TOIE_Msk			(0x01 << UART_CTRL_TOIE_Pos)
#define UART_CTRL_BRKDET_Pos		15		//LIN Break Detect，检测到LIN Break，即RX线上检测到连续11位低电平
#define UART_CTRL_BRKDET_Msk		(0x01 << UART_CTRL_BRKDET_Pos)
#define UART_CTRL_BRKIE_Pos			16		//LIN Break Detect 中断使能
#define UART_CTRL_BRKIE_Msk			(0x01 << UART_CTRL_BRKIE_Pos)
#define UART_CTRL_GENBRK_Pos		17		//Generate LIN Break，发送LIN Break
#define UART_CTRL_GENBRK_Msk		(0x01 << UART_CTRL_GENBRK_Pos)
#define UART_CTRL_DATA9b_Pos		18		//1 9位数据位    0 8位数据位
#define UART_CTRL_DATA9b_Msk		(0x01 << UART_CTRL_DATA9b_Pos)
#define UART_CTRL_PARITY_Pos		19		//000 无校验    001 奇校验   011 偶校验   101 固定为1    111 固定为0
#define UART_CTRL_PARITY_Msk		(0x07 << UART_CTRL_PARITY_Pos)
#define UART_CTRL_STOP2b_Pos		22		//1 2位停止位    0 1位停止位
#define UART_CTRL_STOP2b_Msk		(0x03 << UART_CTRL_STOP2b_Pos)

#define UART_BAUD_BAUD_Pos			0		//串口波特率 = SYS_Freq/16/BAUD - 1
#define UART_BAUD_BAUD_Msk			(0x3FFF << UART_BAUD_BAUD_Pos)
#define UART_BAUD_TXD_Pos			14		//通过此位可直接读取串口TXD引脚上的电平
#define UART_BAUD_TXD_Msk			(0x01 << UART_BAUD_TXD_Pos)
#define UART_BAUD_RXD_Pos			15		//通过此位可直接读取串口RXD引脚上的电平
#define UART_BAUD_RXD_Msk			(0x01 << UART_BAUD_RXD_Pos)
#define UART_BAUD_RXTOIF_Pos		16		//接收&超时的中断标志 = RXIF | TOIF
#define UART_BAUD_RXTOIF_Msk		(0x01 << UART_BAUD_RXTOIF_Pos)
#define UART_BAUD_TXIF_Pos			17		//发送中断标志 = TXTHRF & TXIE
#define UART_BAUD_TXIF_Msk			(0x01 << UART_BAUD_TXIF_Pos)
#define UART_BAUD_BRKIF_Pos			18		//LIN Break Detect 中断标志，检测到LIN Break时若BRKIE=1，此位由硬件置位
#define UART_BAUD_BRKIF_Msk			(0x01 << UART_BAUD_BRKIF_Pos)
#define UART_BAUD_RXTHRF_Pos		19		//RX FIFO Threshold Flag，RX FIFO中数据达到设定个数（RXLVL >  RXTHR）时硬件置1
#define UART_BAUD_RXTHRF_Msk		(0x01 << UART_BAUD_RXTHRF_Pos)
#define UART_BAUD_TXTHRF_Pos		20		//TX FIFO Threshold Flag，TX FIFO中数据少于设定个数（TXLVL <= TXTHR）时硬件置1
#define UART_BAUD_TXTHRF_Msk		(0x01 << UART_BAUD_TXTHRF_Pos)
#define UART_BAUD_TOIF_Pos			21		//TimeOut 中断标志，超过 TOTIME/BAUDRAUD 秒没有接收到新的数据时若TOIE=1，此位由硬件置位
#define UART_BAUD_TOIF_Msk			(0x01 << UART_BAUD_TOIF_Pos)
#define UART_BAUD_RXIF_Pos			22		//接收中断标志 = RXTHRF & RXIE
#define UART_BAUD_RXIF_Msk			(0x01 << UART_BAUD_RXIF_Pos)
#define UART_BAUD_ABREN_Pos			23		//Auto Baudrate Enable，写1启动自动波特率校准，完成后自动清零
#define UART_BAUD_ABREN_Msk			(0x01 << UART_BAUD_ABREN_Pos)
#define UART_BAUD_ABRBIT_Pos		24		//Auto Baudrate Bit，用于计算波特率的检测位长，0 1位，通过测起始位           脉宽计算波特率，要求发送端发送0xFF
											//                                             1 2位，通过测起始位加1位数据位脉宽计算波特率，要求发送端发送0xFE
											//                                             1 4位，通过测起始位加3位数据位脉宽计算波特率，要求发送端发送0xF8
											//                                             1 8位，通过测起始位加7位数据位脉宽计算波特率，要求发送端发送0x80
#define UART_BAUD_ABRBIT_Msk		(0x03 << UART_BAUD_ABRBIT_Pos)
#define UART_BAUD_ABRERR_Pos		26		//Auto Baudrate Error，0 自动波特率校准成功     1 自动波特率校准失败
#define UART_BAUD_ABRERR_Msk		(0x01 << UART_BAUD_ABRERR_Pos)
#define UART_BAUD_TXDOIF_Pos		27		//TX Done 中断标志，发送FIFO空且发送发送移位寄存器已将最后一位发送出去
#define UART_BAUD_TXDOIF_Msk		(0x01 << UART_BAUD_TXDOIF_Pos)
#define UART_BAUD_FRAC_Pos			28		//波特率分频值小数部分
#define UART_BAUD_FRAC_Msk			(0x0Fu << UART_BAUD_FRAC_Pos)

#define UART_FIFO_RXLVL_Pos			0		//RX FIFO Level，RX FIFO 中字符个数
#define UART_FIFO_RXLVL_Msk			(0xFF << UART_FIFO_RXLVL_Pos)
#define UART_FIFO_TXLVL_Pos			8		//TX FIFO Level，TX FIFO 中字符个数
#define UART_FIFO_TXLVL_Msk			(0xFF << UART_FIFO_TXLVL_Pos)
#define UART_FIFO_RXTHR_Pos			16		//RX FIFO Threshold，RX中断触发门限，中断使能时 RXLVL >  RXTHR 触发RX中断
#define UART_FIFO_RXTHR_Msk			(0xFF << UART_FIFO_RXTHR_Pos)
#define UART_FIFO_TXTHR_Pos			24		//TX FIFO Threshold，TX中断触发门限，中断使能时 TXLVL <= TXTHR 触发TX中断
#define UART_FIFO_TXTHR_Msk			(0xFFu<< UART_FIFO_TXTHR_Pos)

#define UART_LINCR_BRKDETIE_Pos		0		//检测到LIN Break中断使能
#define UART_LINCR_BRKDETIE_Msk		(0x01 << UART_LINCR_BRKDETIE_Pos)
#define UART_LINCR_BRKDETIF_Pos		1		//检测到LIN Break中断状态
#define UART_LINCR_BRKDETIF_Msk		(0x01 << UART_LINCR_BRKDETIF_Pos)
#define UART_LINCR_GENBRKIE_Pos		2		//发送LIN Break完成中断使能
#define UART_LINCR_GENBRKIE_Msk		(0x01 << UART_LINCR_GENBRKIE_Pos)
#define UART_LINCR_GENBRKIF_Pos		3		//发送LIN Break完成中断状态
#define UART_LINCR_GENBRKIF_Msk		(0x01 << UART_LINCR_GENBRKIF_Pos)
#define UART_LINCR_GENBRK_Pos		4		//发送LIN Break，发送完成自动清零
#define UART_LINCR_GENBRK_Msk		(0x01 << UART_LINCR_GENBRK_Pos)

#define UART_CTSCR_EN_Pos			0		//CTS流控使能
#define UART_CTSCR_EN_Msk			(0x01 << UART_CTSCR_EN_Pos)
#define UART_CTSCR_POL_Pos			2		//CTS信号极性，0 低有效，CTS输入为低表示可以发送数据
#define UART_CTSCR_POL_Msk			(0x01 << UART_CTSCR_POL_Pos)
#define UART_CTSCR_STAT_Pos			7		//CTS信号的当前状态
#define UART_CTSCR_STAT_Msk			(0x01 << UART_CTSCR_STAT_Pos)

#define UART_RTSCR_EN_Pos			1		//RTS流控使能
#define UART_RTSCR_EN_Msk			(0x01 << UART_RTSCR_EN_Pos)
#define UART_RTSCR_POL_Pos			3		//RTS信号极性    0 低有效，RTS输入为低表示可以接收数据
#define UART_RTSCR_POL_Msk			(0x01 << UART_RTSCR_POL_Pos)
#define UART_RTSCR_THR_Pos			4		//RTS流控的触发阈值    0 1字节    1 2字节    2 4字节    3 6字节
#define UART_RTSCR_THR_Msk			(0x07 << UART_RTSCR_THR_Pos)
#define UART_RTSCR_STAT_Pos			8		//RTS信号的当前状态
#define UART_RTSCR_STAT_Msk			(0x01 << UART_RTSCR_STAT_Pos)

#define UART_CFG_MSBF_Pos			1		//接收发送MSB First
#define UART_CFG_MSBF_Msk			(0x01 << UART_CFG_MSBF_Pos)
#define UART_CFG_BRKTXLEN_Pos		2		//1表示1bit，以此类推，默认值13
#define UART_CFG_BRKTXLEN_Msk		(0x0F << UART_CFG_BRKTXLEN_Pos)
#define UART_CFG_BRKRXLEN_Pos		6		//0表示1bit，以此类推，默认值12
#define UART_CFG_BRKRXLEN_Msk		(0x0F << UART_CFG_BRKRXLEN_Pos)
#define UART_CFG_RXINV_Pos			10		//接收电平翻转
#define UART_CFG_RXINV_Msk			(0x01 << UART_CFG_RXINV_Pos)
#define UART_CFG_TXINV_Pos			11		//发送电平翻转
#define UART_CFG_TXINV_Msk			(0x01 << UART_CFG_TXINV_Pos)

#define UART_TOCR_TIME_Pos			0		//超时时间长度，单位为 10/BAUDRATE 秒
#define UART_TOCR_TIME_Msk			(0xFFF<< UART_TOCR_TIME_Pos)
#define UART_TOCR_MODE_Pos			12		//0 只有当FIFO中有数时才触发超时中断    1 即使FIFO中没有数也可触发超时中断
#define UART_TOCR_MODE_Msk			(0x01 << UART_TOCR_MODE_Pos)
#define UART_TOCR_IFCLR_Pos			13		//TO Interrupt Flag Clear，写1清除超时中断标志
#define UART_TOCR_IFCLR_Msk			(0x01 << UART_TOCR_IFCLR_Pos)




typedef struct {
	__IO uint32_t CTRL;

	__IO uint32_t DATA;

	__IO uint32_t STAT;

	__IO uint32_t IE;

	__IO uint32_t IF;
	
	__IO uint32_t I2SCR;
	
	__IO uint32_t I2SPR;
	
	     uint32_t RESERVED;
	
	__IO uint32_t SPIMCR;					//SPI Flash Memory Interface Control Register
	
	__IO uint32_t SPIMAR;					//SPI Flash Memory Interface Address Register
} SPI_TypeDef;


#define SPI_CTRL_CLKDIV_Pos			0		//Clock Divider, SPI工作时钟 = SYS_Freq/pow(2, CLKDIV+2)
#define SPI_CTRL_CLKDIV_Msk			(0x07 << SPI_CTRL_CLKDIV_Pos)
#define SPI_CTRL_EN_Pos				3
#define SPI_CTRL_EN_Msk				(0x01 << SPI_CTRL_EN_Pos)
#define SPI_CTRL_SIZE_Pos			4		//Data Size Select, 取值3--15，表示4--16位
#define SPI_CTRL_SIZE_Msk			(0x0F << SPI_CTRL_SIZE_Pos)
#define SPI_CTRL_CPHA_Pos			8		//0 在SCLK的第一个跳变沿采样数据	1 在SCLK的第二个跳变沿采样数据
#define SPI_CTRL_CPHA_Msk			(0x01 << SPI_CTRL_CPHA_Pos)
#define SPI_CTRL_CPOL_Pos			9		//0 空闲状态下SCLK为低电平		  1 空闲状态下SCLK为高电平
#define SPI_CTRL_CPOL_Msk			(0x01 << SPI_CTRL_CPOL_Pos)
#define SPI_CTRL_FFS_Pos			10		//Frame Format Select, 0 SPI	1 TI SSI	2 I2S	3 SPI Flash
#define SPI_CTRL_FFS_Msk			(0x03 << SPI_CTRL_FFS_Pos)
#define SPI_CTRL_MSTR_Pos			12		//Master, 1 主模式	0 从模式
#define SPI_CTRL_MSTR_Msk			(0x01 << SPI_CTRL_MSTR_Pos)
#define SPI_CTRL_FAST_Pos			13		//1 SPI工作时钟 = SYS_Freq/2    0 SPI工作时钟由SPI->CTRL.CLKDIV设置
#define SPI_CTRL_FAST_Msk			(0x01 << SPI_CTRL_FAST_Pos)
#define SPI_CTRL_DMATXEN_Pos		14		//1 通过DMA写FIFO    0 通过MCU写FIFO
#define SPI_CTRL_DMATXEN_Msk		(0x01 << SPI_CTRL_DMATXEN_Pos)
#define SPI_CTRL_DMARXEN_Pos		15		//1 通过DMA读FIFO    0 通过MCU读FIFO
#define SPI_CTRL_DMARXEN_Msk		(0x01 << SPI_CTRL_DMARXEN_Pos)
#define SPI_CTRL_FILTE_Pos			16		//1 对SPI输入信号进行去抖操作    0 对SPI输入信号不进行去抖操作
#define SPI_CTRL_FILTE_Msk			(0x01 << SPI_CTRL_FILTE_Pos)
#define SPI_CTRL_SSN_H_Pos			17		//0 传输过程中SSN始终为0    	 1 传输过程中每字符之间会将SSN拉高半个SCLK周期
#define SPI_CTRL_SSN_H_Msk			(0x01 << SPI_CTRL_SSN_H_Pos)
#define SPI_CTRL_RFTHR_Pos			18		//RX FIFO Threshold，0 接收FIFO中至少有1个数据   ...   7 接收FIFO中至少有8个数据
#define SPI_CTRL_RFTHR_Msk			(0x07 << SPI_CTRL_RFTHR_Pos)
#define SPI_CTRL_TFTHR_Pos			21		//TX FIFO Threshold，0 发送FIFO中至多有0个数据   ...   7 发送FIFO中至多有7个数据
#define SPI_CTRL_TFTHR_Msk			(0x07 << SPI_CTRL_TFTHR_Pos)
#define SPI_CTRL_RFCLR_Pos			24		//RX FIFO Clear
#define SPI_CTRL_RFCLR_Msk			(0x01 << SPI_CTRL_RFCLR_Pos)
#define SPI_CTRL_TFCLR_Pos			25		//TX FIFO Clear
#define SPI_CTRL_TFCLR_Msk			(0x01 << SPI_CTRL_TFCLR_Pos)
#define SPI_CTRL_LSBF_Pos			28		//LSB Fisrt
#define SPI_CTRL_LSBF_Msk			(0x01 << SPI_CTRL_LSBF_Pos)
#define SPI_CTRL_NSYNC_Pos			29		//1 对SPI输入信号进行采样同步    0 对SPI输入信号不进行采样同步
#define SPI_CTRL_NSYNC_Msk			(0x01 << SPI_CTRL_NSYNC_Pos)

#define SPI_STAT_WTC_Pos			0		//Word Transmit Complete，每传输完成一个数据字由硬件置1，软件写1清零
#define SPI_STAT_WTC_Msk			(0x01 << SPI_STAT_WTC_Pos)
#define SPI_STAT_TFE_Pos			1		//发送FIFO Empty
#define SPI_STAT_TFE_Msk			(0x01 << SPI_STAT_TFE_Pos)
#define SPI_STAT_TFNF_Pos			2		//发送FIFO Not Full
#define SPI_STAT_TFNF_Msk			(0x01 << SPI_STAT_TFNF_Pos)
#define SPI_STAT_RFNE_Pos			3		//接收FIFO Not Empty
#define SPI_STAT_RFNE_Msk			(0x01 << SPI_STAT_RFNE_Pos)
#define SPI_STAT_RFF_Pos			4		//接收FIFO Full
#define SPI_STAT_RFF_Msk			(0x01 << SPI_STAT_RFF_Pos)
#define SPI_STAT_RFOV_Pos			5		//接收FIFO Overflow
#define SPI_STAT_RFOV_Msk			(0x01 << SPI_STAT_RFOV_Pos)
#define SPI_STAT_TFLVL_Pos			6		//发送FIFO中数据个数， 0 TFNF=0时表示FIFO内有8个数据，TFNF=1时表示FIFO内有0个数据	1--7 FIFO内有1--7个数据
#define SPI_STAT_TFLVL_Msk			(0x07 << SPI_STAT_TFLVL_Pos)
#define SPI_STAT_RFLVL_Pos			9		//接收FIFO中数据个数， 0 RFF =1时表示FIFO内有8个数据，RFF =0时表示FIFO内有0个数据	1--7 FIFO内有1--7个数据
#define SPI_STAT_RFLVL_Msk			(0x07 << SPI_STAT_RFLVL_Pos)
#define SPI_STAT_BUSY_Pos			15
#define SPI_STAT_BUSY_Msk			(0x01 << SPI_STAT_BUSY_Pos)

#define SPI_IE_RFOV_Pos				0
#define SPI_IE_RFOV_Msk				(0x01 << SPI_IE_RFOV_Pos)
#define SPI_IE_RFF_Pos				1
#define SPI_IE_RFF_Msk				(0x01 << SPI_IE_RFF_Pos)
#define SPI_IE_RFHF_Pos				2
#define SPI_IE_RFHF_Msk				(0x01 << SPI_IE_RFHF_Pos)
#define SPI_IE_TFE_Pos				3
#define SPI_IE_TFE_Msk				(0x01 << SPI_IE_TFE_Pos)
#define SPI_IE_TFHF_Pos				4		//发送FIFO中数据个数大于4
#define SPI_IE_TFHF_Msk				(0x01 << SPI_IE_TFHF_Pos)
#define SPI_IE_RFTHR_Pos			5		//接收FIFO中数据个数大于CTRL.RFTHR设定值中断使能
#define SPI_IE_RFTHR_Msk			(0x01 << SPI_IE_RFTHR_Pos)
#define SPI_IE_TFTHR_Pos			6		//发送FIFO中数据个数小于CTRL.TFTHR设定值中断使能
#define SPI_IE_TFTHR_Msk			(0x01 << SPI_IE_TFTHR_Pos)
#define SPI_IE_WTC_Pos				8		//Word Transmit Complete
#define SPI_IE_WTC_Msk				(0x01 << SPI_IE_WTC_Pos)
#define SPI_IE_FTC_Pos				9		//Frame Transmit Complete
#define SPI_IE_FTC_Msk				(0x01 << SPI_IE_FTC_Pos)

#define SPI_IF_RFOV_Pos				0		//写1清零
#define SPI_IF_RFOV_Msk				(0x01 << SPI_IF_RFOV_Pos)
#define SPI_IF_RFF_Pos				1		//写1清零
#define SPI_IF_RFF_Msk				(0x01 << SPI_IF_RFF_Pos)
#define SPI_IF_RFHF_Pos				2		//写1清零
#define SPI_IF_RFHF_Msk				(0x01 << SPI_IF_RFHF_Pos)
#define SPI_IF_TFE_Pos				3		//写1清零
#define SPI_IF_TFE_Msk				(0x01 << SPI_IF_TFE_Pos)
#define SPI_IF_TFHF_Pos				4		//写1清零
#define SPI_IF_TFHF_Msk				(0x01 << SPI_IF_TFHF_Pos)
#define SPI_IF_RFTHR_Pos			5		//写1清零
#define SPI_IF_RFTHR_Msk			(0x01 << SPI_IF_RFTHR_Pos)
#define SPI_IF_TFTHR_Pos			6		//写1清零
#define SPI_IF_TFTHR_Msk			(0x01 << SPI_IF_TFTHR_Pos)
#define SPI_IF_WTC_Pos				8		//Word Transmit Complete，每传输完成一个数据字由硬件置1
#define SPI_IF_WTC_Msk				(0x01 << SPI_IF_WTC_Pos)
#define SPI_IF_FTC_Pos				9		//Frame Transmit Complete，WTC置位时若TX FIFO是空的，则FTC置位
#define SPI_IF_FTC_Msk				(0x01 << SPI_IF_FTC_Pos)

#define SPI_I2SCR_DIEN_Pos			0		//Data Input Enable
#define SPI_I2SCR_DIEN_Msk			(0x01 << SPI_I2SCR_DIEN_Pos)
#define SPI_I2SCR_DOEN_Pos			1		//Data Output Enable
#define SPI_I2SCR_DOEN_Msk			(0x01 << SPI_I2SCR_DOEN_Pos)
#define SPI_I2SCR_MSTR_Pos			2		//Master Mode
#define SPI_I2SCR_MSTR_Msk			(0x01 << SPI_I2SCR_MSTR_Pos)
#define SPI_I2SCR_EN_Pos			3
#define SPI_I2SCR_EN_Msk			(0x01 << SPI_I2SCR_EN_Pos)
#define SPI_I2SCR_FFMT_Pos			4		//I2S Frame Format, 0 I2S philips   1 MSB justified   2 PCM Short   3 PCM Long
#define SPI_I2SCR_FFMT_Msk			(0x03 << SPI_I2SCR_FFMT_Pos)
#define SPI_I2SCR_DLEN_Pos			6		//I2S Data Length,  0 8位   1 16位   2 24位   3 32位
#define SPI_I2SCR_DLEN_Msk			(0x03 << SPI_I2SCR_DLEN_Pos)
#define SPI_I2SCR_PCMSYNW_Pos		8		//I2S PCM Long Mode Sync Width, 0 1 SCLK period   1 1 Data Length
#define SPI_I2SCR_PCMSYNW_Msk		(0x01 << SPI_I2SCR_PCMSYNW_Pos)
#define SPI_I2SCR_MCLKOE_Pos		9		//MCLK Output Enable
#define SPI_I2SCR_MCLKOE_Msk		(0x01 << SPI_I2SCR_MCLKOE_Pos)
#define SPI_I2SCR_CHLEN_Pos			10		//声道宽度，0 16位   1 32位
#define SPI_I2SCR_CHLEN_Msk			(0x01 << SPI_I2SCR_CHLEN_Pos)
#define SPI_I2SCR_CHRIGHT_Pos		16		//1 Right Channel   0 Left Channel
#define SPI_I2SCR_CHRIGHT_Msk		(0x01 << SPI_I2SCR_CHRIGHT_Pos)

#define SPI_I2SPR_MCLKDIV_Pos		0		//Fmclk = Fpclk / (2 * (MCLKDIV + 1))，MCLK一般是SCLK的256或384倍
#define SPI_I2SPR_MCLKDIV_Msk		(0x3F << SPI_I2SPR_MCLKDIV_Pos)
#define SPI_I2SPR_SCLKDIV_Pos		8		//Fsclk = Fpclk / (2 * (SCLKDIV + 1))
#define SPI_I2SPR_SCLKDIV_Msk		(0xFFF<< SPI_I2SPR_SCLKDIV_Pos)

#define SPI_SPIMCR_DUMMY_Pos		0		//Dummy Clock，0表示1个
#define SPI_SPIMCR_DUMMY_Msk		(0x0F << SPI_SPIMCR_DUMMY_Pos)
#define SPI_SPIMCR_EN_Pos			4
#define SPI_SPIMCR_EN_Msk			(0x01 << SPI_SPIMCR_EN_Pos)
#define SPI_SPIMCR_RDLEN_Pos		8		//Recive Data bytes len, 0表示1个
#define SPI_SPIMCR_RDLEN_Msk		(0xFF << SPI_SPIMCR_RDLEN_Pos)




typedef struct {
	__IO uint32_t CR;

	__IO uint32_t SR;

	__IO uint32_t TR;						//Transfer Register

	__IO uint32_t RXDATA;
	
	__IO uint32_t TXDATA;
	
	__IO uint32_t IF;
	
	__IO uint32_t IE;
	
		 uint32_t RESERVED1;
	
	__IO uint32_t MCR;						//Master Control Register
	
	__IO uint32_t CLK;
	
		 uint32_t RESERVED2[2];
	
	__IO uint32_t SCR;						//Slave Control Register
	
	__IO uint32_t SADDR;
} I2C_TypeDef;


#define I2C_CR_EN_Pos				0	
#define I2C_CR_EN_Msk				(0x01 << I2C_CR_EN_Pos)
#define I2C_CR_MASTER_Pos			1		//1 Master   0 Slave
#define I2C_CR_MASTER_Msk			(0x01 << I2C_CR_MASTER_Pos)
#define I2C_CR_HS_Pos				2		//1 High-Speed mode    0 Standard-mode or Fast-mode
#define I2C_CR_HS_Msk				(0x01 << I2C_CR_HS_Pos)
#define I2C_CR_DNF_Pos				3		//Digital Noise Filter, 宽度低于 DNF+1 个的电平被认为是毛刺
#define I2C_CR_DNF_Msk				(0x0F << I2C_CR_DNF_Pos)

#define I2C_SR_BUSY_Pos				0
#define I2C_SR_BUSY_Msk				(0x01 << I2C_SR_BUSY_Pos)
#define I2C_SR_SCL_Pos				1		//SCL Line Level
#define I2C_SR_SCL_Msk				(0x01 << I2C_SR_SCL_Pos)
#define I2C_SR_SDA_Pos				2		//SDA Line Level
#define I2C_SR_SDA_Msk				(0x01 << I2C_SR_SDA_Pos)

#define I2C_TR_TXACK_Pos			0		//作为接收时，反馈ACK位的电平值
#define I2C_TR_TXACK_Msk			(0x01 << I2C_TR_TXACK_Pos)
#define I2C_TR_RXACK_Pos			1		//作为发送时，接收到的ACK位电平值
#define I2C_TR_RXACK_Msk			(0x01 << I2C_TR_RXACK_Pos)
#define I2C_TR_TXCLR_Pos			2		//TX Data Clear, 自动清零
#define I2C_TR_TXCLR_Msk			(0x01 << I2C_TR_TXCLR_Pos)
#define I2C_TR_SLVACT_Pos			8		//Slave Active, 从机模式下被选中时置位
#define I2C_TR_SLVACT_Msk			(0x01 << I2C_TR_SLVACT_Pos)
#define I2C_TR_SLVRD_Pos			9		//Slave Read mode，从机模式下接收到读请求时置位
#define I2C_TR_SLVRD_Msk			(0x01 << I2C_TR_SLVRD_Pos)
#define I2C_TR_SLVWR_Pos			10		//Slave Write mode，从机模式下接收到写请求时置位
#define I2C_TR_SLVWR_Msk			(0x01 << I2C_TR_SLVWR_Pos)
#define I2C_TR_SLVSTR_Pos			11		//Slave clock stretching
#define I2C_TR_SLVSTR_Msk			(0x01 << I2C_TR_SLVSTR_Pos)
#define I2C_TR_SLVRDS_Pos			12		//Slave RXDATA Status, 0 空   1 接收到地址   2 接收到数据   3 接收到Master Code
#define I2C_TR_SLVRDS_Msk			(0x03 << I2C_TR_SLVRDS_Pos)

#define I2C_IF_TXE_Pos				0		//TX Empty，写TXDATA清零此位
#define I2C_IF_TXE_Msk				(0x01 << I2C_IF_TXE_Pos)
#define I2C_IF_RXNE_Pos				1		//RX Not Empty，读RXDATA清零此位
#define I2C_IF_RXNE_Msk				(0x01 << I2C_IF_RXNE_Pos)
#define I2C_IF_RXOV_Pos				2		//RX Overflow，写1清零
#define I2C_IF_RXOV_Msk				(0x01 << I2C_IF_RXOV_Pos)
#define I2C_IF_TXDONE_Pos			3		//TX Done，写1清零
#define I2C_IF_TXDONE_Msk			(0x01 << I2C_IF_TXDONE_Pos)
#define I2C_IF_RXDONE_Pos			4		//RX Done，写1清零
#define I2C_IF_RXDONE_Msk			(0x01 << I2C_IF_RXDONE_Pos)
#define I2C_IF_RXSTA_Pos			8		//从机接收到起始位，写1清零
#define I2C_IF_RXSTA_Msk			(0x01 << I2C_IF_RXSTA_Pos)
#define I2C_IF_RXSTO_Pos			9		//从机接收到停止位，写1清零
#define I2C_IF_RXSTO_Msk			(0x01 << I2C_IF_RXSTO_Pos)
#define I2C_IF_AL_Pos				16		//主机仲裁丢失总线，写1清零
#define I2C_IF_AL_Msk				(0x01 << I2C_IF_AL_Pos)
#define I2C_IF_MLTO_Pos				17		//Master SCL Low Timeout，写1清零
#define I2C_IF_MLTO_Msk				(0x01 << I2C_IF_MLTO_Pos)

#define I2C_IE_TXE_Pos				0
#define I2C_IE_TXE_Msk				(0x01 << I2C_IE_TXE_Pos)
#define I2C_IE_RXNE_Pos				1
#define I2C_IE_RXNE_Msk				(0x01 << I2C_IE_RXNE_Pos)
#define I2C_IE_RXOV_Pos				2
#define I2C_IE_RXOV_Msk				(0x01 << I2C_IE_RXOV_Pos)
#define I2C_IE_TXDONE_Pos			3
#define I2C_IE_TXDONE_Msk			(0x01 << I2C_IE_TXDONE_Pos)
#define I2C_IE_RXDONE_Pos			4
#define I2C_IE_RXDONE_Msk			(0x01 << I2C_IE_RXDONE_Pos)
#define I2C_IE_RXSTA_Pos			8
#define I2C_IE_RXSTA_Msk			(0x01 << I2C_IE_RXSTA_Pos)
#define I2C_IE_RXSTO_Pos			9
#define I2C_IE_RXSTO_Msk			(0x01 << I2C_IE_RXSTO_Pos)
#define I2C_IE_AL_Pos				16
#define I2C_IE_AL_Msk				(0x01 << I2C_IE_AL_Pos)
#define I2C_IE_MLTO_Pos				17
#define I2C_IE_MLTO_Msk				(0x01 << I2C_IE_MLTO_Pos)

#define I2C_MCR_STA_Pos				0		//写1产生起始位，完成后自动清零
#define I2C_MCR_STA_Msk				(0x01 << I2C_MCR_STA_Pos)
#define I2C_MCR_RD_Pos				1
#define I2C_MCR_RD_Msk				(0x01 << I2C_MCR_RD_Pos)
#define I2C_MCR_WR_Pos				2
#define I2C_MCR_WR_Msk				(0x01 << I2C_MCR_WR_Pos)
#define I2C_MCR_STO_Pos				3		//写1产生停止位，完成后自动清零
#define I2C_MCR_STO_Msk				(0x01 << I2C_MCR_STO_Pos)

#define I2C_CLK_SCLL_Pos			0		//SCL Low Time
#define I2C_CLK_SCLL_Msk			(0xFF << I2C_CLK_SCLL_Pos)
#define I2C_CLK_SCLH_Pos			8		//SCL High Time
#define I2C_CLK_SCLH_Msk			(0xFF << I2C_CLK_SCLH_Pos)
#define I2C_CLK_DIV_Pos				16
#define I2C_CLK_DIV_Msk				(0xFF << I2C_CLK_DIV_Pos)
#define I2C_CLK_SDAH_Pos			24		//SDA Hold Time
#define I2C_CLK_SDAH_Msk			(0x0F << I2C_CLK_SDAH_Pos)

#define I2C_SCR_ADDR10_Pos			0		//1 10位地址    0 7位地址
#define I2C_SCR_ADDR10_Msk			(0x01 << I2C_SCR_ADDR10_Pos)
#define I2C_SCR_MCDE_Pos			1		//Master Code Detect Enable
#define I2C_SCR_MCDE_Msk			(0x01 << I2C_SCR_MCDE_Pos)
#define I2C_SCR_STRE_Pos			2		//Clock Stretching Enable
#define I2C_SCR_STRE_Msk			(0x01 << I2C_SCR_STRE_Pos)
#define I2C_SCR_ASDS_Pos			3		//Adaptive Stretching Data Setup
#define I2C_SCR_ASDS_Msk			(0x01 << I2C_SCR_ASDS_Pos)

#define I2C_SADDR_ADDR7_Pos			1		//7位地址模式下的地址
#define I2C_SADDR_ADDR7_Msk			(0x7F << I2C_SADDR_ADDR7_Pos)
#define I2C_SADDR_ADDR10_Pos		0		//10位地址模式下的地址
#define I2C_SADDR_ADDR10_Msk		(0x3FF<< I2C_SADDR_ADDR10_Pos)
#define I2C_SADDR_MASK7_Pos			17		//7位地址模式下的地址掩码，ADDR7 & (~MASK7) 后与接收地址比较
#define I2C_SADDR_MASK7_Msk			(0x7F << I2C_SADDR_MASK7_Pos)
#define I2C_SADDR_MASK10_Pos		16		//10位地址模式下的地址掩码，只掩码低8位
#define I2C_SADDR_MASK10_Msk		(0xFF << I2C_SADDR_MASK10_Pos)




typedef struct {
	__IO uint32_t CTRL;
	
	__IO uint32_t START;
	
	__IO uint32_t IE;
	
	__IO uint32_t IF;
	
	struct {
		__IO uint32_t STAT;
		
		__IO uint32_t DATA;
		
			 uint32_t RESERVED[2];
	} CH[12];
	
	__IO uint32_t CHSEL;					//ADC->CTRL.CH = PWM_Trigger ? ADC->CHSEL.PWM : ADC->CHSEL.SW
	
		 uint32_t RESERVED[47];
	
	__IO uint32_t FIFOSR;
	
	__IO uint32_t FIFODR;
	
		 uint32_t RESERVED2[2];
	
	__IO uint32_t CTRL2;
	
	__IO uint32_t CTRL3;
	
	__IO uint32_t CTRL4;
	
		 uint32_t RESERVED3;
    
	__IO uint32_t TRGMSK;					//对应位置1后，则相应通道触发ADC功能被屏蔽
	
		 uint32_t RESERVED4[16];
	
	__IO uint32_t CALIBSET;
	
	__IO uint32_t CALIBEN;
} ADC_TypeDef;


#define ADC_CTRL_EN_Pos				12
#define ADC_CTRL_EN_Msk				(0x01 << ADC_CTRL_EN_Pos)
#define ADC_CTRL_CONT_Pos			13		//Continuous conversion，只在软件启动模式下有效，0 单次转换，转换完成后START位自动清除停止转换
#define ADC_CTRL_CONT_Msk			(0x01 << ADC_CTRL_CONT_Pos)							//   1 连续转换，启动后一直采样、转换，直到软件清除START位
#define ADC_CTRL_TRIG_Pos			14		//转换触发方式：4 软件启动转换	  5 PWM触发   6 TIMR0   7 TIMR1
#define ADC_CTRL_TRIG_Msk			(0x07 << ADC_CTRL_TRIG_Pos)
#define ADC_CTRL_RES2FIFO_Pos		18		//0 转换结果存储通道数据寄存器   1 转换结果存入FIFO，DMA时必须选此模式
#define ADC_CTRL_RES2FIFO_Msk		(0x01 << ADC_CTRL_RES2FIFO_Pos)
#define ADC_CTRL_FIFOCLR_Pos		19
#define ADC_CTRL_FIFOCLR_Msk		(0x01 << ADC_CTRL_FIFOCLR_Pos)
#define ADC_CTRL_RESET_Pos			20
#define ADC_CTRL_RESET_Msk			(0x01 << ADC_CTRL_RESET_Pos)
#define ADC_CTRL_AVG_Pos			21		//0 1次采样	  1 2次采样取平均值	  3 4次采样取平均值	  7 8次采样取平均值	  15 16次采样取平均值
#define ADC_CTRL_AVG_Msk			(0x0F << ADC_CTRL_AVG_Pos)

#define ADC_START_GO_Pos			0		//软件触发模式下，写1启动ADC采样和转换，在单次模式下转换完成后硬件自动清零，在扫描模式下必须软件写0停止ADC转换
#define ADC_START_GO_Msk			(0x01 << ADC_START_GO_Pos)
#define ADC_START_BUSY_Pos			4
#define ADC_START_BUSY_Msk			(0x01 << ADC_START_BUSY_Pos)

#define ADC_IE_CH0EOC_Pos			0		//End Of Convertion
#define ADC_IE_CH0EOC_Msk			(0x01 << ADC_IE_CH0EOC_Pos)
#define ADC_IE_CH0OVF_Pos			1		//Overflow
#define ADC_IE_CH0OVF_Msk			(0x01 << ADC_IE_CH0OVF_Pos)
#define ADC_IE_CH1EOC_Pos			2
#define ADC_IE_CH1EOC_Msk			(0x01 << ADC_IE_CH1EOC_Pos)
#define ADC_IE_CH1OVF_Pos			3
#define ADC_IE_CH1OVF_Msk			(0x01 << ADC_IE_CH1OVF_Pos)
#define ADC_IE_CH2EOC_Pos			4
#define ADC_IE_CH2EOC_Msk			(0x01 << ADC_IE_CH2EOC_Pos)
#define ADC_IE_CH2OVF_Pos			5
#define ADC_IE_CH2OVF_Msk			(0x01 << ADC_IE_CH2OVF_Pos)
#define ADC_IE_CH3EOC_Pos			6
#define ADC_IE_CH3EOC_Msk			(0x01 << ADC_IE_CH3EOC_Pos)
#define ADC_IE_CH3OVF_Pos			7
#define ADC_IE_CH3OVF_Msk			(0x01 << ADC_IE_CH3OVF_Pos)
#define ADC_IE_CH4EOC_Pos			8
#define ADC_IE_CH4EOC_Msk			(0x01 << ADC_IE_CH4EOC_Pos)
#define ADC_IE_CH4OVF_Pos			9
#define ADC_IE_CH4OVF_Msk			(0x01 << ADC_IE_CH4OVF_Pos)
#define ADC_IE_CH5EOC_Pos			10
#define ADC_IE_CH5EOC_Msk			(0x01 << ADC_IE_CH5EOC_Pos)
#define ADC_IE_CH5OVF_Pos			11
#define ADC_IE_CH5OVF_Msk			(0x01 << ADC_IE_CH5OVF_Pos)
#define ADC_IE_CH6EOC_Pos			12
#define ADC_IE_CH6EOC_Msk			(0x01 << ADC_IE_CH6EOC_Pos)
#define ADC_IE_CH6OVF_Pos			13
#define ADC_IE_CH6OVF_Msk			(0x01 << ADC_IE_CH6OVF_Pos)
#define ADC_IE_CH7EOC_Pos			14
#define ADC_IE_CH7EOC_Msk			(0x01 << ADC_IE_CH7EOC_Pos)
#define ADC_IE_CH7OVF_Pos			15
#define ADC_IE_CH7OVF_Msk			(0x01 << ADC_IE_CH7OVF_Pos)
#define ADC_IE_CH8EOC_Pos			16
#define ADC_IE_CH8EOC_Msk			(0x01 << ADC_IE_CH8EOC_Pos)
#define ADC_IE_CH8OVF_Pos			17
#define ADC_IE_CH8OVF_Msk			(0x01 << ADC_IE_CH8OVF_Pos)
#define ADC_IE_CH9EOC_Pos			18
#define ADC_IE_CH9EOC_Msk			(0x01 << ADC_IE_CH9EOC_Pos)
#define ADC_IE_CH9OVF_Pos			19
#define ADC_IE_CH9OVF_Msk			(0x01 << ADC_IE_CH9OVF_Pos)
#define ADC_IE_CH10EOC_Pos			20
#define ADC_IE_CH10EOC_Msk			(0x01 << ADC_IE_CH10EOC_Pos)
#define ADC_IE_CH10OVF_Pos			21
#define ADC_IE_CH10OVF_Msk			(0x01 << ADC_IE_CH10OVF_Pos)
#define ADC_IE_CH11EOC_Pos			22
#define ADC_IE_CH11EOC_Msk			(0x01 << ADC_IE_CH11EOC_Pos)
#define ADC_IE_CH11OVF_Pos			23
#define ADC_IE_CH11OVF_Msk			(0x01 << ADC_IE_CH11OVF_Pos)
#define ADC_IE_FIFOOV_Pos			24
#define ADC_IE_FIFOOV_Msk			(0x01 << ADC_IE_FIFOOV_Pos)
#define ADC_IE_FIFOHF_Pos			25
#define ADC_IE_FIFOHF_Msk			(0x01 << ADC_IE_FIFOHF_Pos)
#define ADC_IE_FIFOF_Pos			26
#define ADC_IE_FIFOF_Msk			(0x01 << ADC_IE_FIFOF_Pos)

#define ADC_IF_CH0EOC_Pos			0		//写1清零
#define ADC_IF_CH0EOC_Msk			(0x01 << ADC_IF_CH0EOC_Pos)
#define ADC_IF_CH0OVF_Pos			1
#define ADC_IF_CH0OVF_Msk			(0x01 << ADC_IF_CH0OVF_Pos)
#define ADC_IF_CH1EOC_Pos			2
#define ADC_IF_CH1EOC_Msk			(0x01 << ADC_IF_CH1EOC_Pos)
#define ADC_IF_CH1OVF_Pos			3
#define ADC_IF_CH1OVF_Msk			(0x01 << ADC_IF_CH1OVF_Pos)
#define ADC_IF_CH2EOC_Pos			4
#define ADC_IF_CH2EOC_Msk			(0x01 << ADC_IF_CH2EOC_Pos)
#define ADC_IF_CH2OVF_Pos			5
#define ADC_IF_CH2OVF_Msk			(0x01 << ADC_IF_CH2OVF_Pos)
#define ADC_IF_CH3EOC_Pos			6
#define ADC_IF_CH3EOC_Msk			(0x01 << ADC_IF_CH3EOC_Pos)
#define ADC_IF_CH3OVF_Pos			7
#define ADC_IF_CH3OVF_Msk			(0x01 << ADC_IF_CH3OVF_Pos)
#define ADC_IF_CH4EOC_Pos			8
#define ADC_IF_CH4EOC_Msk			(0x01 << ADC_IF_CH4EOC_Pos)
#define ADC_IF_CH4OVF_Pos			9
#define ADC_IF_CH4OVF_Msk			(0x01 << ADC_IF_CH4OVF_Pos)
#define ADC_IF_CH5EOC_Pos			10
#define ADC_IF_CH5EOC_Msk			(0x01 << ADC_IF_CH5EOC_Pos)
#define ADC_IF_CH5OVF_Pos			11
#define ADC_IF_CH5OVF_Msk			(0x01 << ADC_IF_CH5OVF_Pos)
#define ADC_IF_CH6EOC_Pos			12
#define ADC_IF_CH6EOC_Msk			(0x01 << ADC_IF_CH6EOC_Pos)
#define ADC_IF_CH6OVF_Pos			13
#define ADC_IF_CH6OVF_Msk			(0x01 << ADC_IF_CH6OVF_Pos)
#define ADC_IF_CH7EOC_Pos			14
#define ADC_IF_CH7EOC_Msk			(0x01 << ADC_IF_CH7EOC_Pos)
#define ADC_IF_CH7OVF_Pos			15
#define ADC_IF_CH7OVF_Msk			(0x01 << ADC_IF_CH7OVF_Pos)
#define ADC_IF_CH8EOC_Pos			16
#define ADC_IF_CH8EOC_Msk			(0x01 << ADC_IF_CH8EOC_Pos)
#define ADC_IF_CH8OVF_Pos			17
#define ADC_IF_CH8OVF_Msk			(0x01 << ADC_IF_CH8OVF_Pos)
#define ADC_IF_CH9EOC_Pos			18
#define ADC_IF_CH9EOC_Msk			(0x01 << ADC_IF_CH9EOC_Pos)
#define ADC_IF_CH9OVF_Pos			19
#define ADC_IF_CH9OVF_Msk			(0x01 << ADC_IF_CH9OVF_Pos)
#define ADC_IF_CH10EOC_Pos			20
#define ADC_IF_CH10EOC_Msk			(0x01 << ADC_IF_CH10EOC_Pos)
#define ADC_IF_CH10OVF_Pos			21
#define ADC_IF_CH10OVF_Msk			(0x01 << ADC_IF_CH10OVF_Pos)
#define ADC_IF_CH11EOC_Pos			22
#define ADC_IF_CH11EOC_Msk			(0x01 << ADC_IF_CH11EOC_Pos)
#define ADC_IF_CH11OVF_Pos			23
#define ADC_IF_CH11OVF_Msk			(0x01 << ADC_IF_CH11OVF_Pos)
#define ADC_IF_FIFOOV_Pos			24
#define ADC_IF_FIFOOV_Msk			(0x01 << ADC_IF_FIFOOV_Pos)
#define ADC_IF_FIFOHF_Pos			25
#define ADC_IF_FIFOHF_Msk			(0x01 << ADC_IF_FIFOHF_Pos)
#define ADC_IF_FIFOF_Pos			26
#define ADC_IF_FIFOF_Msk			(0x01 << ADC_IF_FIFOF_Pos)

#define ADC_STAT_EOC_Pos			0		//写1清零
#define ADC_STAT_EOC_Msk			(0x01 << ADC_STAT_EOC_Pos)
#define ADC_STAT_OVF_Pos			1		//读数据寄存器清除
#define ADC_STAT_OVF_Msk			(0x01 << ADC_STAT_OVF_Pos)

#define ADC_DATA_VAL_Pos			0
#define ADC_DATA_VAL_Msk			(0xFFF<< ADC_DATA_VAL_Pos)
#define ADC_DATA_NUM_Pos			12
#define ADC_DATA_NUM_Msk			(0x0F << ADC_DATA_NUM_Pos)

#define ADC_CHSEL_SW_Pos			0		//软件启动转换时采样的通道
#define ADC_CHSEL_SW_Msk			(0xFFF<< ADC_CHSEL_SW_Pos)
#define ADC_CHSEL_PWM_Pos			16		//PWM 启动转换时采样的通道
#define ADC_CHSEL_PWM_Msk			(0xFFF<< ADC_CHSEL_PWM_Pos)

#define ADC_FIFOSR_OV_Pos			0
#define ADC_FIFOSR_OV_Msk			(0x01 << ADC_FIFOSR_OV_Pos)
#define ADC_FIFOSR_HF_Pos			1
#define ADC_FIFOSR_HF_Msk			(0x01 << ADC_FIFOSR_HF_Pos)
#define ADC_FIFOSR_FULL_Pos			2		//FIFO Full
#define ADC_FIFOSR_FULL_Msk			(0x01 << ADC_FIFOSR_FULL_Pos)
#define ADC_FIFOSR_EMPTY_Pos		3		//FIFO Empty
#define ADC_FIFOSR_EMPTY_Msk		(0x01 << ADC_FIFOSR_EMPTY_Pos)

#define ADC_FIFDR_VAL_Pos			0
#define ADC_FIFDR_VAL_Msk			(0xFFF<< ADC_FIFDR_VAL_Pos)
#define ADC_FIFDR_NUM_Pos			12
#define ADC_FIFDR_NUM_Msk			(0x07 << ADC_FIFDR_NUM_Pos)

#define ADC_CTRL2_CLKSEL_Pos		0		//0 由SYS->CLKSEL.ADC选择   1 外部晶振
#define ADC_CTRL2_CLKSEL_Msk		(0x01 << ADC_CTRL2_CLKSEL_Pos)
#define ADC_CTRL2_ADJH_Pos			8
#define ADC_CTRL2_ADJH_Msk			(0xFF << ADC_CTRL2_ADJH_Pos)
#define ADC_CTRL2_ADJL_Pos			16
#define ADC_CTRL2_ADJL_Msk			(0x0F << ADC_CTRL2_ADJL_Pos)

#define ADC_CTRL3_REFSEL_Pos		1		//Reference Select, 0 内部REFP   3 外部REFP
#define ADC_CTRL3_REFSEL_Msk		(0x03 << ADC_CTRL3_REFSEL_Pos)
#define ADC_CTRL3_IREFSEL_Pos		16		//Internal Reference Select, 7 3.6V
#define ADC_CTRL3_IREFSEL_Msk		(0x07 << ADC_CTRL3_IREFSEL_Pos)
#define ADC_CTRL3_CLKDIV2_Pos		24
#define ADC_CTRL3_CLKDIV2_Msk		(0x1F << ADC_CTRL3_CLKDIV2_Pos)
#define ADC_CTRL3_CLKDIV1_Pos		29
#define ADC_CTRL3_CLKDIV1_Msk		(0x03 << ADC_CTRL3_CLKDIV1_Pos)

#define ADC_CTRL4_EREFSEL_Pos		2		//External Reference Select, 0 Vrefp pin   1 VDD
#define ADC_CTRL4_EREFSEL_Msk		(0x01 << ADC_CTRL4_EREFSEL_Pos)

#define ADC_TRGMSK_PWM0_Pos			0
#define ADC_TRGMSK_PWM0_Msk			(0x01 << ADC_TRGMSK_PWM0_Pos)
#define ADC_TRGMSK_PWM1_Pos			2
#define ADC_TRGMSK_PWM1_Msk			(0x01 << ADC_TRGMSK_PWM1_Pos)

#define ADC_CALIBSET_OFFSET_Pos		0
#define ADC_CALIBSET_OFFSET_Msk		(0x1FF<< ADC_CALIBSET_OFFSET_Pos)
#define ADC_CALIBSET_K_Pos			16
#define ADC_CALIBSET_K_Msk			(0x1FF<< ADC_CALIBSET_K_Pos)

#define ADC_CALIBEN_OFFSET_Pos		0
#define ADC_CALIBEN_OFFSET_Msk		(0x01 << ADC_CALIBEN_OFFSET_Pos)
#define ADC_CALIBEN_K_Pos			1
#define ADC_CALIBEN_K_Msk			(0x01 << ADC_CALIBEN_K_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t OCR;
	
	__IO uint32_t BRKCR;
	
	__IO uint32_t BRKIN;
	
		 uint32_t RESERVED[4];
	
	__IO uint32_t PERIOD;                   //[15:0] 周期
	
	__IO uint32_t CMPA;                   	//[15:0] A路翻转点比较值
	
	__IO uint32_t CMPB;						//[15:0] B路翻转点比较值
	
	__IO uint32_t DZA;                      //[9:0] 死区
	
	__IO uint32_t DZB;
	
	__IO uint32_t CMPA2;					//非对称中心对齐模式下，向下计数过程中，A路翻转点比较值
	
	__IO uint32_t CMPB2;					//非对称中心对齐模式下，向下计数过程中，B路翻转点比较值
	
		 uint32_t RESERVED2[5];
	
	__IO uint32_t OVFTRG;
	
	__IO uint32_t CMPTRG;
	
	__IO uint32_t CMPTRG2;
	
		 uint32_t RESERVED3;
	
	__IO uint32_t EVMUX;
	
    __IO uint32_t EVMSK;
	
		 uint32_t RESERVED4[2];
	
	__IO uint32_t IE;
	
	__IO uint32_t IF;
	
	__IO uint32_t VALUE;
	
	__IO uint32_t SR;
} PWM_TypeDef;


#define PWM_CR_MODE_Pos				0		//0 边沿对齐模式   1 中心对齐模式   2 非对称中心对齐模式
#define PWM_CR_MODE_Msk				(0x03 << PWM_CR_MODE_Pos)
#define PWM_CR_MULT_Pos				2		//0 单次计数模式   1 多次计数模式
#define PWM_CR_MULT_Msk				(0x01 << PWM_CR_MULT_Pos)
#define PWM_CR_DIR_Pos				3		//计数器计数方向， 0 向上计数   1 向下计数
#define PWM_CR_DIR_Msk				(0x01 << PWM_CR_DIR_Pos)
#define PWM_CR_CLKSRC_Pos			4		//计数时钟源，0 系统时钟   1 PWM_PULSE0输入   2 PWM_PULSE1输入
#define PWM_CR_CLKSRC_Msk			(0x03 << PWM_CR_CLKSRC_Pos)
#define PWM_CR_CLKDIV_Pos			6		//计数时钟预分频， 0 1分频   1 2分频   ...   1023 1024分频
#define PWM_CR_CLKDIV_Msk			(0x3FF<< PWM_CR_CLKDIV_Pos)
#define PWM_CR_RPTNUM_Pos			16		//计数器溢出多少次执行一次寄存器重载，0 1次   1 2次   ...   255 256次
#define PWM_CR_RPTNUM_Msk			(0xFF << PWM_CR_RPTNUM_Pos)

#define PWM_OCR_IDLEA_Pos			0		//A路空闲时输出电平
#define PWM_OCR_IDLEA_Msk			(0x01 << PWM_OCR_IDLEA_Pos)
#define PWM_OCR_IDLEB_Pos			1		//B路空闲时输出电平
#define PWM_OCR_IDLEB_Msk			(0x01 << PWM_OCR_IDLEB_Pos)
#define PWM_OCR_IDLEAN_Pos			2		//AN路空闲时输出电平
#define PWM_OCR_IDLEAN_Msk			(0x01 << PWM_OCR_IDLEAN_Pos)
#define PWM_OCR_IDLEBN_Pos			3		//BN路空闲时输出电平
#define PWM_OCR_IDLEBN_Msk			(0x01 << PWM_OCR_IDLEBN_Pos)
#define PWM_OCR_INVA_Pos			4		//A路输出是否取反
#define PWM_OCR_INVA_Msk			(0x01 << PWM_OCR_INVA_Pos)
#define PWM_OCR_INVB_Pos			5		//B路输出是否取反
#define PWM_OCR_INVB_Msk			(0x01 << PWM_OCR_INVB_Pos)
#define PWM_OCR_INVAN_Pos			6		//AN路输出是否取反
#define PWM_OCR_INVAN_Msk			(0x01 << PWM_OCR_INVAN_Pos)
#define PWM_OCR_INVBN_Pos			7		//BN路输出是否取反
#define PWM_OCR_INVBN_Msk			(0x01 << PWM_OCR_INVBN_Pos)

#define PWM_BRKCR_OUTA_Pos			0		//刹车状态下A路输出电平
#define PWM_BRKCR_OUTA_Msk			(0x01 << PWM_BRKCR_OUTA_Pos)
#define PWM_BRKCR_OFFA_Pos			1		//刹车信号撤销时A路输出，0 立即恢复正常输出   1 保持当前输出直到计数器溢出再恢复正常输出
#define PWM_BRKCR_OFFA_Msk			(0x01 << PWM_BRKCR_OFFA_Pos)
#define PWM_BRKCR_OUTB_Pos			4		//刹车状态下B路输出电平
#define PWM_BRKCR_OUTB_Msk			(0x01 << PWM_BRKCR_OUTB_Pos)
#define PWM_BRKCR_OFFB_Pos			5		//刹车信号撤销时B路输出，0 立即恢复正常输出   1 保持当前输出直到计数器溢出再恢复正常输出
#define PWM_BRKCR_OFFB_Msk			(0x01 << PWM_BRKCR_OFFB_Pos)
#define PWM_BRKCR_OUTAN_Pos			8		//刹车状态下AN路输出电平
#define PWM_BRKCR_OUTAN_Msk			(0x01 << PWM_BRKCR_OUTAN_Pos)
#define PWM_BRKCR_OUTBN_Pos			9		//刹车状态下BN路输出电平
#define PWM_BRKCR_OUTBN_Msk			(0x01 << PWM_BRKCR_OUTBN_Pos)
#define PWM_BRKCR_STPCNT_Pos		10		//刹车状态下是否停止计数器，1 停止计数器   0 继续计数
#define PWM_BRKCR_STPCNT_Msk		(0x01 << PWM_BRKCR_STPCNT_Pos)
#define PWM_BRKCR_ACTIVE_Pos		17		//当前是否处于刹车状态
#define PWM_BRKCR_ACTIVE_Msk		(0x01 << PWM_BRKCR_ACTIVE_Pos)

#define PWM_BRKIN_BRK0A_Pos			0		//A路是否受刹车输入PWM_BRK0影响
#define PWM_BRKIN_BRK0A_Msk			(0x01 << PWM_BRKIN_BRK0A_Pos)
#define PWM_BRKIN_BRK1A_Pos			1
#define PWM_BRKIN_BRK1A_Msk			(0x01 << PWM_BRKIN_BRK1A_Pos)
#define PWM_BRKIN_BRK2A_Pos			2
#define PWM_BRKIN_BRK2A_Msk			(0x01 << PWM_BRKIN_BRK2A_Pos)
#define PWM_BRKIN_BRK0B_Pos			4
#define PWM_BRKIN_BRK0B_Msk			(0x01 << PWM_BRKIN_BRK0B_Pos)
#define PWM_BRKIN_BRK1B_Pos			5
#define PWM_BRKIN_BRK1B_Msk			(0x01 << PWM_BRKIN_BRK1B_Pos)
#define PWM_BRKIN_BRK2B_Pos			6
#define PWM_BRKIN_BRK2B_Msk			(0x01 << PWM_BRKIN_BRK2B_Pos)

#define PWM_OVFTRG_UPEN_Pos			0		//计数器向上溢出Trigger使能
#define PWM_OVFTRG_UPEN_Msk			(0x01 << PWM_OVFTRG_UPEN_Pos)
#define PWM_OVFTRG_DNEN_Pos			1		//计数器向下溢出Trigger使能
#define PWM_OVFTRG_DNEN_Msk			(0x01 << PWM_OVFTRG_DNEN_Pos)
#define PWM_OVFTRG_MUX_Pos			2		//Trigger输出到哪一路，0 trig[0]   1 trig[1]   2 trig[2]   ...   7 trig[7]
#define PWM_OVFTRG_MUX_Msk			(0x07 << PWM_OVFTRG_MUX_Pos)

#define PWM_CMPTRG_CMP_Pos			0		//计数器值与此比较值相等时产生Trigger信号
#define PWM_CMPTRG_CMP_Msk			(0xFFFF<<PWM_CMPTRG_CMP_Pos)
#define PWM_CMPTRG_EN_Pos			16
#define PWM_CMPTRG_EN_Msk			(0x01 << PWM_CMPTRG_EN_Pos)
#define PWM_CMPTRG_MUX_Pos			17		//Trigger输出到哪一路，0 trig[0]   1 trig[1]   2 trig[2]   ...   7 trig[7]
#define PWM_CMPTRG_MUX_Msk			(0x07 << PWM_CMPTRG_MUX_Pos)
#define PWM_CMPTRG_WIDTH_Pos		20		//Trigger输出信号宽度，0 无输出   1 4个计数时钟   2 8个计数时钟   ...   63 252个计数时钟
#define PWM_CMPTRG_WIDTH_Msk		(0x3F << PWM_CMPTRG_WIDTH_Pos)
#define PWM_CMPTRG_DIR_Pos			28		//0 向上计数过程中产生Trigger   1 向下计数过程中产生Trigger
#define PWM_CMPTRG_DIR_Msk			(0x01 << PWM_CMPTRG_DIR_Pos)
#define PWM_CMPTRG_ATP_Pos			29		//AD触发信号在所挖坑中的位置：0 0/8处   1 1/8处   ...   7 7/8处
#define PWM_CMPTRG_ATP_Msk			(0x07u<< PWM_CMPTRG_ATP_Pos)

#define PWM_CMPTRG2_INTV_Pos		0		//Compare Trigger Interval，0 每周期触发   1 间隔1周期触发一次   2 间隔2周期触发一次 ...
#define PWM_CMPTRG2_INTV_Msk		(0x07 << PWM_CMPTRG2_INTV_Pos)

#define PWM_EVMUX_START_Pos			0
#define PWM_EVMUX_START_Msk			(0x07 << PWM_EVMUX_START_Pos)
#define PWM_EVMUX_STOP_Pos			4
#define PWM_EVMUX_STOP_Msk			(0x07 << PWM_EVMUX_STOP_Pos)
#define PWM_EVMUX_PAUSE_Pos			8
#define PWM_EVMUX_PAUSE_Msk			(0x07 << PWM_EVMUX_PAUSE_Pos)
#define PWM_EVMUX_RELOAD_Pos		12
#define PWM_EVMUX_RELOAD_Msk		(0x07 << PWM_EVMUX_RELOAD_Pos)
#define PWM_EVMUX_MASKA_Pos			16
#define PWM_EVMUX_MASKA_Msk			(0x07 << PWM_EVMUX_MASKA_Pos)
#define PWM_EVMUX_MASKB_Pos			20
#define PWM_EVMUX_MASKB_Msk			(0x07 << PWM_EVMUX_MASKB_Pos)
#define PWM_EVMUX_MASKAN_Pos		24
#define PWM_EVMUX_MASKAN_Msk		(0x07 << PWM_EVMUX_MASKAN_Pos)
#define PWM_EVMUX_MASKBN_Pos		28
#define PWM_EVMUX_MASKBN_Msk		(0x07 << PWM_EVMUX_MASKBN_Pos)

#define PWM_EVMSK_OUTA_Pos			0
#define PWM_EVMSK_OUTA_Msk			(0x01 << PWM_EVMSK_OUTA_Pos)
#define PWM_EVMSK_OUTB_Pos			1
#define PWM_EVMSK_OUTB_Msk			(0x01 << PWM_EVMSK_OUTB_Pos)
#define PWM_EVMSK_OUTAN_Pos			2
#define PWM_EVMSK_OUTAN_Msk			(0x01 << PWM_EVMSK_OUTAN_Pos)
#define PWM_EVMSK_OUTBN_Pos			3
#define PWM_EVMSK_OUTBN_Msk			(0x01 << PWM_EVMSK_OUTBN_Pos)
#define PWM_EVMSK_IMME_Pos			4		//1 MASK立即生效   0 计数器溢出时生效
#define PWM_EVMSK_IMME_Msk			(0x01 << PWM_EVMSK_IMME_Pos)
#define PWM_EVMSK_STPCLR_Pos		8		//外部事件导致计数器停止时计数器是否清零，1 清零   0 保持当前值
#define PWM_EVMSK_STPCLR_Msk		(0x01 << PWM_EVMSK_STPCLR_Pos)

#define PWM_IE_UPOVF_Pos			0		//向上计数时计数器溢出中断使能
#define PWM_IE_UPOVF_Msk			(0x01 << PWM_IE_UPOVF_Pos)
#define PWM_IE_DNOVF_Pos			1		//向下计数时计数器溢出中断使能
#define PWM_IE_DNOVF_Msk			(0x01 << PWM_IE_DNOVF_Pos)
#define PWM_IE_UPCMPA_Pos			2		//向上计数时计数器值与CMPA相等中断使能
#define PWM_IE_UPCMPA_Msk			(0x01 << PWM_IE_UPCMPA_Pos)
#define PWM_IE_UPCMPB_Pos			3		//向上计数时计数器值与CMPB相等中断使能
#define PWM_IE_UPCMPB_Msk			(0x01 << PWM_IE_UPCMPB_Pos)
#define PWM_IE_DNCMPA_Pos			4		//向下计数时计数器值与CMPA相等中断使能
#define PWM_IE_DNCMPA_Msk			(0x01 << PWM_IE_DNCMPA_Pos)
#define PWM_IE_DNCMPB_Pos			5		//向下计数时计数器值与CMPB相等中断使能
#define PWM_IE_DNCMPB_Msk			(0x01 << PWM_IE_DNCMPB_Pos)

#define PWM_IF_UPOVF_Pos			0
#define PWM_IF_UPOVF_Msk			(0x01 << PWM_IF_UPOVF_Pos)
#define PWM_IF_DNOVF_Pos			1
#define PWM_IF_DNOVF_Msk			(0x01 << PWM_IF_DNOVF_Pos)
#define PWM_IF_UPCMPA_Pos			2
#define PWM_IF_UPCMPA_Msk			(0x01 << PWM_IF_UPCMPA_Pos)
#define PWM_IF_UPCMPB_Pos			3
#define PWM_IF_UPCMPB_Msk			(0x01 << PWM_IF_UPCMPB_Pos)
#define PWM_IF_DNCMPA_Pos			4
#define PWM_IF_DNCMPA_Msk			(0x01 << PWM_IF_DNCMPA_Pos)
#define PWM_IF_DNCMPB_Pos			5
#define PWM_IF_DNCMPB_Msk			(0x01 << PWM_IF_DNCMPB_Pos)

#define PWM_SR_STAT_Pos				0		//0 IDLE   1 ACTIVE   2 PAUSE
#define PWM_SR_STAT_Msk				(0x03 << PWM_SR_STAT_Pos)
#define PWM_SR_DIR_Pos				4		//0 向上计数   1 向下计数
#define PWM_SR_DIR_Msk				(0x01 << PWM_SR_DIR_Pos)
#define PWM_SR_OUTA_Pos				5
#define PWM_SR_OUTA_Msk				(0x01 << PWM_SR_OUTA_Pos)
#define PWM_SR_OUTB_Pos				6
#define PWM_SR_OUTB_Msk				(0x01 << PWM_SR_OUTB_Pos)
#define PWM_SR_OUTAN_Pos			7
#define PWM_SR_OUTAN_Msk			(0x01 << PWM_SR_OUTAN_Pos)
#define PWM_SR_OUTBN_Pos			8
#define PWM_SR_OUTBN_Msk			(0x01 << PWM_SR_OUTBN_Pos)


typedef struct {
	__IO uint32_t START;
	
	__IO uint32_t SWBRK;					//Software Brake，软件刹车
    
    __IO uint32_t RESET;
	
	union {
		__IO uint32_t RELOADEN;
		
		__IO uint32_t RESTART;
	};
	
    __IO uint32_t PULSE;
	
    __IO uint32_t FILTER;					//外部信号滤波，0 无滤波   1 4个PCLK周期   2 8个PCLK周期   3 16个PCLK周期
	
    __IO uint32_t BRKPOL;					//刹车信号极性，
	
    __IO uint32_t BRKIE;
    
	union {
		__IO uint32_t BRKIF;
		
		__IO uint32_t BRKSR;
	};
	
	__IO uint32_t EVSR;
} PWMG_TypeDef;


#define PWMG_START_PWM0_Pos			0
#define PWMG_START_PWM0_Msk			(0x01 << PWMG_START_PWM0_Pos)
#define PWMG_START_PWM1_Pos			1
#define PWMG_START_PWM1_Msk			(0x01 << PWMG_START_PWM1_Pos)

#define PWMG_SWBRK_PWM0A_Pos		0
#define PWMG_SWBRK_PWM0A_Msk		(0x01 << PWMG_SWBRK_PWM0A_Pos)
#define PWMG_SWBRK_PWM1A_Pos		1
#define PWMG_SWBRK_PWM1A_Msk		(0x01 << PWMG_SWBRK_PWM1A_Pos)
#define PWMG_SWBRK_PWM0B_Pos		8
#define PWMG_SWBRK_PWM0B_Msk		(0x01 << PWMG_SWBRK_PWM0B_Pos)
#define PWMG_SWBRK_PWM1B_Pos		9
#define PWMG_SWBRK_PWM1B_Msk		(0x01 << PWMG_SWBRK_PWM1B_Pos)

#define PWMG_RESET_PWM0_Pos			0
#define PWMG_RESET_PWM0_Msk			(0x01 << PWMG_RESET_PWM0_Pos)
#define PWMG_RESET_PWM1_Pos			1
#define PWMG_RESET_PWM1_Msk			(0x01 << PWMG_RESET_PWM1_Pos)

#define PWMG_RELOADEN_PWM0_Pos		0
#define PWMG_RELOADEN_PWM0_Msk		(0x01 << PWMG_RELOADEN_PWM0_Pos)
#define PWMG_RELOADEN_PWM1_Pos		1
#define PWMG_RELOADEN_PWM1_Msk		(0x01 << PWMG_RELOADEN_PWM1_Pos)

#define PWMG_RESTART_PWM0_Pos		8
#define PWMG_RESTART_PWM0_Msk		(0x01 << PWMG_RESTART_PWM0_Pos)
#define PWMG_RESTART_PWM1_Pos		9
#define PWMG_RESTART_PWM1_Msk		(0x01 << PWMG_RESTART_PWM1_Pos)

#define PWMG_PULSE_EDGE0_Pos		0		//PWM_PULSE0 计数边沿，0 上升沿   1 下降沿
#define PWMG_PULSE_EDGE0_Msk		(0x01 << PWMG_PULSE_EDGE0_Pos)
#define PWMG_PULSE_EDGE1_Pos		1
#define PWMG_PULSE_EDGE1_Msk		(0x01 << PWMG_PULSE_EDGE1_Pos)

#define PWMG_BRKPOL_BRK0_Pos		0		//PWMG_BRK0 刹车信号极性，0 低电平刹车   1 高电平刹车
#define PWMG_BRKPOL_BRK0_Msk		(0x01 << PWMG_BRKPOL_BRK0_Pos)
#define PWMG_BRKPOL_BRK1_Pos		1
#define PWMG_BRKPOL_BRK1_Msk		(0x01 << PWMG_BRKPOL_BRK1_Pos)
#define PWMG_BRKPOL_BRK2_Pos		2
#define PWMG_BRKPOL_BRK2_Msk		(0x01 << PWMG_BRKPOL_BRK2_Pos)

#define PWMG_BRKIE_BRK0_Pos			0
#define PWMG_BRKIE_BRK0_Msk			(0x01 << PWMG_BRKIE_BRK0_Pos)
#define PWMG_BRKIE_BRK1_Pos			1
#define PWMG_BRKIE_BRK1_Msk			(0x01 << PWMG_BRKIE_BRK1_Pos)
#define PWMG_BRKIE_BRK2_Pos			2
#define PWMG_BRKIE_BRK2_Msk			(0x01 << PWMG_BRKIE_BRK2_Pos)

#define PWMG_BRKIF_BRK0_Pos			0
#define PWMG_BRKIF_BRK0_Msk			(0x01 << PWMG_BRKIF_BRK0_Pos)
#define PWMG_BRKIF_BRK1_Pos			1
#define PWMG_BRKIF_BRK1_Msk			(0x01 << PWMG_BRKIF_BRK1_Pos)
#define PWMG_BRKIF_BRK2_Pos			2
#define PWMG_BRKIF_BRK2_Msk			(0x01 << PWMG_BRKIF_BRK2_Pos)

#define PWMG_BRKSR_BRK0_Pos			4		//刹车引脚电平值
#define PWMG_BRKSR_BRK0_Msk			(0x01 << PWMG_BRKSR_BRK0_Pos)
#define PWMG_BRKSR_BRK1_Pos			5
#define PWMG_BRKSR_BRK1_Msk			(0x01 << PWMG_BRKSR_BRK1_Pos)
#define PWMG_BRKSR_BRK2_Pos			6
#define PWMG_BRKSR_BRK2_Msk			(0x01 << PWMG_BRKSR_BRK2_Pos)

#define PWMG_EVSR_EV0_Pos			0		//外部事件信号电平值
#define PWMG_EVSR_EV0_Msk			(0x01 << PWMG_EVSR_EV0_Pos)
#define PWMG_EVSR_EV1_Pos			1
#define PWMG_EVSR_EV1_Msk			(0x01 << PWMG_EVSR_EV1_Pos)
#define PWMG_EVSR_EV2_Pos			2
#define PWMG_EVSR_EV2_Msk			(0x01 << PWMG_EVSR_EV2_Pos)
#define PWMG_EVSR_EV3_Pos			3
#define PWMG_EVSR_EV3_Msk			(0x01 << PWMG_EVSR_EV3_Pos)
#define PWMG_EVSR_EV4_Pos			4
#define PWMG_EVSR_EV4_Msk			(0x01 << PWMG_EVSR_EV4_Pos)
#define PWMG_EVSR_EV5_Pos			5
#define PWMG_EVSR_EV5_Msk			(0x01 << PWMG_EVSR_EV5_Pos)
#define PWMG_EVSR_EV6_Pos			6
#define PWMG_EVSR_EV6_Msk			(0x01 << PWMG_EVSR_EV6_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t POSCNT;					//[15:0] 位置计数器
	__IO uint32_t MAXCNT;					//[15:0] 最大计数值
		
		 uint32_t RESERVED[5];
	
	__IO uint32_t IE;						//Interrupt Enable，为0时IF相应位不置位
	
	__IO uint32_t IM;						//Interrupt Mask，为0时即使IF相应位置位也不触发 QEI_IRQn 中断
	
	__O  uint32_t IC;						//Interrupt Clear
	
	__I  uint32_t IF;						//Interrupt Flag
	
	__IO uint32_t IFOV;						//interrupt Interrupt Overrun
} QEI_TypeDef;


#define QEI_CR_ENA_Pos   			0
#define QEI_CR_ENA_Msk				(0x01 << QEI_CR_ENA_Pos)
#define QEI_CR_ABSWAP_Pos  			4		//1 A、B引脚交换
#define QEI_CR_ABSWAP_Msk			(0x01 << QEI_CR_ABSWAP_Pos)
#define QEI_CR_X2X4_Pos  			5		//0 X2计数模式		1 X4计数模式
#define QEI_CR_X2X4_Msk				(0x01 << QEI_CR_X2X4_Pos)
#define QEI_CR_RSTSRC_Pos  			6		//Reset Source		0 计数匹配复位		1 索引信号复位
#define QEI_CR_RSTSRC_Msk			(0x01 << QEI_CR_RSTSRC_Pos)
#define QEI_CR_MODE_Pos  			7		//工作模式选择		1 QEI模式
#define QEI_CR_MODE_Msk				(0x01 << QEI_CR_MODE_Pos)
#define QEI_CR_INDEX_Pos 			9		//0 索引引脚为低电平		1 索引引脚为高电平
#define QEI_CR_INDEX_Msk			(0x01 << QEI_CR_INDEX_Pos)
#define QEI_CR_PAUSE_Pos 			10		//1 空闲模式停止位
#define QEI_CR_PAUSE_Msk			(0x01 << QEI_CR_PAUSE_Pos)

#define QEI_IE_INDEX_Pos 			0		//检测到Index脉冲
#define QEI_IE_INDEX_Msk			(0x01 << QEI_IE_INDEX_Pos)
#define QEI_IE_MATCH_Pos 			1		//POSCNT递增到与MAXCNT相等，或POSCNT从MAXCNT递减到0
#define QEI_IE_MATCH_Msk			(0x01 << QEI_IE_MATCH_Pos)
#define QEI_IE_CNTOV_Pos 			2		//Counter Overrun，计数器溢出
#define QEI_IE_CNTOV_Msk			(0x01 << QEI_IE_CNTOV_Pos)
#define QEI_IE_ERROR_Pos 			3		//计数器错误
#define QEI_IE_ERROR_Msk			(0x01 << QEI_IE_ERROR_Pos)

#define QEI_IM_INDEX_Pos 			0
#define QEI_IM_INDEX_Msk			(0x01 << QEI_IM_INDEX_Pos)
#define QEI_IM_MATCH_Pos 			1
#define QEI_IM_MATCH_Msk			(0x01 << QEI_IM_MATCH_Pos)
#define QEI_IM_CNTOV_Pos 			2
#define QEI_IM_CNTOV_Msk			(0x01 << QEI_IM_CNTOV_Pos)
#define QEI_IM_ERROR_Pos 			3
#define QEI_IM_ERROR_Msk			(0x01 << QEI_IM_ERROR_Pos)

#define QEI_IC_INDEX_Pos 			0
#define QEI_IC_INDEX_Msk			(0x01 << QEI_IC_INDEX_Pos)
#define QEI_IC_MATCH_Pos 			1
#define QEI_IC_MATCH_Msk			(0x01 << QEI_IC_MATCH_Pos)
#define QEI_IC_CNTOV_Pos 			2
#define QEI_IC_CNTOV_Msk			(0x01 << QEI_IC_CNTOV_Pos)
#define QEI_IC_ERROR_Pos 			3
#define QEI_IC_ERROR_Msk			(0x01 << QEI_IC_ERROR_Pos)

#define QEI_IF_INDEX_Pos 			0
#define QEI_IF_INDEX_Msk			(0x01 << QEI_IF_INDEX_Pos)
#define QEI_IF_MATCH_Pos 			1
#define QEI_IF_MATCH_Msk			(0x01 << QEI_IF_MATCH_Pos)
#define QEI_IF_CNTOV_Pos 			2
#define QEI_IF_CNTOV_Msk			(0x01 << QEI_IF_CNTOV_Pos)
#define QEI_IF_ERROR_Pos 			3
#define QEI_IF_ERROR_Msk			(0x01 << QEI_IF_ERROR_Pos)

#define QEI_IFOV_INDEX_Pos 			0
#define QEI_IFOV_INDEX_Msk			(0x01 << QEI_IFOV_INDEX_Pos)
#define QEI_IFOV_MATCH_Pos 			1
#define QEI_IFOV_MATCH_Msk			(0x01 << QEI_IFOV_MATCH_Pos)
#define QEI_IFOV_CNTOV_Pos 			2
#define QEI_IFOV_CNTOV_Msk			(0x01 << QEI_IFOV_CNTOV_Pos)
#define QEI_IFOV_ERROR_Pos 			3
#define QEI_IFOV_ERROR_Msk			(0x01 << QEI_IFOV_ERROR_Pos)




typedef struct {
	__IO uint32_t CMD;
	
	__IO uint32_t INPUT;					//CORDIC计算输入数据，计算SIN和COS时，表示待计算的弧度
											//计算ARCTAN时，为防止溢出，需要将待计算数处理后再写入INPUT寄存器：
											//待计算数 ∈ (0.05, 0.5]时，将待计算数乘以2后写入INPUT
											//待计算数 ∈ (0.5, 2]时，   将待计算数直接写入INPUT
											//待计算数 > 2时，           将待计算数除以2后写入INPUT
	
	__IO uint32_t COS;						//COS计算结果
	
	__IO uint32_t SIN;						//SIN计算结果
	
	__IO uint32_t ARCTAN;					//ARCTAN计算结果
	
	__IO uint32_t IF;
	
	__IO uint32_t IE;
	
	__IO uint32_t TANH;						//写启动TANH计算，写完再读，返回计算结果
} CORDIC_TypeDef;


#define CORDIC_CMD_START_Pos		0		//写1启动运算，运算完成后自动清零
#define CORDIC_CMD_START_Msk		(0x01 << CORDIC_CMD_START_Pos)
#define CORDIC_CMD_RANGE_Pos		1		//计算ARCTAN时输入数值的范围 0 (0.05, 0.5]   1 (0.5, 2]   2 >2
#define CORDIC_CMD_RANGE_Msk		(0x03 << CORDIC_CMD_RANGE_Pos)
#define CORDIC_CMD_SINCOS_Pos		3		//1 计算SIN和COS    0 计算ARCTAN
#define CORDIC_CMD_SINCOS_Msk		(0x01 << CORDIC_CMD_SINCOS_Pos)
#define CORDIC_CMD_MULDIV_Pos		4		//0 计算SIN\COS\ARCTAN，具体由SINCOS位决定   2 计算除法   3 计算乘法
#define CORDIC_CMD_MULDIV_Msk		(0x03 << CORDIC_CMD_MULDIV_Pos)

#define CORDIC_INPUT_F_Pos			0		//输入数据小数部分
#define CORDIC_INPUT_F_Msk			(0x3FFF << CORDIC_INPUT_F_Pos)
#define CORDIC_INPUT_I_Pos			14		//输入数据整数部分
#define CORDIC_INPUT_I_Msk			(0x03 << CORDIC_INPUT_I_Pos)
#define CORDIC_INPUT_F2_Pos			16		//输入数据小数部分，乘法中的第二个参数、除法中的被除数，乘法结果存于SIN、除法结果存于ARCTAN
#define CORDIC_INPUT_F2_Msk			(0x3FFF << CORDIC_INPUT_F2_Pos)
#define CORDIC_INPUT_I2_Pos			30		//输入数据整数部分，乘法中的第二个参数、除法中的被除数，乘法结果存于SIN、除法结果存于ARCTAN
#define CORDIC_INPUT_I2_Msk			(0x03u<< CORDIC_INPUT_I2_Pos)

#define CORDIC_COS_F_Pos			0		//COS计算结果的小数部分
#define CORDIC_COS_F_Msk			(0x3FFF << CORDIC_COS_F_Pos)
#define CORDIC_COS_I_Pos			14		//COS计算结果的整数部分
#define CORDIC_COS_I_Msk			(0x03 << CORDIC_COS_I_Pos)
#define CORDIC_COS_DONE_Pos			16		//1 COS计算已完成
#define CORDIC_COS_DONE_Msk			(0x01 << CORDIC_COS_DONE_Pos)

#define CORDIC_SIN_F_Pos			0		//SIN计算结果的小数部分
#define CORDIC_SIN_F_Msk			(0x3FFF << CORDIC_SIN_F_Pos)
#define CORDIC_SIN_I_Pos			14		//SIN计算结果的整数部分
#define CORDIC_SIN_I_Msk			(0x03 << CORDIC_SIN_I_Pos)
#define CORDIC_SIN_DONE_Pos			16		//1 SIN计算已完成
#define CORDIC_SIN_DONE_Msk			(0x01 << CORDIC_SIN_DONE_Pos)

#define CORDIC_ARCTAN_F_Pos			0		//ARCTAN计算结果的小数部分
#define CORDIC_ARCTAN_F_Msk			(0x3FFF << CORDIC_ARCTAN_F_Pos)
#define CORDIC_ARCTAN_I_Pos			14		//ARCTAN计算结果的整数部分
#define CORDIC_ARCTAN_I_Msk			(0x03 << CORDIC_ARCTAN_I_Pos)
#define CORDIC_ARCTAN_DONE_Pos		16		//1 ARCTAN计算已完成
#define CORDIC_ARCTAN_DONE_Msk		(0x01 << CORDIC_ARCTAN_DONE_Pos)

#define CORDIC_IF_DONE_Pos			0		//1 计算完成，写1清零
#define CORDIC_IF_DONE_Msk			(0x01 << CORDIC_IF_DONE_Pos)
#define CORDIC_IF_ERR_Pos			1		//1 计算出错，SIN或COS结果不在[0, 1]范围内，或ARCTAN计算结果不在[0, 2]范围内时置位，写1清零
#define CORDIC_IF_ERR_Msk			(0x01 << CORDIC_IF_ERR_Pos)

#define CORDIC_IE_DONE_Pos			0
#define CORDIC_IE_DONE_Msk			(0x01 << CORDIC_IE_DONE_Pos)
#define CORDIC_IE_ERR_Pos			1
#define CORDIC_IE_ERR_Msk			(0x01 << CORDIC_IE_ERR_Pos)

#define CORDIC_TANH_F_Pos			0		//TANH输入和计算结果的小数部分
#define CORDIC_TANH_F_Msk			(0x3FFF << CORDIC_TANH_F_Pos)
#define CORDIC_TANH_I_Pos			14		//TANH输入和计算结果的整数部分
#define CORDIC_TANH_I_Msk			(0x03 << CORDIC_TANH_I_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t SR;
	
	     uint32_t RESERVED[2];
	
	__IO uint32_t DIVIDEND;					//被除数
	
	__IO uint32_t DIVISOR;					//除数
	
	__IO uint32_t QUO;						//商
	
	__IO uint32_t REMAIN;					//余数
	
	__IO uint32_t RADICAND;					//被开方数
	
	__IO uint32_t ROOT;						//平方根，低16位为小数部分，高16位为整数部分
} DIV_TypeDef;


#define DIV_CR_DIVGO_Pos			0		//写1启动除法运算，运算完成后自动清零
#define DIV_CR_DIVGO_Msk			(0x01 << DIV_CR_DIVGO_Pos)
#define DIV_CR_DIVSIGN_Pos			1		//0 有符号除法    1 无符号除法
#define DIV_CR_DIVSIGN_Msk			(0x01 << DIV_CR_DIVSIGN_Pos)
#define DIV_CR_ROOTGO_Pos			8		//写1启动开平方根运算，运算完成后自动清零
#define DIV_CR_ROOTGO_Msk			(0x01 << DIV_CR_ROOTGO_Pos)
#define DIV_CR_ROOTMOD_Pos			9		//开平方根模式：0 结果为整数    1 结果既有整数部分又有小数部分
#define DIV_CR_ROOTMOD_Msk			(0x01 << DIV_CR_ROOTMOD_Pos)

#define DIV_SR_DIVEND_Pos			0		//除法运算完成标志，写1清零
#define DIV_SR_DIVEND_Msk			(0x01 << DIV_SR_DIVEND_Pos)
#define DIV_SR_DIVBUSY_Pos			1		//1 除法运算计算中
#define DIV_SR_DIVBUSY_Msk			(0x01 << DIV_SR_DIVBUSY_Pos)
#define DIV_SR_ROOTENDI_Pos			8		//开方整数运算完成标志，写1清零
#define DIV_SR_ROOTENDI_Msk			(0x01 << DIV_SR_ROOTENDI_Pos)
#define DIV_SR_ROOTENDF_Pos			9		//开方小数运算完成标志，写1清零
#define DIV_SR_ROOTENDF_Msk			(0x01 << DIV_SR_ROOTENDF_Pos)
#define DIV_SR_ROOTBUSY_Pos			10		//1 开方运算计算中
#define DIV_SR_ROOTBUSY_Msk			(0x01 << DIV_SR_ROOTBUSY_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t ADDR;
	
	__IO uint32_t ERASE;
	
	__IO uint32_t REMAP;
	
	__IO uint32_t CFG0;
	
	__IO uint32_t CFG1;
	
	__IO uint32_t CFG2;
	
	     uint32_t RESERVED;
	
	__IO uint32_t STAT;
} FMC_TypeDef;


#define FMC_ADDR_ADDR_Pos			0
#define FMC_ADDR_ADDR_Msk			(0x7FFF<< FMC_ADDR_ADDR_Pos)
#define FMC_ADDR_BUSY_Pos			30		//0 可以写入下一个数据
#define FMC_ADDR_BUSY_Msk			(0x01 << FMC_ADDR_BUSY_Pos)
#define FMC_ADDR_WREN_Pos			31
#define FMC_ADDR_WREN_Msk			(0x01u<< FMC_ADDR_WREN_Pos)

#define FMC_ERASE_PAGE_Pos			9
#define FMC_ERASE_PAGE_Msk			(0x3F << FMC_ERASE_PAGE_Pos)
#define FMC_ERASE_CHIP_Pos			16		//1 整片擦除
#define FMC_ERASE_CHIP_Msk			(0x01 << FMC_ERASE_CHIP_Pos)
#define FMC_ERASE_REQ_Pos			24
#define FMC_ERASE_REQ_Msk			(0xFFu<< FMC_ERASE_REQ_Pos)

#define FMC_REMAP_ON_Pos			0
#define FMC_REMAP_ON_Msk			(0x01 << FMC_REMAP_ON_Pos)
#define FMC_REMAP_OFFSET_Pos		1		//对0x000-0x800这2K地址的访问映射到2K*OFFSET-2K*(OFFSET+1)地址处
#define FMC_REMAP_OFFSET_Msk		(0x0F << FMC_REMAP_OFFSET_Pos)

#define FMC_CFG0_1US_Pos			4		//1us对应多少个时钟周期，0表示1个周期
#define FMC_CFG0_1US_Msk			(0x7F << FMC_CFG0_1US_Pos)
#define FMC_CFG0_CLK_Pos			16		//当前时钟范围，0 25~50MHz   1 <25MHz   2 50~75MHz  3 75~100MHz
#define FMC_CFG0_CLK_Msk			(0x03 << FMC_CFG0_CLK_Pos)

#define FMC_CFG1_TERASE_Pos			0		//terase参数要求多少ms
#define FMC_CFG1_TERASE_Msk			(0x1F << FMC_CFG1_TERASE_Pos)
#define FMC_CFG1_TNVH1_Pos			5
#define FMC_CFG1_TNVH1_Msk			(0x7F << FMC_CFG1_TNVH1_Pos)
#define FMC_CFG1_TRCV_Pos			12
#define FMC_CFG1_TRCV_Msk			(0x03 << FMC_CFG1_TRCV_Pos)
#define FMC_CFG1_TNVH_Pos			14
#define FMC_CFG1_TNVH_Msk			(0x07 << FMC_CFG1_TNVH_Pos)
#define FMC_CFG1_TPROG_Pos			17
#define FMC_CFG1_TPROG_Msk			(0x3F << FMC_CFG1_TPROG_Pos)
#define FMC_CFG1_TPGS_Pos			23
#define FMC_CFG1_TPGS_Msk			(0x0F << FMC_CFG1_TPGS_Pos)
#define FMC_CFG1_TNVS_Pos			27
#define FMC_CFG1_TNVS_Msk			(0x07 << FMC_CFG1_TNVS_Pos)

#define FMC_STAT_ERASEBUSY_Pos		0
#define FMC_STAT_ERASEBUSY_Msk		(0x01 << FMC_STAT_ERASEBUSY_Pos)
#define FMC_STAT_PROGBUSY_Pos		1
#define FMC_STAT_PROGBUSY_Msk		(0x01 << FMC_STAT_PROGBUSY_Pos)
#define FMC_STAT_READBUSY_Pos		2
#define FMC_STAT_READBUSY_Msk		(0x01 << FMC_STAT_READBUSY_Pos)
#define FMC_STAT_PROGDONE_Pos		3
#define FMC_STAT_PROGDONE_Msk		(0x01 << FMC_STAT_PROGDONE_Pos)
#define FMC_STAT_ERASEDONE_Pos		4
#define FMC_STAT_ERASEDONE_Msk		(0x01 << FMC_STAT_ERASEDONE_Pos)




typedef struct {
    __IO uint32_t MINSEC;                   //分秒计数
    
    __IO uint32_t DATHUR;                   //日时计数
    
    __IO uint32_t MONDAY;                   //月周计数
    
    __IO uint32_t YEAR;                     //[11:0] 年计数，支持1901-2199
    
    __IO uint32_t MINSECAL;                 //分秒闹铃设置
    
    __IO uint32_t DAYHURAL;                 //周时闹铃设置
    
    __IO uint32_t LOAD;
    
    __IO uint32_t IE;
    
    __IO uint32_t IF;                       //写1清零
    
    __IO uint32_t EN;                       //[0] 1 RTC使能
    
    __IO uint32_t CFGABLE;                  //[0] 1 RTC可配置
    
    __IO uint32_t TRIM;                     //时钟调整
    
    __IO uint32_t TRIMM;                    //时钟微调整
} RTC_TypeDef;


#define RTC_MINSEC_SEC_Pos			0       //秒计数，取值0--59
#define RTC_MINSEC_SEC_Msk		    (0x3F << RTC_MINSEC_SEC_Pos)
#define RTC_MINSEC_MIN_Pos			6       //分钟计数，取值0--59
#define RTC_MINSEC_MIN_Msk		    (0x3F << RTC_MINSEC_MIN_Pos)

#define RTC_DATHUR_HOUR_Pos			0       //小时计数，取值0--23
#define RTC_DATHUR_HOUR_Msk		    (0x1F << RTC_DATHUR_HOUR_Pos)
#define RTC_DATHUR_DATE_Pos			5       //date of month，取值1--31
#define RTC_DATHUR_DATE_Msk		    (0x1F << RTC_DATHUR_DATE_Pos)

#define RTC_MONDAY_DAY_Pos			0       //day of week，取值0--6
#define RTC_MONDAY_DAY_Msk		    (0x07 << RTC_MONDAY_DAY_Pos)
#define RTC_MONDAY_MON_Pos			3       //月份计数，取值1--12
#define RTC_MONDAY_MON_Msk		    (0x0F << RTC_MONDAY_MON_Pos)

#define RTC_MINSECAL_SEC_Pos		0       //闹钟秒设置
#define RTC_MINSECAL_SEC_Msk		(0x3F << RTC_MINSECAL_SEC_Pos)
#define RTC_MINSECAL_MIN_Pos	    6       //闹钟分钟设置
#define RTC_MINSECAL_MIN_Msk		(0x3F << RTC_MINSECAL_MIN_Pos)

#define RTC_DAYHURAL_HOUR_Pos		0       //闹钟小时设置
#define RTC_DAYHURAL_HOUR_Msk		(0x1F << RTC_DAYHURAL_HOUR_Pos)
#define RTC_DAYHURAL_SUN_Pos		5       //周日闹钟有效
#define RTC_DAYHURAL_SUN_Msk		(0x01 << RTC_DAYHURAL_SUN_Pos)
#define RTC_DAYHURAL_MON_Pos		6       //周一闹钟有效
#define RTC_DAYHURAL_MON_Msk		(0x01 << RTC_DAYHURAL_MON_Pos)
#define RTC_DAYHURAL_TUE_Pos		7       //周二闹钟有效
#define RTC_DAYHURAL_TUE_Msk		(0x01 << RTC_DAYHURAL_TUE_Pos)
#define RTC_DAYHURAL_WED_Pos		8       //周三闹钟有效
#define RTC_DAYHURAL_WED_Msk		(0x01 << RTC_DAYHURAL_WED_Pos)
#define RTC_DAYHURAL_THU_Pos		9       //周四闹钟有效
#define RTC_DAYHURAL_THU_Msk		(0x01 << RTC_DAYHURAL_THU_Pos)
#define RTC_DAYHURAL_FRI_Pos		10      //周五闹钟有效
#define RTC_DAYHURAL_FRI_Msk		(0x01 << RTC_DAYHURAL_FRI_Pos)
#define RTC_DAYHURAL_SAT_Pos		11      //周六闹钟有效
#define RTC_DAYHURAL_SAT_Msk		(0x01 << RTC_DAYHURAL_SAT_Pos)

#define RTC_LOAD_TIME_Pos			0		//将时间、定时、校准相关寄存器中的值加载到RTC时钟域
#define RTC_LOAD_TIME_Msk			(0x01 << RTC_LOAD_TIME_Pos)
#define RTC_LOAD_ALARM_Pos			1		//将闹钟相关寄存器中的值加载到RTC时钟域
#define RTC_LOAD_ALARM_Msk			(0x01 << RTC_LOAD_ALARM_Pos)

#define RTC_IE_SEC_Pos		        0       //秒中断使能
#define RTC_IE_SEC_Msk		        (0x01 << RTC_IE_SEC_Pos)
#define RTC_IE_MIN_Pos		        1
#define RTC_IE_MIN_Msk		        (0x01 << RTC_IE_MIN_Pos)
#define RTC_IE_HOUR_Pos		        2
#define RTC_IE_HOUR_Msk		        (0x01 << RTC_IE_HOUR_Pos)
#define RTC_IE_DATE_Pos		        3
#define RTC_IE_DATE_Msk		        (0x01 << RTC_IE_DATE_Pos)
#define RTC_IE_ALARM_Pos		    4
#define RTC_IE_ALARM_Msk		    (0x01 << RTC_IE_ALARM_Pos)
#define RTC_IE_TRIM_Pos				5		//校准完成中断使能
#define RTC_IE_TRIM_Msk				(0x01 << RTC_IE_TRIM_Pos)
#define RTC_IE_HSEC_Pos				6		//半秒中断使能
#define RTC_IE_HSEC_Msk				(0x01 << RTC_IE_HSEC_Pos)
#define RTC_IE_QSEC_Pos				7		//四分之一秒中断使能
#define RTC_IE_QSEC_Msk				(0x01 << RTC_IE_QSEC_Pos)

#define RTC_IF_SEC_Pos		        0       //写1清零
#define RTC_IF_SEC_Msk		        (0x01 << RTC_IF_SEC_Pos)
#define RTC_IF_MIN_Pos		        1
#define RTC_IF_MIN_Msk		        (0x01 << RTC_IF_MIN_Pos)
#define RTC_IF_HOUR_Pos		        2
#define RTC_IF_HOUR_Msk		        (0x01 << RTC_IF_HOUR_Pos)
#define RTC_IF_DATE_Pos		        3
#define RTC_IF_DATE_Msk		        (0x01 << RTC_IF_DATE_Pos)
#define RTC_IF_ALARM_Pos		    4
#define RTC_IF_ALARM_Msk		    (0x01 << RTC_IF_ALARM_Pos)
#define RTC_IF_TRIM_Pos				5
#define RTC_IF_TRIM_Msk				(0x01 << RTC_IF_TRIM_Pos)
#define RTC_IF_HSEC_Pos				6
#define RTC_IF_HSEC_Msk				(0x01 << RTC_IF_HSEC_Pos)
#define RTC_IF_QSEC_Pos				7
#define RTC_IF_QSEC_Msk				(0x01 << RTC_IF_QSEC_Pos)

#define RTC_TRIM_ADJ_Pos		    0       //用于调整BASECNT的计数周期，默认为32768，如果DEC为1，则计数周期调整为32768-ADJ，否则调整为32768+ADJ
#define RTC_TRIM_ADJ_Msk		    (0xFF << RTC_TRIM_ADJ_Pos)
#define RTC_TRIM_DEC_Pos		    8
#define RTC_TRIM_DEC_Msk		    (0x01 << RTC_TRIM_DEC_Pos)

#define RTC_TRIMM_CYCLE_Pos		    0       //用于计数周期微调，如果INC为1，则第n个计数周期调整为(32768±ADJ)+1,否则调整为(32768±ADJ)-1
                                            //cycles=0时，不进行微调整；cycles=1，则n为2；cycles=7，则n为8；以此类推
#define RTC_TRIMM_CYCLE_Msk		    (0x07 << RTC_TRIMM_CYCLE_Pos)
#define RTC_TRIMM_INC_Pos		    3
#define RTC_TRIMM_INC_Msk		    (0x01 << RTC_TRIMM_INC_Pos)




typedef struct {
	__IO uint32_t RSTVAL;					//计数器计数到此值时产生复位
	
	__IO uint32_t INTVAL;					//计数器计数到此值时产生中断
	
	__IO uint32_t CR;
	
	__IO uint32_t IF;						//[0] 中断标志，写1清零
	
	__IO uint32_t FEED;						//写0x55喂狗
} WDT_TypeDef;


#define WDT_CR_EN_Pos				0
#define WDT_CR_EN_Msk				(0x01 << WDT_CR_EN_Pos)
#define WDT_CR_RSTEN_Pos			1
#define WDT_CR_RSTEN_Msk			(0x01 << WDT_CR_RSTEN_Pos)
#define WDT_CR_INTEN_Pos			2
#define WDT_CR_INTEN_Msk			(0x01 << WDT_CR_INTEN_Pos)
#define WDT_CR_WINEN_Pos			3		//Window function enable
#define WDT_CR_WINEN_Msk			(0x01 << WDT_CR_WINEN_Pos)
#define WDT_CR_CLKDIV_Pos			8		//WDT计数时钟分频值 = pow(2, CLKDIV+1)
#define WDT_CR_CLKDIV_Msk			(0x0F << WDT_CR_CLKDIV_Pos)




/******************************************************************************/
/*						 Peripheral memory map							  */
/******************************************************************************/
#define RAM_BASE		   	0x20000000
#define AHB_BASE			0x40000000
#define APB1_BASE		 	0x40040000
#define APB2_BASE			0x400A0000


/* AHB Peripheral memory map */
#define SYS_BASE			(AHB_BASE + 0x00000)

#define CORDIC_BASE			(AHB_BASE + 0x03000)
#define DIV_BASE			(AHB_BASE + 0x03800)
#define GPIOM_BASE			(AHB_BASE + 0x04000)


/* APB1 Peripheral memory map */
#define GPIOA_BASE			(APB1_BASE + 0x0000)
#define GPIOB_BASE			(APB1_BASE + 0x0800)

#define UART0_BASE			(APB1_BASE + 0x2000)
#define UART1_BASE			(APB1_BASE + 0x2800)

#define SPI0_BASE			(APB1_BASE + 0x4000)

#define PWM0_BASE			(APB1_BASE + 0x6000)
#define PWM1_BASE			(APB1_BASE + 0x6080)
#define PWMG_BASE			(APB1_BASE + 0x6400)

#define TIMR0_BASE			(APB1_BASE + 0x6800)
#define TIMR1_BASE			(APB1_BASE + 0x6840)
#define TIMRG_BASE			(APB1_BASE + 0x6C00)

#define BTIMR0_BASE			(APB1_BASE + 0x8800)
#define BTIMR1_BASE			(APB1_BASE + 0x8840)
#define BTIMR2_BASE			(APB1_BASE + 0x8880)
#define BTIMR3_BASE			(APB1_BASE + 0x88C0)
#define BTIMRG_BASE			(APB1_BASE + 0x8C00)

#define ADC0_BASE			(APB1_BASE + 0x9000)

#define FMC_BASE			(APB1_BASE + 0xA000)		//Flash Memory Controller

#define RTC_BASE			(APB1_BASE + 0xB800)

#define QEI_BASE			(APB1_BASE + 0xD800)


/* APB2 Peripheral memory map */
#define PORTA_BASE			(APB2_BASE + 0x0000)
#define PORTB_BASE			(APB2_BASE + 0x0010)
#define PORTM_BASE			(APB2_BASE + 0x0080)

#define WDT_BASE			(APB2_BASE + 0x0800)

#define I2C0_BASE			(APB2_BASE + 0x6000)



/******************************************************************************/
/*						 Peripheral declaration							 */
/******************************************************************************/
#define SYS					((SYS_TypeDef  *) SYS_BASE)

#define PORTA				((PORT_TypeDef *) PORTA_BASE)
#define PORTB				((PORT_TypeDef *) PORTB_BASE)
#define PORTM				((PORT_TypeDef *) PORTM_BASE)

#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB				((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOM				((GPIO_TypeDef *) GPIOM_BASE)

#define TIMR0				((TIMR_TypeDef *) TIMR0_BASE)
#define TIMR1				((TIMR_TypeDef *) TIMR1_BASE)
#define TIMRG				((TIMRG_TypeDef*) TIMRG_BASE)

#define BTIMR0				((TIMR_TypeDef *) BTIMR0_BASE)
#define BTIMR1				((TIMR_TypeDef *) BTIMR1_BASE)
#define BTIMR2				((TIMR_TypeDef *) BTIMR2_BASE)
#define BTIMR3				((TIMR_TypeDef *) BTIMR3_BASE)
#define BTIMRG				((TIMRG_TypeDef*) BTIMRG_BASE)

#define UART0				((UART_TypeDef *) UART0_BASE)
#define UART1				((UART_TypeDef *) UART1_BASE)

#define SPI0				((SPI_TypeDef  *) SPI0_BASE)

#define I2C0				((I2C_TypeDef  *) I2C0_BASE)

#define ADC0 				((ADC_TypeDef  *) ADC0_BASE)

#define PWM0				((PWM_TypeDef  *) PWM0_BASE)
#define PWM1				((PWM_TypeDef  *) PWM1_BASE)
#define PWMG				((PWMG_TypeDef *) PWMG_BASE)

#define CORDIC				((CORDIC_TypeDef*)CORDIC_BASE)

#define DIV					((DIV_TypeDef  *) DIV_BASE)

#define FMC					((FMC_TypeDef  *) FMC_BASE)

#define RTC					((RTC_TypeDef  *) RTC_BASE)

#define WDT					((WDT_TypeDef  *) WDT_BASE)

#define QEI					((QEI_TypeDef  *) QEI_BASE)


#include "SWM201_port.h"
#include "SWM2X1_gpio.h"
#include "SWM2X1_exti.h"
#include "SWM2X1_timr.h"
#include "SWM2X1_uart.h"
#include "SWM2X1_spi.h"
#include "SWM2X1_i2c.h"
#include "SWM2X1_pwm.h"
#include "SWM2X1_adc.h"
#include "SWM2X1_div.h"
#include "SWM2X1_cordic.h"
#include "SWM2X1_rtc.h"
#include "SWM2X1_wdt.h"
#include "SWM2X1_qei.h"
#include "SWM2X1_flash.h"
#include "SWM2X1_sleep.h"
#include "SWM2X1_iofilt.h"



#ifdef  SW_LOG_RTT
#define log_printf(...)	 	SEGGER_RTT_printf(0, __VA_ARGS__)
#else
#define log_printf(...)	 	printf(__VA_ARGS__)
#endif


#ifndef SW_LOG_LEVEL
#define SW_LOG_LEVEL        0
#endif

#if (SW_LOG_LEVEL > 0)
#define SW_LOG_ERR(...)    	{						 \
							log_printf("ERROR: ");   \
							log_printf(__VA_ARGS__); \
							log_printf("\n");		 \
							}

#if (SW_LOG_LEVEL > 1)
#define SW_LOG_WARN(...) 	{						 \
							log_printf("WARN : ");   \
							log_printf(__VA_ARGS__); \
							log_printf("\n");		 \
							}

#if (SW_LOG_LEVEL > 2)
#define SW_LOG_INFO(...)   	{						 \
							log_printf("INFO : ");   \
							log_printf(__VA_ARGS__); \
							log_printf("\n");		 \
							}
#else
#define SW_LOG_INFO(...)
#endif

#else
#define SW_LOG_WARN(...)
#define SW_LOG_INFO(...)
#endif

#else
#define SW_LOG_ERR(...)
#define SW_LOG_WARN(...)
#define SW_LOG_INFO(...)
#endif


#endif //__SWM201_H__
