/****************************************************************************************************************************************** 
* 文件名称:	system_SWM201.c
* 功能说明:	SWM201单片机的时钟设置
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期: V1.0.0		2016年1月30日
* 升级记录: 
*
*
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology
*******************************************************************************************************************************************/ 
#include <stdint.h>
#include "SWM201.h"


/******************************************************************************************************************************************
 * 系统时钟设定
 *****************************************************************************************************************************************/
#define SYS_CLK_30MHz		0	 	//0 内部高频  30MHz RC振荡器
#define SYS_CLK_3M75Hz		1		//1 内部高频3.75MHz RC振荡器
#define SYS_CLK_60MHz		2		//2 内部高频  60MHz RC振荡器
#define SYS_CLK_7M5Hz		3		//3 内部高频 7.5MHz RC振荡器
#define SYS_CLK_XTAL		4		//4 外部晶体振荡器（2-32MHz）
#define SYS_CLK_XTAL_DIV8	5		//5 外部晶体振荡器（2-32MHz） 8分频
#define SYS_CLK_32KHz		8		//8 内部低频32KHz RC  振荡器
#define SYS_CLK_XTAL_32K	9		//9 外部低频32KHz 晶体振荡器

#define SYS_CLK   SYS_CLK_XTAL


#define __HSI		(30000000UL)		//高速内部时钟
#define __LSI		(   32000UL)		//低速内部时钟
#define __HSE		(12000000UL)		//高速外部时钟
#define __LSE		(   32000UL)		//低速外部时钟


uint32_t SystemCoreClock  = __HSI;   				//System Clock Frequency (Core Clock)
uint32_t CyclesPerUs      = (__HSI / 1000000); 		//Cycles per micro second


/****************************************************************************************************************************************** 
* 函数名称: 
* 功能说明: This function is used to update the variable SystemCoreClock and must be called whenever the core clock is changed
* 输    入: 
* 输    出: 
* 注意事项: 
******************************************************************************************************************************************/
void SystemCoreClockUpdate(void)    
{
	if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)			//SYS  <= HRC
	{
		if(SYS->HRCCR & SYS_HRCCR_DBL_Msk)				//HRC = 60MHz
		{
			SystemCoreClock = __HSI*2;
		}
		else											//HRC = 30MHz
		{
			SystemCoreClock = __HSI;
		}
	}
	else											//SYS  <= CLK
	{
		switch((SYS->CLKSEL & SYS_CLKSEL_CLK_Msk) >> SYS_CLKSEL_CLK_Pos)
		{
		case 0:
			SystemCoreClock = __LSI;
			break;
		
		case 2:
			SystemCoreClock = __LSE;
			break;
		
		case 3:
			SystemCoreClock = __HSE;
			break;
		
		case 4:
			SystemCoreClock = __HSI;
			if(SYS->HRCCR & SYS_HRCCR_DBL_Msk)  SystemCoreClock *= 2;
			break;
		}
		
		if(SYS->CLKSEL & SYS_CLKSEL_CLK_DIVx_Msk)  SystemCoreClock /= 8;
	}
	
	if(SystemCoreClock > 48000000)  *((volatile uint32_t *) 0x4004A010) |= (2 << 16);	//Flash适应60MHz
	
	CyclesPerUs = SystemCoreClock / 1000000;
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
	SYS->CLKEN0 |= (1 << SYS_CLKEN0_ANAC_Pos);
	
	switch(SYS_CLK)
	{
		case SYS_CLK_30MHz:
			switchTo30MHz();
			break;
		
		case SYS_CLK_3M75Hz:
			switchTo3M75Hz();
			break;
		
		case SYS_CLK_60MHz:
			switchTo60MHz();
			break;
		
		case SYS_CLK_7M5Hz:
			switchTo7M5Hz();
			break;
		
		case SYS_CLK_XTAL:
			switchToXTAL(0);
			break;
		
		case SYS_CLK_XTAL_DIV8:
			switchToXTAL(1);
			break;
		
		case SYS_CLK_32KHz:
			switchTo32KHz();
			break;
		
		case SYS_CLK_XTAL_32K:
			switchToXTAL_32K();
			break;
	}
	
	SystemCoreClockUpdate();
	
	PORTB->PULLD &= ~(1 << PIN11);
	PORTB->PULLU &= ~((1 << PIN12) | (1 << PIN14));
}

void switchTo30MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_ON_Pos) |
				 (0 << SYS_HRCCR_DBL_Pos);			//HRC = 30MHz
	
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);		//SYS <= HRC
}

void switchTo3M75Hz(void)
{
	switchTo30MHz();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (4 << SYS_CLKSEL_CLK_Pos);		//CLK <= HRC

	SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_DIVx_Pos);
	
	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= HRC/8
}

void switchTo60MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_ON_Pos) |
				 (1 << SYS_HRCCR_DBL_Pos);			//HRC = 60MHz
	
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);		//SYS <= HRC
}

void switchTo7M5Hz(void)
{
	switchTo60MHz();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (4 << SYS_CLKSEL_CLK_Pos);		//CLK <= HRC

	SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_DIVx_Pos);
	
	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= HRC/8
}

void switchToXTAL(uint32_t div8)
{
	uint32_t i;
	
	switchTo30MHz();
	
	PORTB->PULLU &= ~((1 << PIN11) | (1 << PIN12));
	PORTB->PULLD &= ~((1 << PIN11) | (1 << PIN12));
	PORT_Init(PORTB, PIN11, PORTB_PIN11_XTAL_IN,  0);
	PORT_Init(PORTB, PIN12, PORTB_PIN12_XTAL_OUT, 0);
	SYS->XTALCR |= (1 << SYS_XTALCR_ON_Pos) | (7 << SYS_XTALCR_DRV_Pos) | (1 << SYS_XTALCR_DET_Pos);
	for(i = 0; i < 1000; i++) __NOP();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (3 << SYS_CLKSEL_CLK_Pos);		//CLK <= XTAL

	if(div8) SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_DIVx_Pos);
	else     SYS->CLKSEL &=~(1 << SYS_CLKSEL_CLK_DIVx_Pos);
	
	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= XTAL
}

void switchTo32KHz(void)
{
	switchTo30MHz();
	
	SYS->LRCCR = (1 << SYS_LRCCR_ON_Pos);
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (0 << SYS_CLKSEL_CLK_Pos);		//CLK <= LRC

	SYS->CLKSEL &=~(1 << SYS_CLKSEL_CLK_DIVx_Pos);

	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= LRC
}

void switchToXTAL_32K(void)
{
	uint32_t i;
	
	switchTo30MHz();
	
	SYS->XTALCR |= (1 << SYS_XTALCR_32KON_Pos) | (1 << SYS_XTALCR_32KDRV_Pos) | (1 << SYS_XTALCR_32KDET_Pos);
	for(i = 0; i < 1000; i++) __NOP();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (2 << SYS_CLKSEL_CLK_Pos);		//CLK <= XTAL_32K

	SYS->CLKSEL &=~(1 << SYS_CLKSEL_CLK_DIVx_Pos);

	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= XTAL_32K
}
