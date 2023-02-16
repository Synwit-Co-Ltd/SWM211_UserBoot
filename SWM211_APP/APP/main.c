#include "SWM2X1.h"

void SerialInit(void);
void Flash_remap(uint32_t addr);

int main(void)
{
 	uint32_t i;
	
	Flash_remap(0x4000);
	
	SystemInit();
	
	SerialInit();
	
	GPIO_Init(GPIOA, PIN2, 0, 1, 0, 0);		// 输入，开上拉
	GPIO_Init(GPIOA, PIN5, 1, 0, 0, 0);		// 输出
	
	TIMR_Init(TIMR0, TIMR_MODE_TIMER, CyclesPerUs, 1500000, 1);
	TIMR_Start(TIMR0);
	
 	while(1==1)
 	{
		printf("Running in App\r\n");
		
		for(i = 0; i < SystemCoreClock/4; i++) __NOP();
		
		
		if(GPIO_GetBit(GPIOA, PIN2) == 0)	// 检测到按键按下，跳到UserBoot
		{
			__disable_irq();
			
			WDT_Init(WDT, 0, 5);			// 通过触发WDT复位跳转到UserBoot
			WDT_Start(WDT);
			while(1) __NOP();
		}
 	}
}


void TIMR0_Handler(void)
{
	TIMR_INTClr(TIMR0);
	
	GPIO_InvBit(GPIOA, PIN5);
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTA, PIN0, PORTA_PIN0_UART0_RX, 1);	//GPIOA.0配置为UART0 RXD
	PORT_Init(PORTA, PIN1, PORTA_PIN1_UART0_TX, 0);	//GPIOA.1配置为UART0 TXD
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATA_8BIT;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOP_1BIT;
	UART_initStruct.RXThreshold = 3;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThreshold = 3;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutTime = 10;
	UART_initStruct.TimeoutIEn = 0;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
}

/****************************************************************************************************************************************** 
* 函数名称: fputc()
* 功能说明: printf()使用此函数完成实际的串口打印动作
* 输    入: int ch		要打印的字符
*			FILE *f		文件句柄
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}
