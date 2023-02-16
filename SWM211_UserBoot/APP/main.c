#include "SWM2X1.h"

void SerialInit(void);
void JumpToApp(uint32_t addr);

int main(void)
{
 	uint32_t i, j;
	
	SystemInit();
	
	SerialInit();
 	
	GPIO_Init(GPIOA, PIN2, 0, 1, 0, 0);		// ���룬������
	GPIO_Init(GPIOA, PIN5, 1, 0, 0, 0);		// ���
	
	if(GPIO_GetBit(GPIOA, PIN2) == 0)		// ��⵽�ض��źţ����߼�⵽APP����������
	{
		for(i = 0; i < 10; i++)				// ģ��UserBoot����APP����Ĺ���
		{
			GPIO_InvBit(GPIOA, PIN5);
			
			printf("Running in UserBoot\r\n");
			
			for(j = 0; j < SystemCoreClock/24; j++) __NOP();
		}
	}
	else
	{
		printf("Running in UserBoot\r\n");
	}
	
	/* ��ת��APPǰ��Ҫ��UserBootʹ�õ�����ص�����λ��*/
	__disable_irq();
	
	SYS->PRSTEN = 0x55;
	SYS->PRSTR0 = 0xFFFFFFFF & (~SYS_PRSTR0_ANAC_Msk);
	SYS->PRSTR1 = 0xFFFFFFFF;
	for(i = 0; i < CyclesPerUs; i++) __NOP();
	SYS->PRSTR0 = 0;
	SYS->PRSTR1 = 0;
	SYS->PRSTEN = 0;
	
	SysTick->CTRL = 0;	//�ر�SysTick
	
	NVIC->ICER[0] = 0xFFFFFFFF;
	
	JumpToApp(0x4000);
	
 	while(1==1)
 	{
 	}
}


void JumpToApp(uint32_t addr)
{	
	uint32_t sp = *((volatile uint32_t *)(addr));
	uint32_t pc = *((volatile uint32_t *)(addr + 4));
	
	typedef void (*Func_void_void)(void);
	Func_void_void ResetHandler = (Func_void_void)pc;
	
	__set_MSP(sp);
	
	ResetHandler();
	
	while(1) __NOP();
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTA, PIN0, PORTA_PIN0_UART0_RX, 1);	//GPIOA.0����ΪUART0 RXD
	PORT_Init(PORTA, PIN1, PORTA_PIN1_UART0_TX, 0);	//GPIOA.1����ΪUART0 TXD
 	
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
* ��������: fputc()
* ����˵��: printf()ʹ�ô˺������ʵ�ʵĴ��ڴ�ӡ����
* ��    ��: int ch		Ҫ��ӡ���ַ�
*			FILE *f		�ļ����
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}
