;******************************************************************************************************************************************
; 文件名称:    startup_SWM201.s
; 功能说明:    SWM201单片机的启动文件
; 技术支持:    http://www.synwit.com.cn/e/tool/gbook/?bid=1
; 注意事项:
; 版本日期: V1.0.0        2016年1月30日
; 升级记录:
;
;
;******************************************************************************************************************************************
; @attention
;
; THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION
; REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE
; FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
; OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
; -ECTION WITH THEIR PRODUCTS.
;
; COPYRIGHT 2012 Synwit Technology
;******************************************************************************************************************************************

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler              ; Reset Handler
        DCD     NMI_Handler                ; NMI Handler
        DCD     HardFault_Handler          ; Hard Fault Handler
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     SVC_Handler                ; SVCall Handler
        DCD     0                          ; Reserved
        DCD     0                          ; Reserved
        DCD     PendSV_Handler             ; PendSV Handler
        DCD     SysTick_Handler            ; SysTick Handler

        ; External Interrupts
        DCD    UART0_Handler
		DCD    TIMR0_Handler
		DCD    SPI0_Handler
		DCD    UART1_Handler
		DCD    UART2_Handler
		DCD    TIMR1_Handler
		DCD    DMA_Handler
		DCD    PWM0_Handler
		DCD    BTIMR0_Handler
		DCD    TIMR2_Handler
		DCD    TIMR3_Handler
		DCD    WDT_Handler
		DCD    I2C4_Handler
		DCD    UART3_Handler
		DCD    ADC0_Handler
		DCD    BTIMR1_Handler
		DCD    GPIOA9_GPIOC6_Handler
		DCD    GPIOA6_GPIOC7_Handler
		DCD    GPIOA7_GPIOC8_Handler
		DCD    GPIOA8_GPIOC9_Handler
		DCD    GPIOA10_GPIOC10_Handler
		DCD    GPIOA13_GPIOC12_Handler
		DCD    GPIOA12_GPIOC13_Handler
		DCD    GPIOA11_GPIOC14_Handler
		DCD    XTALSTOPDET_GPIOC0_Handler
		DCD    BTIMR2_GPIOB12_Handler
		DCD    PWM1_GPIOA1_Handler
		DCD    PWM2_UART4_Handler
		DCD    BOD_PWMHALT_Handler
		DCD    PWM3_GPIOB_ACMP_Handler
		DCD    SPI1_HALL_GPIOD_Handler
		DCD    BTIMR3_RTC_Handler
        

        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, =__iar_program_start
        BX      R0
        
        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler


        PUBWEAK UART0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART0_Handler
        B UART0_Handler

        PUBWEAK TIMR0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMR0_Handler
        B TIMR0_Handler

        PUBWEAK SPI0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI0_Handler
        B SPI0_Handler

        PUBWEAK UART1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_Handler
        B UART1_Handler

        PUBWEAK UART2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler
        B UART2_Handler

        PUBWEAK TIMR1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMR1_Handler
        B TIMR1_Handler

        PUBWEAK DMA_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Handler
        B DMA_Handler

        PUBWEAK PWM0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM0_Handler
        B PWM0_Handler

        PUBWEAK BTIMR0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BTIMR0_Handler
        B BTIMR0_Handler

        PUBWEAK TIMR2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMR2_Handler
        B TIMR2_Handler

        PUBWEAK TIMR3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMR3_Handler
        B TIMR3_Handler

        PUBWEAK WDT_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT_Handler
        B WDT_Handler

        PUBWEAK I2C4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C4_Handler
        B I2C4_Handler

        PUBWEAK UART3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART3_Handler
        B UART3_Handler

        PUBWEAK ADC0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC0_Handler
        B ADC0_Handler

        PUBWEAK BTIMR1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BTIMR1_Handler
        B BTIMR1_Handler

        PUBWEAK GPIOA9_GPIOC6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA9_GPIOC6_Handler
        B GPIOA9_GPIOC6_Handler

        PUBWEAK GPIOA6_GPIOC7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA6_GPIOC7_Handler
        B GPIOA6_GPIOC7_Handler

        PUBWEAK GPIOA7_GPIOC8_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA7_GPIOC8_Handler
        B GPIOA7_GPIOC8_Handler

        PUBWEAK GPIOA8_GPIOC9_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA8_GPIOC9_Handler
        B GPIOA8_GPIOC9_Handler

        PUBWEAK GPIOA10_GPIOC10_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA10_GPIOC10_Handler
        B GPIOA10_GPIOC10_Handler

        PUBWEAK GPIOA13_GPIOC12_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA13_GPIOC12_Handler
        B GPIOA13_GPIOC12_Handler

        PUBWEAK GPIOA12_GPIOC13_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA12_GPIOC13_Handler
        B GPIOA12_GPIOC13_Handler

        PUBWEAK GPIOA11_GPIOC14_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA11_GPIOC14_Handler
        B GPIOA11_GPIOC14_Handler

        PUBWEAK XTALSTOPDET_GPIOC0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
XTALSTOPDET_GPIOC0_Handler
        B XTALSTOPDET_GPIOC0_Handler

        PUBWEAK BTIMR2_GPIOB12_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BTIMR2_GPIOB12_Handler
        B BTIMR2_GPIOB12_Handler

        PUBWEAK PWM1_GPIOA1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM1_GPIOA1_Handler
        B PWM1_GPIOA1_Handler

        PUBWEAK PWM2_UART4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM2_UART4_Handler
        B PWM2_UART4_Handler

        PUBWEAK BOD_PWMHALT_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BOD_PWMHALT_Handler
        B BOD_PWMHALT_Handler

        PUBWEAK PWM3_GPIOB_ACMP_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM3_GPIOB_ACMP_Handler
        B PWM3_GPIOB_ACMP_Handler

        PUBWEAK SPI1_HALL_GPIOD_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_HALL_GPIOD_Handler
        B SPI1_HALL_GPIOD_Handler

        PUBWEAK BTIMR3_RTC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BTIMR3_RTC_Handler
        B BTIMR3_RTC_Handler

        END
