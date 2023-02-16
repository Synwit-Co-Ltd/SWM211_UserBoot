    .syntax unified
    .arch armv6-m

/* Memory Model
   The HEAP starts at the end of the DATA section and grows upward.
   
   The STACK starts at the end of the RAM and grows downward     */
    .section .stack
    .align 3
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    0x800
__StackTop:


    .section .heap
    .align 3
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .space    0x000
__HeapLimit:


    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            
    .long    Reset_Handler         
    .long    NMI_Handler          
    .long    HardFault_Handler     
    .long    0     
    .long    0      
    .long    0   
    .long    0                    
    .long    0                    
    .long    0                    
    .long    0
    .long    SVC_Handler          
    .long    0     
    .long    0                     
    .long    PendSV_Handler           
    .long    SysTick_Handler         

    /* External interrupts */
    .long     UART0_Handler
    .long     TIMR0_Handler
    .long     SPI0_Handler
    .long     UART1_Handler
    .long     UART2_Handler
    .long     TIMR1_Handler
    .long     DMA_Handler
    .long     PWM0_Handler
    .long     BTIMR0_Handler
    .long     TIMR2_Handler
    .long     TIMR3_Handler
    .long     WDT_Handler
    .long     I2C4_Handler
    .long     UART3_Handler
    .long     ADC0_Handler
    .long     BTIMR1_Handler
    .long     GPIOA9_GPIOC6_Handler
    .long     GPIOA6_GPIOC7_Handler
    .long     GPIOA7_GPIOC8_Handler
    .long     GPIOA8_GPIOC9_Handler
    .long     GPIOA10_GPIOC10_Handler
    .long     GPIOA13_GPIOC12_Handler
    .long     GPIOA12_GPIOC13_Handler
    .long     GPIOA11_GPIOC14_Handler
    .long     XTALSTOPDET_GPIOC0_Handler
    .long     BTIMR2_GPIOB12_Handler
    .long     PWM1_GPIOA1_Handler
    .long     PWM2_UART4_Handler
    .long     BOD_PWMHALT_Handler
    .long     PWM3_GPIOB_ACMP_Handler
    .long     SPI1_HALL_GPIOD_Handler
    .long     BTIMR3_RTC_Handler

	.section .text.Reset_Handler
    .align 2
    .thumb
    .globl    Reset_Handler
    .type     Reset_Handler, %function
Reset_Handler:
/* Loop to copy data from read only memory to RAM. The ranges
 * of copy from/to are specified by symbols evaluated in linker script.  */
    ldr    r1, =__data_load__
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs   r3, r2
    ble .Lflash_to_ram_done

.Lflash_to_ram_loop:
    subs r3, #4
    ldr r0, [r1, r3]
    str r0, [r2, r3]
    bgt .Lflash_to_ram_loop
.Lflash_to_ram_done:


    ldr    r2, =__bss_start__
    ldr    r3, =__bss_end__

    subs   r3, r2
    ble .Lbss_to_ram_done

    movs    r0, 0
.Lbss_to_ram_loop:
    subs r3, #4
    str r0, [r2, r3]
    bgt .Lbss_to_ram_loop
.Lbss_to_ram_done:

    ldr    r0, =main
    bx     r0
    .pool    


    .text
/* Macro to define default handlers. 
   Default handler will be weak symbol and just dead loops. */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .endm

    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler

	def_default_handler    UART0_Handler
    def_default_handler    TIMR0_Handler
    def_default_handler    SPI0_Handler
    def_default_handler    UART1_Handler
    def_default_handler    UART2_Handler
    def_default_handler    TIMR1_Handler
    def_default_handler    DMA_Handler
    def_default_handler    PWM0_Handler
    def_default_handler    BTIMR0_Handler
    def_default_handler    TIMR2_Handler
    def_default_handler    TIMR3_Handler
    def_default_handler    WDT_Handler
    def_default_handler    I2C4_Handler
    def_default_handler    UART3_Handler
    def_default_handler    ADC0_Handler
    def_default_handler    BTIMR1_Handler
    def_default_handler    GPIOA9_GPIOC6_Handler
    def_default_handler    GPIOA6_GPIOC7_Handler
    def_default_handler    GPIOA7_GPIOC8_Handler
    def_default_handler    GPIOA8_GPIOC9_Handler
    def_default_handler    GPIOA10_GPIOC10_Handler
    def_default_handler    GPIOA13_GPIOC12_Handler
    def_default_handler    GPIOA12_GPIOC13_Handler
    def_default_handler    GPIOA11_GPIOC14_Handler
    def_default_handler    XTALSTOPDET_GPIOC0_Handler
    def_default_handler    BTIMR2_GPIOB12_Handler
    def_default_handler    PWM1_GPIOA1_Handler
    def_default_handler    PWM2_UART4_Handler
    def_default_handler    BOD_PWMHALT_Handler
    def_default_handler    PWM3_GPIOB_ACMP_Handler
    def_default_handler    SPI1_HALL_GPIOD_Handler
    def_default_handler    BTIMR3_RTC_Handler
    
    def_default_handler    Default_Handler

    .end
