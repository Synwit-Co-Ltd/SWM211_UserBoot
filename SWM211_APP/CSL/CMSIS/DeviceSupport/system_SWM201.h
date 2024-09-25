#ifndef __SYSTEM_SWM201_H__
#define __SYSTEM_SWM201_H__

#ifdef __cplusplus
 extern "C" {
#endif


extern uint32_t SystemCoreClock;		// System Clock Frequency (Core Clock)
extern uint32_t CyclesPerUs;			// Cycles per micro second


extern void SystemInit(void);

extern void SystemCoreClockUpdate (void);

	 
extern void switchTo30MHz(void);
extern void switchTo3M75Hz(void);
extern void switchTo60MHz(void);
extern void switchTo7M5Hz(void);
extern void switchToXTAL(uint32_t div8);
extern void switchTo32KHz(void);
extern void switchToXTAL_32K(void);

#ifdef __cplusplus
}
#endif

#endif //__SYSTEM_SWM201_H__
