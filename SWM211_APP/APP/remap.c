#include "SWM2X1.h"

void Flash_remap(uint32_t addr)
{
	/* 只能在APP中REMAP，在UserBoot中REMAP可能会导致访问UserBoot的代码被重定向到APP的代码 */
	FMC->REMAP = (1 << FMC_REMAP_ON_Pos) | ((addr / 2048) << FMC_REMAP_OFFSET_Pos);
	FMC->CACHE |= (1 << FMC_CACHE_CCLR_Pos);
	
	__enable_irq();
}
