#include "SWM2X1.h"

void Flash_remap(uint32_t addr)
{
	/* ֻ����APP��REMAP����UserBoot��REMAP���ܻᵼ�·���UserBoot�Ĵ��뱻�ض���APP�Ĵ��� */
	FMC->REMAP = (1 << FMC_REMAP_ON_Pos) | ((addr / 2048) << FMC_REMAP_OFFSET_Pos);
	FMC->CACHE |= (1 << FMC_CACHE_CCLR_Pos);
	
	__enable_irq();
}
