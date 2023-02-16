#ifndef __SWM2X1_FLASH_H__
#define __SWM2X1_FLASH_H__


uint32_t FLASH_Erase(uint32_t addr);
uint32_t FLASH_Write(uint32_t addr, uint32_t buff[], uint32_t cnt);
void Flash_Param_at_xMHz(uint32_t x);


#define FLASH_RES_OK	0
#define FLASH_RES_TO	1	//Timeout
#define FLASH_RES_ERR	2


#endif //__SWM2X1_FLASH_H__
