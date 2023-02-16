/****************************************************************************************************************************************** 
* �ļ�����:	SWM201_flash.c
* ����˵��:	ʹ��оƬ��IAP���ܽ�Ƭ��Flashģ���EEPROM���������ݣ�����󲻶�ʧ
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
* �汾����: V1.0.0		2016��1��30��
* ������¼: 
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
#ifdef CHIP_SWM201


#include "SWM2X1.h"
#include "SWM2X1_flash.h"


/****************************************************************************************************************************************** 
* ��������:	FLASH_Erase()
* ����˵��:	FLASH����������ÿ������512�ֽ�
* ��    ��: uint32_t addr		Ҫ���������ĵ�ַ������512���룬��addr%512 == 0
* ��    ��: uint32_t			FLASH_RES_OK��FLASH_RES_TO��FLASH_RES_ERR
* ע������: ��
******************************************************************************************************************************************/
#if defined ( __ICCARM__ )
__ramfunc
#endif
uint32_t FLASH_Erase(uint32_t addr)
{
	if(addr >= 32*1024) return FLASH_RES_ERR;
	
	__disable_irq();
	
	FMC->ERASE = FMC_ERASE_REQ_Msk | ((addr >> 9) << FMC_ERASE_PAGE_Pos);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	while(FMC->STAT & FMC_STAT_ERASEBUSY_Msk) __NOP();
	
	__enable_irq();
	
	return FLASH_RES_OK;
}


/****************************************************************************************************************************************** 
* ��������:	FLASH_Write()
* ����˵��:	FLASH����д��
* ��    ��: uint32_t addr		����Ҫд�뵽Flash�еĵ�ַ���ֶ���
*			uint32_t buff[]		Ҫд��Flash�е�����
*			uint32_t cnt		Ҫд�����ݵĸ���������Ϊ��λ�����128
* ��    ��: uint32_t			FLASH_RES_OK��FLASH_RES_TO��FLASH_RES_ERR
* ע������: Ҫд������ݱ���ȫ����ͬһҳ�ڣ�ÿҳ512�ֽڣ���addr/512 == (addr+(cnt-1)*4)/512
******************************************************************************************************************************************/
#if defined ( __ICCARM__ )
__ramfunc
#endif
uint32_t FLASH_Write(uint32_t addr, uint32_t buff[], uint32_t cnt)
{	
	uint32_t i;
	
	if((addr+cnt*4) > 32*1024) return FLASH_RES_ERR;
	
	if(addr/512 != (addr+(cnt-1)*4)/512) return FLASH_RES_ERR;	// ��ҳ
	
	__disable_irq();
	
	FMC->ADDR = (1u << FMC_ADDR_WREN_Pos) | (addr << FMC_ADDR_ADDR_Pos);
	for(i = 0; i < cnt; i++)
	{
		FMC->DATA = buff[i];
		while(FMC->ADDR & FMC_ADDR_BUSY_Msk) __NOP();
	}
	while(FMC->STAT & FMC_STAT_PROGBUSY_Msk) __NOP();
	
	FMC->ADDR = 0;
	
	__enable_irq();
	
	return FLASH_RES_OK;
}


#endif
