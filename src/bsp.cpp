/******************************************************************************
**                                                                           **
**                    STMICROELECTRONICS  WIRELINE                           **
**                                                                           **
** ***************************************************************************/

/**************************** COPYRIGHT INFORMATION **************************
**                                                                           **
**   This program contains proprietary information which is a trade          **
**   secret of STMICROELECTRONICS  WIRELINE and also is protected as an      **
**   unpublished work under applicable Copyright laws. Recipient is to       **
**   retain this program in confidence and is not permitted to use or make   **
**   copies thereof other than as permitted in a written agreement with      **
**   STMICROELECTRONICS  WIRELINE.                                           **
**                                                                           **
** ***************************************************************************/

/******************************************************************************
**
** Project       : ADSL - UNICORN
**
** Module        : MODEM SW ADSL ST_REL: UNICORN_4.6.2
**
** File name     : bsp.cpp
**
** Authors       : 
**
** Created On    : 
**
** Description   : Board Support Package for Unicorn Board
**
** Reference(s)  : 
**
**
** ****************************************************************************

** ****************************************************************************
**
** History
** Date          Revision     Comment
** 
**
** ***************************************************************************/

/***************************** COMPILE DIRECTIVES ****************************
**
**
** ***************************************************************************/
//#define USB_KERNEL_DEBUG
/*******************************************************************************/

/******************************************************************************
**                                                                           **
**   HEADER (INCLUDE) SECTION   (check protection before include)            **
**                                                                           **
** ***************************************************************************/

#include "../include/types.h"
#include "../include/tracetool.h"
#include "../include/hal.h"
#include "../include/hard.h"
#include "../include/rapi.h"
#include "../include/bsp.h"		// AMAS API's and typedefs


unsigned short initializeBoard(void)
{
#ifdef USB_KERNEL_DEBUG
  if ( (status = USB_controlWrite(0x61,0x00)) == FAILURE )
			PRINT_ERROR("Configuration of GPIO_DIR register failed!\n");

	PRINT_INFO("USB> GPIO_DIR: INITIALIZED!\n");
#endif
	return SUCCESS;
}


unsigned long powerUp_Modem_Chipset (unsigned long Chipset)
{
	ST_STATUS status;
	unsigned short  iaddr_val, idata_val, irmw_val;

	iaddr_val = DMT_GPIO_OFFSET;
	idata_val = DMT_OUT_PIN_HIGH;
	irmw_val  = MASK_DMT_OUT_PIN_LOW;

	status = USB_controlWrite(UR_IADR,iaddr_val);
	if (status == FAILURE) return FAILURE;
	status = USB_controlWrite(UR_IDATA,idata_val);
	if (status == FAILURE) return FAILURE;
	status = USB_controlWrite(UR_IRMW,irmw_val);
	if (status == FAILURE) return FAILURE;
	status = USB_controlWrite(UR_STATUS,0x2823);
	if (status == FAILURE) return FAILURE;

	PRINT_ERROR("USB> powerUp_Modem_Chipset completed\n");
	return SUCCESS;
}

unsigned long powerDown_Modem_Chipset (unsigned long Chipset)
{
	return SUCCESS;
}

unsigned long pull_Modem_Chipset_out_of_reset(unsigned long Chipset)
{
	return SUCCESS;
}

unsigned long put_Modem_Chipset_in_reset(unsigned long Chipset)
{
	return SUCCESS;
}

