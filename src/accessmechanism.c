//----------------------------------------------------------------------
// Test driver for the STMicroelectronics UNICORN-II
//----------------------------------------------------------------------
// file: usbdrv.cpp
// Author: 
// Copyright 
// Copyright STMicroelectronics 2000-2005
//----------------------------------------------------------------------
// 
//----------------------------------------------------------------------
// This file contains code specific to the USB requirements
// of the ADSL Unicorn-II driver
//----------------------------------------------------------------------
/*
  Updated to work with Linux kernel >= 3.6.10 by
  Zbigniew Luszpinski 2013-05-04 <zbiggy(a)o2,pl>
*/
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
#include <linux/autoconf.h>
#else
#include <generated/autoconf.h>
#endif
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include "../include/unicorn_usbdrv.h"
#include "../include/types.h"
#include "../include/hal.h"
#include "../include/hard.h"
#include "../include/rapi.h"
#include "../include/amas.h"
#include "../include/amsw_init.h"
#include "../include/amsw_intf_types.h"
#include "../include/usb_protocolcreator.h"
#include "../include/accessmechanism.h"
#include "../include/tracetool.h"

#if  (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif

extern void createclassobject(void);
extern void deletelassobject(void);

struct semaphore acc_sema ;
unsigned long flags;
extern struct unicorn_dev unicorn_usb_dev;

T_EpOut     selectBestEpOut(WORD *epOutSize, WORD requestedSize);
T_EpIn      selectBestEpIn(WORD requestedSize);

void createSemaphore(void)
{
	sema_init(&acc_sema,1);
}

void CreateObject(void)
{
	createclassobject();
}

void DeleteObject(void)
{
	deletelassobject();
}
T_EpOut selectBestEpOut(WORD *epOutSize, WORD requestedSize)
{

	UINT ep2_size = 0;  
	UINT ep6_size = 0;
	T_EpOut bestEpOut;
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
	ep2_size = ((unicorn_usb_dev.usb_dev->epmaxpacketout[EP_OBC_ISO_OUT]) / 2);  //bytes to Words
	ep6_size = ((unicorn_usb_dev.usb_dev->epmaxpacketout[EP_OBC_INT_OUT]) / 2);  //bytes to Words
#else
	ep2_size = ((usb_maxpacket(unicorn_usb_dev.usb_dev,usb_sndisocpipe(unicorn_usb_dev.usb_dev, EP_OBC_ISO_OUT),1)) / 2);  //bytes to Words
	ep6_size = ((usb_maxpacket(unicorn_usb_dev.usb_dev,usb_sndintpipe(unicorn_usb_dev.usb_dev,EP_OBC_INT_OUT),1))  / 2);  //bytes to Words
#endif

	if(requestedSize <= ep6_size) 
	{
		*epOutSize = ep6_size;
		bestEpOut = EP6;
	}
	else 
	{
		*epOutSize = ep2_size;
		bestEpOut = EP2;
	}

	return bestEpOut;
}

T_EpIn  selectBestEpIn(WORD requestedSize)
{
	UINT  ep3_size = 0;  
	UINT  ep7_size = 0;
	T_EpIn bestEpIn;

#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
	ep3_size = ((unicorn_usb_dev.usb_dev->epmaxpacketout[EP_OBC_ISO_IN]) / 2);  //bytes to Words
	ep7_size = ((unicorn_usb_dev.usb_dev->epmaxpacketout[EP_OBC_INT_IN]) / 2);  //bytes to Words
#else
	ep3_size = ((usb_maxpacket(unicorn_usb_dev.usb_dev, usb_rcvisocpipe(unicorn_usb_dev.usb_dev, EP_OBC_ISO_IN),0)) / 2);  //bytes to Words
	ep7_size = ((usb_maxpacket(unicorn_usb_dev.usb_dev, usb_rcvintpipe(unicorn_usb_dev.usb_dev,  EP_OBC_INT_IN),0)) / 2);  //bytes to Words

#endif

	if(requestedSize <= 8)
	{
		bestEpIn = EP7;
	}
	else
	{
		bestEpIn = EP3;
	}

	return bestEpIn;
}


/***************************************WRITE SINGLE****************************************/
ST_STATUS Cinterface_writeSingle(WORD address, WORD data)
{
	ST_STATUS WriteResult;
	T_ShortWrite CommandOut;
	WORD size;
	down(&acc_sema);
	USB_ProtocolCreator_writeSingle(unicorn_usb_dev.usb_mem->CmdBufW1, &size, address, data);

	CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
	CommandOut.frameSize = size;

	WriteResult = USB_S_Write(&CommandOut, EP6);

	up(&acc_sema);

	if (WriteResult == SUCCESS) 
	{
		return SUCCESS;
	}

	PRINT_ERROR("single WRITE ACTION FAILED \n");
	return FAILURE;
}

/***************************************WRITE MASKED****************************************/
ST_STATUS Cinterface_writeMasked(WORD address, WORD data, WORD mask)
{
	ST_STATUS    WriteResult;
	T_ShortWrite CommandOut;
	WORD         size;
	down(&acc_sema);	
	USB_ProtocolCreator_writeMasked(unicorn_usb_dev.usb_mem->CmdBufW1, &size, address, mask, data);

	CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
	CommandOut.frameSize = size;

	WriteResult = USB_S_Write(&CommandOut, EP6);

	up(&acc_sema);

	if(WriteResult == SUCCESS) 
	{
		return SUCCESS;
	}

	return FAILURE;
}

/***************************************WRITE BLOCK****************************************/

ST_STATUS Cinterface_writeBlock(BlockAccess* block, WORD blockSize_blocks)
{
	WORD epOutSize = 0;
	ST_STATUS       writeResult = FAILURE;
	T_ShortWrite    commandOut;
	T_LongWrite	  commandOutLong;
	T_EpOut         epOut;

	WORD totalSize = 0;       
	WORD size = 0;

	WORD nFrames = 0;
	WORD frameSize = 0;
	WORD lastFrameSize = 0;

	WORD  blockSize_words = blockSize_blocks * 2;
	WORD* data = (WORD*) block;

	UINT numberOfMiddleFrames = 0;
	UINT wordsInFirstFrame = 0;
	UINT wordsInMiddleFrame = 0;
	UINT wordsInLastFrame = 0;
	WORD cmd_buf_size = 0;
	UINT numberOfMidFrame_CMD = 0;
	UINT numberOfCmdbuffer = 0;
	WORD blocksize = 0;  
	WORD remainSize = 0;
	UINT incaddr = 0,k=0;

	totalSize = blockSize_words + HDR_DIR_BLOCK;

	epOut = selectBestEpOut(&epOutSize,totalSize);
	down(&acc_sema);	
	if(totalSize <= epOutSize) 
	{
		USB_ProtocolCreator_writeBlock(unicorn_usb_dev.usb_mem->CmdBufW1, &size, blockSize_words, data, epOut);

		commandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
		commandOut.frameSize = size;

		writeResult = USB_S_Write(&commandOut, epOut);
	}
	else // size > epOutSize
	{
		USB_ProtocolCreator_getNumberOfFrames_Block(epOutSize, blockSize_words, &numberOfMiddleFrames, &wordsInLastFrame, &wordsInFirstFrame, &wordsInMiddleFrame);

		nFrames = numberOfMiddleFrames + 2;
		frameSize = epOutSize;
		lastFrameSize = wordsInLastFrame + 2;

		cmd_buf_size = HDR_DIR_BLOCK + blockSize_words + (2 * (nFrames-1));

		if(cmd_buf_size <= 1024)
		{

			USB_ProtocolCreator_writeBlock_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,numberOfMiddleFrames,wordsInFirstFrame,wordsInMiddleFrame,wordsInLastFrame,blockSize_words, epOutSize, data, epOut,&incaddr);

			commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
			commandOutLong.frameSize = frameSize;
			commandOutLong.lastFrameSize = lastFrameSize;
			commandOutLong.nFrames = nFrames;

			writeResult = USB_L_Write(&commandOutLong, epOut);        
			data += incaddr;

		}
		else
		{

			if((cmd_buf_size % 1024) != 0)
			{
				numberOfCmdbuffer = cmd_buf_size / 1024;
			}
			else
			{
				numberOfCmdbuffer = (cmd_buf_size / 1024) - 1;
			}


			numberOfMidFrame_CMD = (1024 / epOutSize) - 2;


			blocksize = 1024 - HDR_DIR_BLOCK - (2 * (numberOfMidFrame_CMD + 1));

			for( k = 0; k < numberOfCmdbuffer ; k++)
			{

				USB_ProtocolCreator_writeBlock_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,numberOfMidFrame_CMD,wordsInFirstFrame,wordsInMiddleFrame,wordsInMiddleFrame,blocksize, epOutSize, data, epOut,&incaddr);

				commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
				commandOutLong.frameSize = frameSize;
				commandOutLong.lastFrameSize = wordsInMiddleFrame + 2 ;
				commandOutLong.nFrames = numberOfMidFrame_CMD + 2 ;

				writeResult = USB_L_Write(&commandOutLong, epOut);            
				data += incaddr;

			}


			remainSize = totalSize - (blocksize*numberOfCmdbuffer);

			if(remainSize < epOutSize)
			{
				USB_ProtocolCreator_writeBlock(unicorn_usb_dev.usb_mem->CmdBufW1, &size, (remainSize-HDR_DIR_BLOCK), data, epOut);

				commandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
				commandOut.frameSize = size;

				writeResult = USB_S_Write(&commandOut, epOut);
			}
			else
			{
				USB_ProtocolCreator_getNumberOfFrames_Block(epOutSize, (remainSize-HDR_DIR_BLOCK), &numberOfMiddleFrames, &wordsInLastFrame, &wordsInFirstFrame, &wordsInMiddleFrame);      


				USB_ProtocolCreator_writeBlock_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,numberOfMiddleFrames,wordsInFirstFrame,wordsInMiddleFrame,wordsInLastFrame,(remainSize-HDR_DIR_BLOCK), epOutSize, data, epOut,&incaddr);
				commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
				commandOutLong.frameSize = frameSize;
				commandOutLong.lastFrameSize = wordsInLastFrame + 2;
				commandOutLong.nFrames = numberOfMiddleFrames + 2 ;

				writeResult = USB_L_Write(&commandOutLong, epOut);            

			}
		}
	}

	up(&acc_sema);

	if(writeResult == SUCCESS) 
	{
		return SUCCESS;
	}

	PRINT_ERROR("block WRITE ACTION FAILED \n");
	return FAILURE;
}

/***************************************WRITE BURST****************************************/

ST_STATUS Cinterface_writeBurst(WORD startAddress, WORD burstSize_words, WORD* data)
{
	WORD            epOutSize = 0; 
	ST_STATUS       writeResult;
	T_ShortWrite    commandOut;
	T_LongWrite	commandOutLong;
	T_EpOut         epOut;

	WORD totalSize = 0;       
	WORD size = 0;

	WORD nFrames = 0;
	WORD frameSize = 0;
	WORD lastFrameSize = 0;

	UINT numberOfMiddleFrames = 0;
	UINT wordsInFirstFrame = 0;
	UINT wordsInMiddleFrame = 0;
	UINT wordsInLastFrame = 0;
	WORD cmd_buf_size = 0;
	UINT numberOfMidFrame_CMD = 0;
	UINT numberOfCmdbuffer = 0;
	UINT k = 0;
	WORD burstsize = 0;  
	WORD remainSize = 0;
	WORD orgaddress;

	totalSize = burstSize_words + HDR_DIR_BURST;

	epOut = selectBestEpOut(&epOutSize,totalSize);
	down(&acc_sema);
	if(totalSize <= epOutSize) 
	{
		USB_ProtocolCreator_writeBurst(unicorn_usb_dev.usb_mem->CmdBufW1, &size, burstSize_words, startAddress,data, epOut);

		commandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
		commandOut.frameSize = size;

		writeResult = USB_S_Write(&commandOut, epOut);
	}
	else // size > epOutSize
	{
		USB_ProtocolCreator_getNumberOfFrames_Burst(epOutSize, burstSize_words, &numberOfMiddleFrames, &wordsInLastFrame, &wordsInFirstFrame, &wordsInMiddleFrame);

		nFrames = numberOfMiddleFrames + 2;
		frameSize = epOutSize;          
		lastFrameSize = wordsInLastFrame + 2;

		cmd_buf_size = HDR_DIR_BURST+ burstSize_words + (2 * (nFrames - 1));

		if(cmd_buf_size <= 1024)
		{


			USB_ProtocolCreator_writeBurst_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,numberOfMiddleFrames,wordsInFirstFrame,wordsInMiddleFrame,wordsInLastFrame,burstSize_words,&startAddress,epOutSize, data, EP2);          
			commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
			commandOutLong.frameSize = frameSize;
			commandOutLong.lastFrameSize = lastFrameSize;
			commandOutLong.nFrames = nFrames;

			writeResult = USB_L_Write(&commandOutLong, epOut);

		}
		else
		{
			if((cmd_buf_size % 1024) != 0)
			{
				numberOfCmdbuffer = cmd_buf_size / 1024;
			}
			else
			{
				numberOfCmdbuffer = (cmd_buf_size / 1024) - 1;
			}


			numberOfMidFrame_CMD = (1024 / epOutSize) - 2;
			burstsize = 1024 - HDR_DIR_BURST - (2 * (numberOfMidFrame_CMD + 1));

			for(k = 0; k < numberOfCmdbuffer; k++)
			{
				orgaddress = startAddress;
				USB_ProtocolCreator_writeBurst_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,numberOfMidFrame_CMD,wordsInFirstFrame,wordsInMiddleFrame,wordsInMiddleFrame,burstsize,&startAddress,epOutSize, data, EP2);            			
				commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
				commandOutLong.frameSize = frameSize;
				commandOutLong.lastFrameSize = wordsInMiddleFrame + 2;
				commandOutLong.nFrames = numberOfMidFrame_CMD + 2;

				writeResult = USB_L_Write(&commandOutLong, epOut);

				data += ((startAddress - orgaddress) / 2) + 1;
			}

			remainSize = totalSize - (burstsize*numberOfCmdbuffer);

			if(remainSize < epOutSize)
			{
				USB_ProtocolCreator_writeBurst(unicorn_usb_dev.usb_mem->CmdBufW1, &size, (remainSize-HDR_DIR_BURST), startAddress,data, epOut);  

				commandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
				commandOut.frameSize = size;

				writeResult = USB_S_Write(&commandOut, epOut);
			}
			else
			{
				USB_ProtocolCreator_getNumberOfFrames_Burst(epOutSize, (remainSize-HDR_DIR_BURST), &numberOfMiddleFrames, &wordsInLastFrame, &wordsInFirstFrame, &wordsInMiddleFrame);

				USB_ProtocolCreator_writeBurst_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,numberOfMiddleFrames,wordsInFirstFrame,wordsInMiddleFrame,wordsInLastFrame,(remainSize-HDR_DIR_BURST),&startAddress,epOutSize, data, EP2);            

				commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
				commandOutLong.frameSize = frameSize;
				commandOutLong.lastFrameSize = wordsInLastFrame + 2;
				commandOutLong.nFrames = numberOfMiddleFrames + 2;
				writeResult = USB_L_Write(&commandOutLong, epOut);
			}

		}
	}

	up(&acc_sema); 

	if(writeResult == SUCCESS) 
	{
		return SUCCESS;
	}

	PRINT_ERROR(" burst WRITE ACTION FAILED \n");
	return FAILURE;
}


/***************************************WRITE MESSAGE****************************************/
ST_STATUS Cinterface_writeMessage(WORD address, WORD* data, WORD burstSize_words, WORD addressCheck, WORD bitPositionCheck)
{
	WORD            epOutSize = 0; 
	ST_STATUS       writeResult;
	T_ShortWrite    commandOut;
	T_LongWrite	  commandOutLong;
	T_EpOut         epOut;

	WORD totalSize = 0;       
	WORD size = 0;

	WORD nFrames = 0;
	WORD frameSize = 0;
	WORD lastFrameSize = 0;

	totalSize = burstSize_words + HDR_MESSAGE;


	epOut = selectBestEpOut(&epOutSize,totalSize);
	down(&acc_sema);
	if(totalSize <= epOutSize) 
	{
		USB_ProtocolCreator_writeMessage(unicorn_usb_dev.usb_mem->CmdBufW1,&size,address,burstSize_words,data,addressCheck,bitPositionCheck,epOut);

		commandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
		commandOut.frameSize = size;

		writeResult = USB_S_Write(&commandOut, epOut);
	}
	else
	{
		USB_ProtocolCreator_writeMessage_MultipleFrames(unicorn_usb_dev.usb_mem->CmdBufW1,&nFrames,&frameSize,&lastFrameSize,data,burstSize_words,address,addressCheck, bitPositionCheck,epOut,epOutSize);

		commandOutLong.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
		commandOutLong.frameSize = frameSize;
		commandOutLong.lastFrameSize = lastFrameSize;
		commandOutLong.nFrames = nFrames;

		writeResult = USB_L_Write(&commandOutLong, epOut);
	}

	up(&acc_sema);	

	if(writeResult == SUCCESS) 
	{
		return SUCCESS;
	}

	PRINT_ERROR("message WRITE ACTION FAILED \n");
	return FAILURE;
}

/***************************************READ SINGLE****************************************/
ST_STATUS Cinterface_readSingle (WORD address, WORD* data)
{   
	ST_STATUS ReadResult;
	T_ShortRead CommandOut;
	WORD size = 0;
	T_EpOut epOut;
	T_EpIn  epIn;
	epOut = EP6;
	epIn  = EP7;
	down(&acc_sema);

	USB_ProtocolCreator_readSingle(unicorn_usb_dev.usb_mem->CmdBufW1, &size, address, epIn);

	CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
	CommandOut.wrSize = size;
	CommandOut.rdSize = 1;

	ReadResult = USB_S_Read(&CommandOut, epOut, epIn);

	up(&acc_sema);	

	if (ReadResult == SUCCESS) 
	{
		*data = unicorn_usb_dev.usb_mem->CmdBufRd[0];
		return SUCCESS;
	}
	return FAILURE;
}

/***************************************READ BLOCK****************************************/

ST_STATUS Cinterface_readBlock(WORD* addresses, WORD blockSize, WORD* data)
{   
	ST_STATUS ReadResult =FAILURE;
	T_LongRead CommandOut;
	WORD size;
	WORD epOutSize = 0;
	T_EpOut epOut;
	T_EpIn  epIn;

	WORD numberOfCmdBuf = 0;
	UINT j = 0;
	UINT i = 0;
	UINT k = 0;
	WORD remainSize;

	epOut = selectBestEpOut(&epOutSize,(HDR_DIR_BLOCK + blockSize));
	epIn  = selectBestEpIn(blockSize);
	down(&acc_sema);
	if(blockSize <= 512)
	{
		USB_ProtocolCreator_readBlock(unicorn_usb_dev.usb_mem->CmdBufW1, &size, blockSize, addresses, epOut,epIn);

		CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
		CommandOut.wrSize = size;
		CommandOut.rdSize = blockSize;

		ReadResult = USB_L_Read(&CommandOut, epOut, epIn);

		if (ReadResult == SUCCESS) 
		{
			for( i = 0; i < blockSize; i++)
			{
				data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i];
			}
		}
	}
	else
	{
		numberOfCmdBuf = blockSize / 512;

		for(k = 0; k < numberOfCmdBuf; k++)
		{
			USB_ProtocolCreator_readBlock(unicorn_usb_dev.usb_mem->CmdBufW1, &size, 512 - HDR_DIR_BLOCK, addresses, epOut,epIn);
			CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
			CommandOut.wrSize = size;
			CommandOut.rdSize = 512 - HDR_DIR_BLOCK;

			ReadResult = USB_L_Read(&CommandOut, epOut, epIn);

			if (ReadResult == SUCCESS) 
			{
				for(i = j; i < (512 - HDR_DIR_BLOCK) + j; i++)
				{            
					data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i - j];
				} 
				j += 512 - HDR_DIR_BLOCK;
				addresses += 512 - HDR_DIR_BLOCK;
			}
		}

		remainSize = blockSize - ((512 - HDR_DIR_BLOCK) * numberOfCmdBuf);

		if(remainSize != 0)
		{
			USB_ProtocolCreator_readBlock(unicorn_usb_dev.usb_mem->CmdBufW1, &size, remainSize, addresses, epOut,epIn);
			CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
			CommandOut.wrSize = size;
			CommandOut.rdSize = remainSize;

			ReadResult = USB_L_Read(&CommandOut, epOut, epIn);
			if (ReadResult == SUCCESS) 
			{
				for(i = j; i < (j + remainSize); i++)
				{
					data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i - j];
				}      
			}
		}

	}

	up(&acc_sema);	

	if (ReadResult == SUCCESS) 
	{
		return SUCCESS;
	}

	PRINT_ERROR("block READ ACTION FAILED \n");
	return FAILURE;
}

/***************************************READ BURST****************************************/
ST_STATUS Cinterface_readBurst(WORD startAddress, WORD burstSize, WORD* data)
{

	ST_STATUS ReadResult=FAILURE;
	T_LongRead CommandOut;
	WORD size;
	WORD epOutSize;   
	T_EpOut epOut;
	T_EpIn  epIn;

	WORD numberOfCmdBuf = 0;
	UINT j = 0;
	UINT i = 0;
	UINT k = 0;
	UINT l = 0;
	WORD remainSize;

	epOut = selectBestEpOut(&epOutSize,HDR_DIR_BURST);
	epIn  = selectBestEpIn(burstSize);
	down(&acc_sema);
	if(burstSize <= 512)
	{
		USB_ProtocolCreator_readBurst(unicorn_usb_dev.usb_mem->CmdBufW1, &size, burstSize, startAddress, epOut, epIn);

		CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
		CommandOut.wrSize = size;
		CommandOut.rdSize = burstSize;

		ReadResult = USB_L_Read(&CommandOut, epOut, epIn);

		if (ReadResult == SUCCESS) 
		{
			for(i = 0; i < burstSize; i++)
			{
				data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i];
			}
		}
	}
	else
	{
		numberOfCmdBuf = burstSize / 512;

		for(j = 0, k = 0; k < numberOfCmdBuf; k++)
		{
			USB_ProtocolCreator_readBurst(unicorn_usb_dev.usb_mem->CmdBufW1, &size, 512, startAddress, epOut, epIn);

			CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
			CommandOut.wrSize = size;
			CommandOut.rdSize = 512;

			ReadResult = USB_L_Read(&CommandOut, epOut, epIn);            

			if (ReadResult == SUCCESS) 
			{

				for(l = 0, i = j; i < (512 + j); i++, l++)
				{
					data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i-j];
				}    

				j += 512;
				startAddress += 1024;
			}

		}
		remainSize = burstSize - (512 * numberOfCmdBuf);

		if(remainSize != 0)
		{
			USB_ProtocolCreator_readBurst(unicorn_usb_dev.usb_mem->CmdBufW1, &size, remainSize, startAddress, epOut, epIn);

			CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
			CommandOut.wrSize = size;
			CommandOut.rdSize = remainSize;

			ReadResult = USB_L_Read(&CommandOut, epOut, epIn);

			if (ReadResult == SUCCESS) 
			{
				for(l = 0, i = j; i < (j + remainSize); i++, l++)
				{
					data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i - j];
				}
			}
		}
	}

	up(&acc_sema);

	if (ReadResult == SUCCESS) 
	{
		return SUCCESS;
	}

	PRINT_ERROR("burst READ ACTION FAILED \n");
	return FAILURE;
}


/*******************************READ MESSAGE**********************************************************/
ST_STATUS Cinterface_readMessage(WORD address, WORD* data, WORD* returnSize, WORD addressCheck, WORD bitPositionCheck, WORD maxReadSize)
{

	ST_STATUS ReadResult;
	T_LongRead CommandOut;
	WORD size;
	WORD epOutSize;

	T_EpOut epOut;
	T_EpIn  epIn;
	UINT i=0;

	epOut = selectBestEpOut(&epOutSize,HDR_MESSAGE);
	epIn  = selectBestEpIn(maxReadSize);
	down(&acc_sema);
	USB_ProtocolCreator_readMessage(unicorn_usb_dev.usb_mem->CmdBufW1, &size, address, maxReadSize, addressCheck, bitPositionCheck, epOut, epIn);

	CommandOut.cmdBuff = unicorn_usb_dev.usb_mem->CmdBufW1;
	CommandOut.wrSize = size;
	CommandOut.rdSize = maxReadSize;

	ReadResult = USB_L_Read(&CommandOut, epOut, epIn);

	up(&acc_sema);

	if (ReadResult == SUCCESS) 
	{
		for(  i = 0; i < CommandOut.rdSize; i++)
		{
			data[i] = unicorn_usb_dev.usb_mem->CmdBufRd[i];
		}
		*returnSize = CommandOut.rdSize;
		return SUCCESS;
	}

	PRINT_ERROR("readMessage action failed \n");
	return FAILURE;
}

ST_STATUS Cinterface_setBitIrqMaskTable(BYTE regIndex, BYTE irqNr)
{
	tosca_softITABLE[regIndex + 14 ] |= (1 << irqNr);
	return SUCCESS ; 
}

ST_STATUS Cinterface_clearBitIrqMaskTable(BYTE regIndex, BYTE irqNr)
{
	tosca_softITABLE[regIndex +14 ] &= ~(1 << irqNr);
	return SUCCESS;
}

ST_STATUS Cinterface_setIrqMaskTableEntry(BYTE regIndex, WORD mask)
{
	tosca_softITABLE[regIndex + 14 ] = mask;
	return SUCCESS;
}

ST_STATUS Cinterface_getIrqTableEntry(BYTE regIndex, WORD *tableEntry)
{
	*tableEntry = tosca_softITABLE[regIndex];
	return SUCCESS;
}


