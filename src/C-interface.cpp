//----------------------------------------------------------------------
// Test driver for the STMicroelectronics ADSL Chip Taurus
//----------------------------------------------------------------------
// file: usbdrv.cpp
// Author: 
// Copyright 
// Copyright STMicroelectronics 2000
//----------------------------------------------------------------------
// 
//----------------------------------------------------------------------
// This file contains code specific to the USB requirements
// of the ADSL Taurus driver
//----------------------------------------------------------------------

#include "../include/Typesdrv.h"
#include "../include/C-interface.h"

AdslDeviceInterface *adslDeviceInterface;

/***************************************WRITE SINGLE****************************************/
myclass::myclass()
{
}

myclass::~myclass()
{
}

#ifdef __cplusplus
extern "C" {
#endif

class myclass *myclassobj;
void createclassobject(void)
{
	myclassobj = new myclass();
	adslDeviceInterface=myclassobj;
}

void deletelassobject(void){
	delete myclassobj;
}
#ifdef __cplusplus
}
#endif 

//#ifdef __cplusplus
extern "C"{
//#endif
ST_STATUS Cinterface_writeSingle(WORD address, WORD data);
ST_STATUS Cinterface_writeMasked(WORD address, WORD data, WORD mask);
ST_STATUS Cinterface_writeBlock(BlockAccess *block, WORD blockSize_blocks);
ST_STATUS Cinterface_writeBurst(WORD startAddress, WORD burstSize_words, WORD* data);
ST_STATUS Cinterface_writeMessage(WORD address, WORD *data, WORD burstSize_words, WORD addressCheck, WORD bitPositionCheck);

ST_STATUS Cinterface_readSingle(WORD address, WORD * data);
ST_STATUS Cinterface_readBlock(WORD* addresses, WORD blockSize, WORD * data);

ST_STATUS Cinterface_readBurst(WORD startAddress, WORD busrtSize, WORD * data);

ST_STATUS Cinterface_readMessage(WORD address, WORD *data, WORD *returnSize, WORD addressCheck, WORD bitPositionCheck, WORD maxReadSize);

ST_STATUS Cinterface_setBitIrqMaskTable(BYTE regIndex, BYTE irqNr);
ST_STATUS Cinterface_clearBitIrqMaskTable(BYTE regIndex, BYTE irqNr);
ST_STATUS Cinterface_setIrqMaskTableEntry(BYTE regIdex, WORD mask);
ST_STATUS Cinterface_getIrqTableEntry(BYTE regIndex, WORD *tableEntry);

//#ifdef __cplusplus
}
//#endif

ST_STATUS myclass::writeSingle(WORD address, WORD data)
{
       return Cinterface_writeSingle(address, data);
}

/***************************************WRITE MASKED****************************************/
ST_STATUS myclass::writeMasked(WORD address, WORD data, WORD mask)
{
   return Cinterface_writeMasked(address,data, mask);
}

/***************************************WRITE BLOCK****************************************/

ST_STATUS myclass::writeBlock(BlockAccess* block, WORD blockSize_blocks)
{
   return Cinterface_writeBlock(block,blockSize_blocks); 
}

/***************************************WRITE BURST****************************************/

ST_STATUS myclass::writeBurst(WORD startAddress, WORD burstSize_words, WORD* data)
{
    return Cinterface_writeBurst(startAddress, burstSize_words, data);
}

/***************************************WRITE MESSAGE****************************************/
ST_STATUS myclass::writeMessage(WORD address, WORD* data, WORD burstSize_words, WORD addressCheck, WORD bitPositionCheck)
{
	return Cinterface_writeMessage(address, data, burstSize_words, addressCheck, bitPositionCheck); 
}

/***************************************READ SINGLE****************************************/
ST_STATUS myclass ::readSingle (WORD address, WORD* data)
{   
	return Cinterface_readSingle(address, data);
}

/***************************************READ BLOCK****************************************/

ST_STATUS myclass::readBlock(WORD* addresses, WORD blockSize, WORD* data)
{   
        return Cinterface_readBlock(addresses, blockSize, data);
}

/***************************************READ BURST****************************************/
ST_STATUS myclass::readBurst(WORD startAddress, WORD burstSize, WORD* data)
{
      return Cinterface_readBurst(startAddress, burstSize, data);
}

/*******************************READ MESSAGE**********************************************************/
ST_STATUS myclass::readMessage(WORD address, WORD* data, WORD* returnSize, WORD addressCheck, WORD bitPositionCheck, WORD maxReadSize)
{
	return Cinterface_readMessage(address, data, returnSize, addressCheck, bitPositionCheck, maxReadSize);
}

/************************************************************************************************************/
ST_STATUS myclass::setBitIrqMaskTable(BYTE regIndex, BYTE irqNr)
{
	return Cinterface_setBitIrqMaskTable(regIndex, irqNr);
}

ST_STATUS myclass::clearBitIrqMaskTable(BYTE regIndex, BYTE irqNr)
{
	return Cinterface_clearBitIrqMaskTable(regIndex, irqNr);
}

ST_STATUS myclass::setIrqMaskTableEntry(BYTE regIndex, WORD mask)
{
	return Cinterface_setIrqMaskTableEntry(regIndex, mask);
}

ST_STATUS myclass::getIrqTableEntry(BYTE regIndex, WORD *tableEntry)
{
	return Cinterface_getIrqTableEntry(regIndex, tableEntry);

}


