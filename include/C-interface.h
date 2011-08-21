#ifndef _CINTERFACE_H
#define _CINTERFACE_H


#include "adsldeviceitf.h"

class myclass:public AdslDeviceInterface
{
	public:
        myclass();
	 	virtual ~myclass();
		
        virtual ST_STATUS readSingle(WORD address, WORD* data) ;
		virtual ST_STATUS readBurst(WORD startAddress, WORD burstSize, WORD* data);
		virtual ST_STATUS readBlock(WORD* addresses, WORD blockSize, WORD* data) ;
		virtual ST_STATUS readMessage(WORD address, WORD* data, WORD* returnSize, WORD addressCheck, WORD bitPositionCheck, WORD maxReadSize) ;

		virtual ST_STATUS writeSingle(WORD address, WORD data) ;
		virtual ST_STATUS writeMasked(WORD address, WORD data, WORD mask) ;
		virtual ST_STATUS writeBlock(BlockAccess* block, WORD blockSize_blocks);
		virtual ST_STATUS writeBurst(WORD startAddress, WORD burstSize_words, WORD* data);
		virtual ST_STATUS writeMessage(WORD address, WORD* data, WORD burstSize_words, WORD addressCheck, WORD bitPositionCheck) ;

		virtual ST_STATUS setBitIrqMaskTable(BYTE regIndex, BYTE irqNr) ;
		virtual ST_STATUS clearBitIrqMaskTable(BYTE regIndex, BYTE irqNr);
		virtual ST_STATUS setIrqMaskTableEntry(BYTE regIndex, WORD mask) ;
		virtual ST_STATUS getIrqTableEntry(BYTE regIndex, WORD* tableEntry);
};
#endif
