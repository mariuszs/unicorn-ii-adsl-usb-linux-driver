#ifndef _ADSLINTERFACE_H
#define _ADSLINTERFACE_H

#ifdef UNICORN_COMMON
#ifndef UNICORN_WIN
#include "USB_ProtocolCreator.h"
#endif
#endif
                                   
struct BlockAccess
{
	WORD address;
       	WORD data;
};

struct DataBlockIndirect
{
      WORD data0;
      WORD data1;
      WORD data2;
};


class AdslDeviceInterface
{
public:

  enum T_Path {TX_PATH,RX_PATH};
  virtual ST_STATUS readSingle(WORD address, WORD* data) = 0;
  virtual ST_STATUS readBurst(WORD startAddress, WORD burstSize, WORD* data) = 0;
  virtual ST_STATUS readBlock(WORD* addresses, WORD blockSize, WORD* data) = 0;
  virtual ST_STATUS readMessage(WORD address, WORD* data, WORD* returnSize, WORD addressCheck, WORD bitPositionCheck, WORD maxReadSize) = 0;

  virtual ST_STATUS writeSingle(WORD address, WORD data) = 0;
  virtual ST_STATUS writeMasked(WORD address, WORD data, WORD mask) = 0;
  virtual ST_STATUS writeBlock(BlockAccess* block, WORD blockSize_blocks) = 0;   
  virtual ST_STATUS writeBurst(WORD startAddress, WORD burstSize_words, WORD* data) = 0;
  virtual ST_STATUS writeMessage(WORD address, WORD* data, WORD burstSize_words, WORD addressCheck, WORD bitPositionCheck) = 0;

  virtual ST_STATUS setBitIrqMaskTable(BYTE regIndex, BYTE irqNr) = 0;
  virtual ST_STATUS clearBitIrqMaskTable(BYTE regIndex, BYTE irqNr) = 0;
  virtual ST_STATUS setIrqMaskTableEntry(BYTE regIndex, WORD mask) = 0;
  virtual ST_STATUS getIrqTableEntry(BYTE regIndex, WORD* tableEntry) = 0;
};
#endif
