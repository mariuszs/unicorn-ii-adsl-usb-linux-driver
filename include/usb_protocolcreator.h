#ifndef _USB_PROTOCOLCREATOR_H_
#define _USB_PROTOCOLCREATOR_H_

/* ###########  Control Commands Bitfields Configuration  ########### */

#define FIRST                   0x0001                  /* 0000 0000 0000 0001 */
#define LAST                    0x0002                  /* 0000 0000 0000 0010 */
#define ONE                     0x0003                  /* 0000 0000 0000 0011 */
#define C_IDLE									0x0000                  /* 0000 0000 0000 0000 */
 
#define EP2_OUT  0x0008                 /* 0000 0000 0000 1000 */
#define EP6_OUT  0x0004                 /* 0000 0000 0000 0100 */
#define EP3_IN   0x00A0                 /* 0000 0000 1010 0000 */
#define EP7_IN   0x0090									/* 0000 0000 1001 0000 */
#define EP5_IN   0x0040                 /* 0000 0000 0100 0000 */

#define E_WR_LINK   0x0300              /* 0000 0011 0000 0000 */
#define E_WR_NLINK  0x0100              /* 0000 0001 0000 0000 */

#define REG_POS     10                  /* bit position of the ADDR field of byte 1 in Control Command */

/* ###########  Control Commands Length for various access modes ########### */
 
#define HDR_DIR_READ   4    /* Control Commands Length for a Single Direct Read (16-bit Words) */
#define HDR_DIR_WRITE  6    /* Control Commands Length for a Single Direct Write (16-bit Words) */
#define HDR_IND_READ   6    /* Control Commands Length for a Single Indirect Read (16-bit Words) */
#define HDR_IND_WRITE  12   /* Control Commands Length for a Single Indirect Write (16-bit Words) */
#define HDR_RM_WRITE   8    /* Control Commands Length for a Read_Modify_Write (16-bit Words) */

#define HDR_DIR_BURST  6      /* Control Commands Length in Rd/Wr Direct Burst (16-bit Words) */
#define HDR_IND_BURST  8      /* Control Commands Length in Rd/Wr Indirect Burst (16-bit Words) */
#define HDR_DIR_BLOCK  2      /* Control Commands Length in Rd/Wr Direct Block (16-bit Words) */
#define HDR_IND_BLOCK  4      /* Control Commands Length in Rd/Wr Indirect Block (16-bit Words) */
#define HDR_MESSAGE    10     /* Control Commands Length in Rd/Wr Message (16-bit Words) */

#define IND_BLOCK_SIZE  3
/* ###########  Address Field [15:10] in Control Commands for ISO and INT pipes ########### */
 
#define C_STATUS        STATUS  << REG_POS    /* Status Control Register in Control Command   */
#define C_IDATA_1       IDATA_1 << REG_POS    /* Indirect Data Register 1 (LSB) */
#define C_IDATA_2       IDATA_2 << REG_POS    /* Indirect Data Register 2       */
#define C_IDATA_3       IDATA_3 << REG_POS    /* Indirect Data Register 3 (MSB) */
#define C_IDATA         IDATA << REG_POS      /* Indirect Data 3 Register (0)   */
#define C_IADDR          IADDR << REG_POS       /* Indirect Address Register      */
#define C_IMASKS        IMASKS << REG_POS     /* Mask to apply to the Slave 
																									Indirect R/W operations        */
#define C_IRMW          IRMW << REG_POS       /* Mask to apply when the Slave   
																									Read-Modify-Write access is used */

#define C_IADDR_TX       IADDR_TX << REG_POS     /* Indirect TX Reference Address Register */
#define C_IADDR_RX       IADDR_RX  << REG_POS    /* Indirect RX Reference Address Register */
#define C_IADDR_MSG      IADDR_MSG  << REG_POS   /* Indirect MSG Reference Address */
#define C_IADDR_CHK      IADDR_CHK << REG_POS    /* Indirect Address Check Register */
#define C_IADDR_BURST    IADDR_BURST << REG_POS  /* Burst Base Address Register */
#define C_SIZE_BURST    SIZE_BURST << REG_POS  /* Number of Burst Accesses to be performed */
#define C_IMASKM        IMASKM << REG_POS      /* Mask to apply to the Master Indirect or MSG */


/************************************************************/

  void USB_ProtocolCreator_writeSingle(WORD* commandBuffer,  WORD* wrSize, WORD address, WORD data);
  void USB_ProtocolCreator_writeMasked(WORD* commandBuffer, WORD* wrSize, WORD address, WORD mask, WORD data);
  void USB_ProtocolCreator_writeBlock(WORD* commandBuffer, WORD* wrSize, UINT BlockSize, WORD *data, T_EpOut epOut);
  void USB_ProtocolCreator_writeBlock_MultipleFrames(WORD* command_Buffer, UINT numberOfMiddleFrames, WORD wordsInFirstFrame, UINT wordsInMiddleFrame, UINT wordsInLastFrame, UINT blockSize, UINT epOutSize, WORD *data,T_EpOut epOut, UINT *count);
  void USB_ProtocolCreator_writeBurst(WORD* commandBuffer, WORD* wrSize, UINT burstSize,WORD startAddress, WORD *data,T_EpOut epOut);
  void USB_ProtocolCreator_writeBurst_MultipleFrames(WORD* command_Buffer, UINT numberOfMiddleFrames, UINT wordsInFirstFrame, UINT wordsInMiddleFrame,UINT wordsInLastFrame, UINT burstSize, WORD *startAddress, UINT epOutSize, WORD *data,T_EpOut epOut);
  void USB_ProtocolCreator_writeMessage(WORD* commandBuffer, WORD* wrSize, WORD fifoAddress, WORD burstSize, WORD* data, WORD addressCheck, WORD bitPositionCheck, T_EpOut epOut);
  void USB_ProtocolCreator_writeMessage_MultipleFrames(WORD* command_Buffer, WORD* nFrames, WORD* frameSize, WORD* lastFrameSize, WORD* data,WORD burstSize, WORD fifoAddress, WORD addressCheck, WORD bitPositionCheck, T_EpOut epOut, UINT epOutSize);
  void USB_ProtocolCreator_readSingle(WORD* commandBuffer, WORD* wrSize, WORD address,T_EpIn epIn);
  void USB_ProtocolCreator_readBurst(WORD* commandBuffer, WORD* wrSize, WORD burstSize, WORD address,T_EpOut epOut, T_EpIn epIn);
  void USB_ProtocolCreator_readBlock(WORD* commandBuffer, WORD* wrSize, WORD blockSize, WORD *addresses, T_EpOut epOut, T_EpIn epIn);
  void USB_ProtocolCreator_readMessage(WORD* commandBuffer, WORD* wrSize, WORD fifoAddress, WORD maxReadSize, WORD addressCheck, WORD bitPositionCheck, T_EpOut epOut, T_EpIn epIn);
  void USB_ProtocolCreator_getNumberOfFrames_Block(UINT EPOutSize, UINT DataSize, UINT *NumberOfMiddleFrames, UINT *WordsInLastFrame, UINT *WordsInFirstFrame, UINT *WordsInMiddleFrame);
  void USB_ProtocolCreator_getNumberOfFrames_Burst(UINT EpOutSize, UINT DataSize, UINT *NumberOfMiddleFrames, UINT *WordsInLastFrame, UINT *WordsInFirstFrame, UINT *WordsInMiddleFrame);

  
  #endif
