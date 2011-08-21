#include "../include/types.h"
#include "../include/hard.h"

#include "../include/hal.h"
#include "../include/tracetool.h"
#include "../include/usb_protocolcreator.h"

static void USB_ProtocolCreator_getNumberOfFrames_Message(UINT EpOutSize, UINT DataSize, UINT *NumberOfMiddleFrames, UINT *WordsInLastFrame, UINT *WordsInFirstFrame, UINT *WordsInMiddleFrame);


WORD pathMask[2] = {TX_MASK,RX_MASK};
WORD epOutMsk[2] = {EP2_OUT, EP6_OUT};
WORD epInMsk[3]  = {EP3_IN, EP5_IN, EP7_IN};


/**************************************WRITE SINGLE************************************/
void USB_ProtocolCreator_writeSingle(WORD* commandBuffer, WORD* wrSize, WORD address, WORD data)
{
  *wrSize = HDR_DIR_WRITE;

  *commandBuffer++ = C_IDLE | E_WR_LINK | C_IADDR;
  *commandBuffer++ = address;
  *commandBuffer++ = E_WR_LINK | C_IDATA;
  *commandBuffer++ = data;
  *commandBuffer++ = E_WR_NLINK | C_STATUS;
  *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | SEX_MASK | SLAVE_DIR | TX_MASK;
  
  return;
}


/**************************************WRITE MASKED************************************/
void USB_ProtocolCreator_writeMasked(WORD* commandBuffer, WORD* wrSize, WORD address, WORD mask, WORD data)
{
  *wrSize = HDR_RM_WRITE;

  *commandBuffer++ = C_IDLE | E_WR_LINK | C_IADDR; 
  *commandBuffer++ = address;
  *commandBuffer++ = E_WR_LINK | C_IRMW;
  *commandBuffer++ = mask;
  *commandBuffer++ = E_WR_LINK | C_IDATA;
  *commandBuffer++ = data;
  *commandBuffer++ = E_WR_NLINK | C_STATUS;
  *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | SEX_MASK | SLAVE_RM | TX_MASK;

  return;
}

/**************************************WRITE BLOCK************************************/
void USB_ProtocolCreator_writeBlock(WORD* commandBuffer, WORD* wrSize, UINT BlockSize, WORD *data,T_EpOut epOut)
{
    *wrSize = BlockSize + HDR_DIR_BLOCK;
    unsigned i;

    *commandBuffer++ = ONE | E_WR_NLINK | C_STATUS | epOutMsk[epOut];
    *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | TX_MASK | MEX_MASK | MASTER_BLOCK_DIR;
    
    for(i = 0; i < BlockSize; i++)
    {
       *commandBuffer++ = *data++;
    }                                     
}

void USB_ProtocolCreator_writeBlock_MultipleFrames(WORD* command_Buffer, UINT numberOfMiddleFrames, WORD wordsInFirstFrame, UINT wordsInMiddleFrame, UINT wordsInLastFrame, UINT blockSize, UINT epOutSize, WORD *data,T_EpOut epOut, UINT *incaddr)
{

    WORD *commandBuffer = command_Buffer;
   UINT count = 0;
   UINT i;
   UINT j;
   UINT k;
   UINT l;
    //first Frame
    *commandBuffer++ = FIRST | E_WR_NLINK | C_STATUS | epOutMsk[epOut] ;
    *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | TX_MASK | MEX_MASK | MASTER_BLOCK_DIR;
    for( i = 0; i < wordsInFirstFrame; i++)
    {
      *commandBuffer++ = *data++;
	  count++;
    }
    
    //middle frames
    for ( j = 0; j < numberOfMiddleFrames; j++)
    {
        *commandBuffer++ = C_IDLE;
        *commandBuffer++ = C_IDLE;
      
        for( k = 0; k < wordsInMiddleFrame; k++)
        {
            *commandBuffer++ = *data++;
	        count++;
        }
    }
    
    //last frame
    *commandBuffer++ = LAST;
    *commandBuffer++ = C_IDLE;
    for( l = 0; l < wordsInLastFrame; l++)
    {
        *commandBuffer++ = *data++;
	    count++;
    }
    *incaddr = count;

}

void USB_ProtocolCreator_getNumberOfFrames_Block(UINT EpOutSize, UINT DataSize, UINT *NumberOfMiddleFrames, UINT *WordsInLastFrame, UINT *WordsInFirstFrame, UINT *WordsInMiddleFrame)
{

  UINT RemainingWords = DataSize;
  UINT MiddleFrames = 0;
  
//FIRST FRAME
  
  *WordsInFirstFrame = EpOutSize - HDR_DIR_BLOCK;
  RemainingWords -= *WordsInFirstFrame;
 
  //MIDDLE FRAMES
  while((RemainingWords+2) > EpOutSize)
  {
        MiddleFrames++;    
        RemainingWords -= (EpOutSize - 2);
  }
  *WordsInMiddleFrame = EpOutSize - 2;
  *NumberOfMiddleFrames = MiddleFrames;

  //LAST FRAME
  *WordsInLastFrame = RemainingWords;

  return;
}


/**************************************WRITE BURST************************************/
void USB_ProtocolCreator_writeBurst(WORD* commandBuffer, WORD* wrSize, UINT burstSize,WORD startAddress, WORD *data, T_EpOut epOut)
{
    UINT i;
    *wrSize = burstSize + HDR_DIR_BURST;

    *commandBuffer++ = ONE | E_WR_LINK | C_IADDR_BURST | epOutMsk[epOut];
    *commandBuffer++ = startAddress;
    *commandBuffer++ = E_WR_LINK | C_SIZE_BURST;
    *commandBuffer++ = burstSize;
    *commandBuffer++ = E_WR_NLINK | C_STATUS;
    *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | TX_MASK | MEX_MASK | MASTER_BURST_DIR;

    for(i = 0; i < burstSize; i++)
    {
        *commandBuffer++ = *data++;
    }
    return;
}

void USB_ProtocolCreator_writeBurst_MultipleFrames(WORD* command_Buffer, UINT numberOfMiddleFrames, UINT wordsInFirstFrame, UINT wordsInMiddleFrame,UINT wordsInLastFrame, UINT burstSize, WORD* startAddress, UINT epOutSize, WORD *data,T_EpOut epOut)
{
    
    WORD *commandBuffer = command_Buffer;
    WORD  address =0;    
    
    UINT i = 0;
    UINT j = 0;
    UINT k = 0;
    UINT l = 0;

    //first Frame
    *commandBuffer++ = FIRST | E_WR_LINK | C_IADDR_BURST  | epOutMsk[epOut];
    *commandBuffer++ = *startAddress;
    *commandBuffer++ = E_WR_LINK | C_SIZE_BURST;
    *commandBuffer++ = burstSize;
    *commandBuffer++ = E_WR_NLINK | C_STATUS;
    *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | TX_MASK | MEX_MASK | MASTER_BURST_DIR;

    for( i = 0; i < wordsInFirstFrame; i++)
    {
        *commandBuffer++ = *data++;
         address++;		 
	}
    
    //middle frames
    for ( j = 0; j < numberOfMiddleFrames; j++)
    {
        *commandBuffer++ = C_IDLE;
        *commandBuffer++ = C_IDLE;
        for( k = 0; k < wordsInMiddleFrame; k++)
        {
            *commandBuffer++ = *data++;
            address++;	
		 
        }
    }
    
    //last frame
    *commandBuffer++ = LAST;
    *commandBuffer++ = C_IDLE;
    for( l = 0; l < wordsInLastFrame; l++)
    {
        *commandBuffer++ = *data++;
         address++;		 
		 
    }
	
    *startAddress = *startAddress + (address*2);		
		

}

void USB_ProtocolCreator_getNumberOfFrames_Burst(UINT EpOutSize, UINT DataSize, UINT *NumberOfMiddleFrames, UINT *WordsInLastFrame, UINT *WordsInFirstFrame, UINT *WordsInMiddleFrame)
{

    UINT RemainingWords = DataSize;
    UINT MiddleFrames = 0;
    
    //FIRST FRAME
    *WordsInFirstFrame = EpOutSize - HDR_DIR_BURST;
    RemainingWords -= *WordsInFirstFrame;
   
    //MIDDLE FRAMES
    while((RemainingWords+2) > EpOutSize)
    {
        MiddleFrames++;    
        RemainingWords -= (EpOutSize - 2);
    }
    *WordsInMiddleFrame = EpOutSize - 2;
    *NumberOfMiddleFrames = MiddleFrames;

    //LAST FRAME
    *WordsInLastFrame = RemainingWords;

    return;
}
    
/**************************************WRITE MESSAGE************************************/

void USB_ProtocolCreator_writeMessage(WORD* commandBuffer, WORD* wrSize, WORD fifoAddress, WORD burstSize, WORD* data, WORD addressCheck, WORD bitPositionCheck, T_EpOut epOut)
{
    UINT i;
    *wrSize = burstSize + HDR_MESSAGE;

                                           
    *commandBuffer++ = ONE | E_WR_LINK | C_IADDR_MSG | epOutMsk[epOut];
    *commandBuffer++ = fifoAddress;
    *commandBuffer++ = E_WR_LINK | C_IADDR_CHK;
    *commandBuffer++ = addressCheck;
    *commandBuffer++ = E_WR_LINK | C_SIZE_BURST;
    *commandBuffer++ = burstSize;
    *commandBuffer++ = E_WR_LINK | C_IMASKM;
    *commandBuffer++ = 1 << bitPositionCheck;
    *commandBuffer++ = E_WR_NLINK | C_STATUS;
    *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | TX_MASK | MEX_MASK | MASTER_MSG;
    
    for ( i=0; i<burstSize;i++) 
    {
        *commandBuffer++ = *data++;
    }
}

void USB_ProtocolCreator_writeMessage_MultipleFrames(WORD* commandBuffer, WORD* nFrames, WORD* frameSize, WORD* lastFrameSize, WORD* data, WORD burstSize, WORD fifoAddress, WORD addressCheck, WORD bitPositionCheck, T_EpOut epOut, UINT epOutSize)
{
   UINT numberOfMiddleFrames = 0;
   UINT wordsInFirstFrame = 0;
   UINT wordsInMiddleFrame = 0;
   UINT wordsInLastFrame = 0;
   UINT i;
   UINT j;
   UINT k;
   UINT l;   
   USB_ProtocolCreator_getNumberOfFrames_Message(epOutSize, burstSize, &numberOfMiddleFrames, &wordsInLastFrame, &wordsInFirstFrame, &wordsInMiddleFrame);

   *nFrames = numberOfMiddleFrames+2;
   *frameSize = epOutSize;
   *lastFrameSize = wordsInLastFrame+2;

   //first Frame
   *commandBuffer++ = FIRST | E_WR_LINK | C_IADDR_MSG | epOutMsk[epOut];
   *commandBuffer++ = fifoAddress;
   *commandBuffer++ = E_WR_LINK | C_IADDR_CHK;
   *commandBuffer++ = addressCheck;
   *commandBuffer++ = E_WR_LINK | C_SIZE_BURST;
   *commandBuffer++ = burstSize;
   *commandBuffer++ = E_WR_LINK | C_IMASKM;
   *commandBuffer++ = 1 << bitPositionCheck;
   *commandBuffer++ = E_WR_NLINK | C_STATUS;
   *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | WR_MASK | TX_MASK|  MEX_MASK | MASTER_MSG;

   for( i = 0; i < wordsInFirstFrame; i++)
   {
      *commandBuffer++ = *data++;
   }


   //middle frames
   for ( j = 0; j < numberOfMiddleFrames; j++)
   {
      *commandBuffer++ = C_IDLE;
      *commandBuffer++ = C_IDLE;
      for( k = 0; k < wordsInMiddleFrame; k++)
      {
         *commandBuffer++ = *data++;
      }
   }
    
   //last frame
   *commandBuffer++ = LAST;
   *commandBuffer++ = C_IDLE;
   for( l = 0; l < wordsInLastFrame; l++)
   {
       *commandBuffer++ = *data++;
   }
}

void USB_ProtocolCreator_getNumberOfFrames_Message(UINT EpOutSize, UINT DataSize, UINT *NumberOfMiddleFrames, UINT *WordsInLastFrame, UINT *WordsInFirstFrame, UINT *WordsInMiddleFrame)
{

  UINT RemainingWords = DataSize;
  UINT MiddleFrames = 0;
  
  //FIRST FRAME
  
  *WordsInFirstFrame = EpOutSize - HDR_MESSAGE;
  RemainingWords -= *WordsInFirstFrame;
 
  //MIDDLE FRAMES
  while((RemainingWords+2) > EpOutSize)
  {
      MiddleFrames++;    
      RemainingWords -= (EpOutSize - 2);
  }
  *WordsInMiddleFrame = EpOutSize - 2;
  *NumberOfMiddleFrames = MiddleFrames;

  //LAST FRAME
  *WordsInLastFrame = RemainingWords;

  return;
}

/**************************************READ SINGLE************************************/

void USB_ProtocolCreator_readSingle(WORD* commandBuffer, WORD* wrSize, WORD address,T_EpIn epIn)
{
  
  *wrSize = HDR_DIR_BLOCK + 1;

  *commandBuffer++ = ONE  | E_WR_NLINK | C_STATUS | epInMsk[epIn];
  *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | RD_MASK | MEX_MASK | MASTER_DIR_RD_BLOCK;
  *commandBuffer++ = address;

  return;
}

/**************************************READ BURST************************************/

void USB_ProtocolCreator_readBurst(WORD* commandBuffer, WORD* wrSize, WORD burstSize, WORD address,T_EpOut epOut, T_EpIn epIn)
{
  *wrSize = HDR_DIR_BURST;
    
  *commandBuffer++ = ONE |  E_WR_LINK | C_IADDR_BURST| epOutMsk[epOut] | epInMsk[epIn];
  *commandBuffer++ = address;
  *commandBuffer++ = C_SIZE_BURST | E_WR_LINK;
  *commandBuffer++ = burstSize;
  *commandBuffer++ = E_WR_NLINK | C_STATUS;
  *commandBuffer++ = UTOPIA_EN | STATUS_INIT | START_OBC | MEX_MASK | RD_MASK | MASTER_BURST_DIR;
  
  return;
}
 
/**************************************READ Block************************************/

void USB_ProtocolCreator_readBlock(WORD* commandBuffer, WORD* wrSize, WORD blockSize, WORD *addresses,T_EpOut epOut, T_EpIn epIn)
{
  UINT i;
  *wrSize = blockSize + HDR_DIR_BLOCK;
  
  *commandBuffer++ = ONE | E_WR_NLINK | C_STATUS | epOutMsk[epOut] | epInMsk[epIn];
  *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | RD_MASK | MEX_MASK | MASTER_DIR_RD_BLOCK;

  for(i = 0; i < blockSize; i++)
  {
    *commandBuffer++ = *addresses++;  
  }
  
  return;
}

/**************************************READ Message************************************/

void USB_ProtocolCreator_readMessage(WORD* commandBuffer, WORD* wrSize, WORD fifoAddress, WORD maxReadSize, WORD addressCheck, WORD bitPositionCheck, T_EpOut epOut, T_EpIn epIn)
{

  *wrSize = HDR_MESSAGE;
  
  *commandBuffer++ = ONE | E_WR_LINK | C_IADDR_MSG | epOutMsk[epOut] | epInMsk[epIn];
  *commandBuffer++ = fifoAddress;
  *commandBuffer++ = C_SIZE_BURST | E_WR_LINK;
  *commandBuffer++ = maxReadSize;
  *commandBuffer++ = C_IADDR_CHK | E_WR_LINK;
  *commandBuffer++ = addressCheck;
  *commandBuffer++ = C_IMASKM | E_WR_LINK;
  *commandBuffer++ = 1 << bitPositionCheck;
  *commandBuffer++ = C_STATUS | E_WR_NLINK;
  *commandBuffer++ = STATUS_INIT | START_OBC | UTOPIA_EN | RD_MASK | MEX_MASK | MASTER_RD_MSG;

  return;
}





