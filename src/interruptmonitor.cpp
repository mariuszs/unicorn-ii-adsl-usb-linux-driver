#include "../include/types.h"
#include "../include/interruptmonitor.h"

enum {INTBUFFER_SIZE = 16};

BOOL monitorActive = FALSE;
BOOL newIntReceived = FALSE;
WORD IntBufferCopy[INTBUFFER_SIZE];
              
void InterruptMonitor(void)
{
    for(UINT i=0; i< INTBUFFER_SIZE;i++)
  {
    IntBufferCopy[i] = 0xdead;
  }
}

BOOL isMonitorActive(void)
{
   return monitorActive;
}

void startIntMonitor(void)
{
  monitorActive = TRUE;
  newIntReceived = FALSE;
}

void stopIntMonitor(void)
{
  monitorActive = FALSE;
  newIntReceived = FALSE;
}

void copyINTbuf(WORD* INTbuf)
{
  WORD* p = IntBufferCopy;
  for (UINT i=0; i < INTBUFFER_SIZE; i++)
  {
    *p++ = *INTbuf++;
  }
  newIntReceived = TRUE;

 }

void retreiveINTbuf(WORD* newInterrupt, WORD* targetLocation)
{
  WORD* p = IntBufferCopy;
  if(newIntReceived == TRUE)
  {
    for (UINT i=0; i < INTBUFFER_SIZE; i++)
    {
        *targetLocation++ = *p++;
    }
    newIntReceived = FALSE;
  }
 
}

