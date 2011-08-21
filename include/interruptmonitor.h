#ifndef _INTERRUPT_MONITOR_H_
#define _INTERRUPT_MONITOR_H_

#include "types.h"
#include "hard.h"

#include "hal.h"
#include "tracetool.h"

    void InterruptMonitor();
    BOOL isMonitorActive();
    void startIntMonitor();
    void stopIntMonitor();
    void copyINTbuf(WORD* INTbuf);
    void retreiveINTbuf(WORD* newInterrupt, WORD* targetLocation);
    
#endif
