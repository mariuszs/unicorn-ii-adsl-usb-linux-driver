//----------------------------------------------------------------------
// Test driver for the ST Microelectronics ADSL Chip Taurus USB
//----------------------------------------------------------------------
// file: msw.cpp
// Authors: Maddalena Brattoli, Christophe Piel
// Copyright STMicroelectronics 2000-2005
// Copyright F.H.L.P. 2000
//----------------------------------------------------------------------
#include <linux/kernel.h> 
//#include <string.h>
#include <stdlib.h>
#include "../include/types.h"
#include "../include/tracetool.h"

#include "../include/hal.h"
#include "../include/rapi.h"
#include "../include/amsw_intf_types.h"
#include "../include/amsw_ant.h"
#include "../include/amsw_init.h"

#include "../include/amas.h"
#include "../include/bsp.h"

int amu_go;

unsigned long amu_init_modem(unsigned short MODE);
void  AMUTask(unsigned long,unsigned long ,unsigned long ,unsigned long);
  
extern unsigned long g_AMUQid;
extern unsigned long g_ModemState;
extern unsigned int  g_WaitForInit;
extern unsigned int  g_WaitForShowtime;
extern bool_t          L3_flag;
extern unsigned long GlobalRemove;


extern void *operator new(size_t size)
{
  void *ptr;

  if (xm_getmem(size,&ptr) == SUCCESS) {
  	//PRINT_INFO("new: size=%d,ptr=%p\n",size,ptr);
  } else {
  	ptr = NULL;
  	PRINT_ERROR("### xm_getmem failed,size=%ld\n",size);
  }
  return ptr;
}

extern void *operator new[](size_t size)
{
  void *ptr;

  if (xm_getmem(size,&ptr) == SUCCESS) {
  	//PRINT_INFO("new[]: size=%d,ptr=%p\n",size,ptr);
  } else {
  	ptr = NULL;
  	PRINT_ERROR("### xm_getmem failed,size=%ld\n",size);
  }
  return ptr;
}

extern void operator delete(void *ptr)
{
  //PRINT_INFO("delete: ptr=%p\n",ptr);
  xm_retmem(ptr);
}

extern void operator delete[](void *ptr)
{
  //PRINT_INFO("delete[]: ptr=%p\n",ptr);
  xm_retmem(ptr);
}

void msw_init(int MODE)
{
	int err = 0;
	unsigned long l_Tid;
	unsigned long l_RetCode;
	unsigned long l_TaskArgs[4];

	err = AMSW_Modem_SW_Init(0,0,1,0,XPRIO_EVENT_HDLR,XPRIO_CRITICAL_APPL);
	if (err)
	{
		PRINT_INFO("AMSW_Modem_SW_Init() error\n");
		return;
	}

	err = initializeBoard();
	if (err)
	{
		PRINT_INFO("initializeBoard() error\n");
		return;
	}

	err = amu_init_modem(MODE);
 	if (err)
	{
		PRINT_INFO("amu_init_modem error\n");
		return;
	}

	l_RetCode = xq_create((char *)"AMUQ", 0, 0, &l_TaskArgs[0]);
	if (l_RetCode != 0)
	{
		PRINT_INFO("***** AMU Queue creation error *****\n");
		return;
	}
	g_AMUQid = l_TaskArgs[0];

	// create the AMU task
	l_RetCode = xt_create((char *)"AMU0", 2, 1024, 0, 0, &l_Tid);
	if (l_RetCode != 0)
	{
		PRINT_INFO("***** AMU task creation error *****\n");
		return;
	}

	// start the AMU Task
	amu_go = TRUE;
	l_RetCode = xt_start(l_Tid, 0, AMUTask, l_TaskArgs);
	if (l_RetCode != 0)
	{
		PRINT_INFO("***** AMU task start error *****\n");
		return;
	}
}

void msw_start(void)
{
	unsigned long 	l_RetCode = C_AMSW_REJ;
	AMSW_ModemState l_modemState;

	l_RetCode = AMSW_ANT_getModemState(&l_modemState);
	if(l_RetCode != C_AMSW_ACK)
	{
		PRINT_ERROR("AMSW_ANT_getModemState error %d!\n",l_RetCode);
	}
	else switch(l_modemState)
	{
		case C_AMSW_SHOWTIME_L0 :
		case C_AMSW_SHOWTIME_LQ :
		case C_AMSW_SHOWTIME_L1 :
			PRINT_ERROR("ATU_R already in SHOWTIME\n");
			break;
		case C_AMSW_ACTIVATING  :
		case C_AMSW_INITIALIZING:
//			AMSW_ANT_requestModemStateChange(C_AMSW_IDLE);
			break;
		case C_AMSW_L3          :
		case C_AMSW_IDLE        :
			l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_ACTIVATING);
			if(l_RetCode != C_AMSW_ACK)
				PRINT_ERROR("AMSW_ANT_requestModemStateChange error!\n");
			else
			{
				// Reset counters
				g_WaitForInit = 0;
				g_WaitForShowtime = 0;
			}
			break;

		default:
			break;
	}
}

void msw_stop(void)
{
	unsigned long 	l_RetCode = C_AMSW_REJ;
	AMSW_ModemState l_modemState;

        PRINT_INFO("msw_stop\n");

        if (GlobalRemove) return;
      
	l_RetCode = AMSW_ANT_getModemState(&l_modemState);
	PRINT_INFO("l_modemState=%d\n",l_modemState);	
	if (l_RetCode != C_AMSW_ACK)
	{
            PRINT_ERROR("AMSW_ANT_getModemState error\n");
	}
	else switch(l_modemState)
	{
		case C_AMSW_L3 :
		case C_AMSW_IDLE :
			PRINT_ERROR("ATU_R is DOWN\n");
			break;
		case C_AMSW_SHOWTIME_L0 :
		case C_AMSW_SHOWTIME_L1 :
		case C_AMSW_SHOWTIME_LQ :

			// Perform orderly shutdown
			l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_L3);
			if(l_RetCode != C_AMSW_ACK)
			{
				// If Error Give Disorderly shutdown
				PRINT_ERROR("AMSW_ANT_requestModemStateChange(C_AMSW_L3) error!\n");
				l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_IDLE);
				if(l_RetCode != C_AMSW_ACK)
					PRINT_ERROR("AMSW_ANT_requestModemStateChange(C_AMSW_IDLE) error!\n");
			}
			else
			{
                            PRINT_ERROR("Waiting 5 sec to verify L3_executed\n");
                            for (int i=0; i<10; i++)
                            {
                                // Abort everything on surprise removal
                                if (GlobalRemove) return;
                                				// verify acceptance of orderly shutdown
                                if ((
                                     g_ModemState != C_AMSW_SHOWTIME_L0 &&
                                     g_ModemState != C_AMSW_SHOWTIME_L1 &&
                                     g_ModemState != C_AMSW_SHOWTIME_LQ
                                     )) 
                                    resetShowtime();

                                xtm_wkafter(500);
                                //break;
                            }
                            if ((
                                 g_ModemState == C_AMSW_SHOWTIME_L0 ||
                                 g_ModemState == C_AMSW_SHOWTIME_L1 ||
                                 g_ModemState == C_AMSW_SHOWTIME_LQ
                                 ))
                            {
                                if (L3_flag == FALSE) // no answer received within 1 sec
                                    PRINT_ERROR("No answer to orderly shutdown request for 1 sec!!\n");
                                				PRINT_ERROR("Performing disorderly shutdown!!!\n");
                                                                l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_IDLE);
                                                                if(l_RetCode != C_AMSW_ACK)
                                                                    PRINT_ERROR("AMSW_ANT_requestModemStateChange(C_AMSW_IDLE) error!\n", l_RetCode);
                            }
                        }
                            break;

			default: 			// IDLE request FIX : 15/01/2001
			  PRINT_ERROR("Performing disorderly shutdown!!!\n");
				l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_IDLE);
				if(l_RetCode != C_AMSW_ACK)
					PRINT_ERROR("AMSW_ANT_requestModemStateChange(C_AMSW_IDLE) error!\n");

			break;
 	}
}

void msw_exit(void)
{

	unsigned long 	l_RetCode = C_AMSW_REJ;
	amu_go = FALSE;

        PRINT_INFO("msw_exit\n");

	if (!GlobalRemove)
        {
            board_disable_intrs();		// disable interrupts before disabling the driver

            l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_IDLE);
            PRINT_ERROR("Modem going to IDLE\n");
            if	(l_RetCode != C_AMSW_ACK)
                PRINT_ERROR("AMSW_ANT_requestModemStateChange(C_AMSW_IDLE) error!\n", l_RetCode);
        } else {
            g_ModemState = C_AMSW_IDLE;
        }
        AMSW_Modem_SW_Exit();
}
