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
** File name     : amas.h
**
** Authors       : 
**
** Created On    : 
**
** Description   : ADSL MANAGEMENT APPLICATION SOFTWARE INTERFACE
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

/******************************************************************************/

/******************************************************************************
**                                                                           **
**   HEADER (INCLUDE) SECTION   (check protection before include)            **
**                                                                           **
** ***************************************************************************/

#ifndef _AMAS_H_
#define _AMAS_H_
#include "types.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float Teq_noise_table[256];
	float hf[512];
	unsigned long a;
	short teq_global[32];
} T_AMSW_TeqX;
#define C_AMSW_TEQ								12

#define C_AMSW_DS_REQ_BITRATE_TOO_HIGH_FOR_LITE         30

#define C_AMSW_AMU_EVENT_ACT_TIMEOUT                  	60 /* AMU_EVENT_ACT_TIMEOUT */
#define C_AMSW_AMU_EVENT_INI_TIMEOUT                  	65 /* AMU_EVENT_INI_TIMEOUT */
#define C_AMSW_AMU_EVENT_SHUTDOWN                  	    70 /* AMU_EVENT_SHUTDOWN */
#define C_AMSW_EVENT_RETRY				                75 /* AMU_EVENT_RETRY */
#define C_AMSW_UNKNOWN									80 
#define C_AMSW_UNKNOWN_FAILURE							85 
#define C_AMSW_NO_HARDWARE								90 
#define C_AMSW_NO_USB_BANDWIDTH							95 


#define STARTMSW_VERSION "UnicornTest 1.02"
//---------------------------------------
/* Bitpmap for Defects */
#define LOM			0x4
#define LCDI		0x8
#define LCDNI		0x10
#define LOF			0x40
#define LOS			0x80


#define C_AMSW_DISORDERLY          0xabcd   //STM
#define C_AMSW_RETRY			   15		//STM


#define AMSW_GOLDEN_OFF (1<<0)
#define AMSW_GOLDEN_ON  (1<<1)

#define AMSW_LQ                   (1<<13)

// STM : Initialization Mode Options

#define	MSW_MODE_UNKNOWN    0
#define MSW_MODE_ANSI       1
#define MSW_MODE_GLITE      2
#define MSW_MODE_MULTI      3
#define MSW_MODE_GDMT       4
#define MSW_MODE_ETSI_ISDN  5
#define MSW_MODE_GDMT_ISDN  6
#define MSW_MODE_MULTI_ISDN 7
#define MSW_MODE_GDMT_BDT   8
#define MSW_MODE_MAX        9 

void msw_init(int mode);            // Cold entry point of the MSW

void msw_exit(void);                // Shutdown of the MSW
void msw_start(void);               // Activation of the line
void msw_stop(void);                // Deactivation of the line

unsigned long msw_get_event(void);  // Wait for changes in the MSW state
void msw_cancel_event(void);		// Cancel the wait

void msw_report_event(unsigned long type,unsigned long code);

//void stwinmsw_Ver(T_AMSW_VersionMS *VersionMSdata); //STM Gian
char * stwinmsw_Ver(void);

// Globals
extern int amu_go;
extern unsigned long GlobalRemove;	// Driver is being removed
extern unsigned long last_report;

//	type of the event returned by msw_get_event()

#define	MSW_EVENT_NONE      0
#define	MSW_EVENT_REPORT    1
#define	MSW_EVENT_FAILURE   2
#define	MSW_EVENT_STATE     3
#define	MSW_EVENT_CANCEL	4

#define AMU_EVENT_ACT_TIMEOUT	5
#define AMU_EVENT_INI_TIMEOUT	25
#define AMU_EVENT_SHUTDOWN		7
#define AMU_EVENT_RETRY			8

#ifdef __cplusplus
}
#endif
#endif
/* end of file */
