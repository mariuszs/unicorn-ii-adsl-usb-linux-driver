#ifndef _AMSW_ANT_H_
#define _AMSW_ANT_H_ 

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
**                                                                           **
**                                 ALCATEL TELECOM                           **
**                                                                           **
** ***************************************************************************/

/**************************** COPYRIGHT INFORMATION **************************
**                                                                           **
**   This program contains proprietary information which is a trade          **
**   secret of ALCATEL TELECOM and also is protected as an unpublished       **
**   work under applicable Copyright laws. Recipient is to retain            **
**   this program in confidence and is not permitted to use or make copies   **
**   thereof other than as permitted in a written agreement with             **
**   ALCATEL TELECOM.                                                        **
**                                                                           **
** ***************************************************************************/

/******************************************************************************
** 
** Project       : ADSL
** 
** Module        : MODEM SW ADSL 3.0
** 
** File name     : AMSW_ANT.h
** 
** CM reference  : 3EC 15576 
** 
** Authors       : 
** 
** Created On    : 
** 
** Description   : header file for ADSL modem interface
** 
** Reference(s)  : 
** 
** ****************************************************************************

** ****************************************************************************
** 
** History
** Date          Revision     Comment
** 
** ***************************************************************************/

/***************************** COMPILE DIRECTIVES ****************************
** 
** 
** ***************************************************************************/


/******************************************************************************
**                                                                           **
**   HEADER (INCLUDE) SECTION   (check protection before include)            **
**                                                                           **
** ***************************************************************************/
#include "amsw_intf_types.h"
/******************************************************************************
**                                                                           **
**   EXTERNAL DATA (+ meaning)                                               **
**                                                                           **
** ***************************************************************************/

/******************************************************************************
**                                                                           **
**   GLOBAL DATA                                                             **
**                                                                           **
** ***************************************************************************/

/******************************************************************************
**                                                                           **
**   LOCAL FUNCTION PROTOTYPES (file scope only)                             **
**                                                                           **
** ***************************************************************************/

/******************************************************************************
**                                                                           **
**   EXPORTED FUNCTIONS      (prototypes in header file)                     **
**                                                                           **
** ***************************************************************************/

  extern unsigned long   AMSW_getInterfaceVersion(void);
  /* modem has copy,check on all data configured once during lifetime modem */
  extern AMSW_ResultCode AMSW_ANT_setModemConfiguration(IN AMSW_ConfigType configType,
                                                       IN void * configData);
  extern AMSW_ResultCode AMSW_ANT_getModemConfiguration(IN AMSW_ConfigType configType,
                                                       OUT void * configData);
  extern AMSW_ResultCode AMSW_ANT_getData(IN AMSW_DataType dataType,
                                      OUT void * data);
  extern AMSW_ResultCode AMSW_ANT_requestModemStateChange(IN AMSW_ModemState requestedState);
  extern void            AMSW_ANT_reportModemStateChange(IN AMSW_ModemState modemState);
  extern void            AMSW_ANT_reportEvent(IN AMSW_ModemEvent event);
  extern void            AMSW_ANT_reportModemFailure(IN AMSW_ModemFailure failureCause);
  extern AMSW_ResultCode AMSW_ANT_getModemState(OUT AMSW_ModemState *modemState);

  extern unsigned long   AMSW_ANT_dyingGasp(void);

  extern void            AMSW_ANT_lowPowerOccured(unsigned long);
  extern AMSW_ResultCode AMSW_ANT_setUtopiaFlush(IN bool_t fast, IN bool_t interl);
  extern AMSW_ResultCode AMSW_ANT_resetUtopiaFlush(IN bool_t fast, IN bool_t interl);

  extern AMSW_ResultCode AMSW_ANT_getSachemID(IN unsigned long sachemBaseAddr, OUT unsigned short* sachemID);

/******************************************************************************
**                                                                           **
**   END                                                                     **
**                                                                           **
** ***************************************************************************/

#ifdef __cplusplus
}
#endif

#endif //_AMSW_ANT_H_
