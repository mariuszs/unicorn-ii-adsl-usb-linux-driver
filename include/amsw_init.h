#ifndef _AMSW_INIT_H
#define _AMSW_INIT_H
#ifdef __cplusplus
extern "C"
{
#endif  
/* *****************************************************************************
**                                                                           **
**                                 ALCATEL TELECOM                           **
**                                                                           **
** ***************************************************************************/

/* *************************** COPYRIGHT INFORMATION **************************
**                                                                           **
**   This program contains proprietary information which is a trade          **
**   secret of ALCATEL TELECOM and also is protected as an unpublished       **
**   work under applicable Copyright laws. Recipient is to retain            **
**   this program in confidence and is not permitted to use or make copies   **
**   thereof other than as permitted in a written agreement with             **
**   ALCATEL TELECOM.                                                        **
**                                                                           **
** ***************************************************************************/

/* *****************************************************************************
** 
** Project       : ADSL
** 
** Module        : MODEM SW ADSL 3.0
** 
** File name     : amsw_init.h
** 
** Authors       : 
** 
** Created On    : 
** 
** Description   : exported initialization header
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

/* **************************** COMPILE DIRECTIVES ****************************
** 
** 
** ***************************************************************************/

/* *****************************************************************************
**                                                                           **
**   HEADER (INCLUDE) SECTION   (check protection before include)            **
**                                                                           **
** ***************************************************************************/

/* *****************************************************************************
**                                                                           **
**   EXPORTED FUNCTIONS      (prototypes in header file)                     **
**                                                                           **
** ***************************************************************************/

#define FULL_RATE_MODEM_VARIANT		   (unsigned long)           0x00
#define G_LITE_MODEM_VARIANT		   (unsigned long)	     0x01
#define ISDN_MODEM_VARIANT		   (unsigned long)	     0x02	
#define G_DMT_MODEM_VARIANT  (unsigned long)	     0x03
#define UADSL_MODEM_VARIANT  (unsigned long)	     0x04

  extern unsigned long AMSW_Modem_SW_Init(unsigned long deviceNumber,
                                          unsigned long chipAddress,
                                          unsigned long intr,
                                          char * trace_debug_fs_path,
                                          unsigned long prioModem,
                                          unsigned long prioAutonomuous);

  extern unsigned long AMSW_Chip_Halt(void);

  extern unsigned long AMSW_Modem_SW_Exit(void);

  extern unsigned long assessStatusChipset(unsigned long deviceNumber,
                                           unsigned long status);
  
/* ****************************************************************************
**                                                                           **
**   END                                                                     **
**                                                                           **
** ************************************************************************* */
#ifdef __cplusplus
}  
#endif
#endif /*_AMSW_INIT_H */
