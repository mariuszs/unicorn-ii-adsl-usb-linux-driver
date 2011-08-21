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
** File name     : amu.cpp
**
** Authors       : 
**
** Created On    :  
**
** Description   : ADSL MANAGEMENT UNIT  
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
 
/*******************************************************************************/
 
/******************************************************************************
**                                                                           **
**   HEADER (INCLUDE) SECTION   (check protection before include)            **
**                                                                           **
** ***************************************************************************/

#include "../include/types.h"
#include "../include/tracetool.h"
#include "../include/rapi.h"  
#include "../include/hard.h"
#include "../include/hal.h"
#include "../include/amsw_intf_types.h"
#include "../include/amsw_ant.h"
#include "../include/amsw_init.h"
#include "../include/amas.h"
#include "../include/amu.h"
#define _PICAP_CODE_
#ifdef _PICAP_CODE_
extern long Vendor_Id_code_ECI;
#endif

#ifdef RFC035  
extern unsigned long useRFC035v = 0;
#endif

extern long pvo_noisdn;

#define RETRY_WAIT_TIME_MIN_MSEC      5000 // 5 seconds (between line disable and reenable)

// Module parameters
unsigned long Vendor_Id_code_Globspan = 0;
unsigned long FmPollingRate     = 1000;
unsigned long InitTimeout       = 20000;
unsigned long ActTimeout        = 300000;	// Fix for Alcatel 4.2.13
unsigned long RetryTime         = 2000;  

unsigned long LCD_Trig          = 15000;
unsigned long LOS_LOF_Trig      = 2000;  

unsigned long nolinedown        = 0;
unsigned long biquadGainOffset  = 0;
unsigned long targetNMOffset    = 0;
unsigned long maxBitToneLimit   = 12;
unsigned long bitswapEnable     = 1;
unsigned long rxRefGainOffset   = 0;
unsigned long maxAddNoiseMargin = 31;
    
// End module parameters

unsigned long NEAR_LCDNI_COUNT;         
unsigned long NEAR_LCDI_COUNT;          
unsigned long NEAR_LOS_COUNT;           
unsigned long NEAR_LOF_COUNT;           
unsigned long FAR_LCDNI_COUNT;          
unsigned long FAR_LCDI_COUNT;						
unsigned long FAR_LOS_COUNT;            
unsigned long FAR_LOF_COUNT;            
unsigned long FAR_LOS_SHORT_COUNT;      
unsigned long FAR_LOF_SHORT_COUNT;      

//ModemSW variables
extern T_AMSW_Identification                g_Identification;
extern T_AMSW_NT_NearEndLineOperData        g_NearEndLineOperData;
extern T_AMSW_NT_FarEndLineOperData         g_FarEndLineOperData;
extern T_AMSW_def_counter_set               g_def_counter_set;
extern T_AMSW_def_bitmap_set                g_def_bitmap_set;
extern T_AMSW_def_counters                  g_def_counters;
extern T_AMSW_NT_ChannelOperData            g_ChannelOperData;
extern T_AMSW_ANT_CustomerConfiguration     g_CustomerCfg;
extern T_AMSW_ANT_StaticConfiguration       g_StaticCfg;
extern T_AMSW_PowerStateConfiguration       g_PowerStateCfg;   

extern "C" void HandleAtmError(void);
extern "C" void HandleLeds(void);
	
unsigned long amu_init_modem(unsigned short MODE);
unsigned long amu_init_modem(unsigned short MODE)
{
	unsigned long l_RetCode;

	//
	// Static Configuration
	//
	g_StaticCfg.utopiaMode = C_AMSW_UTOPIA_LEVEL1;
	g_StaticCfg.utopiaFast = 0;
	g_StaticCfg.utopiaSlow = 0;
	for (l_RetCode=0; l_RetCode<32; l_RetCode++)
		g_StaticCfg.serialNumber[l_RetCode] = '9';
	g_StaticCfg.maximumDownstreamATMRate =  12000;
	g_StaticCfg.managementVersion = 1;
	g_StaticCfg.goldenMode = AMSW_GOLDEN_OFF;
	g_StaticCfg.chipSetVendorIdentif.countryCode = 0x0f;
	g_StaticCfg.chipSetVendorIdentif.reserved = 0x00;
	g_StaticCfg.chipSetVendorIdentif.vendorCode = (0x41 << 24) | (0x4c << 16) | (0x43 << 8) | (0x42); /*"ALCB"*/ 
	g_StaticCfg.chipSetVendorIdentif.vendorSpecific = 0x0451;
    g_StaticCfg.maximumNumberOfBitsPerToneUpstream = 14;
	g_StaticCfg.maximumNumberOfBitsPerToneDownstream = 14;


	//	Customer Configuration
	//	----------------------
	g_CustomerCfg.POTSoverlayOperationModes = 0;
	g_CustomerCfg.ISDNoverlayOperationModes = 0;
	switch(MODE)
	{
	case MSW_MODE_ANSI:
		g_CustomerCfg.POTSoverlayOperationModes = AMSW_ANSI;
		break;
	case MSW_MODE_GLITE:
		g_CustomerCfg.POTSoverlayOperationModes = AMSW_G_LITE;
		break;
	case MSW_MODE_GDMT:
		g_CustomerCfg.POTSoverlayOperationModes = AMSW_G_DMT;
		break;
	case MSW_MODE_MULTI:
		g_CustomerCfg.POTSoverlayOperationModes = AMSW_G_DMT | AMSW_ANSI | AMSW_G_LITE;
        break;
    case MSW_MODE_ETSI_ISDN:
		g_CustomerCfg.ISDNoverlayOperationModes = AMSW_ETSI;
		break;
	case MSW_MODE_GDMT_ISDN:
		g_CustomerCfg.ISDNoverlayOperationModes = AMSW_G_DMT;
		break;
	case MSW_MODE_MULTI_ISDN:
		g_CustomerCfg.ISDNoverlayOperationModes = AMSW_ETSI | AMSW_G_DMT;
		break;
	case MSW_MODE_GDMT_BDT:
		g_CustomerCfg.ISDNoverlayOperationModes = AMSW_G_DMT;
	default:
		g_CustomerCfg.POTSoverlayOperationModes = AMSW_G_DMT | AMSW_ANSI | AMSW_G_LITE;
		break;
	}

  if (pvo_noisdn)
  {
    g_CustomerCfg.ISDNoverlayOperationModes = 0;
  }
  
  if(    (MODE == MSW_MODE_ANSI) || 
         (MODE == MSW_MODE_GLITE) ||
         (MODE == MSW_MODE_GDMT) ||
         (MODE == MSW_MODE_MULTI))
    {
        for (l_RetCode = 0; l_RetCode < 8; l_RetCode++)
		    g_CustomerCfg.ISDNoverlayPermissions[l_RetCode] = 0;
      
	g_CustomerCfg.POTSoverlayPermissions[0] = AMSW_TRELLIS | AMSW_BITSWAP;   // ANSI
	
#ifdef SUICIDE_FIX	
	g_CustomerCfg.POTSoverlayPermissions[1] = AMSW_TRELLIS | AMSW_DS_PILOT_MODULATED | AMSW_POWER_MANAGEMENT;	    //DMT
#else
	g_CustomerCfg.POTSoverlayPermissions[1] = AMSW_TRELLIS | AMSW_DS_PILOT_MODULATED;							// DMT													
#endif
														
	g_CustomerCfg.POTSoverlayPermissions[2] = AMSW_LQ;																														// UAWG
	g_CustomerCfg.POTSoverlayPermissions[3] = AMSW_BITSWAP | AMSW_RS16 | AMSW_TRELLIS | AMSW_DS_PILOT_MODULATED ;	// G.LITE
	g_CustomerCfg.POTSoverlayPermissions[4] = 0;
	g_CustomerCfg.POTSoverlayPermissions[5] = 0;
	g_CustomerCfg.POTSoverlayPermissions[6] = 0;
	g_CustomerCfg.POTSoverlayPermissions[7] = 0;
  }

    if( (MODE == MSW_MODE_ETSI_ISDN) || 
        (MODE == MSW_MODE_GDMT_ISDN) ||
        (MODE == MSW_MODE_MULTI_ISDN) ||
        (MODE == MSW_MODE_GDMT_BDT) )
    {
	    for (l_RetCode = 0; l_RetCode < 8; l_RetCode++)
			g_CustomerCfg.POTSoverlayPermissions[l_RetCode] = 0;
		
	
	g_CustomerCfg.ISDNoverlayPermissions[0] = AMSW_TRELLIS | AMSW_BITSWAP;   // ANSI
	
#ifdef SUICIDE_FIX	
	g_CustomerCfg.ISDNoverlayPermissions[1] = AMSW_TRELLIS | AMSW_DS_PILOT_MODULATED | AMSW_POWER_MANAGEMENT;	    //DMT
#else
	g_CustomerCfg.ISDNoverlayPermissions[1] = AMSW_TRELLIS | AMSW_DS_PILOT_MODULATED;							// DMT													
#endif
														
	g_CustomerCfg.ISDNoverlayPermissions[2] = AMSW_LQ;																														// UAWG
	g_CustomerCfg.ISDNoverlayPermissions[3] = 0;	// G.LITE
	g_CustomerCfg.ISDNoverlayPermissions[4] = 0;
	g_CustomerCfg.ISDNoverlayPermissions[5] = 0;
	g_CustomerCfg.ISDNoverlayPermissions[6] = 0;
	g_CustomerCfg.ISDNoverlayPermissions[7] = 0;
    }


	// ISDN
	// ----
    if(pvo_noisdn)
    {
       g_CustomerCfg.ISDNoverlayOperationModes = 0;
	for (l_RetCode = 0; l_RetCode < 8; l_RetCode++)
		g_CustomerCfg.ISDNoverlayPermissions[l_RetCode] = 0;
	for (l_RetCode = 0; l_RetCode < 8; l_RetCode++)
		g_CustomerCfg.ISDNoverlayPermissions[l_RetCode] = 0;
    }



    g_CustomerCfg.biquadGainOffset = 0x0;
    g_CustomerCfg.targetNMOffset = 0x0;
    g_CustomerCfg.maxBitToneLimit = 12;
    g_CustomerCfg.bitswapEnable = (unsigned long)bitswapEnable;
    g_CustomerCfg.rxRefGainOffset = 0x0;
    g_CustomerCfg.NMOffsetAnsi13 = 0x0;
    g_CustomerCfg.bruteGainOffset = 0x0;
    g_CustomerCfg.rxCROffset = 0x0;
    g_CustomerCfg.powerBoostOffset = 0x0;
    g_CustomerCfg.maxAddNoiseMargin = 31;
    g_CustomerCfg.maxUpstreamCarrier = 0;
    g_CustomerCfg.minDownstreamCarrier = 0;
    g_CustomerCfg.firstToneAliasingNoise = 255;

	// Power State Configuration
	// -------------------------
	g_PowerStateCfg.powerStateControl = AMSW_L3 | AMSW_L1;

	// Setup modem configuration
	// -------------------------
	l_RetCode = AMSW_ANT_setModemConfiguration(C_AMSW_STATIC_CONFIGURATION,&g_StaticCfg);
	if (l_RetCode != C_AMSW_ACK)
		PRINT_ERROR("Error in AMSW_ANT_setModemConfiguration-1 (%d)\n", l_RetCode);
	l_RetCode = AMSW_ANT_setModemConfiguration(C_AMSW_CUSTOMER_CONFIGURATION,&g_CustomerCfg);
	if (l_RetCode != C_AMSW_ACK)
		PRINT_ERROR("Error in AMSW_ANT_setModemConfiguration-2 (%d)\n", l_RetCode);
	l_RetCode = AMSW_ANT_setModemConfiguration(C_AMSW_POWER_STATE_CONTROL,&g_PowerStateCfg);
	if (l_RetCode != C_AMSW_ACK)
		PRINT_ERROR("Error in AMSW_ANT_setModemConfiguration-3 (%d)\n", l_RetCode);
	return l_RetCode;
}

void AMUTask(unsigned long,unsigned long , unsigned long , unsigned long )
{
	unsigned long   l_RetCode = C_AMSW_REJ;
	static bool_t     PM_toggle = TRUE;

	PRINT_ERROR("FmPollingRate=%ldms,InitTimeout=%ldms,ActTimeout=%ld\n",
		   FmPollingRate,InitTimeout,ActTimeout);

	unsigned long PM_FM_POLLING_RATE = FmPollingRate;
	unsigned long WAITFOR_SHOWTIME_COUNT = InitTimeout / PM_FM_POLLING_RATE;
	unsigned long WAITFOR_INIT_COUNT = ActTimeout / PM_FM_POLLING_RATE;
	unsigned long RETRY_WAIT_TIME = RETRY_WAIT_TIME_MIN_MSEC / PM_FM_POLLING_RATE;

    bool_t CheckRetry = FALSE;

	if (RetryTime > RETRY_WAIT_TIME_MIN_MSEC)	// STM Gian Set RetryTime Only if is Bigger then minimum value
		RETRY_WAIT_TIME = RetryTime / PM_FM_POLLING_RATE;
	unsigned long INIT_POLLING_TIME = 5; //(??????) 
	unsigned long WAITFOR_DISORDERLY_COUNT = 3; 

	NEAR_LCDNI_COUNT = LCD_Trig / PM_FM_POLLING_RATE;         
	NEAR_LCDI_COUNT = NEAR_LCDNI_COUNT;          
	NEAR_LOS_COUNT = LOS_LOF_Trig / PM_FM_POLLING_RATE;           
	NEAR_LOF_COUNT = NEAR_LOS_COUNT;           
	FAR_LCDNI_COUNT = NEAR_LCDNI_COUNT;          
	FAR_LCDI_COUNT = NEAR_LCDNI_COUNT;						
	FAR_LOS_COUNT = NEAR_LOS_COUNT;            
	FAR_LOF_COUNT = NEAR_LOS_COUNT;            
	FAR_LOS_SHORT_COUNT = NEAR_LOS_COUNT;      
	FAR_LOF_SHORT_COUNT = NEAR_LOS_COUNT;      

	UINT delay = 0;
		
	while (amu_go)
	{
		xtm_wkafter(500);
		delay += 500;
           
		
		// Exit immediately on surprise removal
		// ------------------------------------
		if (GlobalRemove) return;

		HandleAtmError();
		HandleLeds();
		
		// Modem Software polling
		// ----------------------
		if (delay < PM_FM_POLLING_RATE) continue;
		delay = 0;
           
		// Modem Software polling
		// ----------------------
		switch(g_ModemState)
		{
		case C_AMSW_IDLE       : 
           		g_WaitForInit = 0;
			if (CheckRetry && RetryTime > 0)
			{
				g_WaitForRetry = 0;
				g_ModemState = C_AMSW_RETRY;
			}
			break;
		case C_AMSW_ACTIVATING : 
			g_WaitForInit++;
			if(g_WaitForInit >= WAITFOR_INIT_COUNT)
			{
				msw_report_event(AMU_EVENT_ACT_TIMEOUT,0);
				PRINT_ERROR("Timeout in activation!!!!\n");
				g_ModemState = C_AMSW_DISORDERLY;
				g_WaitForInit = 0;
			}
			break;
		case C_AMSW_L3         : break;
		case C_AMSW_SHOWTIME_L0:
		case C_AMSW_SHOWTIME_LQ:
		case C_AMSW_SHOWTIME_L1:
			// Poll line for defects and performance
			g_ShowtimeCounter++;
			if (g_ShowtimeCounter >= INIT_POLLING_TIME)
			{
				if ( (FM_Polling(PM_toggle) != C_AMSW_ACK ))
				{
					PRINT_ERROR("Error in AMSW_get_Data during Fm Polling....\n");
					PRINT_ERROR("Fm Polling will be stopped for %d sec!!!\n",(INIT_POLLING_TIME*PM_FM_POLLING_RATE)/1000);
					g_ShowtimeCounter = 0;
				}
				else
					PM_toggle = !PM_toggle;
			}
			break;
		case C_AMSW_INITIALIZING :
		case C_AMSW_Ghs_HANDSHAKING :
		case C_AMSW_ANSI_HANDSHAKING :

			// WAIT to reach SHOWTIME
			g_WaitForShowtime++;
			if(g_WaitForShowtime >= WAITFOR_SHOWTIME_COUNT)
			{
				msw_report_event(AMU_EVENT_INI_TIMEOUT,0);
				PRINT_ERROR("Timeout in initialization!!!!\n");
				// Reset count
				g_WaitForShowtime = 0;
				// Disorderly shutdown 
				g_ModemState = C_AMSW_DISORDERLY;
			}
			break;

		case C_AMSW_DISORDERLY :

			// Special state for disorderly shutdown
			// Wait out the period of heavy burst of interrupts
			// from CO side when line length is changed while in
			// showtime
		
			g_WaitForDisorderly++;
			if(g_WaitForDisorderly >= WAITFOR_DISORDERLY_COUNT)
			{
				board_disable_intrs();

				g_ShowtimeCounter = 0;							
				g_WaitForDisorderly = 0;

				// Disorderly shutdown 
				l_RetCode = AMSW_ANT_requestModemStateChange(C_AMSW_IDLE);
				resetShowtime();
				if(l_RetCode != C_AMSW_ACK) {
					PRINT_ERROR("Error in AMSW_ANT_requestModemStateChange(C_AMSW_IDLE) = %d\n", l_RetCode);
				}
				else 
                {
                    for (int i=0; i<6; i++)
                    {
                        if (g_ModemState == C_AMSW_IDLE || GlobalRemove) break;
                        xtm_wkafter(500);
                    }
                }
                
                CheckRetry = TRUE;

				if (RetryTime != 0)
				{
					g_ModemState = C_AMSW_RETRY;
					g_WaitForRetry = 0;
				}
        
			}
			break;
			
		case C_AMSW_RETRY:
			if (RetryTime != 0)
			{
				g_WaitForRetry++;
				if (g_WaitForRetry >= RETRY_WAIT_TIME)
                {	
					msw_report_event(AMU_EVENT_RETRY,0);
					g_WaitForRetry = 0;
                   	CheckRetry = FALSE;
					msw_start();
				}
			}
			else
			{
				PRINT_ERROR("Error in AMUTask (C_AMSW_RETRY) with RetryTime != 0 (%ld)\n",RetryTime);
				g_ModemState = C_AMSW_IDLE;
			}
			break;
		default:
			PRINT_ERROR("Error in AMUTask g_ModemState =%d not catched\n",g_ModemState);
			g_ModemState = C_AMSW_IDLE;
	        break;
		}						//end switch
	}
}

//----------------------------------------------------------------------
// This function  monitors the line for fault management:
// LOS (Loss Of Signal),
// LOF (Loss Of Frame),
// LCDI (Loss Of Cell Delineation Interleaved), 
// LCDNI (Loss Of Cell Delineation Fast)defects
//----------------------------------------------------------------------
unsigned long FM_Polling(bool_t pm_poll)
{
	unsigned long l_RetCode = C_AMSW_REJ;

	if ((g_ModemState == C_AMSW_SHOWTIME_L0) ||
			(g_ModemState == C_AMSW_SHOWTIME_LQ) ||
			(g_ModemState == C_AMSW_SHOWTIME_L1))
	{
		// Get defect bit map
		l_RetCode = AMSW_ANT_getData(C_AMSW_FM_DATA, &g_def_bitmap_set);
		
		if(l_RetCode == C_AMSW_ACK)
		{
//#######################  Loss Of Signal  ##############################   
	
			if( ((g_def_bitmap_set.near_end.status & LOS) == LOS) ||
				((g_def_bitmap_set.near_end.status & LOS) == 0) && 
				((g_def_bitmap_set.near_end.change & LOS) == LOS) )
			{
				g_NEAR_LOS++;
			}
			else
			{
				g_NEAR_LOS = 0;
			}

			if( ((g_def_bitmap_set.far_end.status & LOS) == LOS) ||
				((g_def_bitmap_set.far_end.status & LOS) == 0) && 
				((g_def_bitmap_set.far_end.change & LOS) == LOS) )
			{
				g_FAR_LOS++;
			}
			else
			{
				g_FAR_LOS = 0;
			}

//######################  Loss Of Cell Delineation Fast  ######################	

			if((g_def_bitmap_set.near_end.status & LCDNI) == LCDNI)
			{
				g_NEAR_LCDNI++;
			}
			else
			{
				g_NEAR_LCDNI = 0;
			}

			if((g_def_bitmap_set.far_end.status & LCDNI) == LCDNI)
			{
				g_FAR_LCDNI++;
			}
			else
			{
				g_FAR_LCDNI = 0;
			}

//######################  Loss Of Cell Delineation Interleaved ######################

			if((g_def_bitmap_set.near_end.status & LCDI) == LCDI)
			{
				g_NEAR_LCDI++;
			}
			else
			{
				g_NEAR_LCDI = 0;
			}

			if((g_def_bitmap_set.far_end.status & LCDI) == LCDI)
			{
				g_FAR_LCDI++;
			}
			else
			{
				g_FAR_LCDI = 0;
			}

		// WAIT : If over a period of time defect persists, do a disorderly shutdown
            if (nolinedown==0)
            {

			    if( (g_NEAR_LOS >= NEAR_LOS_COUNT) || (g_NEAR_LCDNI >= NEAR_LCDNI_COUNT) ||
				    (g_NEAR_LCDI >= NEAR_LCDI_COUNT) )
			    {
				    g_FAR_LOS = 0;
				    g_FAR_LCDNI = 0;
				    g_FAR_LCDI = 0;
				    g_ModemState = C_AMSW_DISORDERLY;
			    }
			    else if ( (g_FAR_LOS >= FAR_LOS_COUNT) || (g_FAR_LCDNI >= FAR_LCDNI_COUNT) ||
					    (g_FAR_LCDI >= FAR_LCDI_COUNT) ||
					    ((g_FAR_LOS >= FAR_LOS_SHORT_COUNT) && g_NEAR_LOS ) )
				    g_ModemState = C_AMSW_DISORDERLY;
            }
								
			if(g_ModemState == C_AMSW_DISORDERLY)
			{

				PRINT_ERROR("Bringing down line due to persistent:\n");
				PRINT_ERROR("NEAR_LOS = %d/NEAR_LCD = %d/NEAR_LCDI = %d\nFAR_LOS = %d  FAR_LCDI = %d  FAR_LCDNI = %d\n",
					g_NEAR_LOS,  g_NEAR_LCDNI, g_NEAR_LCDI,g_FAR_LOS, g_FAR_LCDNI, g_FAR_LCDI);
				
				msw_report_event(AMU_EVENT_SHUTDOWN,0);

				g_NEAR_LOS = 0; g_NEAR_LCDNI = 0; g_NEAR_LCDI = 0;
				g_FAR_LOS = 0; g_FAR_LCDNI = 0; g_FAR_LCDI = 0;
				msw_stop();
				msw_start();
			}
		}
	}

	return l_RetCode;
}

//----------------------------------------------------------------------
//	Monitoring of performance counters
//----------------------------------------------------------------------
unsigned long PM_Polling(void)
{
    unsigned long l_RetCode = C_AMSW_REJ;

	if   ((g_ModemState == C_AMSW_SHOWTIME_L0) ||
          (g_ModemState == C_AMSW_SHOWTIME_LQ) ||
	      (g_ModemState == C_AMSW_SHOWTIME_L1)
        	 )
    {
		l_RetCode = AMSW_ANT_getData(C_AMSW_PM_DATA, &g_def_counter_set);
		if(l_RetCode != C_AMSW_ACK)
		{
			PRINT_ERROR("AMSW_ANT_getData error\n");
		}
		else
		{
			PRINT_INFO("\nFast Path Performance Counters:\n\n");	
			PRINT_INFO("Near-end Fec-F = %5u\n", g_def_counter_set.near_end.FecNotInterleaved);
			PRINT_INFO("Far-end Fec-F  = %5u\n", g_def_counter_set.far_end.FecNotInterleaved);
						
			PRINT_INFO("Near-end Crc-F = %5u\n", g_def_counter_set.near_end.CrcNotInterleaved);
			PRINT_INFO("Far-end Crc-F  = %5u\n", g_def_counter_set.far_end.CrcNotInterleaved);

			PRINT_INFO("Near-end Hec-F = %5u\n", g_def_counter_set.near_end.HecNotInterleaved);
			PRINT_INFO("Far-end Hec-F  = %5u\n", g_def_counter_set.far_end.HecNotInterleaved);
						
			PRINT_INFO("Near-end Total Cell-F  = %5u\n", g_def_counter_set.near_end.TotalCellCountNotInterleaved);								
			PRINT_INFO("Near-end Active Cell-F = %5u\n", g_def_counter_set.near_end.ActiveCellCountNotInterleaved);
						
			PRINT_INFO("\nInterleave Path Performance Counters:\n\n");	

			PRINT_INFO("Near-end Fec-I = %5u\n", g_def_counter_set.near_end.FecInterleaved);
			PRINT_INFO("Far-end Fec-I  = %5u\n", g_def_counter_set.far_end.FecInterleaved);
												
			PRINT_INFO("Near-end Crc-I = %5u\n", g_def_counter_set.near_end.CrcInterleaved);
			PRINT_INFO("Far-end Crc-I  = %5u\n", g_def_counter_set.far_end.CrcInterleaved);
						
			PRINT_INFO("Near-end Hec-I = %5u\n", g_def_counter_set.near_end.HecInterleaved);
			PRINT_INFO("Far-end Hec-I  = %5u\n", g_def_counter_set.far_end.HecInterleaved);
						
			PRINT_INFO("Near-end Total Cell-I  = %5u\n", g_def_counter_set.near_end.TotalCellCountInterleaved);
			PRINT_INFO("Near-end Active Cell-I = %5u\n", g_def_counter_set.near_end.ActiveCellCountInterleaved);
        }
	}
	return l_RetCode;
}
