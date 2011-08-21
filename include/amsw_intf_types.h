#ifndef __AMSW_INTF_TYPES_H  
#define __AMSW_INTF_TYPES_H
/************************************************************************
**                                                                     **
**                           ALCATEL TELECOM                           **
**                                                                     **
************************************************************************/

/************************ COPYRIGHT INFORMATION *************************
**                                                                     **
** This program contains proprietary information which is a trade      **
** secret of ALCATEL TELECOM and also is protected as an unpublished   **
** work under applicable Copyright laws. Recipient is to retain this   ** 
** program in confidence and is not permitted to use or make copies    ** 
** thereof other than as permitted in a written agreement with         **
** ALCATEL TELECOM.                                                    **
**                                                                     **
************************************************************************/

/*************************** IDENTIFICATION *****************************
**
** Project       : ADSL R3.0  - ADSL Modem SW subsystem
**
** File name     : AMSW_intf_types.h
**		
** Programmer(s) : 
**                
** Description   : type definitions for the ADSL Modem SW interface
**
************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
#include "types.h"
#ifdef __cplusplus
}
#endif

/* interface specification definitions */
#define IN     /* read only input argument */
#define OUT    /* write only input argument */
#define INOUT  /* mixed read/write input argument */

/* type definition section */
typedef unsigned long AMSW_InterfaceVersion;
typedef unsigned long AMSW_ResultCode;
typedef unsigned long AMSW_DeviceId;
typedef unsigned long AMSW_ConfigType;
typedef unsigned long AMSW_DataType;

/* possible values for AMSW_ResultCode */
#define C_AMSW_ACK                          0
#define C_AMSW_REJ                          1
#define C_AMSW_ERR_COM_INV_PARAM      0x10003

/* possible values for AMSW_ConfigType */
#define C_AMSW_OPERATOR_CONFIGURATION 1
#define C_AMSW_STATIC_CONFIGURATION   2
#define C_AMSW_CUSTOMER_CONFIGURATION 1
#define C_AMSW_POWER_STATE_CONTROL    3

#define C_AMSW_operatorControlledRateAdaptive  1
#define C_AMSW_automaticRateAdaptiveAtStartup  2
#define C_AMSW_dynamicRateAdaptive             3

/* possible structs for configData in setModemConfiguration */
typedef unsigned char  VendorIdentification[16];
typedef struct {
  unsigned char	  MinNoiseMarginDnstr;
  unsigned char	  MinNoiseMarginUpstr;
  unsigned char   MaxAddNoiseMarginDnstr;
  unsigned char   MaxAddNoiseMarginUpstr;
  unsigned char   TargetNoiseMarginDnstr;
  unsigned char   TargetNoiseMarginUpstr;
  signed char     MaxPSDDnstr;
  unsigned char   CarrierMask[32];
  unsigned long   RAModeDnstr;
  unsigned long   RAModeUpstr;
  unsigned short  InterlMinBitrateDnstr;
  unsigned short  InterlMinBitrateUpstr;
  unsigned short  InterlPlannedBitrateDnstr;
  unsigned short  InterlPlannedBitrateUpstr;
  unsigned short  InterlMaxBitrateDnstr;
  unsigned short  InterlMaxBitrateUpstr;
  unsigned char   InterlMaxDelayDnstr;
  unsigned char   InterlMaxDelayUpstr;
  unsigned short  FastMinBitrateDnstr;
  unsigned short  FastMinBitrateUpstr;
  unsigned short  FastPlannedBitrateDnstr;
  unsigned short  FastPlannedBitrateUpstr;
  unsigned short  FastMaxBitrateDnstr;
  unsigned short  FastMaxBitrateUpstr;
  unsigned short  LowPowerBitrateDnstr;
  unsigned short  LowPowerBitrateUpstr;
  unsigned char   POTSoverlayOperationModes;
  unsigned short  POTSoverlayPermissions[8];
  unsigned char   ISDNoverlayOperationModes;
  unsigned short  ISDNoverlayPermissions[8];
} T_AMSW_LT_OperatorConfiguration;

typedef struct {
  unsigned char   USpowerCutback;
  unsigned char   POTSoverlayOperationModes;
  unsigned short  POTSoverlayPermissions[8];
  unsigned char   ISDNoverlayOperationModes;
  unsigned short  ISDNoverlayPermissions[8];

  signed char     biquadGainOffset;
  signed char     targetNMOffset;
  signed char     maxBitToneLimit;
  signed char     bitswapEnable;
  signed char     rxRefGainOffset;
  signed char     NMOffsetAnsi13;
  signed char     bruteGainOffset;
  signed char     rxCROffset;
  signed char     powerBoostOffset;
  unsigned char   maxAddNoiseMargin;
  unsigned char   maxUpstreamCarrier;
  unsigned char   minDownstreamCarrier;
  unsigned char   firstToneAliasingNoise;
} T_AMSW_ANT_CustomerConfiguration;


typedef struct   { 
  unsigned char   countryCode; 
  unsigned char   reserved; 
  unsigned long   vendorCode; 
  unsigned short  vendorSpecific; 
} T_ITU_VendorId; 
  
typedef struct { 
  T_ITU_VendorId    ITU_ChipsetVendorId; 
  T_ITU_VendorId    ITU_ModemVendorId; 
  unsigned char   ITU_StandardRevisionNbr; 
  unsigned short  ANSI_ETSI_VendorId; 
  unsigned char   ANSI_ETSI_VendorRevisionNbr; 
  unsigned char   ANSI_ETSI_StandardRevisionNbr; 
  unsigned long   ALC_ManagementInfo; 
} T_AMSW_Identification; 
  
typedef struct { 
  unsigned long   utopiaMode; 
  unsigned long   utopiaFast; 
  unsigned long   utopiaSlow; 
  T_ITU_VendorId    vendorIdentif; 
  unsigned long   frontEndIdentification;
} T_AMSW_LT_StaticConfiguration; 
  
typedef struct { 
  unsigned long  utopiaMode; 
  unsigned long  utopiaFast; 
  unsigned long  utopiaSlow; 
  char           serialNumber[32]; 
  T_ITU_VendorId chipSetVendorIdentif; 
  T_ITU_VendorId modemVendorIdentif;
  unsigned long  maximumDownstreamATMRate; 
  unsigned long  managementVersion; 
  unsigned long  frontEndIdentification;
  unsigned char  goldenMode; 
  unsigned char  maximumNumberOfBitsPerToneUpstream;
  unsigned char  maximumNumberOfBitsPerToneDownstream;  
} T_AMSW_ANT_StaticConfiguration; 

typedef struct {
  unsigned char powerStateControl;
} T_AMSW_PowerStateConfiguration;

/* possible bitpositions for powerStateControl */
#define AMSW_L1 (1<<1)
#define AMSW_L3 (1<<3)

/* possible bitpositions for POTSoverlayOperationModes */
#define AMSW_ANSI_IDX   0
#define AMSW_ANSI       (1<<AMSW_ANSI_IDX)
#define AMSW_G_DMT_IDX  1
#define AMSW_G_DMT      (1<<AMSW_G_DMT_IDX)
#define AMSW_UAWG_IDX   2
#define AMSW_UAWG       (1<<AMSW_UAWG_IDX)
#define AMSW_G_LITE_IDX 3
#define AMSW_G_LITE     (1<<AMSW_G_LITE_IDX)


/* possible bitpositions for ISDNoverlayOperationModes */
#define AMSW_ETSI_IDX   0
#define AMSW_ETSI       (1<<AMSW_ETSI_IDX)
#define AMSW_G_DMT_IDX  1
#define AMSW_G_DMT      (1<<AMSW_G_DMT_IDX)

/* possible bitpositions for the permissions */
#define AMSW_TRELLIS              (1<<0)
#define AMSW_FAST_RETRAIN         (1<<1)
#define AMSW_POWER_MANAGEMENT     (1<<2)
#define AMSW_BITSWAP              (1<<3)
#define AMSW_RS16                 (1<<4)
#define AMSW_US_PILOT_MODULATED   (1<<5)
#define AMSW_DS_PILOT_MODULATED   (1<<6)

/* possible values for dataType in getData */
#define C_AMSW_PM_DATA                           0
#define C_AMSW_FM_DATA                           1
#define C_AMSW_NEAR_END_IDENTIFICATION           2
#define C_AMSW_FAR_END_IDENTIFICATION            3
#define C_AMSW_NEAR_END_LINE_DATA                4
#define C_AMSW_FAR_END_LINE_DATA                 5
#define C_AMSW_NEAR_END_CHANNEL_DATA_FAST        6
#define C_AMSW_FAR_END_CHANNEL_DATA_FAST         7
#define C_AMSW_NEAR_END_CHANNEL_DATA_INTERLEAVED 8
#define C_AMSW_FAR_END_CHANNEL_DATA_INTERLEAVED  9

#define C_AMSW_LOOP_NOISE_DETECTED              12

#ifdef UNICORN_COMMON
#define C_AMSW_VERSIONMS						10

#ifdef _3EC15576_DEBUG_
#define C_AMSW_DEBUGBUFFER                      13
#define C_AMSW_DEBUGBUFFER_SIZE                 14
#endif

#endif

typedef unsigned long AMSW_OperState;

typedef struct {
  unsigned short relCapacityOccupationUpstr;
  unsigned short  attainableBitrateUpstr;
  signed char	 noiseMarginUpstr;  
  signed char    outputPowerDnstr;   
  unsigned char  attenuationUpstr;
  unsigned char  carrierLoad[128];    
  unsigned long  operationalMode;
} T_AMSW_LT_NearEndLineOperData;

typedef struct {
  unsigned short relCapacityOccupationDnstr;
  unsigned short  attainableBitrateDnstr;
  signed char	 noiseMarginDnstr;
  signed char	 outputPowerUpstr;
  unsigned char	 attenuationDnstr;
} T_AMSW_LT_FarEndLineOperData;

typedef struct {
  unsigned short relCapacityOccupationDnstr;
  signed char	 noiseMarginDnstr;
  signed char    outputPowerUpstr;
  unsigned char  attenuationDnstr;
  unsigned long  operationalMode;
} T_AMSW_NT_NearEndLineOperData;

typedef struct {
  unsigned short relCapacityOccupationUpstr;
  signed char	 noiseMarginUpstr;
  signed char	 outputPowerDnstr;
  unsigned char	 attenuationUpstr;
  unsigned char  carrierLoad[128];
} T_AMSW_NT_FarEndLineOperData;

typedef struct {
  unsigned short  actualBitrate;
} T_AMSW_LT_ChannelOperData;

typedef struct {
  unsigned short  actualBitrate;
} T_AMSW_NT_ChannelOperData;

typedef struct {
  unsigned long  status;
  unsigned long  change;
} T_AMSW_def_bitmaps;

typedef struct {
  T_AMSW_def_bitmaps  near_end;
  T_AMSW_def_bitmaps  far_end;
} T_AMSW_def_bitmap_set;

typedef struct {
  unsigned short  FecNotInterleaved;
  unsigned short  FecInterleaved;
  unsigned short  CrcNotInterleaved;
  unsigned short  CrcInterleaved;
  unsigned short  HecNotInterleaved;
  unsigned short  HecInterleaved;
  unsigned short  TotalCellCountInterleaved;
  unsigned short  TotalCellCountNotInterleaved;
  unsigned short  ActiveCellCountInterleaved;
  unsigned short  ActiveCellCountNotInterleaved;
  unsigned short  BERInterleaved;
  unsigned short  BERNotInterleaved;
} T_AMSW_def_counters;

typedef struct {
  T_AMSW_def_counters  near_end;
  T_AMSW_def_counters   far_end;
} T_AMSW_def_counter_set;

typedef unsigned long AMSW_PIN;
typedef unsigned char AMSW_PINSTATE;

#ifdef UNICORN_COMMON
#define AMSW_VERSION_LENGTH 30

typedef struct {   
		char versionA[AMSW_VERSION_LENGTH];
} T_AMSW_VersionMS;

const int STANDARD_DATA_BUFFER_LENGTH = 25000;  

#ifdef _3EC15576_DEBUG_
typedef struct {
        unsigned long contentLength;
		unsigned char debugBuffer[STANDARD_DATA_BUFFER_LENGTH];
} T_AMSW_DebugBuffer;

typedef struct {
        unsigned long bufferSize;
} T_AMSW_DebugBufferSize;
#endif
#endif // UNICORN_COMMON


/* typedefs for analog frontend activation modes */
#define C_ADSL_CHIPSET_IN_RESET         0
#define C_ADSL_CHIPSET_OUT_OF_RESET     1
#define C_ADSL_FRONT_END_SINGLE_MODE    2
#define C_ADSL_FRONT_END_MUX_MODE       3



/* constants for the utopia mode */
#define C_AMSW_UTOPIA_LEVEL1            1
#define C_AMSW_UTOPIA_LEVEL2            2

/* constants for the states */
typedef unsigned short AMSW_ModemState;

#define C_AMSW_IDLE                0
#define C_AMSW_L3                  1
#define C_AMSW_LISTENING           2
#define C_AMSW_ACTIVATING          3
#define C_AMSW_Ghs_HANDSHAKING     4
#define C_AMSW_ANSI_HANDSHAKING    5
#define C_AMSW_INITIALIZING        6
#define C_AMSW_RESTARTING          7
#define C_AMSW_FAST_RETRAIN        8
#define C_AMSW_SHOWTIME_L0         9
#define C_AMSW_SHOWTIME_LQ         10
#define C_AMSW_SHOWTIME_L1         11
#define C_AMSW_EXCHANGE            12
#define C_AMSW_TRUNCATE            13
#define C_AMSW_ESCAPE              14


/* constants for the events */
typedef unsigned char AMSW_ModemEvent;
#define C_AMSW_PEER_ATU_FOUND           0
#define C_AMSW_RESTART_REQUEST          1
#define C_AMSW_ACTIVATION_REQUEST       2
#define C_AMSW_TO_INITIALIZING          3
#define C_AMSW_SHOWTIME                 4
#define C_AMSW_L3_EXECUTED              5
#define C_AMSW_L3_REJECTED              6
#define C_AMSW_L1_EXECUTED              7
#define C_AMSW_L1_REJECTED              8     
#define C_AMSW_L0_REJECTED              9     
#define C_AMSW_RESTART_ACCEPTABLE       10
#define C_AMSW_SUICIDE_REQUEST          11
#define C_AMSW_RESTART_NOT_ACCEPTABLE   12

/* modemfailures */
typedef unsigned short AMSW_ModemFailure;
#define C_AMSW_UNCOMPATIBLE_LINECONDITIONS               5
#define C_AMSW_NO_LOCK_POSSIBLE                         10
#define C_AMSW_PROTOCOL_ERROR                           15
#define C_AMSW_MESSAGE_ERROR                            20
#define C_AMSW_SPURIOUS_ATU_DETECTED                    25
#define C_AMSW_REQ_BITRATE_TOO_HIGH_FOR_LITE            30
#define C_AMSW_INTERLEAVED_PROFILE_REQUIRED_FOR_LITE    35
#define C_AMSW_FORCED_SILENCE                           40
#define C_AMSW_UNSELECTABLE_OPERATION_MODE              45
#define C_AMSW_STATE_REFUSED_BY_GOLDEN                  50
#ifndef BELLORIG
#define AME_CACTX_DURING_CTONES_DETECTED                0x46
#endif
//added from R3.8.129
#define AME_INTEROP_ERROR                               0x50


#endif /* __AMSW_INTF_TYPES_H */



