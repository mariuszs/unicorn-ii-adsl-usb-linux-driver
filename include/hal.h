//----------------------------------------------------------------------
// Test driver for the STMicroelectronics ADSL Chip Taurus
//----------------------------------------------------------------------
// File: hal.h
// Author: Christophe Piel
// Copyright F.H.L.P. 2000
// Copyright ST Microelectronics 2000
//----------------------------------------------------------------------
// provides definitions for hardware access API to the Taurus chip
//----------------------------------------------------------------------
#ifndef _HAL_H_
#define _HAL_H_

#ifdef __cplusplus
extern "C" {
#endif
#undef SUCCESS
#undef FAILURE

typedef enum { SUCCESS,FAILURE } ST_STATUS;
//----------------------------------------------------------------------
//	structure definitions
//----------------------------------------------------------------------

typedef struct {
	WORD addr;
	DWORD data;
} T_DirData;

typedef struct {
	DWORD idata;
	DWORD iaddr;
	DWORD status;
} T_SlaveDirData;

typedef struct {
	DWORD idata[3];
	DWORD icntrl;
	DWORD iaddr;
	DWORD status;
} T_SlaveIndData;

typedef struct {
	DWORD idata;
	DWORD iaddr;
	DWORD mask;
	DWORD status;
} T_SlaveMaskData;

typedef struct {
	DWORD *cmdBuff;
	DWORD *resBuff;
	DWORD wrSize;
	DWORD rdSize;
	DWORD status;
} T_MasterRBlock;

typedef struct {
	DWORD *cmdBuff;
	DWORD wrSize;
	DWORD status;
} T_MasterWBlock;

typedef struct {
	DWORD *resBuff;
	DWORD baseAddr;
	DWORD wrSize;
	DWORD rdSize;
	DWORD status;
} T_MasterRBurst;

typedef struct {
	DWORD *cmdBuff;
	DWORD baseAddr;
	DWORD wrSize;
	DWORD status;
} T_MasterWBurst;

typedef struct {
	DWORD *cmdBuff;
	DWORD SocAddr;
	DWORD SocCntrl;
	DWORD mask;
	DWORD reqSize;
	DWORD exeSize;
	DWORD status;
} T_Msg;

typedef struct {
	PBYTE buffer;
	DWORD length;
} T_WriteAtmData;
//----------------------------------------------------------------------
//	USB hardware access interface
//----------------------------------------------------------------------
//	structure definitions
//----------------------------------------------------------------------
//enum T_Bool {FALSE=0,TRUE=1};
typedef enum {EP2,EP6} T_EpOut ;
typedef enum {EP3,EP5,EP7} T_EpIn ;
enum T_Path {TX_PATH,RX_PATH};

typedef struct {
	BYTE addr;
	short data;
} T_RegData;

typedef struct {
	WORD *cmdBuff;
	WORD wrSize;
	WORD rdSize;
} T_ShortRead;



typedef struct {
	WORD *cmdBuff;
	WORD wrSize;
	WORD rdSize;
} T_LongRead;

typedef struct {
	WORD *cmdBuff;
	WORD frameSize;
} T_ShortWrite;

typedef struct {
	WORD *cmdBuff;
	WORD nFrames;
	WORD frameSize;
	WORD lastFrameSize;
} T_LongWrite;

typedef struct {
	WORD ep0_size;
	WORD ep1_size;
	WORD ep2_size;
	WORD ep3_size;
	WORD ep4_size;
	WORD ep5_size;
	WORD ep6_size;
	WORD ep7_size;
} T_EpSettings;

typedef struct {
	WORD *CMDptrW1;
	WORD *CMDptrW2;
	WORD *CMDptrRd;
	WORD *CMDptrW_I1;
	WORD *CMDptrW_I2;
	WORD *CMDptrRd_I;
	WORD *ITABLEptr;
	WORD *IMASKptr;
	T_EpSettings ep_setting;
} T_USBinit;

typedef struct {
	WORD *cmdBuff;
	WORD wrSize;
	WORD rdSize;
	T_EpOut ep_out;
	T_EpIn ep_in;
} T_EpReadData;

typedef struct {
  WORD *resultBuffer;
  T_EpReadData rd;
} T_EpResultReadData;

typedef struct {
	T_ShortWrite wr;
	T_EpOut ep_out;
} T_EpShortWrite;

typedef struct {
	T_LongWrite wr;
	T_EpOut ep_out;
} T_EpLongWrite;

//----------------------------------------------------------------------
//	Function prototypes
//----------------------------------------------------------------------
ST_STATUS
USB_init(
	WORD **CMDptrW1,
	WORD **CMDptrW2,
	WORD **CMDptrRd,
	WORD **CMDptrW_I1,
	WORD **CMDptrW_I2,
	WORD **CMDptrRd_I,
	WORD **ITABLEptr,
	WORD **IMASKptr,
	T_EpSettings *ep_setting
	);

ST_STATUS
USB_controlRead(
	BYTE addr,
	WORD *data
	);

ST_STATUS
USB_controlWrite(
	BYTE addr,
	WORD data
	);

ST_STATUS
USB_S_Write(
	T_ShortWrite *dataPtr,
	T_EpOut ep_out
	);

ST_STATUS
USB_L_Write(
	T_LongWrite *dataPtr,
	T_EpOut ep_out
	);

ST_STATUS
USB_S_Read(
	T_ShortRead *dataPtr,
	T_EpOut ep_out,
	T_EpIn ep_in
	);

ST_STATUS USB_L_Read(T_LongRead *dataPtr,T_EpOut ep_out,T_EpIn ep_in);

ST_STATUS
USB_WriteAtm(
	PBYTE buffer,
	DWORD length
	);

ST_STATUS
USB_ReadAtm(
	PBYTE buffer,
	DWORD size,
	DWORD *length
	);

BOOL USB_checkIntContext(void);
ST_STATUS USB_OBC_Reset(void);


VOID WritePort(WORD ad,BYTE b);
//----------------------------------------------------------------------
//	EOF
//----------------------------------------------------------------------
#ifdef __cplusplus
} // extern "C"
#endif
	

#endif
