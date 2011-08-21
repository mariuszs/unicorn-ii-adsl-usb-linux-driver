//----------------------------------------------------------------------
// Test driver for the STMicroelectronics ADSL Chip Taurus
//----------------------------------------------------------------------
// File: adsldrv.h
// Author: Christophe Piel
// Copyright F.H.L.P. 2000
// Copyright STMicroelectronics 2000
//----------------------------------------------------------------------
// Uses Compuware's DriverWorks classes
//----------------------------------------------------------------------
#ifndef _UNICORN_USBDRV_H_
#define _UNICORN_USBDRV_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <linux/usb.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/atmdev.h>
#include <asm/byteorder.h>
#include "types.h"
#include "hal.h"
#include "hard.h"
#include "rapi.h"
#include "amas.h"
#include "amsw_ant.h"
#include "amsw_init.h"
#include "amsw_intf_types.h"  
#include "crc.h"
#include "unicorn.h"
#include "debug.h"

//----------------------------------------------------------------------
// R/W objects
//----------------------------------------------------------------------
#define	CELL_LENGTH             USB_CELL_LENGTH

struct send_atm {
	BOOLEAN started;
	int turn_write;				// Current Write buffer
	int turn_send;				// Current sending buffer
	unsigned char *bufs[ATM_WRITES_BULK];
	int lens[ATM_WRITES_BULK];
	unsigned long busy[ATM_WRITES_BULK];
	int maxlen;
	int bufofs;
	unsigned long long cell_count;
	spinlock_t lock;	
};

#define RECV_ATM_MAX_CELLS 1024

struct recv_atm {
	BOOLEAN started;
	unsigned short turn_read;
	unsigned short turn_recv;
	unsigned char *cells[RECV_ATM_MAX_CELLS];
        int used[RECV_ATM_MAX_CELLS];
	int maxlen;
	int pipe_index;
	int num_reads;
	unsigned long long cell_count;
	spinlock_t lock;	
};


struct unicorn_dev {	
	struct usb_device 	*usb_dev;
	int                     alternate_setting;
	struct send_atm		send_atm;		// US ATM object for transmission
	struct recv_atm		recv_atm;		// DS ATM object for transmission
	struct urb  		*int_in_pipe[2];	// Endpoint 1	
       	struct urb	        *obc_iso_out;		// Endpoint 2	
	struct urb		*obc_iso_in;		// Endpoint 3	
	struct urb	        *atm_write[ATM_WRITES_BULK];	// ATM US Transfer (EP 4)
	struct urb		*atm_read[ATM_READS_BULK];	// ATM DS transfer (EP 5)
	struct urb		*obc_int_out;		// Endpoint 6	
	struct urb		*obc_int_in;		// Endpoint 7	

	USB_MEMORY		*usb_mem;	       // Pointer to the USB buffers
	volatile PBYTE		dma_virtual_addr;      // DMA buffer virtual address
	DWORD                   obc_sem;               // To wait for previous OBC command
	unsigned long		obc_flags;	       // OBC Flags
	struct semaphore	obc_lock;
	BOOLEAN			started;
	BOOLEAN			msw_started;		// True if the MSW is (manually) started
};

#ifdef __cplusplus
}
#endif
#endif			// __unicorn_usbdrv_h__
