//----------------------------------------------------------------------
// Test driver for the STMicroelectronics ADSL Chip Taurus
//----------------------------------------------------------------------
// File: hard.h
// Author: Christophe Piel
// Copyright F.H.L.P. 2000
// Copyright ST Microelectronics 2000
//----------------------------------------------------------------------
// Contains definitions of the hardware registers and configuration
//----------------------------------------------------------------------
#ifndef	_HARD_H_
#define _HARD_H_

#define SACHEM_RX_INDIRECT_ACC_DATA0_OFFSET 0x0
#define SACHEM_TX_INDIRECT_ACC_DATA0_OFFSET 0x8
#define TRANSFER                            0x8000

       
#define	OBC_CMD_TIMEOUT	1000	// ms
#define	TOSCA_INTR_WDOG	200	// ms

#define	ATM_CELL_LENGTH	53

/* ADSL uP Register Map: REGs */

#define STATUS          0x00             /* Status Control Register        */
#define IDATA_1         0x01             /* Indirect Data Register 1 (LSB) */
#define IDATA_2         0x02             /* Indirect Data Register 2       */
#define IDATA_3         0x03             /* Indirect Data Register 3 (MSB) */
#define IDATA           0x04             /* Indirect Data 3 Register (0)   */
#define IADDR           0x05             /* Indirect Address Register      */
#define IMASKS          0x06             /* Mask to apply to the Slave 
	                                        Indirect R/W operations        */
#define IRMW            0x07             /* Mask to apply when the Slave   
	                                        Read-Modify-Write access is used */

#define IADDR_TX        0x08             /* Indirect TX Reference Address Register */
#define IADDR_RX        0x09             /* Indirect RX Reference Address Register */
#define IADDR_MSG       0x0A             /* Indirect MSG Reference Address */
#define IADDR_CHK       0x0B             /* Indirect Address Check Register */
#define IADDR_BURST     0x0C             /* Burst Base Address Register */
#define SIZE_BURST      0x0D             /* Number of Burst Accesses to be performed */
#define IMASKM          0x0E             /* Mask to apply to the Master Indirect or MSG
                                            read/write operations */

#define IADDR_IRQ       0x0F             /* Indirect Tosca IRQ Table Address Register */
#define ITABLE          0x10             /* Start Address of the Interrupt Table Data Registers */
#define ISR             0x1F             /* ADSL Interrupt status register */


//	Interrupt Status Register bit description
//	-----------------------------------------
#define	ISR_TOIF		0x0001	// TOSCA Direct Interrupt Flag
#define	ISR_TOIE		0x0002	// TOSCA Direct Interrupt Enable
#define	ISR_TOIFS		0x0004	// TOSCA Delayed Interrupt
#define	ISR_TOIES		0x0008	// TOSCA Delayed Interrupt Enable
#define	ISR_TIMIF		0x0010	// Timer Interrupt Flag
#define	ISR_TIMIE		0x0020	// Timer Interrupt Enable
#define	ISR_OSIF		0x0040	// OBC Slave Interrupt Flag
#define	ISR_OSIE		0x0080	// OBC Slave Interrupt Enable
#define	ISR_OMIF		0x0100	// OBC Master Interrupt Flag
#define	ISR_OMIE		0x0200	// OBC Master Interrupt Enable
#define	ISR_GPIFA		0x0400	// GPIN[0] Interrupt Flag
#define	ISR_GPIEA		0x0800	// GPIN[0] Interrupt Enable
#define	ISR_GPIFB		0x1000	// GPIN[4] Interrupt Flag
#define	ISR_GPIEB		0x2000	// GPIN[4] Interrupt Enable

#define	ISR_IF			0x1555	// Mask for all interrupts flags


//----------------------------------------------------------------------
//	USB interface hardware definitions
//----------------------------------------------------------------------
#define	USB_CELL_LENGTH	56

#define ATM_READS_ISO                   4  	// max ATM downstream URB outstanding in ISO mode (0b100)
#define ATM_READS_BULK			        32	// max ATM downstream URB outstanding in BULK mode (0b100000)

#define	ATM_DS_CELLS_PER_PKT_AS5 	    17  // cells per ISO packet AS5
#define	ATM_DS_CELLS_PER_PKT_AS4 	    4  	// cells per ISO packet AS4
#define	ATM_DS_CELLS_PER_PKT_AS3 	    10  	// cells per ISO packet AS3
#define ATM_DS_CELLS_PER_PKT_AS2_56	    1	// cells per ISO packet AS2...For 56 byte device, is 1
#define ATM_DS_CELLS_PER_PKT_AS2_64	    8	// cells per ISO packet AS2...For 64 byte device, is 8
#define	ATM_DS_CELLS_PER_PKT_AS1 	    8  	// cells per ISO packet AS1

#define	READ_ISO_PACKETS_PER_URB    	12	// ATM downstream ISO packets per URB
#define READ_BULK_PACKETS_PER_URB	    1	// ATM downstream BULK packets per URB

#define USB_DS_BUF_SIZE			        32768

#define	ATM_WRITES_ISO	                4	// max ATM upstream URB outstanding in ISO mode (0b100)
#define ATM_WRITES_BULK			       4// max ATM upstream URB outstanding in BULK mode (0b100000)

#define	ATM_US_CELLS_PER_PKT_AS1_3_4   	3	// ATM upstream cells per ISO packet
#define ATM_US_CELLS_PER_PKT_AS2_5	    3	// ATM upstream cells per BULK packet

#define	WRITE_ISO_PACKETS_PER_URB   	12  // ATM upstream ISO packets per URB
#define	WRITE_BULK_PACKETS_PER_URB   	12   // ATM upstream BULK packets per URB

#define USB_US_BUF_SIZE			        4096

typedef struct {
	WORD CmdBufW1[1024];
	WORD CmdBufW2[1024];
	WORD CmdBufRd[1024];

	WORD CmdBufW_I1[1024];
	WORD CmdBufW_I2[1024];
	WORD CmdBufRd_I[1024];

	WORD AtmUsBuf[USB_US_BUF_SIZE];
	WORD AtmDsBuf[USB_DS_BUF_SIZE];
	WORD IntBuf[2][17];
} USB_MEMORY, *PUSB_MEMORY;

//	USB Endpoints
//	-------------
enum {
	USB_EP0,
	EP_INTERRUPT,
	EP_OBC_ISO_OUT,
	EP_OBC_ISO_IN,
	EP_ATM_ISO_OUT,
	EP_ATM_ISO_IN,
	EP_OBC_INT_OUT,
	EP_OBC_INT_IN,
	EP_MAX,
};

//	USB_BRIDGE Register Map
//	=======================
#define	UR_CFW			0x00	// USB Configuration Word
#define	UR_ISDR			0x01	// USB Interrupt Status & Data Register
#define	UR_STAT			0x02	// USB Internal Status Register
#define UR_SHRT_PKT		0x03

//	CFG_MEM Register Map
//	--------------------
#define	UR_C_ADR		0x20	// SPI ADR & CONTROL Register
#define	UR_C_DATA		0x21	// SPI DATA Register

//	ADSL uP Register Map: REGS
//	--------------------------
#define	UR_STATUS		0x40	// Status Control Register
#define	UR_IDATA_1		0x41	// Indirect Data 1 Register (LSB)
#define	UR_IDATA_2		0x42	// Indirect Data 2 Register
#define	UR_IDATA_3		0x43	// Indirect Data 3 Register (MSB)
#define	UR_IDATA		0x44	// Indirect Data 3 Register
#define	UR_IADR			0x45	// Indirect Address Register
#define	UR_IMASKS		0x46	// Mask to apply to Slave Indirect R/W
#define	UR_IRMW			0x47	// Mask to apply to Slave R/M/W access
#define	UR_IADR_TX		0x48	// Indirect TX Reference Address Register
#define	UR_IADR_RX		0x49	// Indirect RX Reference Address Register
#define	UR_IADR_MSG		0x4A	// Indirect MSG Reference Address Register
#define	UR_IADR_CHK		0x4B	// Indirect Address Check Register
#define	UR_IADR_BURST	0x4C	// Burst Base Address Register
#define	UR_SIZE_BURST	0x4D	// Number of Burst accesses to be executed
#define	UR_IMASKM		0x4E	// Mask to apply to the Master Ind or Msg R/W
#define	UR_IADR_IRQ		0x4F	// Indirect Address of Tosca IRQ Table
#define	UR_ITABLE		0x50	// Interrupt Table Data Register
#define	UR_ISR			0x5F	// Interrupt Status Register

//	ADSL uP Register Map: PERIPHERALs
//	---------------------------------
#define	UR_GPIO_DATA	0x60	// Status of the GPIO pins
#define	UR_GPIO_DIR		0x61	// Direction of the GPIO pins
#define	UR_GPIO_PER		0x62	// Persistency values of the GPIO pins
#define	UR_TIM_REG_A	0x63	// Timer A mode and preset value
#define	UR_TIM_REG_B	0x64	// Timer B mode and preset value
#define	UR_AFE_TEST		0x65	// AFE Feedback Control Register
#define UR_TIMEOUT_TIMER 0x6A

//	USB insterrupt status register bitfields
//	----------------------------------------
#define	UISDR_TIRQ1		0x0001	// ADSL uP interrupt flag 1
#define	UISDR_TIRQ2		0x0002	// ADSL uP interrupt flag 2
#define	UISDR_UTIRQ1	0x0004	// UTOPIA rising edge FIFO interrupt flag
#define	UISDR_UTIRQ2	0x0008	// UTOPIA falling edge FIFO interrupt flag
#define	UISDR_ERR_ATM	0x0010	// ATM operation Error flag
#define	UISDR_ERR_OBC	0x0020	// WR OBC operation Error flag
#define	UISDR_ERR_PIPE	0x0040	// WR OBC Access Error flag
#define	UISDR_ACTDIF	0x0080	// ACTD interrupt flag
#define	UISDR_INT_LO	0x0100	// INT_LO flag
#define	UISDR_TIE1		0x0200	// Enable ADSL uP interrupt 1
#define	UISDR_TIE2		0x0400	// Enable ADSL uP interrupt 2
#define	UISDR_IE		0x0800	// Enable Error interrupts
#define	UISDR_LOE		0x1000	// Enable INT_LO interrupts
#define	UISDR_UTIE1		0x2000	// Enable UTOPIA FIFO interrupts
#define	UISDR_UTIE2		0x4000	// Enable UTOPIA FIFO interrupts
#define	UISDR_ACTDIE	0x8000	// Enable ACTD interrupts

#define	UISDR_IF		0x018F	// Mask for all interrupts flags
#define	UISDR_ERF		0x0070	// Mask for all Error flags
#define	UISDR_IEF		0xFE00	// Mask for all Enable flags


//	Control COMMAND bitfields
//	-------------------------
#define	CTRL_FIRST		0x0001	// First control command in pipe
#define	CTRL_LASTI		0x0002	// Last Frame of control cmds in pipe
#define	CTRL_SET_OBCI	0x0004	// Connect WR OBC FIFO to EP6
#define	CTRL_RST_OBCI	0x0008	// Connect WR OBC FIFO to EP2
#define	CTRL_SET_OBCO	0x0010	// Connect RD OBC FIFO to EP7
#define	CTRL_RST_OBCO	0x0020	// Connect RD OBC FIFO to EP3
#define	CTRL_SET_ATM	0x0040	// Connect RD OBC FIFO to EP5
#define	CTRL_RST_ATM	0x0080	// Connect RD OBC FIFO to EP3/EP7
#define	CTRL_EWRITE		0x0100	// Enable Write Command
#define	CTRL_LINK		0x0200	// Next DWORD is another control command
#define	CTRL_ADR		0xFC00	// Mask for ADR field

//	LED bitfields
//	-------------
#define LED_POWER       0x0800
#define LED_SHOWTIME    0x0200
#define LED_INIT        0x0400


/* Definition of OBC Status Register */

enum SEX_FIELD {SLAVE_STOP  = 0x0000,   /* 0000 0000 0000 0000 */
								SLAVE_DIR   = 0x0001,   /* 0000 0000 0000 0001 */
								SLAVE_IND   = 0x0002,   /* 0000 0000 0000 0010 */
								SLAVE_RM    = 0x0003    /* 0000 0000 0000 0011 */
};

enum MEX_FIELD {MASTER_STOP      = 0x0000,   /* 0000 0000 0000 0000 */
                MASTER_BLOCK_DIR = 0x0001,   /* 0000 0000 0000 0001 */
								MASTER_BURST_DIR = 0x0002,   /* 0000 0000 0000 0010 */
								MASTER_BLOCK_IND = 0x0003,   /* 0000 0000 0000 0011 */
								MASTER_BURST_IND = 0x0004,   /* 0000 0000 0000 0100 */
								MASTER_MSG       = 0x0005    /* 0000 0000 0000 0101 */
}; 

/* Masks for setting Status Bitfields */

#define STATUS_INIT    	0x2800         /* 0010 1000 0000 0000 */ /* MIRRORE+TOSCA_EN */
#define RX_MASK    	    0x0000         /* 0000 0000 0000 0000 */
#define TX_MASK    	    0x0400         /* 0000 0100 0000 0000 */
#define RD_MASK    	    0x0008         /* 0000 0000 0000 1000 */
#define WR_MASK    	    0x0000			   /* 0000 0000 0000 0000 */
#define MEX_MASK        0x0010         /* 0000 0000 0001 0000 */
#define SEX_MASK        0x0000         /* 0000 0000 0000 0000 */
#define START_OBC       0x0020         /* 0000 0000 0010 0000 */
#define OBC_CLEAR       0x0200         /* 0000 0010 0000 0000 */

#define INDIRECT_SLAVE_MASK  0x8000

#define UTOPIA_EN			 0xC000         /* 1100 0000 0000 0000 */

#define SLAVE_DIR_RD    0x2829         /* 0010 1000 0010 1001 */ 
#define SLAVE_DIR_WR    0x2821         /* 0010 1000 0010 0001 */
#define SLAVE_IND_RD    0x282A         /* 0010 1000 0010 1010 */
#define SLAVE_IND_WR    0x2822         /* 0010 1000 0010 0010 */
#define SLAVE_RM_WR     0x2823         /* 0010 1000 0010 0011 */

#define MASTER_DIR_RD_BLOCK    0x2839         /* 0010 1000 0011 1001 */  /* OBC_FIFO_CLEAR is setted in MASTER */
#define MASTER_DIR_WR_BLOCK    0x2831         /* 0010 1000 0011 0001 */
#define MASTER_IND_RD_BLOCK    0x283B         /* 0010 1000 0011 1011 */
#define MASTER_IND_WR_BLOCK    0x2833         /* 0010 1000 0011 0011 */

#define MASTER_DIR_RD_BURST    0x283A         /* 0010 1000 0011 1010 */ 
#define MASTER_DIR_WR_BURST    0x2832         /* 0010 1000 0011 0010 */
#define MASTER_IND_RD_BURST    0x283C         /* 0010 1000 0011 1100 */
#define MASTER_IND_WR_BURST    0x2834         /* 0010 1000 0011 0100 */

#define MASTER_RD_MSG          0x283D         /* 0010 1000 0011 1101 */
#define MASTER_WR_MSG		   0x2835         /* 0010 1000 0011 0101 */

#define U_CFW           0x00            /* USB Configuration Word */
#define U_ISDR          0x01            /* USB Interrupt Status & Data Register */
#define U_STAT          0x02            /* USB Interrupt Status & Data Register */

/* CFG_MEM Register Map: Offsets used */
                                  
#define C_ADDR          0x20             /* SPI ADDR & CONTROL register*/
#define C_DATA          0x21             /* SPI DATA register */

#endif
