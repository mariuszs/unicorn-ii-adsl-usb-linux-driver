/*
   This driver supports the Unicorn ADSL chipset from STMicroelectronics.
   The chipset consists of the ADSL DMT transceiver ST70138 and the ST70174 
   Analog Front End (AFE).
   This file contains the USB specific routines.
 */
#include <linux/autoconf.h>
#include <linux/version.h>

#if defined(CONFIG_MODVERSIONS) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#include <linux/modversions.h>
#endif

#ifndef PKG_VERSION
#   define PKG_VERSION "unknown"
#endif

#ifndef DRIVER_VERSION
#   define DRIVER_VERSION "unknown"
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include "../include/unicorn_usbdrv.h"
#include "../include/tracetool.h"

// Compatability stuff
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
typedef struct usb_iso_packet_descriptor iso_packet_descriptor_t;
#ifndef FILL_BULK_URB
#define FILL_BULK_URB usb_fill_bulk_urb
#endif

#ifndef FILL_INT_URB
#define FILL_INT_URB usb_fill_int_urb
#endif

#ifndef USB_ISO_ASAP
#define USB_ISO_ASAP URB_ISO_ASAP
#endif

#ifndef USB_ST_DATAUNDERRUN 
#define USB_ST_DATAUNDERRUN	(-EREMOTEIO)
#endif

#ifndef USB_ST_BANDWIDTH_ERROR
#define USB_ST_BANDWIDTH_ERROR	(-ENOSPC)			/* too much bandwidth used */
#endif

#define ALLOC_URB(iso_pkts) usb_alloc_urb(iso_pkts,GFP_ATOMIC)

#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,20))
typedef struct iso_packet_descriptor iso_packet_descriptor_t;
#define ALLOC_URB(iso_pkts) usb_alloc_urb(iso_pkts)
#else
#define ALLOC_URB(iso_pkts) usb_alloc_urb(iso_pkts)
#endif


MODULE_AUTHOR ("ashutosh.sharma@st.com");
MODULE_DESCRIPTION ("ATM driver for the ST UNICORN II ADSL modem.");

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

unsigned long AlternativeSetting = 0;
extern unsigned long bitswapEnable;

#define ALT_SETTING_INIT 2

#define RETRY_UNDERRUN 3

#ifdef CRTSTUFFS_O
void * __dso_handle= & __dso_handle;
#else
void * __dso_handle =0;
#endif

int  __cxa_atexit(void (*func)(void*x),void *arg, void *d )
{
	return 0;
}

extern void CreateObject(void);
extern void DeleteObject(void);
extern void createSemaphore(void);
static void SetUsbTimer(BOOL bStart);
static int init_usb_urb(struct unicorn_dev *dev);
static int term_usb_urb(struct unicorn_dev *dev);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void IntInComplete(struct urb *urb,struct pt_regs *pt_regs);
#else
static void IntInComplete(struct urb *urb);
#endif

extern struct semaphore acc_sema ;
struct unicorn_dev unicorn_usb_dev;

/*
 *  * Unlink an URB with error reporting. This is a macro so
 *   * the __FUNCTION__ returns the caller function name.
 * 
 */

#if  (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10))
#define UNLINK_URB(urb) \
({ \
 	if (GlobalRemove == FALSE ) {\
		int status; \
		if (urb) { \
			DBG(USB_D,"usb_unlink_urb,urb=%p,actual_length=%d\n",urb,urb->actual_length); \
			if ((status = usb_unlink_urb(urb)) < 0) { \
				DBG(1,"usb_unlink_urb failed,status=%d\n", status); \
			} \
		} else { \
			DBG(USB_D,"no urb\n"); \
		status = -1; \
		} \
		status; \
	} else{ \
		if (urb) { \
			usb_kill_urb(urb); \
		} \
	} \
})
#else
#define UNLINK_URB(urb) \
({ \
	int status; \
	if (urb) { \
		DBG(USB_D,"usb_unlink_urb,urb=%p,actual_length=%d\n",urb,urb->actual_length); \
		if ((status = usb_unlink_urb(urb)) < 0) { \
			DBG(1,"usb_unlink_urb failed,status=%d\n", status); \
		} \
	} else { \
		DBG(USB_D,"no urb\n"); \
	status = -1; \
	} \
	status; \
})
#endif

/*
 * Submit an URB with error reporting
 */
static int SUBMIT_URB(struct urb* urb) 
{ 
	int status; 

	if (urb && !GlobalRemove) { 
		DBG(USB_D,"usb_submit_urb,urb=%p,length=%d\n",urb,urb->transfer_buffer_length); 
		urb->hcpriv = 0; 
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
		status = usb_submit_urb(urb,GFP_ATOMIC); 
#else
		status = usb_submit_urb(urb); 
#endif
		if (status  < 0) { 
			DBG(USB_D,"usb_submit_urb failed,status=%d\n", status); 
		}
	} 
	else { 
		if (GlobalRemove) {
			DBG(USB_D,"GlobalRemove detected in SUBMIT_URB\n");
		}
		if (!urb) {
			DBG(USB_D,"SUBMIT_URB: urb: %p\n",urb);
		} 
		status = -1; 
	} 
	return status;
}

/*static void dump_urb(struct urb *urb)
{
#if DEBUG
	int i;

	printk(KERN_DEBUG "hcpriv=%p,dev=%p,pipe=%x,status=%d,transfer_flags=%d\n",
			urb->hcpriv,urb->dev,urb->pipe,urb->status,urb->transfer_flags);

#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,3))
	printk(KERN_DEBUG "bandwidth=%d,start_frame=%u,interval=%d,error_count=%d,timeout=%d\n",
			urb->bandwidth,urb->start_frame,urb->interval,urb->error_count,urb->timeout);
#else
	printk(KERN_DEBUG "start_frame=%u,interval=%d,error_count=%d\n",
			urb->start_frame,urb->interval,urb->error_count);

#endif
	// buffers
	printk(KERN_DEBUG "transfer_buffer=%p,transfer_buffer_length=%d,actual_length=%d\n",
			urb->transfer_buffer,urb->transfer_buffer_length,urb->actual_length);
	for (i=0; i < urb->number_of_packets; i++) {
		iso_packet_descriptor_t *frame = &urb->iso_frame_desc[i]; 
		printk(KERN_DEBUG "[%d],status=%d,actual_length=%d,length=%d,offset=%d\n",
				i,frame->status,frame->actual_length,frame->length,frame->offset);

	}
#endif
}*/

#define DUMP_URB(urb) //dump_urb(urb)

static void fill_isoc_urb(struct urb *urb, struct usb_device *dev, unsigned int pipe, 
		void *buf, int length, int packet_size, usb_complete_t complete, void *context) 
{
#if 	(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
	spin_lock_init(&urb->lock);
#endif
	urb->dev=dev;
	urb->pipe=pipe;
	urb->transfer_buffer=buf;
	urb->transfer_buffer_length=length;
	urb->actual_length = 0;
	urb->transfer_flags=USB_ISO_ASAP;
	urb->start_frame = -1;
	urb->interval = 1; 
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,3))
	urb->timeout = 0;
#endif
	urb->complete=complete;
	urb->context=context;

	{
		int offset = 0;
		int packetNr=0;

		while (length > 0) {
			urb->iso_frame_desc[packetNr].offset = offset;
			urb->iso_frame_desc[packetNr].length = MIN(packet_size,length);
			urb->iso_frame_desc[packetNr].actual_length = 0;
			offset += urb->iso_frame_desc[packetNr].length;
			length -= urb->iso_frame_desc[packetNr].length;
			packetNr++;	
		}
		urb->number_of_packets = packetNr;
	}
}

//----------------------------------------------------------------------
// Macros to read/write the little endian USB bus
//----------------------------------------------------------------------
#ifdef __BIG_ENDIAN__
static inline void USB_SWAP_BUF(WORD *buf,WORD size)
{
	int i;
	for (i=0; i < size; i++) 
		buf[i] = cpu_to_le16(buf[i]);
}
#else
#define USB_SWAP_BUF(buf,size)
#endif

//----------------------------------------------------------------------
// driver parameters
//----------------------------------------------------------------------
#if DEBUG
unsigned long DebugLevel=0;
#endif
int FrameNumber = 6; 
//unsigned long ledScenario = 0;
unsigned long ledScenario = 1;

//----------------------------------------------------------------------
// MSW paramters
//----------------------------------------------------------------------

extern unsigned long LCD_Trig;
extern unsigned long LOS_LOF_Trig;
extern unsigned long RetryTime;
extern unsigned long TruncateMode;
extern unsigned long TNumberOfCarrier = 80;
extern unsigned long g_ModemState;

unsigned long ActivationMode = MSW_MODE_MULTI ;
unsigned long DownstreamRate = 8128;	// In Kbits/sec	
unsigned long g_RefGain;
unsigned short g_TeqMode;
unsigned long Interoperability;
unsigned long TrainingDelay;
unsigned long pilotRealloc;
unsigned long _newCToneDetection_;
unsigned long useRFC019v;
unsigned long useRFC029v;
unsigned long useRFC040v;
unsigned long useRFCFixedRate;
unsigned long useVCXO;
unsigned long txPower;
unsigned long _gi_step_;   
unsigned long _teq_new_delay_;
unsigned long highCarrierOff;
unsigned long decreaseHighCarrier;
unsigned long _boostPowerGdmt_; 
unsigned long AutoActivation=1;
unsigned long LoopbackMode=0;
unsigned long MswDebugLevel=2;
unsigned long last_report = 0L;
AMSW_ModemFailure last_failure = C_AMSW_NO_HARDWARE;
AMSW_ModemEvent last_event = (AMSW_ModemEvent)-1L;
DWORD adsl_system_time = 0L;
ADSL_STATUS adsl_status = ADSL_STATUS_NOHARDWARE;
unsigned short adsl_us_cellrate = 0;
unsigned short adsl_ds_cellrate = 0;

unsigned long GlobalRemove = TRUE;

int gLedXmit = 0;		// Transmit data indicator
int gLedRecv = 0;		// Receive data indicator
int gAtmUsbError=0;

long subfunction = /*0xA1*/161;
long globSubFunctNoAdsf = 1;
long globSubFunctNoAdsg = 0;
long pvo_noboost2db = 0;
long azuma_boost2db = 1;
long pvo_usesachemirq = 1;
long pvo_noadse = 1;
long pvo_noisdn = 1;
long ej_utopiaFifoHecCheck = 0;
long ej_utopiaFifoTraffic = 0;
long isdn_up_to_tone_53 = 0;
long cisco_hs_fix_enable = 0;
long pvo_pembdpllpolarity = 1;
long pvo_vendorId_near_end = 0x22;
long pvo_enableDma = 0;
long pvo_initialDAC = -1;
long pvo_trellisallowed = 1;
long wd_bertestgenactive = 0;
long wd_freezeMedley = 0;
long wd_freezeReverb = 0;
long wd_freezePilot = 0;
long ber_on_std_cell = 0;
long s_half_allowed = 0;


//----------------------------------------------------------------------
// Exported functions
//----------------------------------------------------------------------
unsigned char *unicorn_snd_getcell(struct unicorn_dev *dev);
int unicorn_start_transmit(struct unicorn_dev *dev);
unsigned char *unicorn_rcv_getcell(struct unicorn_dev *dev);
int unicorn_msw_control(struct unicorn_dev *dev,T_MswCtrl *ctrl);
ADSL_STATUS unicorn_get_adsl_status(struct unicorn_dev *dev);
int unicorn_get_adsl_linkspeed(struct unicorn_dev *dev,
		unsigned long *us_rate,unsigned long *ds_rate);

#define OBC_READ_CMD 	0x01	// Set if Read OBC command
#define OBC_CMD_INT_LO 	0x02	// Set after the USB INT_LO interrupt
#define OBC_CMD_INT 	0x04	// Set after the USB OBC interrupt
#define OBC_WRITE_CPLT 	0x08	// Set after the OBC write completion
#define OBC_READ_CPLT 	0x10	// Set after the OBC read IRP completion

#define OBC_LOCK(sem) down(sem)
#define OBC_UNLOCK(sem) up(sem)


struct unicorn_entrypoints unicorn_usb_entrypoints = {
	&unicorn_usb_dev,
	unicorn_snd_getcell,
	unicorn_rcv_getcell,
	unicorn_start_transmit,
	unicorn_msw_control,
	unicorn_get_adsl_status,
	unicorn_get_adsl_linkspeed
};

//----------------------------------------------------------------------
// atm_send_complete:
//----------------------------------------------------------------------
static void atm_send_complete(struct unicorn_dev *dev)
{
	struct send_atm *send_atm = &dev->send_atm;
	int t;

	DBG(RW_D,"atm_send_complete:\n");

	t = send_atm->turn_send;
	if ((dev->alternate_setting != 2) && (dev->alternate_setting != 5)) {
		if (test_and_clear_bit(0,&send_atm->busy[t])) {
			send_atm->lens[t] = 0;
			t = (t+1)&(ATM_WRITES_ISO-1);
			send_atm->turn_send = t;
		} else {
			DBG(ATM_D,"busy ??\n");
		}
	}
	else {
		if (test_and_clear_bit(0,&send_atm->busy[t])) {
			send_atm->lens[t] = 0;
			t = (t+1)&(ATM_WRITES_BULK-1);
			send_atm->turn_send = t;
		} else {
			DBG(ATM_D,"busy ??\n");
		}
	}
}

//----------------------------------------------------------------------
//	ATM US transfer complete
//----------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void AtmUsXferComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void AtmUsXferComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;

	DBG(RW_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
			urb->status,urb->transfer_buffer_length,urb->actual_length);	

	if (urb->status == 0) {
		atm_send_complete(dev);
	} else {
		DBG(ATM_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);	
		atm_send_complete(dev);
	}
}

//----------------------------------------------------------------------
//	Start the ATM upstream DMA
//----------------------------------------------------------------------
static void StartAtmUsXfer(struct unicorn_dev *dev,int turn,unsigned char *buffer,int length)
{
	struct urb *urb = dev->atm_write[turn];

	DBG(RW_D,"(%d,%d)\n",turn,length);

	if (GlobalRemove) 
	{
		return;
	}
	if ((dev->alternate_setting != 2) &&( dev->alternate_setting != 5)) {
		// Fill the isochronous URB
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
		fill_isoc_urb(urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_ATM_ISO_OUT),
				buffer, length,  dev->usb_dev->epmaxpacketout[EP_ATM_ISO_OUT], 
				AtmUsXferComplete, dev);
#else
		//Changes for kernel 2.6.13
		fill_isoc_urb(urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_ATM_ISO_OUT),
				buffer, length,
				usb_maxpacket(dev->usb_dev,
					usb_sndisocpipe(dev->usb_dev,EP_ATM_ISO_OUT),1), 
				AtmUsXferComplete, dev);


#endif
	} else {
		// Fill the bulk URB
		FILL_BULK_URB(urb, dev->usb_dev, usb_sndbulkpipe(dev->usb_dev,EP_ATM_ISO_OUT),
				buffer, length, AtmUsXferComplete, dev);
	}
	SUBMIT_URB(urb);
}

//----------------------------------------------------------------------
//	ATM DS transfer complete
//----------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void AtmDsXferComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void AtmDsXferComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;
	struct recv_atm *recv_atm = &dev->recv_atm;
	int cells;
	int x;
	unsigned char *cell;

	DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
			urb->status,urb->transfer_buffer_length,urb->actual_length);	

	if (GlobalRemove) {
		return;
	}

	if ((urb->status == USB_ST_DATAUNDERRUN) && (urb->actual_length > 0)) {
		urb->status = 0;
	}

	if (urb->status) {
		DBG(ATM_D,"urb->status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);	
		// try again...
	}

	if (urb->actual_length > 0) {
		if (usb_pipeisoc(urb->pipe)) {

			int i;

			// ISOC
			for (i=0; i < urb->number_of_packets; i++) {
				iso_packet_descriptor_t *frame = &urb->iso_frame_desc[i];
				cells = frame->actual_length/USB_CELL_LENGTH;
				cell = (unsigned char *)urb->transfer_buffer+frame->offset;
				while (cells--) {
					DBG(RW_D,"ISOC: turn_recv=%d,cell=%p\n",recv_atm->turn_recv,cell);

					if (!hecCheck(cell)) {
						DBG(ATM_D,"HEC error\n");
						gAtmUsbError=1;
					}

					for (x = 0; x < ATM_CELL_LENGTH; x++) {
						recv_atm->cells[recv_atm->turn_recv][x] = cell[x];
					}

					recv_atm->used[recv_atm->turn_recv++] = TRUE;
					if (recv_atm->turn_recv > (RECV_ATM_MAX_CELLS-1)) {
						recv_atm->turn_recv = 0;
					}

					if (recv_atm->turn_recv == recv_atm->turn_read)
					{
						DBG(ATM_D,"recv_atm buffer overrun, %d, %d", recv_atm->turn_recv, recv_atm->turn_read);
					}
					cell += USB_CELL_LENGTH;
				}
			}
		} else {

			// BULK
			cells = urb->actual_length/USB_CELL_LENGTH;
			cell = (unsigned char *)urb->transfer_buffer;
			while (cells--) {
				DBG(RW_D,"BULK: turn_recv=%d,cell=%p\n",recv_atm->turn_recv,cell);

				// check CRC
				if (!hecCheck(cell)) {
					DBG(ATM_D,"HEC error\n");
					gAtmUsbError=1;
				}

				for (x = 0; x < ATM_CELL_LENGTH; x++) {
					recv_atm->cells[recv_atm->turn_recv][x] = cell[x];
				};

				recv_atm->used[recv_atm->turn_recv++] = TRUE;
				if (recv_atm->turn_recv >= RECV_ATM_MAX_CELLS) {
					recv_atm->turn_recv = 0;
				}

				if (recv_atm->turn_recv == recv_atm->turn_read) {
					DBG(ATM_D,"recv_atm buffer overrun, %d, %d", recv_atm->turn_recv, recv_atm->turn_read);
				}
				cell += USB_CELL_LENGTH;
			}
		}
	}
	urb->dev = dev->usb_dev;
	urb->transfer_flags = USB_ISO_ASAP;
	SUBMIT_URB(urb);
}

//----------------------------------------------------------------------
// atm_stop_rcv:
//----------------------------------------------------------------------
static void atm_stop_rcv(struct unicorn_dev *dev)
{
	struct recv_atm *recv_atm = &dev->recv_atm;
	int i;
	struct urb *urb;
	DBG(USB_D,"atm_stop_rcv started=%d\n",recv_atm->started);

	if (!recv_atm->started) return;
	for (i=0; i < recv_atm->num_reads; i++) {
		DBG(USB_D,"UNLINK_URB");
		//struct urb *urb = dev->atm_read[i];
		urb = (struct urb*)dev->atm_read[i];
//		UNLINK_URB(urb);
	}

	recv_atm->started = FALSE;
}

//----------------------------------------------------------------------
// atm_start_rcv
//----------------------------------------------------------------------
static void atm_start_rcv(struct unicorn_dev *dev)
{
	struct recv_atm *recv_atm = &dev->recv_atm;
	int i;
	unsigned char *buffer;
	int size;
	int chk_size;
	int packet_size;

	if (recv_atm->started) {
		return;
	}

	recv_atm->turn_recv = 0;
	recv_atm->turn_read = 0;
	recv_atm->pipe_index = 0;

	// Initialize the R/W
	buffer = (unsigned char *)(dev->usb_mem->AtmDsBuf);

	switch (dev->alternate_setting)
	{
		case 1 :
			size = ATM_DS_CELLS_PER_PKT_AS1*USB_CELL_LENGTH*READ_ISO_PACKETS_PER_URB;
			recv_atm->num_reads = ATM_READS_ISO;
			break;
		case 2 :
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
        		chk_size = dev->usb_dev->epmaxpacketin[EP_ATM_ISO_IN];
#else
        		chk_size = usb_maxpacket(dev->usb_dev,usb_rcvisocpipe(dev->usb_dev,EP_ATM_ISO_IN),0);
#endif
			if(chk_size == 56) {
				size = ATM_DS_CELLS_PER_PKT_AS2_56*USB_CELL_LENGTH*READ_BULK_PACKETS_PER_URB;
			} else {
				size = ATM_DS_CELLS_PER_PKT_AS2_64*USB_CELL_LENGTH*READ_BULK_PACKETS_PER_URB;
			}
			recv_atm->num_reads = ATM_READS_BULK;
			break;
		case 3 :
			size = ATM_DS_CELLS_PER_PKT_AS3*USB_CELL_LENGTH*READ_ISO_PACKETS_PER_URB;
			recv_atm->num_reads = ATM_READS_ISO;
			break;
		case 4 :
			size = ATM_DS_CELLS_PER_PKT_AS4*USB_CELL_LENGTH*READ_ISO_PACKETS_PER_URB;
			recv_atm->num_reads = ATM_READS_ISO;
			break;
		case 5 :
			size = ATM_DS_CELLS_PER_PKT_AS5*USB_CELL_LENGTH*READ_ISO_PACKETS_PER_URB;
			recv_atm->num_reads = ATM_READS_ISO;
			break;
		default :
			DBG(ATM_D,"Unsupported alternate setting\n");
			size = ATM_DS_CELLS_PER_PKT_AS5*USB_CELL_LENGTH*READ_ISO_PACKETS_PER_URB;
			recv_atm->num_reads = ATM_READS_ISO;
			break;
	}
	recv_atm->maxlen = size;

#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
	packet_size = dev->usb_dev->epmaxpacketin[EP_ATM_ISO_IN];
#else
	packet_size = usb_maxpacket(dev->usb_dev,usb_rcvisocpipe(dev->usb_dev,EP_ATM_ISO_IN),0);
#endif

	PRINT_INFO("buffer=%p,size=%d,packet_size=%d,num_reads=%d\n",
			buffer,size,packet_size,recv_atm->num_reads);

	for (i=0; i < recv_atm->num_reads; i++) {
		struct urb *urb = dev->atm_read[i];		

		if (dev->alternate_setting != 2) {
			// Fill the isochronous URB
			fill_isoc_urb(urb, dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,EP_ATM_ISO_IN),
					buffer + (i * size), recv_atm->maxlen, packet_size, AtmDsXferComplete, dev);
			SUBMIT_URB(dev->atm_read[i]);
		}                                           
		else {
			// Fill the bulk URB
			FILL_BULK_URB(urb, dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev,EP_ATM_ISO_IN),
					buffer + (i * size), recv_atm->maxlen, AtmDsXferComplete, dev);
			SUBMIT_URB(dev->atm_read[i]);
		}
	}

	recv_atm->cell_count = 0;
	recv_atm->started = TRUE;
}

//----------------------------------------------------------------------
// atm_stop_snd
//----------------------------------------------------------------------
static void atm_stop_snd(struct unicorn_dev *dev)
{
	struct send_atm *send_atm = &dev->send_atm;
	int i;

	DBG(USB_D,"atm_stop_snd started=%d\n",send_atm->started);

	if (!send_atm->started) {
		return;
	}

	send_atm->started = FALSE;
	send_atm->turn_send = 0;
	send_atm->turn_write = 0;

	if((dev->alternate_setting != 2) && (dev->alternate_setting != 5))
	{
		for (i=0; i < ATM_WRITES_ISO; i++) {
			send_atm->lens[i] = 0;
		}
	}
	else {
		for (i=0; i < ATM_WRITES_BULK; i++) {
			send_atm->lens[i] = 0;
		}
	}
}

//----------------------------------------------------------------------
// atm_start_snd
//----------------------------------------------------------------------
static void atm_start_snd(struct unicorn_dev *dev)
{
	struct send_atm *send_atm = &dev->send_atm;
	int i;
	unsigned char *buffer;
	int size;

	DBG(1,"started=%d\n",send_atm->started);

	if (send_atm->started) {
		return;
	}

	send_atm->bufofs = 0;
	send_atm->turn_send = 0;
	send_atm->turn_write = 0;

	// Initialize the R/W
	buffer = (unsigned char *)dev->usb_mem->AtmUsBuf;

	if((dev->alternate_setting != 2) && (dev->alternate_setting != 5)) {
		size = ATM_US_CELLS_PER_PKT_AS1_3_4 * USB_CELL_LENGTH * WRITE_ISO_PACKETS_PER_URB;
		send_atm->maxlen = size;
		for (i=0; i < ATM_WRITES_ISO; i++) {
			send_atm->bufs[i] = buffer + (i * size);
			send_atm->lens[i] = 0;
			send_atm->busy[i] = 0;
		}
	}
	else {
		size = ATM_US_CELLS_PER_PKT_AS2_5 * USB_CELL_LENGTH * WRITE_BULK_PACKETS_PER_URB;
		send_atm->maxlen = size;
		for (i=0; i < ATM_WRITES_BULK; i++) {
			send_atm->bufs[i] = buffer + (i * size);
			send_atm->lens[i] = 0;
			send_atm->busy[i] = 0;
		}
	}

	send_atm->cell_count = 0;
	send_atm->started = TRUE;
}


//----------------------------------------------------------------------
// adjust_bandwidth()
//----------------------------------------------------------------------
static void adjust_bandwidth(struct unicorn_dev *dev,unsigned short AdslUsCellRate ,unsigned short AdslDsCellRate, short NewSetting)
{

	struct usb_device *usb_dev = dev->usb_dev;
	struct urb *urb;

	unsigned long TotalBandwidth = 0;
	unsigned long AvailableBandwidth = 0;

	int status = 0;

	// Prevent any access to OBC from the Modem Software
	// -------------------------------------------------

	PRINT_INFO("Adsl Cell Rate = %d\n", AdslDsCellRate);

	// Not known yet how to know ConsumedBandwidth

	//m_Lower.GetUsbBandwidth(TotalBandwidth,ConsumedBandwidth);
	//AvailableBandwidth = TotalBandwidth - ConsumedBandwidth;
	AvailableBandwidth = 12000;
	TotalBandwidth = 12000;

	PRINT_INFO("Total bw: %d available bw: %d \n",(int) TotalBandwidth,(int) AvailableBandwidth);


	// Select alternate setting to match speed/bandwidth used
	if(NewSetting == 3)
	{
		if(AvailableBandwidth < 10700) NewSetting = 2;
	}
	else if (NewSetting == 1)
	{
		if(AvailableBandwidth < 9718) NewSetting = 2;

	}
	else if (NewSetting == 5)
	{
		if(AvailableBandwidth < 10210) NewSetting = 2;

	}
	else if (NewSetting == 4)
	{
		if(AvailableBandwidth < 7078) NewSetting = 2;

	}
	else if (NewSetting < 0)		
	{
		// New configuration is dynamic (based on negotiated bitrate)
		// ----------------------------------------------------------
		if (AdslDsCellRate > 10000)
		{
			if(AvailableBandwidth > 10210) NewSetting = 5;			
			else NewSetting = 2;
		}			
		else if( AdslDsCellRate > 8000) 
		{
			if(AvailableBandwidth > 10700) NewSetting = 3;						
			else NewSetting = 2	;			
		}
		else if( AdslDsCellRate > 4000)
		{
			if(AvailableBandwidth > 9718) NewSetting = 1;			
			else NewSetting = 2	;			

		}
		else 
		{
			if(AvailableBandwidth > 7080) NewSetting = 4;			
			else NewSetting = 2;
		}
	}

	if (dev->alternate_setting != NewSetting) {

		down(&acc_sema);

		term_usb_urb(dev);          
		usb_reset_configuration(usb_dev);

		dev->alternate_setting = NewSetting;

		if ((status = usb_set_interface (usb_dev,0, dev->alternate_setting)) < 0) {
			if (status == USB_ST_BANDWIDTH_ERROR) {
				PRINT_INFO("insufficient USB bandwidth\n");
			}
			else {
				PRINT_INFO("usb_set_interface (alt %d) failed,status=%d\n",dev->alternate_setting,status);
			}
		} else {
			PRINT_INFO("Alternate setting is %d\n",NewSetting);
		}

		init_usb_urb(dev);

		urb = dev->int_in_pipe[0];
		FILL_INT_URB(urb, usb_dev,
				usb_rcvintpipe(usb_dev, EP_INTERRUPT),
				dev->usb_mem->IntBuf[0],sizeof(dev->usb_mem->IntBuf[0]),
				IntInComplete,dev,1);
		SUBMIT_URB(urb);

		up(&acc_sema);
	}else {
		PRINT_INFO("Alternate setting is(init) %d\n",NewSetting);
	}
}

//======================================================================
//	REPORTS ADSL status from the MSW
//======================================================================

//----------------------------------------------------------------------
//	Modem Software Event Report for user-mode application
//----------------------------------------------------------------------
static const char *get_msw_event_string(AMSW_ModemEvent event)
{
	static char s[8];

	switch(event) {
		case C_AMSW_PEER_ATU_FOUND: return "PEER ATU FOUND";
		case C_AMSW_RESTART_REQUEST: return "RESTART REQUES";
		case C_AMSW_ACTIVATION_REQUEST: return "ACTIVATION REQUEST";       
		case C_AMSW_TO_INITIALIZING: return "TO INITIALIZING";          
		case C_AMSW_SHOWTIME: return "AMSW SHOWTIME";                 
		case C_AMSW_L3_EXECUTED: return "L3 EXECUTED";              
		case C_AMSW_L3_REJECTED: return "L3 REJECTED";              
		case C_AMSW_L1_EXECUTED: return "L1 REJECTED";                 
		case C_AMSW_L1_REJECTED: return "L1 REJECTED";                
		case C_AMSW_L0_REJECTED: return "L0 REJECTED";                
		case C_AMSW_RESTART_ACCEPTABLE: return "RESTART ACCEPTABLE";         
		case C_AMSW_SUICIDE_REQUEST: return "SUICIDE REQUEST";            
		case C_AMSW_RESTART_NOT_ACCEPTABLE: return "RESTART NOT ACCEPTABLE";     
	}
	sprintf(s,"(%d)",event);
	return s;
}

static const char *get_msw_failure_string(AMSW_ModemFailure failure)
{ 
	static char s[8];

	switch(failure) {
		case C_AMSW_UNCOMPATIBLE_LINECONDITIONS: return "UNCOMPATIBLE LINECONDITION";
		case C_AMSW_NO_LOCK_POSSIBLE: return "NO LOCK POSSIBLE";
		case C_AMSW_PROTOCOL_ERROR: return "PROTOCOL ERROR";
		case C_AMSW_MESSAGE_ERROR: return "MESSAGE ERROR";
		case C_AMSW_SPURIOUS_ATU_DETECTED: return "SPURIOUS ATU DETECTED";
		case C_AMSW_DS_REQ_BITRATE_TOO_HIGH_FOR_LITE: return "DS REQ BITRATE TOO HIGH FOR LITE";
		case C_AMSW_INTERLEAVED_PROFILE_REQUIRED_FOR_LITE: return "INTERLEAVED PROFILE REQUIRED FOR LITE";
		case C_AMSW_FORCED_SILENCE: return "FORCED SILENCE";
		case C_AMSW_UNSELECTABLE_OPERATION_MODE: return "UNSELECTABLE OPERATION MODE";
		case C_AMSW_STATE_REFUSED_BY_GOLDEN: return "STATE REFUSED BY GOLDEN";
	}
	sprintf(s,"(%d)",failure);
	return s;
}

static const char *get_msw_state_string(AMSW_ModemState state)
{
	static char s[8];

	switch(state) {
		case C_AMSW_IDLE: return "IDLE";
		case C_AMSW_L3: return "L3";
		case C_AMSW_LISTENING: return "LISTENING";
		case C_AMSW_ACTIVATING: return "ACTIVATING";
		case C_AMSW_Ghs_HANDSHAKING: return "Ghs HANDSHAKING";
		case C_AMSW_ANSI_HANDSHAKING: return "ANSI HANDSHAKING";
		case C_AMSW_INITIALIZING: return "INITIALIZING";
		case C_AMSW_RESTARTING: return "RESTARTING";
		case C_AMSW_FAST_RETRAIN: return "FAST RETRAIN";
		case C_AMSW_SHOWTIME_L0: return "SHOWTIME L0";
		case C_AMSW_SHOWTIME_LQ: return "SHOWTIME LQ";
		case C_AMSW_SHOWTIME_L1: return "SHOWTIME L1";
		case C_AMSW_EXCHANGE: return "EXCHANGE";
		case C_AMSW_TRUNCATE: return "TRUNCATE";
		case C_AMSW_ESCAPE: return "ESCAPE";
		case C_AMSW_DISORDERLY: return "DISORDERLY";
		case C_AMSW_RETRY: return "RETRY";
	}
	sprintf(s,"(%d)",state);
	return s;
}

void msw_report_event(DWORD type,DWORD code)
{
	DBG(RAPI_D,"type=%ld,code=%ld\n",type,code);

	last_report = (type << 16) | code;
	switch (type) {
		case MSW_EVENT_NONE:
			break;
		case MSW_EVENT_REPORT:
			PRINT_INFO("MSW event: %s\n",get_msw_event_string(code));
			last_event = code;

			if(code == 4)
			{
				T_AMSW_NT_ChannelOperData m_cod[4];

				AMSW_ANT_getData(C_AMSW_NEAR_END_CHANNEL_DATA_FAST,&m_cod[0]);
				AMSW_ANT_getData(C_AMSW_FAR_END_CHANNEL_DATA_FAST,&m_cod[1]);
				AMSW_ANT_getData(C_AMSW_NEAR_END_CHANNEL_DATA_INTERLEAVED,&m_cod[2]);
				AMSW_ANT_getData(C_AMSW_FAR_END_CHANNEL_DATA_INTERLEAVED,&m_cod[3]);	 	

				if(m_cod[0].actualBitrate == 0)
				{
					setAtmRate((m_cod[3].actualBitrate*1000)/(ATM_CELL_LENGTH*8),(m_cod[2].actualBitrate*1000)/(ATM_CELL_LENGTH*8));

				}
				else
				{				
					setAtmRate((m_cod[1].actualBitrate*1000)/(ATM_CELL_LENGTH*8),(m_cod[0].actualBitrate*1000)/(ATM_CELL_LENGTH*8));

				}
				setShowtime();

			}

			break;
		case MSW_EVENT_FAILURE:
			PRINT_INFO("MSW failure: %s\n",get_msw_failure_string(code));
			last_failure = code;
			break;
		case MSW_EVENT_STATE:
			PRINT_INFO("MSW state: %s\n",get_msw_state_string(code));
			break;
		case MSW_EVENT_CANCEL:
			break;
		case AMU_EVENT_ACT_TIMEOUT:
			last_failure = C_AMSW_AMU_EVENT_ACT_TIMEOUT;
			break;
		case AMU_EVENT_INI_TIMEOUT:
			last_failure = C_AMSW_AMU_EVENT_INI_TIMEOUT;
			break;
		case AMU_EVENT_SHUTDOWN:
			last_failure = C_AMSW_AMU_EVENT_SHUTDOWN;
			break;
		case AMU_EVENT_RETRY:
			last_failure = C_AMSW_EVENT_RETRY;
			break;
		default:
			PRINT_INFO("Unknown msw_event\n");
	}
}

//----------------------------------------------------------------------
// Set the showtime FLAG
//----------------------------------------------------------------------
void setShowtime(void)
{
	int NewSetting = -1;

	struct unicorn_dev *dev = &unicorn_usb_dev;

	switch(adsl_status)
	{
		case ADSL_STATUS_NOHARDWARE:
		case ADSL_STATUS_ATMREADY:
			break;
		case ADSL_STATUS_NOLINK:
			adsl_system_time = xtm_gettime();
			adsl_status = ADSL_STATUS_ATMREADY;
			dev->recv_atm.cell_count = 0;
			dev->send_atm.cell_count = 0;

			if (AlternativeSetting == 0) {
				NewSetting = -1;
			}
			else
			{
				NewSetting = AlternativeSetting;
			}

			adjust_bandwidth(dev,adsl_us_cellrate,adsl_ds_cellrate, NewSetting);
			atm_start_snd(dev);
			atm_start_rcv(dev);		
			break;
	}

	if(dev->alternate_setting == 2)
	{
		SetUsbTimer(0);
		USB_controlWrite(UR_SHRT_PKT, 0x2);		
	}
}

static void SetUsbTimer(BOOL bStart)
{
	if(bStart)
	{
		USB_controlWrite(UR_TIMEOUT_TIMER, 0x4003);
	}
	else
	{
		USB_controlWrite(UR_TIMEOUT_TIMER,0);
	}
}
//----------------------------------------------------------------------
// Reset the showtime FLAG
//----------------------------------------------------------------------
void resetShowtime(void)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;

	switch(adsl_status)
	{
		case ADSL_STATUS_NOHARDWARE:
		case ADSL_STATUS_NOLINK:
			break;
		case ADSL_STATUS_ATMREADY:
			adsl_status = ADSL_STATUS_NOLINK;
			adsl_us_cellrate = 0;
			adsl_ds_cellrate = 0;
			atm_stop_snd(dev);
			atm_stop_rcv(dev);
			if (!GlobalRemove) {
				adjust_bandwidth(dev, 0, 0, ALT_SETTING_INIT);

			}
			break;
	}
}

void HandleAtmError(void)
{
	if (gAtmUsbError) {
		gAtmUsbError = 0;
		DBG(ATM_D,"Reset ATM FIFO\n");
		USB_controlWrite(UR_CFW,0xA2);
		USB_controlWrite(UR_CFW,0x22);
	}
}


// --------------------------------
// Led monitoring (FHLP 10/30/2001)
// --------------------------------
void HandleLeds(void)
{
	// called periodic frm AMUTask
	static WORD Leds = 0;
	static WORD LastLeds = 0;
	static int LedDelay = 0;

	// Led monitoring 
	// --------------------------------
	switch (ledScenario) 
	{
		case 1:    //Antwerp LED scenario
			{
				switch(g_ModemState)
				{
					case C_AMSW_ACTIVATING:
					case C_AMSW_INITIALIZING:
					case C_AMSW_Ghs_HANDSHAKING:
					case C_AMSW_ANSI_HANDSHAKING:
						{
							if (++LedDelay == 2)
							{
								LedDelay = 0;
								Leds = LED_POWER + LED_INIT;
							}
							else
							{
								Leds = LED_POWER;
							}
							break;
						}

					case C_AMSW_SHOWTIME_L0:
					case C_AMSW_SHOWTIME_LQ:
					case C_AMSW_SHOWTIME_L1:
						{
							Leds = LED_POWER + LED_INIT;
							if (gLedXmit || gLedRecv)
							{
								gLedXmit = FALSE;
								gLedRecv = FALSE;
								Leds += (LastLeds & LED_SHOWTIME) ^ LED_SHOWTIME;
							}
							break;
						}

					default:
						{
							LedDelay = 0;
							Leds = LED_POWER;
							break;
						}
				}
				break;
			}
		case 2:    //Ghent LED scenario
			{
				switch(g_ModemState)
				{
					case C_AMSW_ACTIVATING:
					case C_AMSW_INITIALIZING:
					case C_AMSW_Ghs_HANDSHAKING:
					case C_AMSW_ANSI_HANDSHAKING:
						{
							if (++LedDelay == 2)
							{
								LedDelay = 0;
								Leds = LED_POWER + LED_INIT;
							}
							else
							{
								Leds = LED_POWER;
							}
							break;
						}

					case C_AMSW_SHOWTIME_L0:
					case C_AMSW_SHOWTIME_LQ:
					case C_AMSW_SHOWTIME_L1:
						{
							Leds = LED_INIT;
							if (gLedXmit || gLedRecv)
							{
								gLedXmit = FALSE;
								gLedRecv = FALSE;
								Leds += (LastLeds & LED_POWER) ^ LED_POWER;
							}
							else 
							{
								Leds += LED_POWER;
							}
							break;
						}

					default:
						{
							LedDelay = 0;
							Leds = LED_POWER;
							break;
						}
				}
				break;
			}
		case 0:    //Brussels LED scenario
		default:
			{
				switch(g_ModemState)
				{
					case C_AMSW_ACTIVATING:
					case C_AMSW_INITIALIZING:
					case C_AMSW_Ghs_HANDSHAKING:
					case C_AMSW_ANSI_HANDSHAKING:
						{
							if (++LedDelay == 2)
							{
								LedDelay = 0;
								Leds = LED_POWER + LED_INIT;
							}
							else
							{
								Leds = LED_POWER;
							}
							break;
						}

					case C_AMSW_SHOWTIME_L0:
					case C_AMSW_SHOWTIME_LQ:
					case C_AMSW_SHOWTIME_L1:
						{
							Leds = LED_POWER;
							if (gLedXmit || gLedRecv)
							{
								gLedXmit = FALSE;
								gLedRecv = FALSE;
								Leds += (LastLeds & LED_SHOWTIME) ^ LED_SHOWTIME;
							}
							else 
							{
								Leds += LED_SHOWTIME;
							}
							break;
						}

					default:
						{
							LedDelay = 0;
							Leds = LED_POWER + LED_INIT;
							break;
						}
				}
				break;
			}
	}

	if (Leds != LastLeds)
	{
		USB_controlWrite(UR_GPIO_DATA,Leds);
	}

	LastLeds = Leds;
}

//----------------------------------------------------------------------
// reports ATM cell rates
//----------------------------------------------------------------------
void setAtmRate(
		unsigned short upRate,
		unsigned short downRate
	       )
{

	adsl_us_cellrate= upRate;
	adsl_ds_cellrate= downRate;
}

//----------------------------------------------------------------------
//	Copy the TOSCA hardware interrupt table applying a OR to the result
//	This function is called from the hardware ISR
//----------------------------------------------------------------------
static void CopyHardIntrTable(struct unicorn_dev *dev,WORD *IntBuf)
{
	int i;

	for (i=0; i<14; i++) {
		tosca_hardITABLE[i] |= cpu_to_le16(IntBuf[i+3]);
	}
}

//----------------------------------------------------------------------
//	OBC command interrupt received
//----------------------------------------------------------------------
static void ObcCmdCompletion(struct unicorn_dev *dev)
{	
	DBG(INTR_D,"obc_flags=%02lx\n",dev->obc_flags);

	if ((dev->obc_flags == (OBC_CMD_INT | OBC_CMD_INT_LO | OBC_WRITE_CPLT)) || 
			(dev->obc_flags == (OBC_CMD_INT | OBC_CMD_INT_LO | OBC_WRITE_CPLT | OBC_READ_CMD | OBC_READ_CPLT))) {
		dev->obc_flags = 0;
		xsm_v(dev->obc_sem);
	}

}

//----------------------------------------------------------------------
//	Interrupt In pipe completion routine
//----------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void IntInComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void IntInComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;
	WORD *IntBuf = urb->transfer_buffer;
	WORD isdr = cpu_to_le16(IntBuf[0]);
	WORD mask;
	WORD isr;

	DBG(INTR_D,"status=%d,actual_length=%d,ISDR=%04x\n",urb->status,urb->actual_length,isdr);		

	if (urb->status != 0) {
		if (urb->status == USB_ST_DATAUNDERRUN) {
			// retry
			DBG(USB_D,"retry,status=%d\n",urb->status);
			goto retry;
		} 
		else {
			if (urb->status == (-ENOENT)) {
				//urb unlinked
				PRINT_INFO("URB was unlinked before... just return\n");
				return;
			}
			else {
				// fatal error
				DBG(USB_D,"fatal error,status=%d\n",urb->status);
				//			GlobalRemove = TRUE;
				//			dev->started = FALSE;
				urb->dev = dev->usb_dev;
				urb->interval = 0;
				//UNLINK_URB(urb);
				return;
			}
		}
	}

	if ((urb->actual_length != 2) && (urb->actual_length != 6) && (urb->actual_length != 34)) {
		DBG(INTR_D,"actual_length=%d\n",urb->actual_length);
		goto retry;
	}

	// ACTD interrupt
	if (isdr & UISDR_ACTDIF) {
		DBG(INTR_D,"ACTD interrupt\n");
	}

	// INT_LO interrupt
	if (isdr & UISDR_INT_LO) {
		DBG(INTR_D,"INT_LO interrupt\n");
		dev->obc_flags |= OBC_CMD_INT_LO;
		ObcCmdCompletion(dev);
	}

	// Utopia FIFO interrupts
	if (isdr & UISDR_UTIRQ1) {
		DBG(INTR_D,"Utopia rising edge FIFO interrupt\n");
	}
	if (isdr & UISDR_UTIRQ2) {
		DBG(INTR_D,("Utopia falling edge FIFO interrupt\n"));
	}

	// Error interrupts
	if (isdr & UISDR_ERF) {
		if (isdr & UISDR_ERR_ATM) {
			DBG(INTR_D,"ATM Operation Error interrupt\n");
			gAtmUsbError = 1;
		}
		if (isdr & UISDR_ERR_OBC) {
			DBG(INTR_D,"WR OBC Operation Error interrupt\n");
		}
		if (isdr & UISDR_ERR_PIPE) {
			DBG(INTR_D,"ERROR: WR OBC Access Error interrupt\n");
		}
	}

	// ADSL interrupts
	mask = UISDR_TIRQ1 | UISDR_TIRQ2;
	if (isdr & mask) {

		isr = cpu_to_le16(IntBuf[1]);
		DBG(INTR_D,"ADSL uP Interrupt,ISR = %04x\n",isr);

		// TOSCA macrocell interrupt
		mask = ISR_TOIFS;
		if (isr & mask) {
			if (urb->actual_length >= 34) {
				DBG(INTR_D,("TOSCA macrocell interrupt\n"));
				CopyHardIntrTable(dev,IntBuf);
				tosca_interrupt();
			} else {
				DBG(INTR_D,"TOSCA macrocell interrupt, too short, actual_length=%d\n",urb->actual_length);
			}
		}

		// Timer interrupt
		mask = ISR_TIMIF;
		if (isr & mask) {
			DBG(INTR_D,"Timer interrupt\n");
		}

		// GPIO interrupt
		mask = ISR_GPIFA | ISR_GPIFB;
		if (isr & mask) {
			DBG(INTR_D,"GPIO interrupt\n");
		}

		// OBC Slave Command Complete interrupt
		mask = ISR_OSIF;
		if (isr & mask){
			DBG(INTR_D,"OBC Slave Command Complete interrupt\n");
			dev->obc_flags |= (OBC_CMD_INT | OBC_CMD_INT_LO);
			ObcCmdCompletion(dev);
		}

		// OBC Master Command Complete interrupt
		mask = ISR_OMIF;
		if (isr & mask) {
			DBG(INTR_D,"OBC Master Command Complete interrupt\n");
			dev->obc_flags |= OBC_CMD_INT;
			ObcCmdCompletion(dev);
		}
	}
retry:	
	// Prepare URB for next transfer
	urb->dev = dev->usb_dev;
	urb->status = 0;
	urb->actual_length = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
	SUBMIT_URB(urb); 
#endif
	return;	
}

//=====================================================================
//	FUNCTIONS TO ACCESS THE HARDWARE
//======================================================================


//----------------------------------------------------------------------
//	Starts the hardware
//----------------------------------------------------------------------
static ST_STATUS start_hardware(struct unicorn_dev *dev)
{
	DBG(1,"\n");

	// Initialize the USB Configuration register
	if (USB_controlWrite(UR_CFW,0x22) == FAILURE) {
		DBG(USB_D,"USB_controlWrite(UR_CFW) failed\n");
		return FAILURE;
	}

	// Initialize the UR_IADR_IRQ register
	if (USB_controlWrite(UR_IADR_IRQ,0x100) == FAILURE) {
		DBG(USB_D,"USB_controlWrite(UR_IADR_IRQ) failed\n");
		return FAILURE;
	}

	// Initialize the ADSL uP Interrupt register
	if (USB_controlWrite(UR_ISR,
				ISR_OSIE | ISR_OMIE | ISR_TOIES) == FAILURE) {
		DBG(USB_D,"USB_controlWrite(UR_ISR) failed\n");
		return FAILURE;
	}

	// Initialize the USB Interrupt & Data register
	if (USB_controlWrite(UR_ISDR,
				UISDR_IE | UISDR_LOE | 
				UISDR_TIE1 | UISDR_TIE2) == FAILURE) {
		DBG(USB_D,"USB_controlWrite(UR_ISDR) failed\n");
		return FAILURE;
	}

	// LED initialization
	USB_controlWrite(UR_GPIO_DIR,0xFFF1);	// Activate GPIO for leds
	USB_controlWrite(UR_GPIO_DATA,0x0E00);		// Leds are off

	return SUCCESS;
}

//----------------------------------------------------------------------
//	Stops the hardware
//----------------------------------------------------------------------
static void stop_hardware(struct unicorn_dev *dev)
{
	DBG(1,"\n");

	USB_controlWrite(UR_ISDR,0);
	USB_controlWrite(UR_ISR,0);
	USB_controlWrite(UR_CFW,0x1800);
	USB_controlWrite(UR_GPIO_DATA,0x0E00);		// Leds are off
}

//----------------------------------------------------------------------
//	Sets the MSW parameters to correct values
//----------------------------------------------------------------------
long set_msw_params(void)
{
	_boostPowerGdmt_ = 1;
	_gi_step_ = 1;
	_teq_new_delay_ = 1;
	decreaseHighCarrier = 12;
	g_RefGain = 20;
	highCarrierOff = 230;
	Interoperability = 1;
	useRFC019v = 0;
	useRFC029v = 8000;
	useRFC040v = 0;
	useRFCFixedRate = 1;
	useVCXO = 0;
	pilotRealloc = 1;
	_newCToneDetection_ = 1;
	TrainingDelay = 120;
	g_TeqMode = 7;
	txPower = 14;

	PRINT_INFO("Driver debug level = %u \n Msw Debug level = %u \n",(int) DebugLevel,(int) MswDebugLevel);

	if ((ActivationMode > MSW_MODE_UNKNOWN) && (ActivationMode < MSW_MODE_MAX)) {
		if((ActivationMode >= MSW_MODE_ANSI) && (ActivationMode <= MSW_MODE_GDMT)) {
			subfunction = 0xA1;
		}
		else if ((ActivationMode >= MSW_MODE_ETSI_ISDN) && (ActivationMode <=  MSW_MODE_MULTI_ISDN )) {
			subfunction = 0xB0;
		}
		else if ((ActivationMode = MSW_MODE_GDMT_BDT)) {
			subfunction = 0xD0;
		}
	}
	else subfunction=0xA1;	//If no ActivationMode is given, Activate in Azuma multimode

	switch (subfunction) {
		case 0x17:
		case 0xA0:
			pvo_noboost2db = 0;
			azuma_boost2db = 0;
			pvo_pembdpllpolarity = 1;
			pvo_noadse = 1;
			globSubFunctNoAdsf = 1;
			globSubFunctNoAdsg = 0;
			pvo_noisdn = 1;
			DBG(USB_D,"Subfunction 0xA0\n");
			break;
		case 0xA1:
			pvo_noboost2db = 0;
			azuma_boost2db = 1;
			pvo_pembdpllpolarity = 1;
			pvo_noadse = 1;
			globSubFunctNoAdsf = 1;
			globSubFunctNoAdsg = 0;
			DBG(USB_D,"Subfunction 0xA1\n");
			break;

		case 0x90 :
		case 0xB0 :
			pvo_noboost2db = 1;
			azuma_boost2db = 0;
			pvo_pembdpllpolarity = 1;
			pvo_noadse =1;
			globSubFunctNoAdsf = 1;
			globSubFunctNoAdsg = 0;
			pvo_noisdn = 0;
			isdn_up_to_tone_53 = 0;
			DBG(USB_D,"Subfunction 0xB0\n");
			break;

		case 0xD0 :
			pvo_noboost2db = 1;
			azuma_boost2db = 0;
			pvo_noadse = 1;
			pvo_pembdpllpolarity = 1;
			pvo_noadse = 1;
			globSubFunctNoAdsf = 1;
			globSubFunctNoAdsg = 0;
			pvo_noisdn = 0;
			isdn_up_to_tone_53 = 1;
			DBG(USB_D,"Subfunction 0xD0\n");
			break;
		default :
			pvo_noboost2db = 1;
			pvo_pembdpllpolarity = 1;
			pvo_noadse = 1;
			pvo_noisdn = 1;
			DBG(USB_D,"Subfunction not valid\n");
			break;
	};
	return subfunction;
}

//----------------------------------------------------------------------
//	Start the USB device
//----------------------------------------------------------------------
static int start_device(struct unicorn_dev *dev,struct usb_device *usb_dev)
{
	int status;
	struct urb *urb;
	int i;
	int x;

	struct recv_atm *recv_atm = &dev->recv_atm;

	dev->usb_dev = usb_dev;

	CreateObject();
	createSemaphore();

	// Select alternate setting to match speed/bandwidth used
	dev->alternate_setting = ALT_SETTING_INIT;

	if ((status = usb_set_interface (usb_dev, 0, dev->alternate_setting)) < 0) {
		if (status == USB_ST_BANDWIDTH_ERROR) {
			DBG(USB_D,"insufficient USB bandwidth\n");
			return status;
		} else {
			DBG(USB_D,"usb_set_interface (alt %d) failed,status=%d\n",
					dev->alternate_setting,status);
			return status;
		}
	}
	//============================
	// Initialize the USB pipes...
	//============================

	init_usb_urb(dev);
	// Allocate memory
	if ((dev->usb_mem = kmalloc(sizeof(USB_MEMORY),GFP_DMA)) == NULL) {
		DBG(USB_D,"kmalloc failed\n");
		return -1;
	}

	for (x = 0; x < RECV_ATM_MAX_CELLS; x++)
	{
		recv_atm->cells[x] = kmalloc(ATM_CELL_LENGTH,GFP_DMA);
		recv_atm->used[x] = FALSE;
	}

	// Initialize OBC objects
	init_MUTEX(&dev->obc_lock);
	if ((xsm_create("OBC ",0,0,&dev->obc_sem)) != SUCCESS) {
		return -1;
	}

	for (i=0; i < 1; i++) {
		urb = dev->int_in_pipe[i];
		// Fill the interrupt URB ...
		FILL_INT_URB(urb, usb_dev,
				usb_rcvintpipe(usb_dev, EP_INTERRUPT),
				dev->usb_mem->IntBuf[i],sizeof(dev->usb_mem->IntBuf[i]),
				IntInComplete,dev,1);

		// ... and start it
		SUBMIT_URB(urb);
	}

	// Start the Hardware Device
	start_hardware(dev);

	dev->started = TRUE;
	adsl_status = ADSL_STATUS_NOLINK;


	// initialize interrupt tables
	// -------------------------
	for (i = 0; i < 14; i++)
	{
		tosca_hardITABLE[i] = 0;
	}
	for (i = 0; i < 28; i++)
	{
		tosca_softITABLE[i] = 0;
	}


	return 0;
}

static int init_usb_urb(struct unicorn_dev *dev)
{
	struct urb *urb;
	int i;

	// EP_INTERRUPT
	for (i=0; i < 2; i++) {
		if ((urb = ALLOC_URB(0)) == NULL) {
			DBG(USB_D,"usb_alloc_urb failed\n");
			return -ENOMEM;
		}
		dev->int_in_pipe[i] = urb;
	}

	// EP_OBC_OUT
	if (dev->alternate_setting != 5) {
		//ISO
		if ((urb = ALLOC_URB(WRITE_ISO_PACKETS_PER_URB)) == NULL) {
			DBG(USB_D,"usb_alloc_urbfailed\n");
			return -ENOMEM;
		}
	}
	else {
		//BULK
		if ((urb = ALLOC_URB(0)) == NULL) {
			DBG(USB_D,"usb_alloc_urbfailed\n");
			return -ENOMEM;
		}
	}
	dev->obc_iso_out = urb; 


	// EP_OBC_IN
	if (dev->alternate_setting != 5) {
		//ISO
		if ((urb = ALLOC_URB(READ_ISO_PACKETS_PER_URB)) == NULL) {
			DBG(USB_D,"usb_alloc_urb failed\n");
			return -ENOMEM;
		}
	}
	else {
		//BULK
		if ((urb = ALLOC_URB(0)) == NULL) {
			DBG(USB_D,"usb_alloc_urb failed\n");
			return -ENOMEM;
		}
	}
	dev->obc_iso_in = urb; 

	// EP_ATM_OUT
	if ((dev->alternate_setting != 2) && (dev->alternate_setting != 5)) {
		//ISO
		for (i=0; i < ATM_WRITES_ISO; i++) {
			if ((urb = ALLOC_URB(WRITE_ISO_PACKETS_PER_URB)) == NULL) {
				DBG(USB_D,"usb_alloc_urb failed\n");
				return -ENOMEM;
			}
			dev->atm_write[i] = urb; 
		}
	}
	else {
		//BULK
		for (i=0; i < ATM_WRITES_BULK; i++) {
			if ((urb = ALLOC_URB(0)) == NULL) {
				DBG(USB_D,"usb_alloc_urb failed\n");
				return -ENOMEM;
			}
			dev->atm_write[i] = urb; 
		}
	}

	// EP_ATM_IN
	if(dev->alternate_setting != 2){ 
		//ISO
		for (i=0; i < ATM_READS_ISO; i++) {
			if ((urb = ALLOC_URB(READ_ISO_PACKETS_PER_URB)) == NULL) {
				DBG(USB_D,"usb_alloc_urb failed\n");
				return -ENOMEM;
			}
			dev->atm_read[i] = urb; 
		}
	}
	else
	{
		//BULK
		for(i=0; i< ATM_READS_BULK; i++)
		{
			if((urb = ALLOC_URB(0))== NULL)
			{
				DBG(USB_D,"usb_alloc_urb failed\n");
				return -ENOMEM;
			}
			dev->atm_read[i]=urb;
		}
	}	

	// EP_OBC_INT_OUT
	if ((urb = ALLOC_URB(0)) == NULL) {
		DBG(USB_D,"usb_alloc_urb failed\n");
		return -ENOMEM;
	}
	dev->obc_int_out = urb; 

	// EP_OBC_INT_IN
	if ((urb = ALLOC_URB(0)) == NULL) {
		DBG(USB_D,"usb_alloc_urb failed\n");
		return -ENOMEM;
	}
	dev->obc_int_in = urb; 

	return 0;

}
//----------------------------------------------------------------------
//	Stop the USB device
//----------------------------------------------------------------------
static void stop_device(struct unicorn_dev *dev)
{
	struct recv_atm *recv_atm = &dev->recv_atm;
	int x;

	DBG(1,"\n");

	adsl_status = ADSL_STATUS_NOHARDWARE;


	if (!GlobalRemove) {
		stop_hardware(dev);
	}

	//============================
	// Terminate the USB pipes...
	//============================

	term_usb_urb(dev);

	if (dev->usb_mem) {
		kfree(dev->usb_mem);
		dev->usb_mem = NULL;
	}
	// Stop receiving and transmitting
	atm_stop_rcv(dev);
	atm_stop_snd(dev);

	for (x = 0; x < RECV_ATM_MAX_CELLS; x++) {
		kfree (recv_atm->cells[x]);
		recv_atm->cells[x] = NULL;
	}
}

//----------------------------------------------------------------------
//	Terminate the USB URBs
//----------------------------------------------------------------------
static int term_usb_urb(struct unicorn_dev *dev)
{
	struct urb *urb;
	int i;

	/*endpoint  1 */
	for (i=0; i < 2; i++) {
		if ((urb = dev->int_in_pipe[i])) {
//			UNLINK_URB(urb);
			usb_free_urb(urb);
			dev->int_in_pipe[i] = NULL;
		}
	}

	/*endpoint  2 */
	if ((urb = dev->obc_iso_out)) {
//		UNLINK_URB(urb);
		usb_free_urb(urb);
		dev->obc_iso_out = NULL;
	}

	/*endpoint  3 */
	if ((urb = dev->obc_iso_in)) {
//		UNLINK_URB(urb);
		usb_free_urb(urb);
		dev->obc_iso_in = NULL;
	}

	/* for alternate setting 1,3,4; unlinking iso urb on endpoint 4 */
	if ((dev->alternate_setting != 2) && (dev->alternate_setting != 5)) {
		for (i=0; i < ATM_WRITES_ISO; i++) {
			if ((urb = dev->atm_write[i])) {
//				UNLINK_URB(urb);
				usb_free_urb(urb);
				dev->atm_write[i] = NULL;
			}
		}
	}
	else {
		/* for alternate setting 2 and 5; unlinking bulk urb on endpoint 4 */
		for (i=0; i < ATM_WRITES_BULK; i++) {
			if ((urb = dev->atm_write[i])) {
//				UNLINK_URB(urb);
				usb_free_urb(urb);
				dev->atm_write[i] = NULL;
			}
		}
	}

	/* for alternate setting 1,3,4,5; unlinking iso urb on endpoint 5 */
	if (dev->alternate_setting != 2) {
		for (i=0; i < ATM_READS_ISO; i++) {
			if ((urb = dev->atm_read[i])) {
//				UNLINK_URB(urb);
				usb_free_urb(urb);
				dev->atm_read[i] = NULL;
			}
		}
	}
	else {
		/* for alternate setting 2; unlinking bulk urb on endpoint 5 */
		for (i=0; i < ATM_READS_BULK; i++) {
			if ((urb = dev->atm_read[i])) {
//				UNLINK_URB(urb);
				usb_free_urb(urb);
				dev->atm_read[i] = NULL;
			}
		}
	}

	/*endpoint  6 */
	if ((urb = dev->obc_int_out)) {
//		UNLINK_URB(urb);
		usb_free_urb(urb);
		dev->obc_int_out = NULL;
	}

	/*endpoint  7 */
	if ((urb = dev->obc_int_in)) {
//		UNLINK_URB(urb);
		usb_free_urb(urb);
		dev->obc_int_in = NULL;
	}

	return 0;
}

//----------------------------------------------------------------------
// CheckObcBuffer:
//----------------------------------------------------------------------
static BOOLEAN CheckObcBuffer(struct unicorn_dev *dev,WORD *buf,UINT n)
{
	PBYTE p = (PBYTE)buf + n*sizeof(WORD);
	if (GlobalRemove) return FALSE;
	if (p < (PBYTE)(dev->usb_mem)) return FALSE;
	if (p > (PBYTE)(dev->usb_mem+offsetof(USB_MEMORY,AtmUsBuf))) return FALSE;
	return TRUE;
}

//----------------------------------------------------------------------
//	Waits for OBC command complete
//----------------------------------------------------------------------
static ST_STATUS WaitForObcCmdComplete(struct unicorn_dev *dev)
{
	// wait for semaphore to be free 
	if (xsm_p(dev->obc_sem, 0, OBC_CMD_TIMEOUT) != SUCCESS) {
		DBG(INTR_D,"wait for obc failed (timed out),obc_flags=%02lx\n",dev->obc_flags);
		dev->obc_flags = 0;
		return FAILURE;
	}
	return SUCCESS;
}

//-----------------------------------------------------------------------------
// ObcWriteIsocComplete:
//-----------------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void ObcWriteIsocComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void ObcWriteIsocComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;

	DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
			urb->status,urb->transfer_buffer_length,urb->actual_length);	


	if (urb->transfer_buffer_length == urb->actual_length) urb->status = 0;
	if (urb->status == 0) {
		dev->obc_flags |= OBC_WRITE_CPLT;
		ObcCmdCompletion(dev);
	} else {
		DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);
	}
}

//-----------------------------------------------------------------------------
// ObcWriteBulkComplete:
//-----------------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void ObcWriteBulkComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void ObcWriteBulkComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;
	if (urb->transfer_buffer_length == urb->actual_length) urb->status = 0;
	if (urb->status == 0) {
		dev->obc_flags |= OBC_WRITE_CPLT;
		ObcCmdCompletion(dev);
	} else {
		DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);
	}
}

//-----------------------------------------------------------------------------
// ObcWriteIntComplete:
//-----------------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void ObcWriteIntComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void ObcWriteIntComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;

	DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
			urb->status,urb->transfer_buffer_length,urb->actual_length);	

	if (urb->transfer_buffer_length == urb->actual_length) urb->status = 0;

	if (urb->status == 0) {
		dev->obc_flags |= OBC_WRITE_CPLT;
		ObcCmdCompletion(dev);
	} else {
		DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);	
	}
	urb->interval = 0;  // set this to 0 to avoid to be re-scheduled
}

//-----------------------------------------------------------------------------
// ObcReadIsocComplete:
//-----------------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void ObcReadIsocComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void ObcReadIsocComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;
	DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d,start_frame=%u\n",
			urb->status,urb->transfer_buffer_length,urb->actual_length,urb->start_frame);

	if (GlobalRemove) return;
	if ((urb->status == USB_ST_DATAUNDERRUN) && (urb->actual_length > 0)) urb->status = 0;
	if ((urb->status == 0) && (urb->actual_length == 0)) urb->status = USB_ST_DATAUNDERRUN;

	if (urb->status == 0) {
		dev->obc_flags |= OBC_READ_CPLT;
		ObcCmdCompletion(dev);
#if RETRY_UNDERRUN
	} else if (urb->status == USB_ST_DATAUNDERRUN) {
		DBG(INTR_D,"retry,transfer_buffer_length=%d,actual_length=%d\n",
				urb->transfer_buffer_length,urb->actual_length);	
		urb->dev = dev->usb_dev;
		if (FrameNumber) {
			urb->transfer_flags = 0;
			urb->start_frame = usb_get_current_frame_number(dev->usb_dev)+FrameNumber;
		} else {
			urb->transfer_flags = USB_ISO_ASAP;
		}
		SUBMIT_URB(urb);
#endif
	} else {
		DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);	
	}
}

//-----------------------------------------------------------------------------
// ObcReadBulkComplete:
//-----------------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void ObcReadBulkComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void ObcReadBulkComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;
	if (GlobalRemove) return;
	if ((urb->status == USB_ST_DATAUNDERRUN) && (urb->actual_length > 0)) urb->status = 0;
	if ((urb->status == 0) && (urb->actual_length == 0)) urb->status = USB_ST_DATAUNDERRUN;

	if (urb->status == 0) {
		dev->obc_flags |= OBC_READ_CPLT;
		ObcCmdCompletion(dev);
	} else {
		DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);
	}
}

//-----------------------------------------------------------------------------
// ObcReadIntComplete:
//-----------------------------------------------------------------------------
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static void ObcReadIntComplete(struct urb *urb,struct pt_regs *pt_regs)
#else
static void ObcReadIntComplete(struct urb *urb)
#endif
{
	struct unicorn_dev *dev = (struct unicorn_dev *)urb->context;

	DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
			urb->status,urb->transfer_buffer_length,urb->actual_length);	

	if (GlobalRemove) return;
	if (urb->actual_length == urb->transfer_buffer_length) urb->status=0;

	if (urb->status == 0) {
		dev->obc_flags |= OBC_READ_CPLT;
		ObcCmdCompletion(dev);
	} else {
		DBG(INTR_D,"status=%d,transfer_buffer_length=%d,actual_length=%d\n",
				urb->status,urb->transfer_buffer_length,urb->actual_length);	
	}

	urb->interval = 0;  // set this to 0 to avoid to be re-scheduled
}

//-----------------------------------------------------------------------------
// USB_init:
//-----------------------------------------------------------------------------
ST_STATUS USB_init(
		WORD **CMDptrW1,WORD **CMDptrW2,WORD **CMDptrRd,
		WORD **CMDptrW_I1,WORD **CMDptrW_I2,WORD **CMDptrRd_I,
		WORD **ITABLEptr,WORD **IMASKptr,T_EpSettings *ep_setting
		)
{	
	struct unicorn_dev *dev = &unicorn_usb_dev;
	PBYTE p = (PBYTE)dev->usb_mem;

	DBG(1,"dev=%p\n",dev);

	DBG(USB_D,"dma_virtual_addr=%p\n",p);

	dev->dma_virtual_addr = p;
	*CMDptrW1   = (PWORD)(p + offsetof(USB_MEMORY,CmdBufW1));
	*CMDptrW2   = (PWORD)(p + offsetof(USB_MEMORY,CmdBufW2));
	*CMDptrRd   = (PWORD)(p + offsetof(USB_MEMORY,CmdBufRd));
	*CMDptrW_I1 = (PWORD)(p + offsetof(USB_MEMORY,CmdBufW_I1));
	*CMDptrW_I2 = (PWORD)(p + offsetof(USB_MEMORY,CmdBufW_I2));
	*CMDptrRd_I = (PWORD)(p + offsetof(USB_MEMORY,CmdBufRd_I));

	ep_setting->ep0_size = dev->usb_dev->descriptor.bMaxPacketSize0;
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
	ep_setting->ep1_size = dev->usb_dev->epmaxpacketin[EP_INTERRUPT];
	ep_setting->ep2_size = dev->usb_dev->epmaxpacketout[EP_OBC_ISO_OUT];
	ep_setting->ep3_size = dev->usb_dev->epmaxpacketin[EP_OBC_ISO_IN];
	ep_setting->ep4_size = dev->usb_dev->epmaxpacketout[EP_ATM_ISO_OUT];
	ep_setting->ep5_size = dev->usb_dev->epmaxpacketin[EP_ATM_ISO_IN];
	ep_setting->ep6_size = dev->usb_dev->epmaxpacketout[EP_OBC_INT_OUT];
	ep_setting->ep7_size = dev->usb_dev->epmaxpacketin[EP_OBC_INT_IN];
#else
	ep_setting->ep1_size = usb_maxpacket(dev->usb_dev, usb_rcvintpipe(dev->usb_dev,EP_INTERRUPT),0);
	ep_setting->ep2_size = usb_maxpacket(dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_OBC_ISO_OUT),1);
	ep_setting->ep3_size = usb_maxpacket(dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,EP_OBC_ISO_IN),0);
	ep_setting->ep4_size = usb_maxpacket(dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_ATM_ISO_OUT),1);
	ep_setting->ep5_size = usb_maxpacket(dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,EP_ATM_ISO_IN),0);
	ep_setting->ep6_size = usb_maxpacket(dev->usb_dev, usb_sndintpipe(dev->usb_dev,EP_OBC_INT_OUT),1);
	ep_setting->ep7_size = usb_maxpacket(dev->usb_dev, usb_rcvintpipe(dev->usb_dev,EP_OBC_INT_IN),0);


#endif
	DBG(USB_D,"max packet size,ep0 %d,ep1 %d,ep2 %d,ep3 %d,ep4 %d,ep5 %d,ep6 %d,ep7 %d\n",
			ep_setting->ep0_size,ep_setting->ep1_size,ep_setting->ep2_size,ep_setting->ep3_size,
			ep_setting->ep4_size,ep_setting->ep5_size,ep_setting->ep6_size,ep_setting->ep7_size);


	*ITABLEptr = tosca_softITABLE;
	*IMASKptr  = tosca_softITABLE + 14;

	return SUCCESS;
}

//-----------------------------------------------------------------------------
// USB_controlRead:
//	Read an USB register
//-----------------------------------------------------------------------------
ST_STATUS USB_controlRead(BYTE addr,WORD *data)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	int err;
	WORD Data[2];

	err = usb_control_msg(dev->usb_dev,usb_rcvctrlpipe(dev->usb_dev,0),
			addr,USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,0,Data,sizeof(WORD),HZ);	
	if (err < 0) {
		goto fail;
	}		
	le16_to_cpus(Data[0]);
	*data=Data[0];

	DBG(USB_D,"addr=%02x,data=%04x\n",addr,*data);

	return SUCCESS;

fail:
	DBG(USB_D,"err=%d\n",err);
	return FAILURE;
}

//-----------------------------------------------------------------------------
// USB_controlWrite:
//	Write to an USB register
//-----------------------------------------------------------------------------
ST_STATUS USB_controlWrite(BYTE addr,WORD data)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	int  err;


	DBG(USB_D,"addr=%02x,data=%04x\n",addr,data);
	err = usb_control_msg(dev->usb_dev,usb_sndctrlpipe(dev->usb_dev,0),
			addr,0x40,
			data,0,NULL,0,HZ);	
	if (err < 0) {
		goto fail;
	}	
	return SUCCESS;

fail:
	DBG(USB_D,"err=%d\n",err);
	return FAILURE;
}

//----------------------------------------------------------------------
//     ObcReset
//----------------------------------------------------------------------
static void ObcReset(void)
{
	// Force STOP ACCESS in status register and OBC FIFO clear pulse
	WORD data;
	if (GlobalRemove) return;

	DBG(INTR_D,"Resetting OBC...\n");
	if(USB_controlRead(UR_STATUS,&data) == SUCCESS) {
		USB_controlWrite(UR_STATUS,(data&~3)|0x200);
	}
}

#define MAX_RETRIES 2

//-----------------------------------------------------------------------------
// USB_S_Write:
//-----------------------------------------------------------------------------
ST_STATUS USB_S_Write(T_ShortWrite *dataPtr,T_EpOut ep_out)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	struct urb *urb;
	int retry;
	ST_STATUS status;
	int err;

	if (!CheckObcBuffer(dev,dataPtr->cmdBuff,dataPtr->frameSize)) return FAILURE;
	if ((ep_out != EP2) && (ep_out != EP6)) {
		DBG(USB_D,"Invalid ep_out parameter\n");
		return FAILURE;
	}

	OBC_LOCK(&dev->obc_lock);

	USB_SWAP_BUF(dataPtr->cmdBuff,dataPtr->frameSize);

	for (retry=0; retry < MAX_RETRIES; retry++) {

		DBG(USB_D,"cmdBuff=%p,frameSize=%d,ep_out=%d\n",dataPtr->cmdBuff,dataPtr->frameSize,ep_out);

		dev->obc_flags = 0x00;

		if (ep_out == EP2) {
			urb = dev->obc_iso_out;
			if(dev->alternate_setting != 5)
			{
				//ISO

#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
				fill_isoc_urb(urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_OBC_ISO_OUT),
						dataPtr->cmdBuff, dataPtr->frameSize*sizeof(WORD),
						dev->usb_dev->epmaxpacketout[EP_OBC_ISO_OUT], 
						ObcWriteIsocComplete, dev);
#else
				fill_isoc_urb(urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_OBC_ISO_OUT),
						dataPtr->cmdBuff, dataPtr->frameSize*sizeof(WORD),
						usb_maxpacket(dev->usb_dev,
							usb_sndisocpipe(dev->usb_dev,EP_OBC_ISO_OUT),1), 
						ObcWriteIsocComplete, dev);

#endif
			}
			else
			{
				//BULK
				FILL_BULK_URB(urb,dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, EP_OBC_ISO_OUT),dataPtr->cmdBuff,dataPtr->frameSize*sizeof(WORD),ObcWriteBulkComplete,dev);
			}
		} else {
			// INT
			urb = dev->obc_int_out;
			FILL_INT_URB(urb, dev->usb_dev, usb_sndintpipe(dev->usb_dev,EP_OBC_INT_OUT),
					dataPtr->cmdBuff, dataPtr->frameSize*sizeof(WORD), 
					ObcWriteIntComplete, dev, 1);
		}

		err = SUBMIT_URB(urb);
		if (err < 0) {
			status = FAILURE;
			goto fail;
		}
		status = WaitForObcCmdComplete(dev);
		if (status == FAILURE) {
			DBG(USB_D,"cmdBuff=%p,frameSize=%d,ep_out=%d\n",dataPtr->cmdBuff,dataPtr->frameSize,ep_out);

			DUMP_URB(urb);
			urb->dev = dev->usb_dev,
			urb->interval = 0,
//			UNLINK_URB(urb);
			ObcReset();
		} else {
			break;
		}		
	}

	USB_SWAP_BUF(dataPtr->cmdBuff,dataPtr->frameSize);
fail:
	OBC_UNLOCK(&dev->obc_lock);
	return status;
}

//-----------------------------------------------------------------------------
// USB_L_Write:
//-----------------------------------------------------------------------------
ST_STATUS USB_L_Write(T_LongWrite *dataPtr,T_EpOut ep_out)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	ULONG size = (dataPtr->nFrames-1)*dataPtr->frameSize + dataPtr->lastFrameSize;
	struct urb *urb;
	int retry;
	ST_STATUS status;
	int err;

	if (!CheckObcBuffer(dev,dataPtr->cmdBuff,size)) return FAILURE;

	if ((ep_out != EP2) && (ep_out != EP6)) {
		DBG(USB_D,"Invalid ep_out parameter\n");
		return FAILURE;
	}

	OBC_LOCK(&dev->obc_lock);

	USB_SWAP_BUF(dataPtr->cmdBuff,size);

	for (retry=0; retry < MAX_RETRIES; retry++) {

		DBG(USB_D,"cmdBuff=%p,nFrames=%d,frameSize=%d,lastFrameSize=%d,ep_out=%d\n",
				dataPtr->cmdBuff,dataPtr->nFrames,dataPtr->frameSize,dataPtr->lastFrameSize,ep_out);
		dev->obc_flags = 0x00;

		if (ep_out == EP2) {
			urb = dev->obc_iso_out;
			if(dev->alternate_setting  !=5)
			{
				//ISO
				fill_isoc_urb(urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,EP_OBC_ISO_OUT),
						dataPtr->cmdBuff, size*sizeof(WORD),
						dataPtr->frameSize*sizeof(WORD), 
						ObcWriteIsocComplete, dev);
			}
			else
			{
				//BULK
				FILL_BULK_URB(urb,dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, EP_OBC_ISO_OUT),dataPtr->cmdBuff,dataPtr->frameSize*sizeof(WORD),ObcWriteBulkComplete,dev);

			}

		} else {
			// INT
			urb = dev->obc_int_out;
			FILL_INT_URB(urb, dev->usb_dev, usb_sndintpipe(dev->usb_dev,EP_OBC_INT_OUT),
					dataPtr->cmdBuff, size*sizeof(WORD),
					ObcWriteIntComplete, dev, 1);
		}
		err = SUBMIT_URB(urb);
		if (err < 0) {
			status = FAILURE;
			goto fail;
		}
		status = WaitForObcCmdComplete(dev);
		if (status == FAILURE) {
			DBG(USB_D,"cmdBuff=%p,nFrames=%d,frameSize=%d,lastFrameSize=%d,ep_out=%d\n",
					dataPtr->cmdBuff,dataPtr->nFrames,dataPtr->frameSize,dataPtr->lastFrameSize,ep_out);
			DUMP_URB(urb);
			urb->dev = dev->usb_dev;
			urb->interval = 0;
//			UNLINK_URB(urb);
			ObcReset();
		} else {
			break;
		}		
	}

	USB_SWAP_BUF(dataPtr->cmdBuff,size);
fail:
	OBC_UNLOCK(&dev->obc_lock);
	return status;

}

//-----------------------------------------------------------------------------
// USB_S_Read:
//-----------------------------------------------------------------------------
ST_STATUS USB_S_Read(T_ShortRead *dataPtr,T_EpOut ep_out,T_EpIn ep_in)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	BOOLEAN isoIn;
	PWORD ptr;
	struct urb *read_urb,*write_urb;
	int retry;
	ST_STATUS status;
	int err;

	if (!CheckObcBuffer(dev,dataPtr->cmdBuff,dataPtr->wrSize)) return FAILURE;

	if ((ep_out != EP2) && (ep_out != EP6)) {
		DBG(USB_D,"Invalid ep_out parameter\n");
		return FAILURE;
	}

	if ((ep_in != EP3) && (ep_in != EP7)) {
		DBG(USB_D,"Invalid ep_in parameter\n");
		return FAILURE;
	}

	OBC_LOCK(&dev->obc_lock);

	USB_SWAP_BUF(dataPtr->cmdBuff,dataPtr->wrSize);


	for (retry=0; retry < MAX_RETRIES; retry++) {

		DBG(USB_D,"cmdBuff=%p,wrSize=%d,rdSize=%d,ep_out=%d,ep_in=%d\n",
				dataPtr->cmdBuff,dataPtr->wrSize,dataPtr->rdSize,ep_out,ep_in);

		dev->obc_flags = OBC_READ_CMD;
		ptr = dev->usb_mem->CmdBufRd;


		if (ep_in == EP3) {
			int ep = EP_OBC_ISO_IN;

			read_urb = dev->obc_iso_in;  

			if(dev->alternate_setting == 5){
				//BULK
				isoIn = FALSE;
				FILL_BULK_URB(read_urb, dev->usb_dev,usb_rcvbulkpipe(dev->usb_dev,ep),
						ptr, dataPtr->rdSize*sizeof(WORD),
						ObcReadBulkComplete, dev);
				err = SUBMIT_URB(read_urb);
				if (err < 0) {
					status = FAILURE;
					goto fail;
				}

			}
			else{
				//ISO
				isoIn = TRUE;	

#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
				fill_isoc_urb(read_urb, dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,ep),
						ptr, dataPtr->rdSize*sizeof(WORD),
						(dev->usb_dev->epmaxpacketin[ep]), ObcReadIsocComplete, dev);
#else
				fill_isoc_urb(read_urb, dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,ep),
						ptr, dataPtr->rdSize*sizeof(WORD),
						usb_maxpacket(dev->usb_dev,usb_rcvisocpipe(dev->usb_dev,ep),0),
						ObcReadIsocComplete, dev);

#endif
			}

		} else {     //EP7
			int ep =  EP_OBC_INT_IN;

			read_urb = dev->obc_int_in;

			//INT
			isoIn = FALSE;	
			FILL_INT_URB(read_urb, dev->usb_dev, usb_rcvintpipe(dev->usb_dev,ep),
					ptr, dataPtr->rdSize*sizeof(WORD),
					ObcReadIntComplete, dev, 1);
			err = SUBMIT_URB(read_urb);
			if (err < 0) {
				status = FAILURE;
				goto fail;
			}
		}

		if (ep_out == EP2) {
			int ep = EP_OBC_ISO_OUT;

			write_urb = dev->obc_iso_out;

			if(dev->alternate_setting  !=5){
				//ISO
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
				fill_isoc_urb(write_urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,ep),
						dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
						(dev->usb_dev->epmaxpacketout[ep]), ObcWriteIsocComplete, dev);
#else
				fill_isoc_urb(write_urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,ep),
						dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
						usb_maxpacket(dev->usb_dev,usb_sndisocpipe(dev->usb_dev,ep),1),
						ObcWriteIsocComplete, dev);

#endif
			}
			else
			{
				//BULK
				FILL_BULK_URB(write_urb, dev->usb_dev, usb_sndbulkpipe(dev->usb_dev,ep),
						dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
						ObcWriteBulkComplete,dev);
			}

		} else {
			int ep = EP_OBC_INT_OUT;

			write_urb = dev->obc_int_out;
			//INT
			FILL_INT_URB(write_urb, dev->usb_dev, usb_sndintpipe(dev->usb_dev,ep),
					dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
					ObcWriteIntComplete, dev, 1);

		}

		err = SUBMIT_URB(write_urb);
		if (err < 0) {
			status = FAILURE;
			goto fail;
		}

		if (isoIn) {
			if (FrameNumber) {
				read_urb->transfer_flags &= ~USB_ISO_ASAP;
				read_urb->start_frame = usb_get_current_frame_number(dev->usb_dev) + FrameNumber;
			} else {
				// ASAP
			}
			err = SUBMIT_URB(read_urb);
			if (err < 0) {
				status = FAILURE;
				goto fail;
			}
		}
		status = WaitForObcCmdComplete(dev);
		if (status == FAILURE) {
			DBG(USB_D,"cmdBuff=%p,wrSize=%d,rdSize=%d,ep_out=%d,ep_in=%d\n",
					dataPtr->cmdBuff,dataPtr->wrSize,dataPtr->rdSize,ep_out,ep_in);
			// Cancel
			DUMP_URB(write_urb);
			read_urb->dev = dev->usb_dev;
			read_urb->interval = 0;
//			UNLINK_URB(read_urb);
			write_urb->dev = dev->usb_dev;
			write_urb->interval = 0;
//			UNLINK_URB(write_urb);
			ObcReset();
		} else {
			dataPtr->rdSize = read_urb->actual_length/2;
			DBG(USB_D,"rdSize=%d\n",dataPtr->rdSize);
			USB_SWAP_BUF(ptr,dataPtr->rdSize);
			break;
		}	
	}

	USB_SWAP_BUF(dataPtr->cmdBuff,dataPtr->wrSize);
fail:
	OBC_UNLOCK(&dev->obc_lock);
	return status;
}

//-----------------------------------------------------------------------------
// USB_L_Read:
//-----------------------------------------------------------------------------
ST_STATUS USB_L_Read(T_LongRead *dataPtr,T_EpOut ep_out,T_EpIn ep_in)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	BOOLEAN isoIn;
	PWORD ptr;
	struct urb *read_urb,*write_urb;
	int retry;
	ST_STATUS status;
	int err;

	if (!CheckObcBuffer(dev,dataPtr->cmdBuff,dataPtr->wrSize)) return FAILURE;

	if ((ep_out != EP2) && (ep_out != EP6)) {
		DBG(USB_D,"Invalid ep_out parameter\n");
		return FAILURE;
	}

	if ((ep_in != EP3) && (ep_in != EP7)) {
		DBG(USB_D,"Invalid ep_in parameter\n");
		return FAILURE;
	}

	OBC_LOCK(&dev->obc_lock);

	USB_SWAP_BUF(dataPtr->cmdBuff,dataPtr->wrSize);

	for (retry=0; retry < MAX_RETRIES; retry++) {

		DBG(USB_D,"cmdBuff=%p,wrSize=%d,rdSize=%d,ep_out=%d,ep_in=%d\n",
				dataPtr->cmdBuff,dataPtr->wrSize,dataPtr->rdSize,ep_out,ep_in);

		dev->obc_flags = OBC_READ_CMD;

		ptr = dev->usb_mem->CmdBufRd;


		if (ep_in == EP3) {
			int ep = EP_OBC_ISO_IN;

			read_urb = dev->obc_iso_in;  
			if((dev->alternate_setting == 5))
			{
				//BULK
				isoIn = FALSE;
				FILL_BULK_URB(read_urb, dev->usb_dev,usb_rcvbulkpipe(dev->usb_dev,ep),
						ptr, dataPtr->rdSize*sizeof(WORD), ObcReadBulkComplete, dev);
				err = SUBMIT_URB(read_urb);
				if (err < 0) {
					status = FAILURE;
					goto fail;
				}

			}
			else{
				//ISO
				isoIn = TRUE;
#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
				fill_isoc_urb(read_urb, dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,ep),
						ptr, dataPtr->rdSize*sizeof(WORD), (dev->usb_dev->epmaxpacketin[ep]), 
						ObcReadIsocComplete, dev);
#else
				fill_isoc_urb(read_urb, dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,ep),
						ptr, dataPtr->rdSize*sizeof(WORD),
						usb_maxpacket(dev->usb_dev, usb_rcvisocpipe(dev->usb_dev,ep),0),
						ObcReadIsocComplete, dev);

#endif
			}
		} else {
			int ep = EP_OBC_INT_IN;

			read_urb = dev->obc_int_in;  
			//INT
			isoIn = FALSE;
			FILL_INT_URB(read_urb, dev->usb_dev, usb_rcvintpipe(dev->usb_dev,ep),
					ptr, dataPtr->rdSize*sizeof(WORD), ObcReadIntComplete, dev, 1);
			err = SUBMIT_URB(read_urb);
			if (err < 0) {
				status = FAILURE;
				goto fail;
			}
		}

		if (ep_out == EP2) {
			int ep = EP_OBC_ISO_OUT;

			write_urb = dev->obc_iso_out;
			if(dev->alternate_setting  !=5)
			{
				//ISO

#if  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10))
				fill_isoc_urb(write_urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,ep),
						dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
						(dev->usb_dev->epmaxpacketout[ep]), ObcWriteIsocComplete, dev);
#else
				fill_isoc_urb(write_urb, dev->usb_dev, usb_sndisocpipe(dev->usb_dev,ep),
						dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
						usb_maxpacket(dev->usb_dev, usb_sndisocpipe(dev->usb_dev,ep),1),
						ObcWriteIsocComplete, dev);

#endif
			}
			else
			{
				//BULK
				FILL_BULK_URB(write_urb, dev->usb_dev, usb_sndbulkpipe(dev->usb_dev,ep),
						dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
						ObcWriteBulkComplete,dev);
			}
		} else {
			int ep = EP_OBC_INT_OUT;
			write_urb = dev->obc_int_out;
			//INT
			FILL_INT_URB(write_urb, dev->usb_dev, usb_sndintpipe(dev->usb_dev,ep),
					dataPtr->cmdBuff, dataPtr->wrSize*sizeof(WORD),
					ObcWriteIntComplete, dev, 1);
		}
		err = SUBMIT_URB(write_urb);
		if (err < 0) {
			status = FAILURE;
			goto fail;
		}

		if (isoIn) {
			if (FrameNumber) {
				read_urb->transfer_flags &= ~USB_ISO_ASAP;
				read_urb->start_frame = usb_get_current_frame_number(dev->usb_dev)+FrameNumber;
			} else {
				// ASAP
			}
			err = SUBMIT_URB(read_urb);
			if (err < 0) {
				status = FAILURE;
				goto fail;
			}
		}

		status = WaitForObcCmdComplete(dev);
		if (status == FAILURE) {
			DBG(USB_D,"cmdBuff=%p,wrSize=%d,rdSize=%d,ep_out=%d,ep_in=%d\n",
					dataPtr->cmdBuff,dataPtr->wrSize,dataPtr->rdSize,ep_out,ep_in);
			// Cancel
			DUMP_URB(write_urb);
			read_urb->dev = dev->usb_dev;
			read_urb->interval = 0;
//			UNLINK_URB(read_urb);
			write_urb->dev = dev->usb_dev;
			write_urb->interval = 0;
//			UNLINK_URB(write_urb);
			ObcReset();
		} else {
			dataPtr->rdSize = read_urb->actual_length/2;
			DBG(USB_D,"rdSize=%d\n",dataPtr->rdSize);
			USB_SWAP_BUF(ptr,dataPtr->rdSize);
			break;
		}	
	}

	USB_SWAP_BUF(dataPtr->cmdBuff,dataPtr->wrSize);
fail:
	OBC_UNLOCK(&dev->obc_lock);
	return status;
}


//----------------------------------------------------------------------
// unicorn_snd_getcell:	
// Returns a pointer to the transmit buffer
//----------------------------------------------------------------------
unsigned char *unicorn_snd_getcell(struct unicorn_dev *dev)
{
	struct send_atm *send_atm = &dev->send_atm;
	int t;
	ULONG n;
	PBYTE p;

	if (!send_atm->started) {
		DBG(ATM_D,"ATM Tx I/O error\n");
		return NULL;
	}

	t = send_atm->turn_write;
	if (send_atm->lens[t] > 0)
	{
		PRINT_INFO("unicorn_snd_getcell:send_atm->lens[%d]=%d\n",t,send_atm->lens[t]);
		return NULL;
	}

	n = send_atm->maxlen - send_atm->bufofs;
	if (n < CELL_LENGTH)
	{
		PRINT_INFO("unicorn_snd_getcell:n <CELL_LENGTH :: n=%u\n",(unsigned int)n);
		return NULL;
	}

	p = send_atm->bufs[t] + send_atm->bufofs;

	// next cell
	send_atm->bufofs += CELL_LENGTH;
	gLedXmit = 1;
	send_atm->cell_count++;
	return p;	
}

//----------------------------------------------------------------------
// unicorn_start_transmit:	
// Start transmit DMA
//----------------------------------------------------------------------
int unicorn_start_transmit(struct unicorn_dev *dev)
{
	struct send_atm *send_atm = &dev->send_atm;
	int t;
	unsigned long n;

	DBG(RW_D,"\n");

	t = send_atm->turn_write;
	n = send_atm->bufofs;
	if ((dev->alternate_setting != 2) && (dev->alternate_setting != 5)) {
		if (n > 0) {
			send_atm->lens[t] = n;
			send_atm->turn_write = (t+1)&(ATM_WRITES_ISO-1);
			send_atm->bufofs = 0;
			test_and_set_bit(0,&send_atm->busy[t]);
			StartAtmUsXfer(dev,t,send_atm->bufs[t],n);
			return n;
		}
	}
	else {
		if (n > 0) {
			send_atm->lens[t] = n;
			send_atm->turn_write = (t+1)&(ATM_WRITES_BULK-1);
			send_atm->bufofs = 0;
			test_and_set_bit(0,&send_atm->busy[t]);
			StartAtmUsXfer(dev,t,send_atm->bufs[t],n);
			return n;
		}
	}
	return 0;	
}

//----------------------------------------------------------------------
// unicorn_rcv_getcell:	
// Returns a pointer to the next cell header in the receive buffer
//----------------------------------------------------------------------
unsigned char *unicorn_rcv_getcell(struct unicorn_dev *dev)
{
	struct recv_atm *recv_atm = &dev->recv_atm;
	unsigned char *cell;

	if (recv_atm->used[recv_atm->turn_read] == TRUE) {
		cell = recv_atm->cells[recv_atm->turn_read];
		DBG(RW_D,"turn_read=%d,cell=%p\n",recv_atm->turn_read,cell);
		// next cell
		recv_atm->used[recv_atm->turn_read++] = FALSE;

		if (recv_atm->turn_read >= RECV_ATM_MAX_CELLS) recv_atm->turn_read = 0;

		gLedRecv = 1;
		recv_atm->cell_count++;
		return cell;
	}
	return NULL;
}

//----------------------------------------------------------------------
//	Modem Software Control functions
//----------------------------------------------------------------------
int unicorn_msw_control(struct unicorn_dev *dev,T_MswCtrl *ctrl)
{
	static unsigned long old_report = 0L;
	ctrl->retcode = -1;

	DBG(RW_D,"code=%ld,subcode=%ld,length=%ld\n",ctrl->code,ctrl->subcode,ctrl->length);

	switch(ctrl->code) {
		case MSW_CTL_INIT:
			if (ctrl->subcode <= 0) break;
			if (ctrl->subcode  >= 16) break;
			ActivationMode = ctrl->subcode ;
			dev->msw_started = TRUE;
			rapi_lock();
			msw_init(ActivationMode);
			rapi_unlock();
			ctrl->retcode = 0;
			ctrl->length = 0;
			break;

		case MSW_CTL_EXIT:
			rapi_lock();
			msw_exit();
			rapi_unlock();
			dev->msw_started = FALSE;
			ctrl->retcode = 0;
			ctrl->length = 0;
			break;

		case MSW_CTL_START:
			rapi_lock();
			msw_start();
			rapi_unlock();
			ctrl->retcode = 0;
			ctrl->length = 0;
			break;

		case MSW_CTL_STOP:
			rapi_lock();
			msw_stop();
			rapi_unlock();
			ctrl->retcode = 0;
			ctrl->length = 0;
			break;

		case MSW_CTL_SET_CONFIG:
			if (ctrl->buffer == NULL) break;
			rapi_lock();
			ctrl->retcode = AMSW_ANT_setModemConfiguration(ctrl->subcode,ctrl->buffer);
			rapi_unlock();
			ctrl->length = 0;
			break;

		case MSW_CTL_GET_CONFIG:
			if (ctrl->buffer == NULL) break;
			rapi_lock();
			ctrl->retcode = AMSW_ANT_getModemConfiguration(ctrl->subcode,ctrl->buffer);
			rapi_unlock();
			break;

		case MSW_CTL_GET_DATA:
			if (ctrl->buffer == NULL) break;
			rapi_lock();
			ctrl->retcode = AMSW_ANT_getData(ctrl->subcode,ctrl->buffer);
			rapi_unlock();
			break;

		case MSW_CTL_GET_STATE:
			if (ctrl->length < sizeof(AMSW_ModemState)) break;
			if (ctrl->buffer == NULL) break;
			rapi_lock();
			ctrl->retcode = AMSW_ANT_getModemState((AMSW_ModemState*)ctrl->buffer);
			rapi_unlock();
			break;

		case MSW_CTL_REQ_CHANGE:
			rapi_lock();
			ctrl->retcode = AMSW_ANT_requestModemStateChange((AMSW_ModemState)ctrl->subcode);
			rapi_unlock();
			ctrl->length = 0;
			break;

		case MSW_CTL_DYING_GASP:
			rapi_lock();
			ctrl->retcode = AMSW_ANT_dyingGasp();
			rapi_unlock();
			ctrl->length = 0;
			break;
		case MSW_CTL_WAIT_EVENT:
			if (ctrl->length < sizeof(unsigned long)) break;
			if (ctrl->buffer == NULL) break;
			if (last_report != old_report) {		
				rapi_lock();
				//			ctrl->retcode = AMSW_ANT_wait_event((unsigned long*)ctrl->buffer);
				rapi_unlock();
				old_report = last_report;
			} else {
				ctrl->buffer = 0;
				//ctrl->length = 0;
			}
			break;
		case MSW_CTL_DVERSION_INFO:
			if (ctrl->length < sizeof(T_VersionDriver)) break;
			if (ctrl->buffer == NULL) break;
			{
				T_VersionDriver *version= (T_VersionDriver *)ctrl->buffer;
				strcpy(version->versionD,DRIVER_VERSION);
			}	
			ctrl->retcode = 0;
			break;
		case MSW_CTL_PVERSION_INFO:
			if (ctrl->length < sizeof(T_VersionPackage)) break;
			if (ctrl->buffer == NULL) break;
			{
				T_VersionPackage *version= (T_VersionPackage *)ctrl->buffer;
				strcpy(version->versionP, ""PKG_VERSION);
			}	
			ctrl->retcode = 0;
			break;
		case MSW_CTL_STATE_INFO:
			if (ctrl->length < sizeof(T_StateInfo)) break;
			if (ctrl->buffer == NULL) break;
			{
				static BOOLEAN last_info_temp=TRUE;//babu
				T_StateInfo *info = (T_StateInfo *)ctrl->buffer;
				info->State = g_ModemState;
				ctrl->retcode = 0;
				if(last_info_temp){
					if (info->State == C_AMSW_IDLE) {
						last_event = (AMSW_ModemEvent)-1;
						last_failure = (AMSW_ModemFailure)-1;
						last_info_temp=FALSE;
					}
				}else{
					if (info->State == C_AMSW_IDLE) {
						last_event = (AMSW_ModemEvent)-1;
					}
				}
				if (info->State == C_AMSW_SHOWTIME_L0) {
					last_failure = (AMSW_ModemFailure)-1;
					info->TimeCnctd = xtm_elapse(adsl_system_time);
				} else {
					info->TimeCnctd = 0;
				}
				info->Report = last_event;
				info->Failure = last_failure;
			}
			break;
		case MSW_CTL_SET_DEBUG_LEVEL:
			DebugLevel = ctrl->subcode;
			ctrl->retcode = 0;
			ctrl->length = 0;
			break;
		case MSW_CTL_SET_MSW_DEBUG_LEVEL:
			MswDebugLevel = ctrl->subcode;
			ctrl->retcode = 0;
			ctrl->length = 0;
			break;
		case MSW_CTL_GET_ACTIVATION_MODE:
			if (ctrl->length < sizeof(T_ActivationMode)) break;
			if (ctrl->buffer == NULL) break;
			{
				T_ActivationMode *amode= (T_ActivationMode *)ctrl->buffer;
				amode->mode=ActivationMode;
			}	
			ctrl->retcode = 0;
			break;
	}

	DBG(RW_D,"retcode=%ld,length=%ld\n",ctrl->retcode,ctrl->length);

	return 0;
}

//----------------------------------------------------------------------
// unicorn_get_adsl_status	
//----------------------------------------------------------------------
ADSL_STATUS unicorn_get_adsl_status(struct unicorn_dev *dev)
{
	return adsl_status;
}

//----------------------------------------------------------------------
// unicorn_get_adsl_linkspeed
//----------------------------------------------------------------------
int unicorn_get_adsl_linkspeed(struct unicorn_dev *dev,
		unsigned long *us_rate,unsigned long *ds_rate)
{
	int status;

	if (adsl_status != ADSL_STATUS_ATMREADY) {
		*us_rate = 0;
		*ds_rate = 0;
		status = -ENXIO;
	} else {
		// adjust upstream rate to avoid overflow
		unsigned long speed = (adsl_us_cellrate*15UL)/16UL;
		*us_rate = (speed * ATM_CELL_SIZE*8UL)/1000UL;
		*ds_rate = (adsl_ds_cellrate * ATM_CELL_SIZE*8UL)/1000UL;
		status = 0;
	}
	return status;
}

//----------------------------------------------------------------------
// get_frame_number_offset
//----------------------------------------------------------------------
static int get_frame_number_offset(struct usb_device *usb_dev)
{
	int ret=0; // default, USB_ISOC_ASAP 

	if (usb_dev->bus) {
		struct usb_device *root_hub = usb_dev->bus->root_hub;
		if (root_hub->descriptor.iProduct) {
			char *buf = kmalloc(256, GFP_KERNEL);
			if (buf) {
				if (usb_string(root_hub, 
							root_hub->descriptor.iProduct, buf, 256) > 0) {
					PRINT_INFO("%s\n",buf);
					if (strcmp(buf,"USB UHCI Root Hub")==0) {
						// usb-uhci driver:
						// need to wait 12 frames before reading OBC
						ret = 12;
					}
				}
				kfree(buf);
			}
		}
	}
	return ret;
}

//----------------------------------------------------------------------
// probe_unicorn_usb
//----------------------------------------------------------------------
static int do_probe(struct usb_device *usb_dev,unsigned long driver_info)
{
        struct unicorn_dev *dev = &unicorn_usb_dev;
        int status;

	PRINT_INFO("found adapter VendorId %04x, ProductId %04x, driver_info=%ld\n",
			usb_dev->descriptor.idVendor, usb_dev->descriptor.idProduct,driver_info);

	GlobalRemove = FALSE;
	// Initialize RAPI
	if ((status = rapi_init()) != 0) {
		DBG(USB_D,"initialization of RAPI failed\n");
		return status;
	}

	// tweaks for different host controller drivers
	FrameNumber = get_frame_number_offset(usb_dev);
	DBG(1,"FrameNumber=%d\n",FrameNumber);


	// Initialize USB adapter
	if ((status = start_device(dev,usb_dev)) != 0) {
		DBG(USB_D,"initialization of USB failed\n");
		return status;
	}

	// Tell ATM driver we are initialized
	if ((status = unicorn_attach(&unicorn_usb_entrypoints)) != 0) {
		DBG(USB_D,"initialization of ATM driver failed\n");
		return status;
	}

	if(!set_msw_params()) {
		DBG(USB_D,"incorrect ActivationMode\n");
		return 0;
	}
	msw_init(ActivationMode);
	PRINT_INFO("msw_init MSW_ActivationMode %ld\n",ActivationMode);

	dev->msw_started = TRUE;
	if (AutoActivation) {
		msw_start();
		PRINT_INFO("msw_start \n");
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
static int __devinit probe_unicorn_usb(struct usb_interface *usb_intf,
		const struct usb_device_id *id)
{
	struct usb_device *usb_dev = interface_to_usbdev(usb_intf);
	int status;
	status = do_probe(usb_dev, id->driver_info);
	usb_set_intfdata(usb_intf,&unicorn_usb_dev);
	return status;
}
#else
static void * __devinit probe_unicorn_usb(struct usb_device *usb_dev,
		unsigned int ifnum,
		const struct usb_device_id *id)
{

	if (do_probe(usb_dev,id->driver_info)) {
		return NULL;
	} else {
		return &unicorn_usb_dev;
	}
}
#endif

//----------------------------------------------------------------------
// disconnect_unicorn_usb
//----------------------------------------------------------------------
static void do_disconnect(struct unicorn_dev *dev)
{
	GlobalRemove = TRUE;

	msw_stop();
	msw_exit();

	unicorn_detach();
	stop_device(dev);
	rapi_exit();
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
static void __devexit disconnect_unicorn_usb(struct usb_interface *usb_intf)
{
	do_disconnect(usb_get_intfdata(usb_intf));
}
#else
static void __devexit disconnect_unicorn_usb(struct usb_device *usb_dev, void *arg)
{
	do_disconnect(arg);
}
#endif


static struct usb_device_id unicorn_usb_ids[] = {
	{ USB_DEVICE(0x0483, 0x0138),driver_info: 20174}, // STMicro reference design
	{ }
};

MODULE_DEVICE_TABLE (usb, unicorn_usb_ids);

static struct usb_driver unicorn_usb_driver = {
name: "unicorn_usb",
probe: probe_unicorn_usb,
disconnect: __devexit_p(disconnect_unicorn_usb),
id_table: unicorn_usb_ids,
};

/* module parameters for MSW */
module_param(ActivationMode, long, 0);
module_param(AlternativeSetting, long, 0);
module_param(AutoActivation, long, 1);
module_param(DownstreamRate, long, 0);
module_param(LoopbackMode, long, 0);
module_param(MswDebugLevel, long, 0);
module_param(RetryTime, long, 0);
#if DEBUG
module_param(DebugLevel, long, 0);
#endif
module_param(ledScenario, long, 1);

//----------------------------------------------------------------------
// unicorn_usb_init
//----------------------------------------------------------------------
static int __init unicorn_usb_init(void)
{
	int status;
	struct unicorn_dev *dev = &unicorn_usb_dev;

	dev->started = FALSE;

	PRINT_INFO("Package version: %s, Driver version %s, %s, %s\n", __TIME__, __DATE__,  ""PKG_VERSION, ""DRIVER_VERSION);
	PRINT_INFO("driver parameters: DebugLevel=%ld\n",DebugLevel);

	status = usb_register(&unicorn_usb_driver);
	if (status < 0) {
	}
	return status;
}

static void __exit unicorn_usb_cleanup(void)
{
	struct unicorn_dev *dev = &unicorn_usb_dev;
	if (GlobalRemove == FALSE) {
		stop_hardware(dev);
	} else {
		unicorn_detach();
	}
	usb_deregister(&unicorn_usb_driver);
}

module_init(unicorn_usb_init);
module_exit(unicorn_usb_cleanup);
