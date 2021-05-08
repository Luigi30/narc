#pragma module PKQDRIVER "X-10A1"
/*
 *****************************************************************************
 * 
 * Copyright ? Digital Equipment Corporation, 1993-1995 All Rights Reserved.
 * Unpublished rights reserved under the copyright laws of the United States.
 * 
 * The software contained on this media is proprietary to and embodies the
 * confidential technology of Digital Equipment Corporation.  Possession, use,
 * duplication or dissemination of the software and media is authorized only
 * pursuant to a valid written license from Digital Equipment Corporation.
 * 
 * RESTRICTED RIGHTS LEGEND   Use, duplication, or disclosure by the U.S.
 * Government is subject to restrictions as set forth in Subparagraph
 * (c)(1)(ii) of DFARS 252.227-7013, or in FAR 52.227-19, as applicable.
 * 
 *****************************************************************************
 *
 *
 * FACILITY:
 *
 *	Alpha VMS/SCSI device driver
 *
 * ABSTRACT:
 * 
 *	PKQDRIVER is a SCSI port driver.  It supports the QLogic ISP1020 chip.
 *
 * AUTHOR:
 *
 *	David E. Eiche
 *
 * CREATION DATE:
 *
 *	1-Jun-1994
 *
 * REVISION HISTORY:
 *
 *      X-11    RJS001          Richard Stammers                23-Mar-1997
 *              - Changed pk$send_command to test the CDB for an INQUIRE
 *                command being sent. If an INQUIRE is being sent then
 *                re-negotiation of wide/synch with the target is forced if
 *                if required, and if firmware version >= 2.00
 *              - Made corresponding changes to ISP1020DEF.SDL to
 *                describe required bits in control flags
 *
 *	X-10A1	MED003          Mark DiFabio                    05-Feb-1997
 *		(Changing IDENT to match Library Generation number.)
 *              Change PK$SEND_COMMAND to use EXE$GL_BLAKHOLE as the pad
 *              buffer for unaligned reads. For writes, EXE$GL_ERASEPB is
 *              used. By using EXE$GL_ERASEPB for reads, the ERASEPB buffer
 *              was getting corrupted with the 'pad' data. 
 *
 *      X-10    RTS001          Bob Silver                      15-Nov-1996
 *              - Do mailbox I/O to change clock rate only when the 60MHZ_CLOCK
 *                flag is set.  Otherwise, leave it as is.
 *		- Call routine to set clock rate only with firmware > 3.00.
 *
 *	X-9	RAR088		Buzzy Ritter			28-Oct-1996
 *	    	- Make target mode code conditional on running firmware > 3.00.
 *	    	- fix uninitialized pointer in queue full path.
 *	    	- fix register dump routine to get right registers.
 *	    
 *	X-8	RAR086		Buzzy Ritter			16-Sep-1996
 *		Fix pause_risc problems with x-7.
 *
 *	X-7	RAR086		Buzzy Ritter			13-Sep-1996
 *		- Fix fast20 implementation by pausing risc execution before
 *		  reading trig pin ( bit 5 of processor status register).
 *		- Setup IO's so they will timeout on firmware hang.
 *		- minimize synchronous offset with 8 and fifo threshold with
 *		  0 ( 0 represents a threshold of 8). 
 *		- V3.2 of the isp firmware hung at high IO load. V3.21 of the
 *		  fw fixes that problem but sometimes caused an asynchronous
 *		  status code of 0x8002 which is a system error. The driver
 *		  wasn't handling that error proberly and the system hangs.
 *		  This change reinits to chip on system error.
 *		- Take out troubling message if a fast20 board's nvram  is set
 *		  to fast10. This will be a normal thing.
 *		- Add debugging code to dump nvram contents to the console
 *		  at uint init. 
 *		
 *      X-6A7	TLC           Tony Camuso                     	11-June-1996
 *              pkq_struc_init(): Raise DIPL to 21 for all platforms
 *
 *	X-6A6	RAR078		Buzzy Ritter			6-Feb-1996
 *		- Add support for fast20 boards , compare nvram to istat line 5
 *		  if they're the same set speed accordingly. If diffrent output
 *		  message and set slower speed.
 *		- fix additional length of sense response message.
 *
 *	X-6A5	RAR077		Buzzy Ritter			17-Jan-1996
 *		- Add support for target mode for clusters.
 *		  This includes canges to unit init to initialize data
 *		  structures, changes to the interrupt service routine to handle
 *		  target mode messages in the response queue and changes in
 *		  send command to handle reset synchronization and ensure
 *		  enough room in the request queue for target messages send
 *		  from the interrupt fork.
 *		- Fix bug in call to deallocate on dump_ram failure.
 *		- Fix isr to avoid fork routine reentrancy. (and take out 
 *		  double fork)
 *
 *	X-6A4	RCL001		Rick Lord			1-Dec-95
 *		- Modify the newly-created SPDT to reflect maximum and
 *		  current bus widths of 16, which is the largest that
 *		  this port can support. Search for "RCL001". There are
 *		  hardcoded 16's in several places in the driver so I'm
 *		  assuming that this is the most it can handle.
 *
 *	X-6A3	RAR076		Buzzy Ritter			6-Nov-1995
 *		Merge the following ghost changes into gryphon.
 *		X-4U4A3	RAR075		Buzzy Ritter		2-Nov-1995
 *		Add check for rawhide platform in DIPL init code. Rawhide uses
 *		IPL21 (like ALCOR) rather than IPL20. 
 *
 *		X-4U4A2	RAR073		Buzzy Ritter		3-Oct-1995
 *		-Add support for overrun recovery. Setup FW to interrupt on
 *		overrun, pad IO's (using scdrp$l_pad_cnt) and reset the bus
 *		if an interrupt occurs. 
 *		- Fix bug in dump_ram implementation for segmented dumps.
 *
 *		X-4U4A1	RAR071		Buzzy Ritter		3-Oct-1995
 *		Add delay into wait loops in pkq_mailbox_io CSR wait loops
 *		to avoid PCI bus saturation.
 *
 *	X-6A2	RAR071		Buzzy Ritter			2-Aug-1995
 *		Add the following zeta-zeta tima fixes.
 *		X-4U5	DEE0235		David E. Eiche		17-Jul-1995
 *		Modify pkq_dump_firmware to checksum the firmware image in
 *		non-paged pool and to retry the dump operation if the
 *		checksum fails to recover from intermittent ISP1020 DUMP_RAM
 *		mailbox command failures.  In pkq_mailbox_io, add code to
 *		return an error on command failure rather than issuing a
 *		bugcheck.  Provide additional console messages to reflect
 *		the error conditions.
 *
 *		RAR070		Buzzy Ritter			24-Jul-1995
 *		Replace mailbox DUMP_RAM call with call to dump_ram routine 
 *		that reads the ram a word at a time. This avoids a problem that
 *		we were seeing on PnSE with the DUMP_RAM mailbox command
 *		terminating early.
 *
 *		X-4U4	DEE0234		David E. Eiche		14-Jun-1995
 *		Fix problems in error logging code:  force byte alignment of
 *		error logging structures and fix miscellaneous typos which
 *		cause data to be overwritten; also add code to freeze the
 *		ISP1020 RISC processor while doing the register dump and to
 *		allow SXP register selection and dumping.
 *
 *		Add code to set the SCSI ID in the standard SPDT field, so
 *		that SDA/CLUE can find it.
 *
 * 	X-6A1	JPJ   		James P. Janetos		07-Aug-1995
 *              IOC_ROUTINES.H now contains the prototype for
 *              IOC$READ_PCI_CONFIG, so remove the prototype
 *              declaration from this module.  Fix the 5th argument in
 *              the call to IOC$READ_PCI_CONFIG -- it is supposed to
 *              be a pointer to an int rather than a pointer to an
 *              int64.
 *
 * 	X-6	RAR068		Buzzy Ritter			26-Apr-1995
 *		Merge the following zeta changes:
 *
 *		X-4U3	DEE0230		David E. Eiche		31-Mar-1995
 *		Change prototype for sc$rl_create_port to contain argument
 *		types.  
 *
 *		Add routines to implement response IDs in order to eliminate a
 *		stale pointer problem when the adaptor returns a completion
 *		response for a request that had already been completed as the
 *		result of a bus reset.  Invoke the new routines from unit
 *		initialization, command buffer allocate and deallocate and
 *		the interrupt fork routine.
 *
 *		Incorporate Buzzy's change to pk$cmd_wait_completion to check
 *		the error status returned by exe$kp_stall_general as a result
 *		of a bus reset.  Also incorporate his change to correctly
 *		terminate the version string in pkq_convert_ver.
 *
 *		Add routines to print the firmware version and to indicate
 *		whether the firmware came from the console or driver image.
 *		Do check of firmware checksum.
 *
 *		Add comments to include files.
 *		Set device ipl to 21 for alcor.
 *
 *		Add flag to indicate to reset_scsi_bus that the reset is from
 *		unit_init.
 *
 *		X-4U2	DEE0229		David E. Eiche		12-Mar-1995
 *		In pkq_load_firmware() correct the firmware length calculation
 *		and add a mailbox command to verify the firmware checksum
 *		before starting it up.
 *		RAR066		Buzzy Ritter			20-Mar-1995
 *		fix multiple interrupt problems with read of icr after it is
 *		written.
 *
 *		X-4U1	DEE0228		David E. Eiche		02-Mar-1995
 *		Correct length calculation in pkq_dump_firmware.  Also incorporate
 *		Buzzy's change to pkq_interrupt_fork to add another call to
 *		bus reset.
 *
 *		DEE0227		David E. Eiche			23-Feb-1995
 *		Relocate the target parameters to the SPDT because the STDT
 *		don't exist at port unit initialization time.  Correct a data
 *		type error in pkq_nvram_read_array() which caused a checksum error.
 *
 *		RAR065		BUZZY RITTER			06-MAR-1995
 * 		Add flow control to stall queue manager when request queue
 *		is full. Also add debug tracing. Add hysteresis to the stall
 *		mechanism so that 25% of buffer size must complete.
 *
 *	X-5	EMB		Ellen M. Batbouta		10-Mar-1995
 *      	Declare the system data cells, MMG$GL_PAGE_SIZE and MMG$GL_
 *		BWP_MASK, as external integers which have a constant value.
 *
 *	X-4	DEE0226		David E. Eiche			03-Feb-1995
 *		Add code to get adaptor operating parameters from the on-board
 *		NVRAM.  Add code to compare the version of the firmware loaded
 *		by the console to the version linked with the driver and use
 *		the newer of the two.  Also, provide the ability to force the
 *		use of default operating parameters rather than the NVRAM and
 *		force the use of the firmware loaded with the driver.
 *
 *		Add code to enable bus reset processing and to detect firmware
 *		hangs.
 *
 *	X-3	DEE0224		David E. Eiche			22-Dec-1994
 *		Add code to pk$buffer_map and pk$send_command to convert
 *		a PTE to a PFN if the PTE is not valid.  Remove odd byte
 *		padding code.  Add mailbox completion codes to pkq_interrupt_fork
 *		and ignore mailbox command completion  and self-test completion
 *		interrupts.
 *
 *	X-2	DEE0223		David E. Eiche			09-Dec-1994
 *		Get isp1020def.h from obj$:
 *
 *	X-1	DEE0222		David E. Eiche			09-Dec-1994
 *		Initial Checkin.
 *
 */

/* Include files for system structures from SYS$LIB_C.TLB */

#include <adpdef.h>				/* Adaptor control block definitions		*/
#include <busarraydef.h>			/* Bus array header and entry definitions	*/
#include <ccbdef.h>				/* Channel control block definitions		*/
#include <crbdef.h>				/* Channel request block definitions		*/
#include <ctype.h>				/* Character type macro definitions		*/
#include <dcdef.h>				/* Device adaptor, class and type definitions	*/
#include <ddbdef.h>				/* Device data block definitions		*/
#include <ddtdef.h>				/* Driver dispatch table definitions		*/
#include <devdef.h>				/* Device characteristics definitions		*/
#include <dptdef.h>				/* Driver prologue table definitions		*/
#include <dyndef.h>				/* Data structure type definitions		*/
#include <fdtdef.h>				/* Function decision table definitions		*/				
#include <hwrpbdef.h>				/* Hardware RPB definitions			*/
#include <idbdef.h>				/* Interrupt dispatch block definitions		*/
#include <ints.h>				/* Integer type definitions			*/
#include <iocdef.h>				/* Ioc$node_data function code definitions	*/
#include <iodef.h>				/* I/O function code definitions		*/
#include <iohandledef.h>			/* I/O mapping handle definitions		*/
#include <irpdef.h>				/* I/O request packet definitions		*/
#include <fkbdef.h>				/* Fork block definitions			*/
#include <orbdef.h>				/* Object rights block definitions		*/				
#include <pcbdef.h>				/* Process control block definitions		*/
#include <pcidef.h>				/* PCI bus definitions				*/
#include <scsidef.h>				/* SCSI definitions				*/
#include <scdtdef.h>				/* SCSI connection descriptor table definitions	*/
#include <scdrpdef.h>				/* SCSI class driver request packet definitions	*/
#include <spdtdef.h>				/* SCSI port descriptor table definitions	*/
#include <stdtdef.h>				/* SCSI target descriptor table definitions	*/
#include <spldef.h>				/* Spinlock control block definitions		*/
#include <splcoddef.h>				/* Spinlock index definitions			*/
#include <ssdef.h>				/* System service status code definitions	*/
#include <stsdef.h>				/* System service status definitions	*/
#include <stddef.h>				/* ISO common definitions			*/
#include <string.h>				/* String processing function definitions	*/
#include <ucbdef.h>				/* Unit control block definitions		*/
#include <vecdef.h>				/* CRB interrupt transfer vector definitions	*/

/* Include files which define exec routine prototypes, from SYS$LIB_C.TLB */

#include <exe_routines.h>
#include <ioc_routines.h>
#include <sch_routines.h>
#include <smp_routines.h>

/* Driver Macros from SYS$LIB_C.TLB */

#include <vms_drivers.h>
#include <vms_macros.h>

/* Include files from DECC$RTLDEF.TLB */

#include <builtins.h>

/* QLogic ISP 1020 definitions */

#include "obj$:isp1020def.h"

/* Local macro definitions */

#define SUCCESS( _status ) (( _status ) & 1 )
#define ERROR( _status ) ( !(( _status ) & 1 ))
#define TRUE 1
#define FALSE 0

#ifndef PKQ_DEBUG
#define PKQ_DEBUG 0
#endif

#define PKQ_MAXBCNT 65535			/* Port maximum byte count			*/
#define PKQ_NUM_RSPID 255			/* Number of RSPIDs in table			*/
#define PKQ_USE_NVRAM_DEFAULTS FALSE		/* Forces use of compiled-in defaults		*/
#define PKQ_USE_LINKED_FIRMWARE FALSE		/* Forces use of linked-in firmware		*/
#define PKQ_WATCHDOG_INTERVAL 30
#define MAX_ATIO_ENTRIES 4			/* max atio entries from fw */
#define MAX_NOTIFY_ENTRIES 4			/* max notify entries from fw */
#define MAX_MARKER_ENTRIES 2			/* max marker entries requires */
/* 
**   min_free entries is the space that is needed in the request queue before
**   command initiation can happen. 
*/
#define MIN_FREE_ENTRIES (MAX_ENTRIES_PER_REQUEST+MAX_ATIO_ENTRIES+MAX_NOTIFY_ENTRIES+MAX_MARKER_ENTRIES)
/*
**	These two values should probably be in scsidef.h but they aren't.
**	They are the command codes for the inquiry and request sense commands.
*/
#define SCSI$K_REQUEST_SENSE 0x3
#define SCSI$K_INQUIRY 0x12

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

#define DECLARE_ISP() \
    uint64		_temp; \
    uint32		_offset; \
    ISP_REGISTERS	*_isp

#define READ_ISP( _adp, _iohandle, _regname ) \
    (( ioc$read_io(_adp, _iohandle, \
		   ( _offset = ( uint64 )(( uint8 * )&_isp->_regname - ( uint8 * )&_isp->isp$w_bus_id_low )), \
		   sizeof( uint16 ), &_temp ) & 1 ) ?  \
     (( _offset & 2 ) ? \
      (( _temp >> 16 ) & 0xffff ) : \
      ( _temp & 0xffff )) : \
     pkq_bugcheck() )

#define WRITE_ISP( _adp, _iohandle, _regname, _wvalue ) \
    _offset = ( uint64 )(( uint8 * )&_isp->_regname - ( uint8 * )&_isp->isp$w_bus_id_low ); \
    _temp = (( _offset & 2 ) ? (( _wvalue & 0xffff ) << 16 ) : ( _wvalue & 0xffff )); \
    if( !(ioc$write_io( _adp, \
                        _iohandle, \
		        _offset, \
		        sizeof( uint16 ), \
			&_temp ) & 1 )) { \
        pkq_bugcheck(); \
   }




#if PKQ_DEBUG
/*
** define structures for debugging purposes here.
*/

/*
** trace buffer definitions 
*/


struct pkq_trace_entry {
    uint32	trace_type;
    uint32	trace_p1;
    uint32	trace_p2;
    uint32	trace_p3;
};
#define TRACE_ENTRIES 1000
#define TRACE_SIZE sizeof(struct pkq_trace_entry)*TRACE_ENTRIES

#endif

/* Response ID structure definitions:
 *  - the RSPID itself
 *  - the RSPID Table Entry
 */
#pragma member_alignment save
#pragma nomember_alignment LONGWORD
typedef struct _header {
	   void	*flink;
	   void *blink;
	   uint16 size;
	   uint8  type;
	   uint8 subtype;
	   uint32 fill;
	   } HEADER;
typedef struct _rspid {				/* ReSPonse IDentifer				*/
    uint16		index;			/* RSPID index					*/
    uint16		seq_no;			/* RSPID sequence number			*/
} RSPID;
#pragma member_alignment restore

#pragma member_alignment save
#pragma nomember_alignment QUADWORD
typedef struct _rte {
    struct _rte	*flink;				/* Free entry forward link			*/
    RSPID	rspid;				/* RSPID itself					*/
    void	*p1;				/* User structure pointer			*/
    void	*spare;				/* Spare, for expansion				*/
} RTE;
#pragma member_alignment restore


/*
**	The definitions for inquiry data and sense data are in 
**	scsidef.h.
** 	These are just typedefs for those structures
*/

typedef struct inquiry_data INQ_RESP;
typedef struct sense_data   REQ_SEN_RESP;
/*
**  pkq_fill is the # of bytes required to longword align the spdt extensiion
**  after adding the inquiry response and the request sense response.
**  The +1 is to accomodate the 1 byte message returned for a short inquiry 
**  request.
*/

#define pkq_fill ((4-((sizeof(INQ_RESP)+sizeof(REQ_SEN_RESP))&3))&3)


/* Define the port-specific extensions to the SPDT
 */
#pragma member_alignment save
#pragma nomember_alignment QUADWORD

typedef struct _pkq_spdt {
    SPDT	spdt$r_base_spdt;			/* Base SPDT				*/
    uint64	spdt$q_iohandle_reg;			/* I/O Handle that maps ISP registers	*/
    uint64	spdt$q_mem_base;			/* Adaptor's memory space base address	*/
    uint64	spdt$q_direct_dma_base;			/* Base PA of direct DMA window		*/
    uint32	spdt$l_direct_dma_size;			/* Size of direct DMA window in bytes	*/
    KPB		*spdt$ps_kpb;				/* Pointer to adaptor-wide KPB		*/
    ISP_ENTRY	*spdt$ps_requestq_va;			/* Request queue virtual address	*/
    ISP_ENTRY	*spdt$ps_responseq_va;			/* Response queue virtual address	*/
    uint64	spdt$ps_requestq_pa;			/* Response queue physical address	*/
    uint64	spdt$ps_responseq_pa;			/* Response queue physical address	*/
    uint32	spdt$l_requestq_in;			/* Request queue in index		*/
    uint32	spdt$l_responseq_out;			/* Response queue out index		*/
    uint16	*spdt$ps_firmware_va;			/* Firmware virtual address		*/
    uint32	spdt$l_firmware_version;		/* Version of firmware in use		*/
    uint32	spdt$l_pkq_flags;			/* PKQ port-specific flags		*/
    uint16	spdt$w_mailbox_0;			/* Outgoing mailboxes			*/
    uint16	spdt$w_mailbox_1;			/*  from most				*/
    uint16	spdt$w_mailbox_2;			/*  recently 				*/
    uint16	spdt$w_mailbox_3;			/*  issued				*/
    uint16	spdt$w_mailbox_4;			/*  mailbox				*/
    uint16	spdt$w_mailbox_5;			/*  command				*/
    uint16	spdt$w_mailbox_6;			/*  (reserved for future expansion)	*/
    uint16	spdt$w_mailbox_7;			/*  (reserved for future expansion)	*/
    uint16	spdt$w_initiator_scsi_id;		/* Initiator SCSI ID			*/
    uint16	spdt$w_bus_reset_delay;			/* Delay after reset until 1st command	*/
    uint16	spdt$w_retry_count;			/* Error retry count			*/
    uint16	spdt$w_retry_delay;			/* Delay between error retries		*/
    uint16	spdt$w_tag_age_limit;			/* Tag aging limit counter		*/
    uint16	spdt$w_selection_timeout;		/* Selection timeout (ms)		*/
    uint16	spdt$w_max_queue_depth;			/* Maximum queue depth			*/
    uint16	spdt$v_fifo_threshhold		: 2;	/* FIFO threshhold			*/
    uint16	spdt$v_adaptor_enable		: 1;	/* Adaptor is enabled if 1 (ignored)	*/
    uint16	spdt$v_asynch_setup_time	: 4;	/* Asynch setup time (clock periods)	*/
    uint16	spdt$v_req_ack_active_negation	: 1;	/* Enable SCSI REQ/ACK active pullups	*/
    uint16	spdt$v_data_active_negation	: 1;	/* Enable SCSI Data active pullups	*/
    uint16	spdt$v_data_dma_burst_enable	: 1;	/* Data DMA channel burst enable	*/
    uint16	spdt$v_command_dma_burst_enable : 1;	/* Command DMA channel burst enable	*/
    uint16	spdt$v_termination_low_enable	: 1;	/* Enable termination of low 8 bits	*/
    uint16	spdt$v_termination_high_enable	: 1;	/* Enable termination of high 8 bits	*/
    uint16	spdt$v_pcmc_burst_enable	: 1;	/* PCMC burst enable			*/
    uint16	spdt$v_60mhz_enable		: 1;	/* Enable 60MHZ ISP1020 clock		*/
    uint32					: 32;	/* Pad to QUAD boundary			*/
#pragma member_alignment save
#pragma nomember_alignment WORD
    struct _tparams {
	union {
	    uint16	spdt$w_capabilities;
	    struct {					/* Bit order required by firmware	*/
		uint16				   : 8; /* Not used				*/
		uint16	spdt$v_renegotiate_on_error  : 1; /* Renegotiate on check condition	*/
		uint16	spdt$v_stop_queue_on_check : 1; /* Stop queue on check			*/
		uint16	spdt$v_auto_request_sense  : 1; /* Auto request sense on check		*/
		uint16	spdt$v_tagged_queuing	   : 1; /* Tagged queuing support		*/
		uint16	spdt$v_synch_data_transfers    : 1; /* Synchronous data transfers	*/
		uint16	spdt$v_wide_data_transfers     : 1; /* Wide data transfers enabled	*/
		uint16	spdt$v_parity_checking     : 1; /* Parity checking enabled		*/
		uint16	spdt$v_disconnect_allowed  : 1; /* Disconnect privilege enabled		*/
	    } c_flags;
	} u1;
	union {
	    uint16	spdt$w_synch_params;
	    struct {					/* Field order required by firmware	*/
		uint8	spdt$b_synch_period;		/* Synchronous transfer period		*/
		uint8	spdt$b_synch_offset;		/* REQ/ACK offset			*/
	    } synch;
	} u2;
	uint16	spdt$w_execution_throttle;		/* Maximum cmds firmware to device	*/
	union {
	    uint16	spdt$w_misc_flags;
	    struct {
		uint16	spdt$v_device_enable	: 1;	/* Device enabled (ignored)		*/
		uint16				: 15;	/* Unused				*/
	    } m_flags;
	} u3;
    } spdt$r_tparams[ 16 ];
#pragma member_alignment restore

    uint32	spdt$l_qman_stalled;			/* Queue manager stalled flag */
    RTE		*spdt$ps_rspid_table;			/* array of rspid blocks*/
    RTE		*spdt$ps_rspid_listhead;		/* RSPID table listhead				*/
    uint32	spdt$is_pkq_rspid_alloc_failures;	/* Count of RSPID allocation failures		*/
    uint32	spdt$is_pkq_rspid_stale_count;		/* Count of stale RSPIDs encountered		*/
    INQ_RESP	spdt$s_inquiry_response;		/* inquiry command response */
    REQ_SEN_RESP spdt$s_request_sense_response;  	/* request sense command response */

    char	align_fill[pkq_fill];			/* fill */

    ISP_ENTRY	spdt$s_atio_entries[ MAX_ATIO_ENTRIES ]; /* buffers for saving atio buffers */
    char	spdt$b_short_inquiry_response;
    char	spdr$b_short_inq_resp_fill[3];

/*
**	add following line if an odd number of longwords is added
**    uint32	spdt$l_fill;
*/

#if PKQ_DEBUG
    uint32	spdt$l_requestq_out;			/* Request queue out index		*/
    uint32	spdt$l_responseq_in;			/* Response queue in index		*/
    struct	pkq_trace_entry  *spdt$ps_log_buffer;	/* log buffer start */
    struct	pkq_trace_entry  *spdt$ps_next_log;	/* next log buffer entry*/
    uint32	spdt$l_log_size;				/* log buffer size */
    uint32	spdt$l_fill_deb;
#endif
} PKQ_SPDT;
#define basespdt spdt$r_base_spdt
#define SPDT$M_PKQ_SEND_MARKER	1			/* Need to send marker after bus reset	*/
#define SPDT$M_PKQ_PARAMS_READ	2			/* Params are stored in SPDT and STDT	*/
#define SPDT$M_PKQ_UNIT_INIT	4			/* Unit initialization in progress	*/
#define SPDT$M_PKQ_CONSOLE_FW	8			/* Active firmware originated in console */
#define SPDT$M_PKQ_SEND_ENABLE_LUN 	16		/* send enable_lun sent */
#define SPDT$M_PKQ_GOT_ENABLE_LUN 	32		/*  enable_lun returned */

#if PKQ_DEBUG
#define trace_event(a,b,c,d,e) pkq_trace_event((a),(b),(c),(d),(e))
#define trace_entry(a,b) pkq_trace_entry((a),(b))
#else
#define trace_event(a,b,c,d,e) 
#define trace_entry(a,b) 
#endif



#pragma member_alignment restore			/* resore for save just before nvram definitions */

#define spdt$v_auto_request_sense u1.c_flags.spdt$v_auto_request_sense
#define spdt$w_capabilities u1.spdt$w_capabilities
#define spdt$v_device_enable u3.m_flags.spdt$v_device_enable
#define spdt$v_disconnect_allowed u1.c_flags.spdt$v_disconnect_allowed
#define spdt$w_misc_flags u3.spdt$w_misc_flags
#define spdt$v_parity_checking u1.c_flags.spdt$v_parity_checking
#define spdt$v_renegotiate_on_error u1.c_flags.spdt$v_renegotiate_on_error
#define spdt$v_stop_queue_on_check u1.c_flags.spdt$v_stop_queue_on_check	
#define spdt$v_synch_data_transfers u1.c_flags.spdt$v_synch_data_transfers
#define spdt$b_synch_offset u2.synch.spdt$b_synch_offset
#define spdt$w_synch_params u2.spdt$w_synch_params
#define spdt$b_synch_period u2.synch.spdt$b_synch_period
#define spdt$v_tagged_queuing u1.c_flags.spdt$v_tagged_queuing
#define spdt$v_wide_data_transfers u1.c_flags.spdt$v_wide_data_transfers


/* Define the QBUF structure which is used as an intermediate
 * buffer to hold the SCSI CDB, status and autosense information
 */
#define QBUF$K_CDB_SIZE			44		/* SCSI Command Data Block size		*/
#define QBUF$K_AUTOSENSE_SIZE		32		/* SCSI Autosense size			*/
#define QBUF$M_FL_AUTOSENSE_VALID	1		/* Autosense info is valid		*/
#pragma member_alignment save
#pragma nomember_alignment QUADWORD
typedef struct _qbuf {
    void	*qbuf$ps_flink;				/* Forward link				*/
    void	*qbuf$ps_blink;				/* Backward link			*/
    uint16	qbuf$w_size;				/* Structure size			*/
    uint8	qbuf$b_type;				/* Structure type			*/
    uint8	qbuf$b_subtype;				/* Structure subtype			*/
    RSPID	qbuf$r_rspid;				/* Response ID				*/
    SCDRP	*qbuf$ps_scdrp;				/* SCDRP pointer			*/
    uint32	qbuf$l_bytes_not_xfrd;			/* Residual byte count			*/
    uint32	qbuf$l_flags;				/* Reserved for QBUF status flags	*/
    uint16	qbuf$w_autosense_length;		/* Length of returned autosense data	*/
    uint8	qbuf$b_scsi_status;			/* SCSI status				*/
    uint8	qbuf$b_spare;				/* Not used				*/
    uint32	qbuf$l_port_status;			/* Completion status returned by port	*/
    uint32	qbuf$l_class_driver;			/* Class-driver overhead space		*/
    uint32	qbuf$l_scsi_cdb_size;			/*  including Command Data Block size	*/
    uint8	qbuf$b_scsi_cdb[ QBUF$K_CDB_SIZE ];	/*  and SCSI Command Data Block		*/
    uint8	qbuf$b_autosense[ QBUF$K_AUTOSENSE_SIZE ]; /* Autosense data			*/
} QBUF;
#pragma member_alignment restore			/* restore from just before PKQ_SPDT start */


/* Define the layout of the ISP1020 firmware header
 *
 * ** TODO ** relocate this to ISP1020DEF.SDL
 */
typedef struct _isp_fw_hdr {
    uint16	jmpop;					/* ISP1020 jump opcode			*/
    uint16	jmpaddr;				/* Jump address (end of copyright + 1)	*/
    uint16	keyword;				/* A keyword of unknown use		*/
    uint16	firmware_length;			/* Firmware length in words		*/
    uint16	copy_flag;				/* A copy flag of unknown use		*/
} ISP_FW_HDR;


/* Error status mapping table, used to convert a QLogic
 * completion status code into a VMS status.
 */
typedef struct _err_status_map {
    int		vms_status;
    int		flags;
} ERR_STATUS_MAP;

#define PKQ_M_ESM_USEOF			1		/* Use operation flags to aid mapping	*/
#define PKQ_M_ESM_LOG			2		/* Log this error			*/
#define PKQ_M_ESM_LOGI			4		/* Log this error only after dev init	*/

ERR_STATUS_MAP	err_status_map[] = {
    { SS$_NORMAL,	0 },				/* 0x00 Normal xport completion		*/
    { SS$_NORMAL,					/* 0x01 Incomplete xport (sel timeout)	*/
	  ( PKQ_M_ESM_USEOF | PKQ_M_ESM_LOGI )},
    { SS$_DEVCMDERR,	PKQ_M_ESM_LOG },		/* 0x02 DMA direction error		*/
    { SS$_NORMAL,					/* 0x03 Transport error error		*/
	  ( PKQ_M_ESM_USEOF | PKQ_M_ESM_LOG ) },
    { SS$_MEDOFL,	PKQ_M_ESM_LOG },		/* 0x04 SCSI bus reset			*/
    { SS$_ABORT,	PKQ_M_ESM_LOG },		/* 0x05 Driver aborted command		*/
    { SS$_TIMEOUT,	PKQ_M_ESM_LOG },		/* 0x06 Transport timeout		*/
    { SS$_NORMAL,	PKQ_M_ESM_LOG },		/* 0x07 Data overrun			*/
    { SS$_NORMAL,	PKQ_M_ESM_LOG },		/* 0x08 Command overrun			*/
    { SS$_NORMAL,	PKQ_M_ESM_LOG },		/* 0x09 Status overrun			*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x0a Bad message after status phase	*/ 
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x0b No message out after select	*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x0c	Extended ID message rejected	*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x0d	IDE message rejected		*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x0e	Abort message rejected		*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x0f	Reject message rejected		*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x10	NOP message rejected		*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x11	Parity error message rejected	*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x12	Device reset message rejected	*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x13	Identify message rejected	*/
    { SS$_CTRLERR,	PKQ_M_ESM_LOG },		/* 0x14	Unexpected bus free		*/
    { SS$_NORMAL,	PKQ_M_ESM_LOG },		/* 0x15	Data underrun			*/
    { SS$_NORMAL,	PKQ_M_ESM_LOG },		/* 0x16	Command underrun		*/
    { SS$_NORMAL,	PKQ_M_ESM_LOG },		/* 0x17	Message underrun		*/
    { SS$_DEVCMDERR,	PKQ_M_ESM_LOG },		/* 0x18	Q'd xaction w/o disconnect priv	*/
    { SS$_DEVCMDERR,	PKQ_M_ESM_LOG },		/* 0x19	Q'd xaction w/ target routine	*/
    { SS$_DEVCMDERR,	PKQ_M_ESM_LOG }			/* 0x1a	Q'd xaction w/ q'ing disabled	*/
};
static int err_status_map_size = sizeof( err_status_map ) / sizeof( err_status_map[ 0 ] );


/* Error log structure definitions
 */
#pragma member_alignment save
#pragma nomember_alignment BYTE

typedef struct _pkq_erl_hdr {
    uint32	pkq_erl$l_lth_in_lw;			/* Entry length	(longwords!)		*/
    uint8	pkq_erl$b_rev;				/* Entry revision level			*/
    uint16	pkq_erl$w_type;				/* Entry type and subtype		*/
    uint8	pkq_erl$b_scsi_id;			/* SCSI ID				*/
} PKQ_ERL_HDR;

typedef struct _pkq_erl_cmd {				/* SCSI command				*/
    uint8	pkq_erl$b_scsi_cdb_size;		/* SCSI CDB Size (bytes)		*/
    uint8	pkq_erl$b_scsi_cdb[ QBUF$K_CDB_SIZE ];	/* SCSI Command Data Block		*/
} PKQ_ERL_CMD;

typedef struct _pkq_erl_msg {
    uint8	pkq_erl$b_msg_lth;			/* Length of error log message portion	*/
    uint8	pkq_erl$b_request[ ISP$K_ENTRY_LTH ];	/* Request entry			*/
    uint8	pkq_erl$b_response[ ISP$K_ENTRY_LTH ];	/* Response entry			*/
} PKQ_ERL_MSG;

typedef struct _pkq_erl_status {
    uint8	pkq_erl$b_status;			/* The SCSI status byte itself		*/
} PKQ_ERL_STATUS;

#define ISP$K_REG_FILE_CNT ( ISP$K_REG_FILE_SIZE / 2 )
typedef struct _pkq_erl_regs {
    uint8	pkq_erl$b_regs_lth;			/* Length of ISP registers portion	*/
    uint16	pkq_erl$w_regs[ ISP$K_REG_FILE_CNT ];	/* The ISP register contents		*/
} PKQ_ERL_REGS;

#define ISP$K_SXP_REG_CNT ( ISP$K_SXP_REG_SIZE / 2 )
typedef struct _pkq_erl_sxp_regs {
    uint8	pkq_erl$b_sxp_regs_lth;			/* Length of ISP SXP registers portion	*/
    uint16	pkq_erl$w_sxp_regs[ ISP$K_SXP_REG_CNT ]; /* The ISP SXP register contents	*/
} PKQ_ERL_SXP_REGS;

#define PKQ_ERL_REV 1					/* Save error log revision level	*/
#define PKQ_ERL_BUFSIZE ( sizeof(PKQ_ERL_HDR) + sizeof(PKQ_ERL_CMD) + sizeof(PKQ_ERL_MSG) + \
    sizeof(PKQ_ERL_STATUS) + sizeof(PKQ_ERL_REGS) + sizeof(PKQ_ERL_SXP_REGS))
                                                        /* Error log buffer size		*/
/* Define SCSI error log type and subtype code
 */
#define SCSI$C_BUS_HUNG 1				/* SCSI BUS (or chip) is hung		*/
#define SCSI$C_ARB_FAIL 2				/* Port didn't arbitrate		*/
#define SCSI$C_SEL_FAIL 3				/* Target device selection failed	*/
#define SCSI$C_TIMEOUT	4				/* Target device timed out		*/
#define  STMO$K_STO 1					/*  subcode - chip detected		*/
#define  STMO$K_INTTMO 2				/*  subcode - host detected		*/
#define SCSI$C_PARITY 5					/* Parity error				*/
#define SCSI$C_PHASE_ERROR 6				/* SCSI bus phase error			*/
#define SCSI$C_BUS_RESET 7				/* SCSI bus reset detected		*/
#define SCSI$C_UNEXPECTED_INTERRUPT 8			/* Unexpected interrupt received	*/
#define SCSI$C_BUS_RESET_ISSUED 9			/* SCSI bus reset issued		*/
#define SCSI$C_RESEL_ERR 10				/* Error during reselection		*/
#define SCSI$C_CTL_ERR 11				/* Controller error			*/
#define  SCTL$K_MAPOVRR 1				/*  subcode - mapping overrun		*/
#define SCSI$C_BUS_ERR 12				/* Bus error				*/
#define SCSI$C_ILLEGAL_MSG 13				/* Illegal message received		*/
#define SCSI$C_ILLEGAL_RSP 14				/* Illegal response received from port	*/
#define SCSI$C_DEVICE_RESET 15				/* SCSI device has been reset		*/
#define SCSI$C_CMD_ABORT 16				/* Command  aborted in progress		*/
#define SCSI$C_PORT_INIT 17				/* Port (re)initialized			*/

#pragma member_alignment restore

/* Define codes which are stuffed into the first longword of
 * device dependent information in the port UCB to indicate
 * why the port failed to come online.
 */
#define PKQ$K_ERR_MAPREG 1				/* Err mapping ISP1020 registers	*/
#define PKQ$K_ERR_ALOQUE 2				/* Err allocating request/response rings */
#define PKQ$K_ERR_ALOKPB 3				/* Err allocating KPB			*/
#define PKQ$K_ERR_KPSTRT 4				/* Err starting kernel process		*/
#define PKQ$K_ERR_ALORSP 5				/* Err allocating rspid */

/* External routines. Note that most external routines are defined above
 * by including	the following files:
 *		exe_routines.h
 *		ioc_routines.h
 *		sch_routines.h
 *		smp_routines.h
 */
extern	void	erl_std$deviceattn();
extern	void	exe_std$outcstring( char * );		/* ** TEMP m/b defined in exe_routines.h */
extern	void	exe_std$outcrlf();			/* ** TEMP m/b defined in exe_routines.h */
extern	void	ini$brk();
extern	void	sa$startio();
extern	void	sa$cancelio();
extern	void	sc$rl_create_port( UCB *, int, SPDT ** );

/* External global variables
 */
extern uint8	clu$gl_allocls;
extern uint8	clu$gl_tape_allocls;
extern DDT	driver$ddt;
extern DPT	driver$dpt;
extern FDT	sa$functab;
extern int	exe$gl_abstim;
extern int	exe$gl_blakhole;
extern PTE	*exe$gl_eraseppt;
extern HWRPB	*exe$gpq_hwrpb;
extern uint32	ioc$gl_naming;
extern char	sys$gq_version;
extern const int mmg$gl_bwp_mask;			/* Byte within page mask		*/
extern const int mmg$gl_page_size;
extern int	mmg$gl_vpn_to_va;
extern uint16	risc_code_addr01;
extern uint16	risc_code_length01;
extern uint16	risc_code01[];
extern uint32   scs$gb_systemid;
extern int	sgn$gl_userd1;

#pragma extern_model save
#pragma extern_model globalvalue
extern int	PK$K_MAX_SCSI_ID = 7;
extern int 	SA$K_KP_STKSIZ;
extern int 	SA$K_KP_REGMSK;
#pragma extern_model restore
extern int	not_valid_counter = 0;

/* Forward routines -- Table of Contents
 */
int	driver$init_tables	();		
int	pk$abort_command	( PKQ_SPDT *spdt,
				 SCDRP *scdrp,
				 SCDT *scdt );
int	pk$map_buffer		( PKQ_SPDT *spdt,
				 SCDRP *scdrp,
				 uint32 priority );
int	pk$cmd_buffer_alloc	( PKQ_SPDT *spdt,
				 SCDRP *scdrp,
				 uint32 request_size,
				 void **ret_addr );
int	pk$cmd_buffer_dealloc	( PKQ_SPDT *spdt,
				 SCDRP *scdrp );
void	pk$interrupt		( IDB *idb );
int	pk$negotiate_synch	( PKQ_SPDT *spdt,
				 STDT *stdt );
int     pk$reset_scsi_bus       ( PKQ_SPDT *spdt,
				 SCDRP *scdrp );
int	pk$send_command		( PKQ_SPDT *spdt,
				 SCDRP *scdrp,
				 SCDT *scdt,
				 STDT *stdt,
				 UCB *ucb );
int	pk$unmap_buffer		( PKQ_SPDT *spdt,
				 SCDRP *scdrp );
int	pk$wait_completion	( PKQ_SPDT *spdt,
				 SCDRP *scdrp,
				 SCDT *scdt,
				 STDT *stdt );

int	pkq_alloc_queues	( PKQ_SPDT *spdt );
RSPID	pkq_alloc_rspid		( PKQ_SPDT *spdt,SCDRP *scdrp );
int	pkq_buffer_map_restart	( CRCTX *crctx );
int	pkq_bugcheck		();
void	pkq_convert_ver		( PKQ_SPDT *spdt,
				 char *message );
int	pkq_ctrl_init		( IDB *idb,
				 DDB *ddb,
				 CRB *crb );
void	pkq_dealloc_rspid	( PKQ_SPDT *spdt, RSPID rspid );
int	pkq_dump_firmware	( PKQ_SPDT *spdt,
				 uint16 **console_fw,
				 int *console_fw_length );
int	pkq_get_firmware_ver	( uint16 *firmware );
void	pkq_get_parameters	( PKQ_SPDT *spdt );
QBUF	*pkq_get_qbuf		( SCDRP *scdrp );
STDT	*pkq_get_stdt		( PKQ_SPDT *spdt,
				 int scsi_id );
void	pkq_init_rspid_table	( PKQ_SPDT *spdt, RTE *p_rspid_table );
void	pkq_interrupt_fork	( IDB *idb,
				 PKQ_SPDT *spdt,
				 SCSI_UCB *ucb );
int	pkq_load_firmware	( PKQ_SPDT *spdt);
void	pkq_log_error		( uint8 type,
				 uint8 subtype,
				 SCDT *scdt,
				 PKQ_SPDT *spdt );
int	pkq_mailbox_io		( PKQ_SPDT *spdt,
				 uint32 rcount,
				 uint16 command,
				 uint16 r1,
				 uint16 r2,
				 uint16 r3,
				 uint16 r4,
				 uint16 r5 );
int	pkq_map_registers	( PKQ_SPDT *spdt );
void	pkq_nvram_command	( ADP *adp,
				 uint64 *handle,
				 uint16 cmd );
void	pkq_nvram_delay		( int32 delta );
void	pkq_nvram_off		( ADP *adp,
				 uint64 *handle );
void	pkq_nvram_on		( ADP *adp,
				 uint64 *handle );
int	pkq_nvram_read_array	( PKQ_SPDT *spdt,
				 ISP_NVRAM *nvram );
uint16	pkq_nvram_read_word	( PKQ_SPDT *spdt,
				 uint16 nvaddr );
int	pkq_set_speed		( PKQ_SPDT *spdt);
void	pkq_nvram_toggle	( ADP *adp,
				 uint64 *handle );
void	pkq_nvram_write_word	( PKQ_SPDT *spdt,
				 uint16 nvaddr,
				 uint16 data );
void	pkq_port_init		( KPB *kpb );
void	pkq_print_message	( PKQ_SPDT *spdt,
				 char *message,
				 int severity);
void	pkq_print_firmware_ver	( PKQ_SPDT *spdt,
				 int severity);
void	pkq_process_error	( PKQ_SPDT *spdt,
				 QBUF *qbuf,
				 ISP_ENTRY *isp_entry);
void	pkq_register_dump	( uint8 *buffer,
				 PKQ_SPDT *spdt,
				 SCSI_UCB *ucb );
int 	pkq_reset		( PKQ_SPDT *spdt );
int	pkq_reset_scsi_bus	( PKQ_SPDT *spdt );
void	pkq_ret_qbuf		( QBUF *qbuf );
SCDRP	*pkq_rspid_to_scdrp	( PKQ_SPDT *spdt, RSPID rspid );
int	pkq_select_firmware	( PKQ_SPDT *spdt );
int	pkq_set_parameters	( PKQ_SPDT *spdt );
int	pkq_send_marker		( PKQ_SPDT *spdt );
void	pkq_struc_init		( CRB *crb_ptr,
				 DDB *ddb_ptr,
				 IDB *idb_ptr,
				 ORB *orb_ptr,
				 UCB *ucb_ptr );
void	pkq_struc_reinit	( CRB *crb_ptr,
				 DDB *ddb_ptr,
				 IDB *idb_ptr,
				 ORB *orb_ptr,
				 UCB *ucb_ptr );
int	pkq_unit_init		( IDB *idb,
				 SCSI_UCB * ucb );
void	pkq_unit_init_fork	( IDB *idb,
				 SCSI_UCB *ucb,
				 FKB *fkb );
void	pkq_watchdog		( CRB * crb );
void	stall_qman		( PKQ_SPDT *spdt );
void	resume_qman		( PKQ_SPDT *spdt, uint32 uncond_flag);
int 	dump_ram		( PKQ_SPDT *spdt, uint16 *buffer, 
						uint32 risc_addr, uint32  size);
uint32 	backoff_delay		( uint32 delay, uint32 max);
void 	init_target_reply	(PKQ_SPDT *);
int 	alloc_atio_buffer	(PKQ_SPDT *, ISP_ENTRY *, ISP_ENTRY **);
void 	dealloc_atio_buffer	(ISP_ENTRY *);
void 	build_ctio		(PKQ_SPDT *, ISP_ENTRY *,ISP_ENTRY *);
void 	set_reqq_entry		(PKQ_SPDT *, int );		    
void 	ack_notify		( PKQ_SPDT *, ISP_ENTRY *);
void 	reset_ack_notify	( PKQ_SPDT *);
void 	clear_entry		(ISP_ENTRY *entry);
void 	return_atio		(PKQ_SPDT *, ISP_ENTRY *);
void 	send_enable_lun		( PKQ_SPDT *);
void 	wait_enable_lun		( PKQ_SPDT *);
uint32 	pause_risc		( PKQ_SPDT *);
void	release_risc		( PKQ_SPDT *);
unsigned int target_mode	( PKQ_SPDT *);

#if	PKQ_DEBUG

int 	pkq_alloc_trace		( PKQ_SPDT *spdt );
void	pkq_trace_event		(PKQ_SPDT *spdt, uint32 type, uint32 p1, uint32 p2, uint32 p3);
void    print64	 		( char *, int64);
void	pkq_dump_nvram		(PKQ_SPDT * ,ISP_NVRAM * );
void 	print_field		(int , char *, int *);
void	out_value		(int);
char	hex_digit		(int);

void	pkq_trace_entry		(PKQ_SPDT * ,ISP_ENTRY * );

#endif



/*
 *   Name:	driver$init_tables - Initialize driver tables
 *
 *   Abstract:	Complete the initialization of the DPT, DDT, and FDT structures.
 *		If a driver image contains a routine named DRIVER$INIT_TABLES then
 *		this routine is called by the $LOAD_DRIVER service immediately
 *		after the driver image is loaded and before any validity checks are
 *		performed on the DPT, DDT, and FDT.  A prototype version of these
 *		structures is built into this image at link time from the
 *		VMS$VOLATILE_PRIVATE_INTERFACES.OLB library.  Note that the device
 *		related data structures (e.g. DDB, UCB, etc.) have not yet been
 *		created when this routine is called.  Thus the actions of this
 *		routine must be confined to the initialization of the DPT, DDT,
 *		and FDT structures which are contained in the driver image.
 *
 *   Inputs:	None.
 *
 *   Implicit
 *   inputs:	driver$dpt, driver$ddt, driver$fdt
 *		These are the externally defined names for the prototype
 *		DPT, DDT, and FDT structures that are linked into this driver.
 *
 *   Outputs:	None.
 *
 *   Implicit
 *   outputs:	DPT, DDT and FDT fields are initialized to the desired values.
 *
 *   Return
 *   values:	SS$_NORMAL
 *
 */

int driver$init_tables()
{
    DPT				*dpt = &driver$dpt;
    DDT				*ddt = &driver$ddt;

    ini_dpt_name		( dpt, "PKQDRIVER" );
    ini_dpt_flags		( dpt, DPT$M_SMPMOD |
				 DPT$M_SCSI_PORT |
				 DPT$M_NOUNLOAD |
				 DPT$M_SNAPSHOT );
    ini_dpt_adapt		( dpt, AT$_PCI );
    ini_dpt_defunits		( dpt, 1 );
    ini_dpt_ucbsize		( dpt, sizeof( SCSI_UCB ));
    ini_dpt_struc_init		( dpt, pkq_struc_init );
    ini_dpt_struc_reinit	( dpt, pkq_struc_reinit );
    ini_dpt_end			( dpt );

    ini_ddt_start		( ddt, exe_std$kp_startio );
    ini_ddt_kp_startio		( ddt, sa$startio );
    ini_ddt_ctrlinit		( ddt, pkq_ctrl_init );
    ini_ddt_unitinit		( ddt, pkq_unit_init );
/*  ini_ddt_functab		( ddt, sa$functab ); */
    ddt->ddt$ps_fdt_2 = &sa$functab;
    ini_ddt_cancel	     	( ddt, sa$cancelio );
    ini_ddt_kp_stack_size	( ddt, SA$K_KP_STKSIZ );
    ini_ddt_kp_reg_mask		( ddt, SA$K_KP_REGMSK );
    ini_ddt_regdmp		( ddt, pkq_register_dump );
    ini_ddt_diagbf		( ddt, 0 );
    ini_ddt_erlgbf		( ddt, PKQ_ERL_BUFSIZE );
    ini_ddt_end			( ddt );

    return( SS$_NORMAL );
}

/*
 *   Name:	pkq_struc_init - Driver database initialization
 *
 *   Abstract:	Called when the driver is loaded to initialize elements
 *		of the driver database.
 *
 *   Inputs:	CRB address
 *		DDB address
 *		IDB address
 *		ORB address
 *		UCB address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	UCB fields are initialized.
 *
 *   Return
 *   values:	None
 */ 

void pkq_struc_init( CRB *crb,			/* Channel request block			*/
		    DDB *ddb,			/* Device data block				*/
		    IDB *idb,			/* Interrupt dispatch block			*/
		    ORB *orb,			/* Object rights block				*/
		    UCB *ucb )			/* Unit control block				*/
{
    HWRPB	*hwrpb = exe$gpq_hwrpb;		/* Hardware Restart Parameter Block		*/


    /* Set up the device class and type.
     */
    ucb->ucb$b_devclass = DC$_BUS;
    ucb->ucb$b_devtype = DT$_ISP1020;

    /* Set the device characteristics to indicate:
     */
    ucb->ucb$l_devchar = DEV$M_AVL | DEV$M_ELG | DEV$M_IDV | DEV$M_ODV;
    ucb->ucb$l_devchar2 = DEV$M_NNM;

    /* Set the fork lock to IOLOCK8 and set the device IPL
     * based on platform.
     */
    ucb->ucb$b_flck = SPL$C_IOLOCK8;

    ucb->ucb$b_dipl = 21;

    /*
     */
    ucb->ucb$w_devbufsiz = 65535;
    ucb->ucb$l_devsts = UCB$M_NOCNVRT;
}

/*
 *   Name:	struc_reinit - Structure reinitialization routine
 *
 *   Abstract:	This routine is called every time the driver is loaded
 *  		or reloaded to initialize elements of the device database.
 *
 *   Inputs:	CRB address
 *		DDB address
 *		IDB address
 *		ORB address
 *		UCB address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	The structure elements are initialized.
 *
 *   Return
 *   values:	None
 *
 */ 
void pkq_struc_reinit (CRB *crb,		/* Channel request block			*/
		       DDB *ddb,		/* Device data block				*/
		       IDB *idb,		/* Interrupt dispatch block			*/
		       ORB *orb,		/* Object rights block				*/
		       UCB *ucb )		/* Unit control block				*/
{
    /* Store the addresses of the interrupt routines procedure
     * descriptor and entry point in the VEC portion of the CRB.
     */
    ddb->ddb$ps_ddt = &driver$ddt;
    dpt_store_isr( crb, pk$interrupt );
}

/*
 *
 *   Name:	pkq_ctrl_init - Controller initialization entry point
 *
 *   Abstract:  Currently, this routine does nothing except return SS$_NORMAL.
 *		
 *
 *   Inputs:	IDB address
 *
 *   Implicit
 *   inputs:	IPL 31
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL
 *
 */
int pkq_ctrl_init( IDB *idb,				/* Interrupt Data Block pointer		*/
		  DDB *ddb,				/* Device Data Block pointer		*/
		  CRB *crb )				/* Channel Resource Block pointer	*/
{
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pkq_unit_init - Unit initialization entry point
 *
 *   Abstract:	It schedules a routine to run at fork IPL to do
 *		the real work of port initialization.
 *
 *   Inputs:	IDB address
 *		UCB address
 *   Implicit
 *   inputs:	IPL 31
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL
 *
 */
int pkq_unit_init( IDB *idb,			/* Interrupt Data Block pointer			*/
		  SCSI_UCB * ucb )		/* Unit Control Block pointer			*/
{
    FKB		*fkb;				/* Fork Block pointer				*/

    /* Set up IDB fields and schedule the fork routine if this
     * hasn't already been done.
     */
    idb->idb$ps_owner = ( UCB * )ucb;
    idb->idb$ps_auxstruc = ucb;
    if( !_BBSSI( 0, &ucb->ucb$il_pk_exflags )) {
	fkb = ( FKB * )&ucb->ucb$ib_pk_inifkblk;
	fkb->fkb$b_flck = SPL$C_IOLOCK8;
	fork( pkq_unit_init_fork, idb, ucb, fkb );
    }
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pkq_unit_init_fork - Build port structures and init port
 *
 *   Abstract:	This fork routine is scheduled from unit
 *		initialization to allocate port structures
 *		and to initialize the port hardware.
 *
 *   Inputs:	IDB address
 *		UCB address
 *		FKB address
 *
 *   Implicit
 *   inputs:	Fork lock held
 *
 *   Outputs:
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_unit_init_fork( IDB *idb,		/* Interrupt Data Block pointer			*/
		    SCSI_UCB *ucb,		/* Unit Control Block pointer			*/
		    FKB *fkb )			/* Fork Block pointer				*/
{
    HEADER	*head;				/* pointer to use for rspid header*/
    int32	retlen;				/* actual allocation length */
    PKQ_SPDT	*spdt;				/* SCSI Port Descriptor Table			*/
    KPB		*kpb;				/* Kernel Process Block				*/
    CRB		*crb;				/* Channel Request Block			*/
    RTE		*p_rspid_table;			/* pointer to rspid table */

    /* Release the fork routine busy lock. If entry was due to power
     *  failure recovery, get the address of the SPDT from the UCB.
     */
    _BBCCI( 0, &ucb->ucb$il_pk_exflags );
    ucb->ucb$r_erlucb.ucb$r_ucb.ucb$l_fpc = 0;
    if( ucb->ucb$r_scsi_ucb.ucb$l_sts & UCB$M_POWER ) {
	spdt = (PKQ_SPDT *)ucb->ucb$r_scsi_ucb.ucb$l_pdt;

    } else {
/*
**  allocate a rspid_table
*/
	if(ERROR(  exe_std$alononpaged(
				PKQ_NUM_RSPID *(sizeof(RTE))+sizeof(HEADER), 
				&retlen,  
				(void **)&p_rspid_table )))
	{
	    ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_ALORSP ;
	    return;
	}
	else
	{
	    head =  (HEADER *)p_rspid_table;
	    head->size = retlen;
	    head->flink = ucb;
	    p_rspid_table = (RTE *)(++head);
	}
	

	/* Create and initialize the SPDT using sc$rl_create port().
	 */
	sc$rl_create_port( ( UCB * )ucb, sizeof( PKQ_SPDT ), ( SPDT ** )&spdt );

	
   	spdt->basespdt.spdt$iw_max_bus_width = 16;	/* RCL001 	*/
   	spdt->basespdt.spdt$iw_config_bus_width = 16;	/* RCL001 	*/
 
	/* Initialize port-specific data in the SPDT:
	 *  - rspid table address;
	 *  - port type;
	 *  - structure versions;
	 *  - port capability flags;
	 *  - extra mapping registers (2 required for PCI);
	 *  - scsi port ID;
	 *  - maximum byte count;
	 *  - mapping and unmapping routine names.
	 */
	spdt->spdt$ps_rspid_table = p_rspid_table;
	pkq_init_rspid_table( spdt, p_rspid_table );
	spdt->basespdt.spdt$w_spdt_type = SPDT$C_TYPE_PKQ;
	spdt->basespdt.spdt$l_version_check = ( SCDRP$C_VERSION |
				( SPDT$C_VERSION << 8 ) |
				( SCDT$C_VERSION << 16 ));
	spdt->basespdt.spdt$l_port_flags = ( SPDT$M_PFLG_SYNCH |
				   SPDT$M_PFLG_ASYNCH |
				   SPDT$M_PFLG_MAPPING_REG |
				   SPDT$M_PFLG_DIR_DMA |
				   SPDT$M_PFLG_LUNS |
				   SPDT$M_PFLG_CMDQ |
				   SPDT$M_PFLG_PORT_AUTOSENSE |
				   SPDT$M_PFLG_SMART_PORT );
	spdt->basespdt.spdt$is_extmapreg = 2;
	{
	    DDB	*ddb = ucb->ucb$r_scsi_ucb.ucb$l_ddb;
	    spdt->basespdt.spdt$l_scsi_port_id = ddb->ddb$t_name[ 3 ] - 'A';
	}
	spdt->basespdt.spdt$l_maxbytecnt = PKQ_MAXBCNT;
	spdt->basespdt.spdt$ps_cd_buffer_map = pk$map_buffer;
	spdt->basespdt.spdt$ps_cd_buffer_unmap = pk$unmap_buffer;
/*
**	Initialize the inquiry and request sense reply's that will be returned
**	in target mode for those two messages. Also initialize atio message 
**	buffers.
*/
	if ( target_mode( spdt) ) init_target_reply(spdt);

	/* Set up the watchdog timer routine.
	 */
	crb = ucb->ucb$r_scsi_ucb.ucb$l_crb;
	crb->crb$l_scs_struc = ( unsigned int )spdt;
	crb->crb$l_toutrout = pkq_watchdog;
	crb->crb$b_flck = ucb->ucb$r_scsi_ucb.ucb$b_flck;
	crb->crb$l_duetime = -1;

	/* Map the ISP1020 device registers.
	 */
	if( ERROR( pkq_map_registers( spdt ))) {
	    ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_MAPREG;
	    return;
	}
	/* Allocate the ISP1020 request and response queues.
	 */
	if( ERROR( pkq_alloc_queues( spdt ))) {
	    ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_ALOQUE;
	    return;
	}
#if PKQ_DEBUG
	/* Allocate a trace buffer for command tracing
	 */
	if( ERROR( pkq_alloc_trace( spdt ))) {
	    ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_ALOQUE;
	    return;
	}

#endif
	/* Allocate a KPB to do port initialization.
	 */
	if( ERROR( exe$kp_allocate_kpb( &kpb,
				       SA$K_KP_STKSIZ,
				       ( KP$M_IO & ~KP$M_DEALLOC_AT_END ),
				       0 ))) {
	    ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_ALOKPB;
	    return;
	}
	spdt->spdt$ps_kpb = kpb;
	kpb->kpb$ps_ucb = ( UCB * )ucb;
	kpb->kpb$ps_scsi_ptr1 = idb;
    }

    /* For both normal initialization and power fail recovery, start
     * a kernel process to do port hardware initialization.
     */
    spdt->spdt$l_pkq_flags |= SPDT$M_PKQ_UNIT_INIT;
    if( ERROR( exe$kp_start( kpb, pkq_port_init, SA$K_KP_REGMSK ))) {
	ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_KPSTRT;
	return;
    }
/*
**	Setup TQE for timeing out IO's if the fw hangs.
*/
    spdt->basespdt.spdt$ps_rl_timeout_setup_tqe ( spdt );
    spdt->spdt$l_pkq_flags &= ~SPDT$M_PKQ_UNIT_INIT;
}

/*
 *
 *   Name:	pkq_port_init
 *
 *   Abstract:	Perform ISP1020 port hardware initialization or
 *		re-initialization.
 *
 *   Inputs:	kpb			Kernel Process Block
 *		  kpb$ps_ucb		Unit Control Block
 *		  kpb$ps_scsi_ptr1	Interrupt Dispatch Block
 *
 *   Implicit
 *   inputs:	Fork lock held
 *		Kernel process context
 *
 *   Outputs:
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_port_init( KPB *kpb )
{
    DECLARE_ISP();				/* Declare ISP register file and temps		*/
    SCSI_UCB			*ucb;		/* Unit Control Block w/SCSI extensions		*/
    PKQ_SPDT			*spdt;		/* SCSI Port Descriptor Table			*/
    CRB				*crb;		/* CRB 						*/
    /*
     *  1. Reset the ISP1020 port;
     *	2. Load and start the ISP firmware;
     *	3. Read parameters out of the NVRAM;
     *  4. Load parameters into the ISP and initialize
     *     the ISP request and response queues;
     *  5. Reset the SCSI Bus;
     *  6. Mark UCB and SPDT online;
     *  7. Enable interrupts.
     */
    ucb = ( SCSI_UCB * )kpb->kpb$ps_ucb;
    crb = ucb->ucb$r_scsi_ucb.ucb$l_crb;
    spdt = ( PKQ_SPDT *)ucb->ucb$r_scsi_ucb.ucb$l_pdt;
    if( SUCCESS( pkq_reset( spdt )) &&
       SUCCESS( pkq_select_firmware( spdt )) &&
       SUCCESS( pkq_set_parameters( spdt )) &&
       SUCCESS( pkq_reset_scsi_bus( spdt ))) {
	ucb->ucb$r_scsi_ucb.ucb$l_sts |= UCB$M_ONLINE;
	spdt->basespdt.spdt$l_sts |= SPDT$M_STS_ONLINE;
	ioc$node_function( crb, IOC$K_ENABLE_INTR );
	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_hccr, ISP$K_HCCR_CLR_RISC_INT ); 
	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_bus_semaphore, 0 );
	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_bus_icr, ( ISP$M_ICR_ALL_INT_ENB | ISP$M_ICR_RISC_INT_ENB ));
	pkq_print_message( spdt, "initialization complete; port online",
			  STS$K_INFO );
    } else {
	pkq_print_message( spdt, "initialization error; port marked offline",
			  STS$K_WARNING );
    }
}

/*
 *
 *   Name:	pkq_map_registers - Map ISP1020 registers
 *
 *   Abstract:	Map the ISP1020 registers into PCI memory space and
 *		associate them with a handle that can be used in
 *		later I/O calls.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	
 *
 *   Implicit
 *   outputs:	spdt$q_mem_base
 *		spdt$q_iohandle_reg
 *		spdt$q_direct_dma_base
 *		spdt$l_direct_dma_size
 *
 *   Return
 *   Values:	SS$_NORMAL 	routine completed normally
 *		Other		errors returned from called routines
 *
 */
int pkq_map_registers( PKQ_SPDT *spdt )		/* SCSI port descriptor table			*/
{
    ADP				*adp = spdt->basespdt.spdt$l_adp;
    SCSI_UCB			*ucb = ( SCSI_UCB * )spdt->basespdt.spdt$l_port_ucb;
    CRB				*crb = ucb->ucb$r_scsi_ucb.ucb$l_crb;
    int				status;
    /*
     *  - Get the PCI base address of memory space using the node
     *    number in the CRB;
     *	- 'AND' off the irrelevant low order bits;
     *	- Map the ISP1020 registers, returning a handle that is stored
     *    in the SPDT and can be used in subsequent ioc$read_io and
     *    ioc$write_io function calls.
     *  - Get the start and size of the direct DMA window (converting the
     *    size from megabytes (?!) to bytes) and store them in the SPDT
     *	  for subsequent use in buffer mapping.
     */
    if( ERROR( status = ioc$read_pci_config( adp,
					    crb->crb$l_node,
					    offsetof( PCI, pci$l_base_address_1 ),
					    sizeof( uint32 ),
					    (int *) &spdt->spdt$q_mem_base )))
	return( status );
    spdt->spdt$q_mem_base &= PCI$M_BASE_ADDRESS_BITS_31_4;
    if( ERROR( status = ioc$map_io( adp,
				   crb->crb$l_node,
				   &spdt->spdt$q_mem_base,
				   sizeof( ISP_REGISTERS ),
				   IOC$K_BUS_MEM_BYTE_GRAN,
				   &spdt->spdt$q_iohandle_reg )))
	return( status );
    if( ERROR( status = ioc$node_data( crb,
				      IOC$K_DIRECT_DMA_BASE,
				      &spdt->spdt$q_direct_dma_base )))
	      return( status );
/*
 *	Since this driver uses 32 bit addresses to access memory we need to 
 *	ensure that the direct dma mapped space is in the 1'st 4gig of physical
 *	address space. 
 */

    if( (spdt->spdt$q_direct_dma_base & 0xffffffff00000000) != 0 ) 
								 pkq_bugcheck();
 
    if( ERROR( status = ioc$node_data( crb,
				      IOC$K_DIRECT_DMA_SIZE,
				      &spdt->spdt$l_direct_dma_size )))
	return( status );
    spdt->spdt$l_direct_dma_size <<= 20;
/*
 *	Make sure that it the direct dma map starts in the 1'st gig then it ends
 *	there as well.
 */
    if( (( spdt->spdt$q_direct_dma_base + spdt->spdt$l_direct_dma_size ) & 
						      0xffffffff00000000) != 0 ) 
	pkq_bugcheck();

    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pkq_alloc_queues - allocate ISP1020 queue space
 *
 *   Abstract:	Allocate ISP1020 request and response queue rings and initialize
 *		the SPDT cells that manage them.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	spdt$ps_requestq_va
 *		spdt$ps_responseq_va
 *		spdt$ps_requestq_pa
 *		spdt$ps_responseq_pa
 *		spdt$ps_requestq_in
 *		spdt$ps_responseq_out
 *
 *   Return
 *   Values:	SS$_NORMAL		routine completed successfully
 *		other			errors returned from called routines
 *
 */
int pkq_alloc_queues( PKQ_SPDT *spdt )
{
    uint32	rqlen;					/* Requested length in bytes		*/
    uint32	rqlip;					/* Requested length in pages		*/
    uint32	alolen;					/* Allocated length in bytes		*/
    ISP_ENTRY	*alobuf;				/* Address of allocated buffer		*/
    int		status;					/* VMS status				*/
#define REQUEST_QUEUE_SIZE	64			/* Size of ISP1020 request queue	*/
#define RESPONSE_QUEUE_SIZE	64			/* Size of ISP1020 response queue	*/


/*
 * 1.  Calculate the total length required to contain the ISP request and
 *     response queues and convert to pages;
 * 2.  Allocate physically contiguous space for the queues, returning errors
 *     to our caller;
 * 3.  Store the base virtual addresses for the request and response queues;
 * 4.  Calculate and store the base physical addresses for the request and
 *     response queues.
 * 5.  Initialize the request queue in and response queue out indices.
 */
    rqlen = ( sizeof( ISP_ENTRY) * ( REQUEST_QUEUE_SIZE + RESPONSE_QUEUE_SIZE )) +
	mmg$gl_bwp_mask;
    rqlip = rqlen >> mmg$gl_vpn_to_va;
    if( ERROR( status = spdt->basespdt.spdt$ps_rl_pool_alloc_physical( spdt,
								       rqlip,
								       &alolen,
								       &alobuf )))
	return( status );
    spdt->spdt$ps_requestq_va = alobuf;
    spdt->spdt$ps_responseq_va = alobuf +  REQUEST_QUEUE_SIZE;
    spdt->spdt$ps_requestq_pa = ioc$sva_to_pa( spdt->spdt$ps_requestq_va, 0, 0, 0 ) +
	spdt->spdt$q_direct_dma_base;
    spdt->spdt$ps_responseq_pa = ioc$sva_to_pa( spdt->spdt$ps_responseq_va, 0, 0, 0 ) +
	spdt->spdt$q_direct_dma_base;
    spdt->spdt$l_requestq_in = 0;
    spdt->spdt$l_responseq_out = 0;
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pkq_reset - Reset the ISP1020
 *
 *   Abstract:	Reset the ISP1020 RISC processor to put it in a known
 *		initial state.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL		reset succeeded
 *		SS$_NOSUCHDEV		reset failed
 *		other			errors from timed wait
 *					(reset failed)
 *
 */
int pkq_reset( PKQ_SPDT *spdt )
{
    DECLARE_ISP();
    ADP			*adp = spdt->basespdt.spdt$l_adp;
    uint64		*handle = &spdt->spdt$q_iohandle_reg;
    int			status;
    int64		interval;			/* Interval in ns being timed */
    int64		due_time;			/* Time event becomes due */

    /* Reset the RISC processor by performing the following steps:
     *	1. Soft reset the ISP (dunno why );
     *	2. Hard reset the ISP which puts the ISP in reset mode, clearing
     *     "all" control and status bits;
     *  3. Release the ISP from reset mode;
     *  4. Clear the BIOS enable bit to allow the address bits to access
     *	   external RAM;
     *  5. Clear the burst mode bits in the config 1 register which were
     *	   missed by the hard reset;
     *	6. Wait for mailbox register 0 to clear, indicating that the reset
     *     is complete;
     *  7. Read mailbox registers 1 - 4 and compare them to the expected
     *	   reset values, returning success if they match or failure if not.
     */
    WRITE_ISP( adp, handle, isp$w_bus_icr, ISP$M_ICR_SOFT_RESET );
    WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_RESET ); 
    WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_RELEASE );
    WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_WRITE_BIOS );
    WRITE_ISP( adp, handle, isp$w_bus_config_1, 0 );
    interval = 10000;
    if( READ_ISP( adp, handle, isp$w_mailbox_0 ) != 0 ) {
	if( ERROR( status = exe$timedwait_setup( &interval, &due_time )))
	    return( status );
	while(( status = exe$timedwait_complete( &due_time )) == SS$_CONTINUE ) {
	    if( READ_ISP( adp, handle, isp$w_mailbox_0 ) == 0 ) {
		status == SS$_NORMAL;
		break;
	    }
	}
	if( ERROR (status ))
	    return( status );
    }
    if(( READ_ISP( adp, handle, isp$w_mailbox_1 ) == 0x4953 ) &&
       ( READ_ISP( adp, handle, isp$w_mailbox_2 ) == 0x5020 ) &&
       ( READ_ISP( adp, handle, isp$w_mailbox_3 ) == 0x2020 ) &&
       ( READ_ISP( adp, handle, isp$w_mailbox_4 ) == 0x0001 )) 
	return( SS$_NORMAL );
    else
	return( SS$_NOSUCHDEV );
}

/*
 *
 *   Name:	pkq_select_firmware - select operational ISP1020 firmware
 *
 *   Abstract:	Select the version of firmware to be used in operation:
 *		dump the ISP1020 firmware that was loaded by the console
 *		into a page-aligned buffer.  Extract the version numbers
 *		from the dumped and linked copies of the firmware and
 *		set up the SPDT to use the newer of the two unless the
 *		PKQ_USE_LINKED_FIRMWARE	symbol is set to 1.  If the
 *		linked version is used, deallocate the buffer that contains
 *		the dumped firmware.  Finally, load and start the copy of
 *		the firmware whose address is in the SPDT.
 *		
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL		firmware dumped successfully
 *		SS$_TIMEOUT		mailbox command timed out (load failed)
 *
 */
int pkq_select_firmware( PKQ_SPDT *spdt )
{
    uint16	*console_fw;			/* Console firmware pointer			*/
    int		console_fw_length;		/* Console firmware length			*/
    int		console_fw_version;		/* Console firmware version			*/
    int		severity;			/* Error message severity			*/
    /* If the firmware hasn't been set up, select and load it now.
     *  -  Point the firmware virtual address at the linked firmware;
     *  -  Get the version number of the linked firmware into the SPDT;
     *  -  Reset the "firmware-from-console" indicator;
     *  -  Dump the firmware loaded by the console into non-paged pool;
     *  -  Get the version number of the linked firmware;
     *  -  If the console firmware version is higher than the linked version:
     *      o  Point the firmware virtual address at the dumped firmware;
     *      o  Get the version number of the dumped firmware into the SPDT;
     *      o  Set the "firmware-from-console" indicator.
     *  -  Otherwise, deallocate the pool containing the dumped firmware;
     *  -  Print the firmware version number;
     *  -  Load the firmware pointed to by the firmware virtual address
     *     in the SPDT.
     */

    severity = STS$K_INFO;
    if( spdt->spdt$ps_firmware_va == 0 ) {
	spdt->spdt$ps_firmware_va = risc_code01;
	spdt->spdt$l_firmware_version = pkq_get_firmware_ver( risc_code01 );
	spdt->spdt$l_pkq_flags &= ~SPDT$M_PKQ_CONSOLE_FW;
	if( PKQ_USE_LINKED_FIRMWARE == FALSE ) {
	    if( SUCCESS( pkq_dump_firmware( spdt, &console_fw, &console_fw_length ))) {
		console_fw_version = pkq_get_firmware_ver( console_fw );
		if( spdt->spdt$l_firmware_version < console_fw_version ) {
		    spdt->spdt$ps_firmware_va = console_fw;
		    spdt->spdt$l_firmware_version = console_fw_version;
		    spdt->spdt$l_pkq_flags |= SPDT$M_PKQ_CONSOLE_FW;
		} else {
		    spdt->basespdt.spdt$ps_rl_pool_dealloc( spdt, console_fw, console_fw_length );
		}
	    } else {
		severity = STS$K_WARNING;
		pkq_print_message( spdt, "console firmware checksum error", severity );
	    }
	}
    }
    pkq_print_firmware_ver( spdt, severity );
    return( pkq_load_firmware( spdt ));
}

/*
 *
 *   Name:	pkq_dump_firmware - dump the current firmware into memory
 *
 *   Abstract:	Allocate a  memory buffer and dump the contents of
 *		the ISP1020 RAM into it.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL		firmware dumped successfully
 *		SS$_TIMEOUT		mailbox command timed out (load failed)
 *
 */
int pkq_dump_firmware( PKQ_SPDT *spdt, uint16 **console_fw, int *console_fw_length )
{
    uint8	*buf;				/* SVA of the firmware buffer			*/
    int		len;				/* Length of the firmware buffer (bytes)	*/
    uint64	pa;				/* Physical address of the buffer		*/
    uint64	pcia;				/* PCI address of the buffer			*/
    int32	size_remaining;			/* Size that remains to be loaded		*/
    int32	size_in_segment;		/* Part of buffer in current contiguous segment	*/
    uint8	*cur_sva;			/* Current buffer SVA during load		*/
    uint16	fw_length;			/* Firmware length in 16-bit words		*/
    uint16	length_offset;			/* Offset of fw length within firmware image	*/
    uint16	cur_risc_addr;			/* RISC-space address of current load		*/
    uint16	checksum;			/* Driver-computed checksum			*/
    int		retries;			/* Number of times to try dump operation	*/
    int		status;				/* VMS status					*/

    /* Dump the contents of ISP1020 RAM into memory:
     *   -  Verify the checksum of the RAM load;
     *   -  Get the length of the firmware;
     *	 -  Allocate a buffer;
     *   -  While data remains to be dumped:
     *	       	o Convert the source virtual address to be a physical address and determine the
     *		  amount of data (bytes) contained in the current page;
     *   	o Convert the physical address to be a PCI address;
     *	 	o Invoke a mailbox command to dump the portion of the firmware contained in the
     *       	  current page;
     *   	o Update the amount remaining to be dumped, the source address and the
     *	          destination address;
     *   -  Return the firmware buffer address and length to the caller.
     */

    length_offset = risc_code_addr01 + ( offsetof( ISP_FW_HDR, firmware_length ) / 2 );
    *console_fw = 0;
    *console_fw_length = 0;
    
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_VERIFY_CHECKSUM, risc_code_addr01, 0, 0, 0, 0 ))) {
	return( status );
    }
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_READ_RAM_WORD, length_offset, 0, 0, 0, 0 ))) {
	return( status );
    }	

    /* Allocate non-paged pool for the firmware.
     */
    fw_length = spdt->spdt$w_mailbox_2;
    if( ERROR( status = spdt->basespdt.spdt$ps_rl_pool_alloc( spdt,
							     ( fw_length * 2 ),
							     &len,
							     &buf ))) {
	return( status );
    }

    /* Dump the firmware into the buffer just allocated, then checksum the result;
     * if the checksum fails, try again until the dump succeeds or until retries
     * are exhausted.
     */
    retries = 5;
    do {
	cur_sva = buf;
	cur_risc_addr = risc_code_addr01;
	size_remaining = fw_length * 2;

	/* While there is data left to dump, calculate the physical address and size
	 * of the data contained in the next physically contiguous segment, then dump it.
	 */
	while( size_remaining > 0 ) {
	    pa = ioc$sva_to_pa( cur_sva, 0, size_remaining, &size_in_segment );
	    pcia = pa + spdt->spdt$q_direct_dma_base;

	    /* If this segment of the dump succeeded, update the parameters to
	     * get to the next segment.
	     */
/*
**
** There has been some problems with this command so it's being replaced with
** a routine that reads ram a word at a time.
**
**	    if( SUCCESS( status = pkq_mailbox_io( spdt, 5,
**					       ISP$K_MB_DUMP_RAM, cur_risc_addr,
**					       (( pcia >> 16 ) & 0xffff ), ( pcia & 0xffff ),
**					       size_in_segment / 2, 0 ))) {
**
*/
	    if( SUCCESS( status = dump_ram( spdt, 
				(uint16 *)cur_sva, 
				cur_risc_addr, 
				size_in_segment/2))) {

	        cur_sva += size_in_segment;
	        cur_risc_addr += ( size_in_segment / 2 );
	        size_remaining -= size_in_segment;

	    /* If the DUMP_RAM command failed, don't attempt to dump additional
	     * segments during this iteration of the do while loop.
	     */
	    } else {
		break;
	    }
	}

	/* If the dump command succeeded, compute the firmware checksum.
	 */
	if( SUCCESS( status )) {
	    int		i;			/* Loop counter					*/
	    uint16	*fwp;			/* Firmware pointer				*/

	    checksum = 0;
	    fwp = ( uint16 * )buf;

	    for( i = 0; i < fw_length; i++ ) {
		checksum += fwp[ i ];
	    }
	}
    } while( checksum &&  --retries );

    /* If the checksum failed, set SS$_BADPARAM status.
     */
    if( checksum )  {
	status = SS$_BADPARAM;
    }	

    /* If either the dump or the checksum failed, deallocate the firmware buffer.
     */
    if( ERROR( status ))  {
	spdt->basespdt.spdt$ps_rl_pool_dealloc( spdt, buf, len );

    /* Both the dump and checksum succeeded; set the address and length of
     * the firmware buffer.
     */
    } else {
	*console_fw = ( uint16 *)buf;
	*console_fw_length = len;
    }

    return( status );
}

/*
 *
 *   Name:	pkq_load_firmware - load the ISP1020 firmware
 *
 *   Abstract:	Load the ISP1020 firmware and start it up.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL		firmware loaded/started successfully
 *		SS$_TIMEOUT		mailbox command timed out (load failed)
 *
 */
int pkq_load_firmware( PKQ_SPDT *spdt)			/* SCSI PDT with driver extensions	*/
{
    uint8	*sva;					/* SVA of the firmware buffer		*/
    uint64	pa;					/* Physical address of the buffer	*/
    uint64	pcia;					/* PCI address of the buffer		*/
    int32	size_remaining;				/* Size that remains to be loaded	*/
    int32	size_in_page;				/* Part of buffer in current page	*/
    uint16	cur_risc_addr;				/* RISC-space address of current load	*/
    int		status;					/* VMS status				*/

    /* Load and start the firmware given its address in the SPDT.
     *   -  Initialize the source (system virtual) address, the destination (RISC) address,
     *      and the total size in 16-bit words remaining to be loaded;
     *   -  While data remains to be loaded:
     *	       	o Convert the source virtual address to be a physical address and determine the
     *		  amount of data (bytes) contained in the current page;
     *   	o Convert the physical address to be a PCI address;
     *	 	o Invoke a mailbox command to load the portion of the firmware contained in the
     *       	  current page;
     *   	o Update the amount remaining to be loaded, the source address and the
     *	          destination address;
     *	 -  Verify the checksum of the firmware just loaded;
     *   -  Start the firmware.
     */

    sva = ( uint8 * )spdt->spdt$ps_firmware_va;
    cur_risc_addr = risc_code_addr01;
    size_remaining = ( *( uint16 * )( sva + offsetof( ISP_FW_HDR, firmware_length ))) * 2;
    while( size_remaining > 0 ) {
	pa = ioc$sva_to_pa( sva, 0, size_remaining, &size_in_page );
	pcia = pa + spdt->spdt$q_direct_dma_base;
	if( ERROR( status = pkq_mailbox_io( spdt, 5,
					   ISP$K_MB_LOAD_RAM, cur_risc_addr,
					   (( pcia >> 16 ) & 0xffff ), ( pcia & 0xffff ),
					   size_in_page / 2, 0 ))) {
	    return( status );
	}
	sva += size_in_page;
	cur_risc_addr += ( size_in_page / 2 );
	size_remaining -= size_in_page;
    }
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_VERIFY_CHECKSUM, risc_code_addr01, 0, 0, 0, 0 ))) {
	return( status );
    }
    if( ERROR( status = pkq_mailbox_io( spdt, 2, 
				       ISP$K_MB_EXEC_FIRMWARE, risc_code_addr01, 0, 0, 0, 0 ))) {
	return( status );
    }
    return( SS$_NORMAL );
}


/*
 *
 *   Name:	pkq_get_firmware_ver - get version from firmware image
 *
 *   Abstract:	Locate the copyright test imbedded within the firmware
 *		image, then locate the version number at the end of the
 *		copyright and convert it to binary.  Return the binary
 *		version number to the caller.
 *
 *   Inputs:	Address of the firmware buffer.
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	Firmware version number
 *
 */
int pkq_get_firmware_ver( uint16 *firmware )
{
    ISP_FW_HDR	*hdr;
    uint16	*copyright;
    int		copyright_length;
    char	string[ 128 ], *sp, *subp;
    char	temp;
    int		version;

    /* Locate the beginning of the copyright notice as the first byte
     * beyond the fixed firmware header.  Then calculate the length of
     * the copyright notice in words as the difference between the target
     * of the jump instruction and the end of the fixed header.
     * Pretty funky.
     */
    hdr = ( ISP_FW_HDR * )firmware;
    copyright = ( uint16 * )(( char * )firmware + sizeof( ISP_FW_HDR ));
    copyright_length = hdr->jmpaddr - risc_code_addr01 - ( sizeof( ISP_FW_HDR ) / 2 );

    /* Copy the copyright notice into a local string buffer, byte-swapping,
     * down-casing and converting nulls to blanks as we go.  Finally, null
     * terminate the string.
     */
    sp = string;
    while( copyright_length ) {
	temp = (( *copyright >> 8 ) & 0xff );
	*sp++ = temp ? ( isupper( temp ) ? temp | 0x20 : temp ) : ' '; 
	temp = ( *copyright++ & 0xff );
	*sp++ = temp ? ( isupper( temp ) ? temp | 0x20 : temp ) : ' '; 
	--copyright_length;
    }
    *sp = 0;

    /* Scan the local string for the substring "version ", and bump by
     * it to find the version number string.  Convert the decimal
     * version number to binary, ignoring the decimal point but
     * terminating on any other non-digit character.
     */
    subp = strstr( string, "version " );
    subp += sizeof( "version " ) - 1;
    version = 0;
    while( *subp ) {
	temp = *subp++;
	if( isdigit( temp )) {
	    version = ( 10 * version ) + temp - '0';
	} else if( temp != '.' ) {
	    break;
	}
    }

    /* Return the binary version number.  YOW!
     */
    return( version );
}


/*
 *
 *   Name:	pkq_set parameters - set ISP1020 parameters
 *
 *   Abstract:	Set the ISP1020 operational parameters based on 
 *		values stored in the NVRAM or on supplied defaults.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	ISP1020 parameters are set up in the firmware
 *
 *   Return
 *   Values:	SS$_NORMAL		parameters were set
 *		SS$_TIMEOUT		error occurred in a mailbox command
 *
 */
int pkq_set_parameters( PKQ_SPDT *spdt )
{
    DECLARE_ISP();					/* Declare ISP register file and temps	*/
    ADP			*adp;				/* Adaptor control block		*/
    uint64		*handle;			/* ISP register I/O handle		*/
    int			status;				/* VMS status				*/
    int			scsi_id;			/* SCSI (target) ID used in loop	*/

    /* Get the operational parameters from the NVRAM or defaults.
     */
    pkq_get_parameters( spdt );
    
    /* Set the initiator's SCSI ID mask and number based on the NVRAM value.
     */
    spdt->basespdt.spdt$l_scsi_bus_id = ( 1 << spdt->spdt$w_initiator_scsi_id );
    spdt->basespdt.spdt$is_scsi_id_num = spdt->spdt$w_initiator_scsi_id;

    /* Set up the ADP pointer and I/O handle.
     */
    adp = spdt->basespdt.spdt$l_adp;
    handle = &spdt->spdt$q_iohandle_reg;

    /* Set the FIFO threshhold.
     */
    WRITE_ISP( adp, handle, isp$w_bus_config_1,
	      ( spdt->spdt$v_fifo_threshhold | ISP$M_CONFIG_1_BURST_ENABLE ));

    /* Set Data and CMD channel burst.  Must precede queue setup.
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 3,
				       ISP$K_MB_SET_PCI_CONTROL_PARAMS,
				       ( spdt->spdt$v_data_dma_burst_enable << 1 ),
				       ( spdt->spdt$v_command_dma_burst_enable << 1 ),
				       0, 0, 0 ))) {
	return( status );
    }
    /* Initialize request queue
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 5,
			      ISP$K_MB_INIT_REQUEST_Q,
			      REQUEST_QUEUE_SIZE,
			      ( spdt->spdt$ps_requestq_pa >> 16 ) & 0xFFFF,
			      spdt->spdt$ps_requestq_pa & 0xFFFF,
			      spdt->spdt$l_requestq_in,
			      0 ))) {
	return( status );
    }
    /* Initialize response queue
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 6,
				       ISP$K_MB_INIT_RESPONSE_Q,
				       RESPONSE_QUEUE_SIZE,
				       ( spdt->spdt$ps_responseq_pa >> 16 ) & 0xFFFF,
				       spdt->spdt$ps_responseq_pa & 0xFFFF,
				       0,
				       spdt->spdt$l_responseq_out ))) {
	return( status );
    }

    /* Set the SCSI ID
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_INITIATOR_SCSI_ID,
				       spdt->spdt$w_initiator_scsi_id,
				       0, 0, 0, 0 ))) {
	return( status );
    }
    /* Set the Selection Timeout
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_SELECTION_TIMEOUT,
				       spdt->spdt$w_selection_timeout,
				       0, 0, 0, 0 ))) {
	return( status );
    }
    /* Set retry count and delay
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 3,
				       ISP$K_MB_SET_RETRY_COUNT,
				       spdt->spdt$w_retry_count,
				       spdt->spdt$w_retry_delay,
				       0, 0, 0 ))) {
	return( status );
    }
    /* Set active negation for REQ/ACK and data.
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_ACTIVE_NEGATION,
				       (( spdt->spdt$v_req_ack_active_negation << 5 ) |
					( spdt->spdt$v_data_active_negation << 4 )),
				       0, 0, 0, 0 ))) {
	return( status );
    }
    /* Set tag age limit
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_TAG_AGE_LIMIT,
				       spdt->spdt$w_tag_age_limit,
				       0, 0, 0, 0 ))) {
	return( status );
    }
    
    /* Set clock speed  (plus adjust spdt$v_asynch_setup_time if necessary)
     */
    if( target_mode( spdt ))
	{
	if( ERROR( status = pkq_set_speed( spdt )))
	    {
	    return( status );
	    }
	}

    /* Set asynchronous data setup time
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_ASYNCH_SETUP_TIME,
				       spdt->spdt$v_asynch_setup_time,
				       0, 0, 0, 0 ))) {
	return( status );
    }
    /* Set target parameters
     */
    for( scsi_id = 0; scsi_id < 16; scsi_id++ ) {
	if( ERROR( status = pkq_mailbox_io( spdt, 4,
					   ISP$K_MB_SET_TARGET_PARAMETERS,
					   ( scsi_id << 8 ),
					   spdt->spdt$r_tparams[ scsi_id].spdt$w_capabilities,
					   spdt->spdt$r_tparams[ scsi_id].spdt$w_synch_params,
					   0, 0 )))
	    return( status );
    }
    /* Set overrun recovery mode
     */
    if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_OVERRUN_RECOVERY_MODE,
				       ISP$K_MB1_INTERRUPT_WITHOUT_RESET,
				       0, 0, 0, 0 ))) {
	return( status );
    }
    if( target_mode( spdt ) )
    {
        /* Enable Target Mode
         */
    	if( ERROR( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_ENABLE_TARGET,
				       ISP$K_MB1_ENABLE_TARGET,
				       0, 0, 0, 0 ))) {
		return( status );
    	}
    }

    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pkq_get_parameters - read ISP parameters from NVRAM
 *
 *   Abstract:	Read the parameters from the NVRAM and store them in
 *		the driver-specific portion of the SPDT.  If the
 *		contents of the NVRAM are invalid, supply defaults.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	ISP parameters are stored in the SPDT
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_get_parameters( PKQ_SPDT *spdt )
{
    int			scsi_id;
    ISP_NVRAM		isp_nvram, *nvram;

    /* If the parameters have already been read from the NVRAM, exit.
     */
    if( spdt->spdt$l_pkq_flags & SPDT$M_PKQ_PARAMS_READ ) {
	return;
    }
    /* Read the entire NVRAM.  If the NVRAM read succeeds (iff NVRAM contents
     * have checksummed correctly and passed other sanity checks), store the
     * NVRAM parameter values in the SPDT.
     */
    nvram = &isp_nvram;
    if(( PKQ_USE_NVRAM_DEFAULTS == FALSE ) && 
       SUCCESS( pkq_nvram_read_array( spdt, nvram ))) {
#if PKQ_DEBUG
/*	pkq_dump_nvram(spdt, nvram);   */
#endif

	/* Store the adaptor-wide parameters in the SPDT.
	**
	** Note that fifo threshold is minimized to 0 here and sync offset is
	** minimized to 8 for data integrety reasons.
	**
	*/
 	spdt->spdt$w_initiator_scsi_id = nvram->isp$v_nvr_initiator_scsi_id;
	spdt->spdt$w_bus_reset_delay = nvram->isp$b_nvr_bus_reset_delay;
	spdt->spdt$w_retry_count = nvram->isp$b_nvr_retry_count;
	spdt->spdt$w_retry_delay = nvram->isp$b_nvr_retry_delay;
	spdt->spdt$w_tag_age_limit = nvram->isp$b_nvr_tag_age_limit;
	spdt->spdt$w_selection_timeout = nvram->isp$w_nvr_selection_timeout;
	spdt->spdt$w_max_queue_depth = nvram->isp$w_nvr_max_queue_depth;
	spdt->spdt$v_fifo_threshhold = min(0,nvram->isp$v_nvr_fifo_threshhold);
	spdt->spdt$v_adaptor_enable = nvram->isp$v_nvr_adaptor_enable;
	spdt->spdt$v_asynch_setup_time = nvram->isp$v_nvr_asynch_setup_time;
	spdt->spdt$v_req_ack_active_negation = nvram->isp$v_nvr_req_ack_active_negation;
	spdt->spdt$v_data_active_negation = nvram->isp$v_nvr_data_active_negation;
	spdt->spdt$v_data_dma_burst_enable = nvram->isp$v_nvr_data_dma_burst_enable;
	spdt->spdt$v_command_dma_burst_enable = nvram->isp$v_nvr_command_dma_burst_enable;
	spdt->spdt$v_termination_low_enable = nvram->isp$v_nvr_termination_low_enable;
	spdt->spdt$v_termination_high_enable = nvram->isp$v_nvr_termination_high_enable;
	spdt->spdt$v_pcmc_burst_enable = nvram->isp$v_nvr_pcmc_burst_enable;
	spdt->spdt$v_60mhz_enable = nvram->isp$v_nvr_60mhz_enable;

	/* Store the per-target parameters in the SPDT
	 */
	for( scsi_id = 0; scsi_id < 16; scsi_id++ ) {
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_renegotiate_on_error =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_renegotiate_on_error;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_stop_queue_on_check =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_stop_queue_on_check;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_auto_request_sense =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_auto_request_sense;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_tagged_queuing =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_tagged_queuing;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_synch_data_transfers =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_synch_data_transfers;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_wide_data_transfers =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_wide_data_transfers;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_parity_checking =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_parity_checking;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_disconnect_allowed =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_disconnect_allowed;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$b_synch_period =
		nvram->isp$r_nvr_target[ scsi_id ].isp$b_nvr_synch_period;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$b_synch_offset =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_synch_offset;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$w_execution_throttle =
		nvram->isp$r_nvr_target[ scsi_id ].isp$b_nvr_execution_throttle;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_device_enable =
		nvram->isp$r_nvr_target[ scsi_id ].isp$v_nvr_device_enable;
	}
    /* The NVRAM read failed for some reason.  Use the default parameter settings.
     */
    } else {
 	spdt->spdt$w_initiator_scsi_id = ISP$K_NVR_INITIATOR_SCSI_ID;
	spdt->spdt$w_bus_reset_delay = ISP$K_NVR_BUS_RESET_DELAY;
	spdt->spdt$w_retry_count = ISP$K_NVR_RETRY_COUNT;
	spdt->spdt$w_retry_delay = ISP$K_NVR_RETRY_DELAY;
	spdt->spdt$w_tag_age_limit = ISP$K_NVR_TAG_AGE_LIMIT;
	spdt->spdt$w_selection_timeout = ISP$K_NVR_SELECTION_TIMEOUT;
	spdt->spdt$w_max_queue_depth = ISP$K_NVR_MAX_QUEUE_DEPTH;
	spdt->spdt$v_fifo_threshhold = ISP$K_NVR_FIFO_THRESHHOLD;
	spdt->spdt$v_adaptor_enable = ISP$K_NVR_ADAPTOR_ENABLE;
	spdt->spdt$v_asynch_setup_time = ISP$K_NVR_ASYNCH_SETUP_TIME;
	spdt->spdt$v_req_ack_active_negation = ISP$K_NVR_REQ_ACK_ACTIVE_NEGATION;
	spdt->spdt$v_data_active_negation =  ISP$K_NVR_DATA_ACTIVE_NEGATION;
	spdt->spdt$v_data_dma_burst_enable = ISP$K_NVR_DATA_DMA_BURST_ENABLE;
	spdt->spdt$v_command_dma_burst_enable = ISP$K_NVR_COMMAND_DMA_BURST_ENABLE;
	spdt->spdt$v_termination_low_enable = ISP$K_NVR_TERMINATION_LOW_ENABLE;
	spdt->spdt$v_termination_high_enable = ISP$K_NVR_TERMINATION_HIGH_ENABLE;
	spdt->spdt$v_pcmc_burst_enable = ISP$K_NVR_PCMC_BURST_ENABLE;
	spdt->spdt$v_60mhz_enable = 0 ;
	for( scsi_id = 0; scsi_id < 16; scsi_id++ ) {
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_renegotiate_on_error =
		ISP$K_NVR_RENEGOTIATE_ON_ERROR;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_stop_queue_on_check =
		ISP$K_NVR_STOP_QUEUE_ON_CHECK;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_auto_request_sense =
		ISP$K_NVR_AUTO_REQUEST_SENSE;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_tagged_queuing =
		ISP$K_NVR_TAGGED_QUEUING;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_synch_data_transfers =
		ISP$K_NVR_SYNCH_DATA_TRANSFERS;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_wide_data_transfers =
		ISP$K_NVR_WIDE_DATA_TRANSFERS;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_parity_checking =
		ISP$K_NVR_PARITY_CHECKING;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_disconnect_allowed =
		ISP$K_NVR_DISCONNECT_ALLOWED;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$b_synch_period =
		ISP$K_NVR_SYNCH_PERIOD;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$b_synch_offset =
		min(8,ISP$K_NVR_SYNCH_OFFSET);
	    spdt->spdt$r_tparams[ scsi_id ].spdt$w_execution_throttle =
		ISP$K_NVR_EXECUTION_THROTTLE;
	    spdt->spdt$r_tparams[ scsi_id ].spdt$v_device_enable =
		ISP$K_NVR_DEVICE_ENABLE;
	}
    }
}

/*
 *
 *   Name:	pkq_reset_scsi_bus - reset the SCSI bus
 *
 *   Abstract:	Call the common SCSI bus reset code as if we were a class
 *		driver in order to provide correct synchronization.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	The bus is reset.
 *
 *   Return
 *   Values:	SS$_NORMAL		Bus reset succeeded
 *
 */
int pkq_reset_scsi_bus( PKQ_SPDT *spdt )
{
    SCDRP	local_scdrp, *scdrp;		/* Fabricate a SCDRP and a pointer thereto	*/
    int		status;				/* VMS status					*/

    /* Fabricate and initialize an SCDRP on the KP stack, then call
     * the class drivers' reset SCSI bus routine which will make sure
     * that the bus reset is synchronized with other activity.
     */
    scdrp = &local_scdrp;
    memset( scdrp, 0, sizeof( SCDRP )); 
    scdrp->scdrp$w_scdrpsize = SCDRP$K_LENGTH;
    scdrp->scdrp$b_cd_type = DYN$C_SCDRP;
    scdrp->scdrp$b_flck = spdt->basespdt.spdt$is_flck;
    scdrp->scdrp$ps_spdt = ( SPDT * )spdt;
    scdrp->scdrp$ps_kpb = spdt->spdt$ps_kpb;
    status = spdt->basespdt.spdt$ps_cd_reset_scsi_bus( spdt, scdrp );
    return( status );
}

/*
 *
 *   Name:	pk$reset_scsi_bus - reset the SCSI bus
 *
 *   Abstract:	Issue the mailbox command to do a SCSI bus
 *		reset and set the wait period before the
 *		firmware will put the next command on the bus.
 *
 *   Inputs:	SPDT address
 *              SCDRP address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	The bus is reset.
 *
 *   Return
 *   Values:	SS$_NORMAL		Bus reset mailbox command succeeded
 *		SS$_TIMEOUT		The mailbox command timed out
 *
 */
int pk$reset_scsi_bus( PKQ_SPDT *spdt,
		       SCDRP *scdrp )
{
    DECLARE_ISP();
    int			status;

    /* Reset the SCSI bus using a mailbox command.
     */
    if( SUCCESS( status = pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_BUS_RESET,
				       spdt->spdt$w_bus_reset_delay,
				       0, 0, 0, 0 ))) {
	/* The bus reset succeeded.  If this routine was called during
	 * port (re-)initialization, exit immediately with success.
	 * Otherwise, call the reset-detected code to clean up outstanding
	 * requests on the port.
	 */
	if( spdt->spdt$l_pkq_flags & SPDT$M_PKQ_UNIT_INIT ) {
	    status = SS$_NORMAL;
	} else {
	    status = spdt->basespdt.spdt$ps_rl_reset_detected_fork( spdt, scdrp );
	}

	/* Set the send-marker-required flag
	 */
	if( target_mode( spdt ) )
	{
	    spdt->spdt$l_pkq_flags |= (SPDT$M_PKQ_SEND_MARKER |
					SPDT$M_PKQ_SEND_ENABLE_LUN);
	}
	else
	{
	    spdt->spdt$l_pkq_flags |= (SPDT$M_PKQ_SEND_MARKER );
	}
    }
    return( status );
}

/*
 *
 *   Name:	pkq_mailbox_io - send a mailbox command
 *
 *   Abstract:	Send an ISP1020 mailbox command and wait for it
 *		to complete.  Then read the outgoing mailbox
 *		registers to see if the command succeeded
 *
 *   Inputs:	SCSI Port Descriptor Table address
 *		Register load count
 *		Mailbox command and parameters 1 - 5
 *		
 *   Outputs:   Outgoing mailbox contents
 *
 *   Return
 *   Values:	SS$_NORMAL		mailbox command succeeded
 *		SS$_TIMEOUT		mailbox command timed out
 *		SS$_
 *
 */
int pkq_mailbox_io( PKQ_SPDT *spdt,		/* SCSI Port Descriptor Table address		*/
			uint32	rcount,		/* Mailbox register load count			*/
			uint16	command,	/* Mailbox 0 (command) load value		*/	
			uint16	r1,		/* Mailbox 1 - 5 load values			*/
			uint16	r2,		/*						*/
			uint16	r3,		/*						*/
			uint16	r4,		/*						*/
			uint16	r5 )		/*						*/
{
#define MBX_INTERVAL  100000000			/* 100 ms					*/
    uint16		mb_status;		/* Mailbox command status			*/
    int			status;			/* Routine status				*/
    DECLARE_ISP();				/* Temps for READ_ISP/WRITE_ISP			*/
    ADP			*adp;			/* Adaptor block pointer			*/
    uint64		*handle;		/* Pointer to ISP register file handle		*/
    int64		interval;		/* Interval in ns being timed			*/
    int64		due_time;		/* Time event becomes due			*/
    uint16		icr_image;		/* Copy of Interrupt Control Register		*/
    int			saved_ipl;		/* IPL saved/restored by lock/unlock		*/
    uint32		delay;			/* backoff delay value in nano seconds */

    /* Get the ADP address and I/O handle from the SPDT.
     */
    adp =  spdt->basespdt.spdt$l_adp;
    handle = &spdt->spdt$q_iohandle_reg;

    /* Lock the device, save the interrupt control register, disable interrupts if
     * they were enabled, and unlock the device.
     */
    device_lock( spdt->basespdt.spdt$l_dlck, RAISE_IPL, &saved_ipl );
    if(( icr_image = READ_ISP( adp, handle, isp$w_bus_icr )) & ISP$M_ICR_ALL_INT_ENB ) {
	WRITE_ISP( adp, handle, isp$w_bus_icr, 0 );
	READ_ISP(adp, handle, isp$w_bus_icr );
    }
    device_unlock( spdt->basespdt.spdt$l_dlck, saved_ipl, SMP_RESTORE );

    /* Wait for the mailbox registers to become available by spinning on the host interrupt
     * bit in the HCCR.  If they don't become available within the specified timeout period,
     * return an error to the caller.
     */
    interval = MBX_INTERVAL;
    if(( READ_ISP( adp, handle, isp$w_hccr ) & ISP$M_HCCR_HOST_INTERRUPT ) != 0 ) {
	if( SUCCESS( status = exe$timedwait_setup( &interval, &due_time ))) {
	    delay = 1000;
	    while(( status = exe$timedwait_complete( &due_time )) == SS$_CONTINUE ) {
		if(( READ_ISP( adp, handle, isp$w_hccr ) & ISP$M_HCCR_HOST_INTERRUPT ) == 0 ) {
		    status == SS$_NORMAL;
		    break;
		} else {
		   delay = backoff_delay( delay , 20000);

		}
	    }
	}
	if( ERROR ( status )) {
	    WRITE_ISP( adp, handle, isp$w_bus_icr, icr_image );
	    return( status );
	}
    }
#if PKQ_DEBUG	
trace_event(spdt,'mio1',rcount,r1,r2);
trace_event(spdt,'mio2',r3 ,r4,r5);
#endif

    /* Load the mailbox registers specified in the rcount parameter.
     * An rcount value of "n" means load registers 0 through n-1.  The single
     * exception is that the MB_INIT_RESPONSE_Q command doesn't load mailbox
     * register 4 which normally contains the command queue address.
     */
    switch( rcount ) {
    case 6:
	WRITE_ISP( adp, handle, isp$w_mailbox_5, r5 );
    case 5:
	if( command != ISP$K_MB_INIT_RESPONSE_Q )
	    WRITE_ISP( adp, handle, isp$w_mailbox_4, r4 );
    case 4:
	WRITE_ISP( adp, handle, isp$w_mailbox_3, r3 );
    case 3:
	WRITE_ISP( adp, handle, isp$w_mailbox_2, r2 );
    case 2:
	WRITE_ISP( adp, handle, isp$w_mailbox_1, r1 );
    case 1:
	WRITE_ISP( adp, handle, isp$w_mailbox_0, command );
	break;

    /* Anything else represents an internal driver error.  Bugcheck.
     */
    default:
	pkq_bugcheck();
    }

    /* Send an interrupt request to the ISP processor to get its attention.
     */
    WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_SET_HOST_INT ); 

    /* In order not to overwrite its outgoing mailbox registers, the RISC processor
     * synchronizes with the host by acquiring the semaphore lock, then fills in the
     * registers and requests a RISC interrupt.
     * This routine therefore waits until the semaphore lock and RISC interrupt bits
     * are set, clears the RISC interrupt bit, reads and stores the mailbox registers,
     * then clears the semaphore lock bit to free the mailbox registers.
     */
    interval = MBX_INTERVAL;
    if( !( READ_ISP( adp, handle, isp$w_bus_semaphore ) & ISP$M_SEMAPHORE_LOCK ) ||
       !( READ_ISP( adp, handle, isp$w_bus_isr ) & ISP$M_ISR_RISC_INT )) {
	if( SUCCESS( status = exe$timedwait_setup( &interval, &due_time ))) {
	    delay = 1000;
	    while(( status = exe$timedwait_complete( &due_time )) == SS$_CONTINUE ) {
		if(( READ_ISP( adp, handle, isp$w_bus_semaphore ) & ISP$M_SEMAPHORE_LOCK ) &&
		   ( READ_ISP( adp, handle, isp$w_bus_isr ) & ISP$M_ISR_RISC_INT )) {
		    status == SS$_NORMAL;
		    break;
		} else {
		   delay = backoff_delay( delay , 20000);
		}
	    }
	}
	if( ERROR( status )) {
	    WRITE_ISP( adp, handle, isp$w_bus_icr, icr_image );
	    return( status );
	}
    }
    WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_CLR_RISC_INT ); 
    mb_status = READ_ISP( adp, handle, isp$w_mailbox_0 );
    spdt->spdt$w_mailbox_0 = mb_status;
    spdt->spdt$w_mailbox_1 = READ_ISP( adp, handle, isp$w_mailbox_1 );
    spdt->spdt$w_mailbox_2 = READ_ISP( adp, handle, isp$w_mailbox_2 );
    spdt->spdt$w_mailbox_3 = READ_ISP( adp, handle, isp$w_mailbox_3 );
    spdt->spdt$w_mailbox_4 = READ_ISP( adp, handle, isp$w_mailbox_4 );
    spdt->spdt$w_mailbox_5 = READ_ISP( adp, handle, isp$w_mailbox_5 );
    WRITE_ISP( adp, handle, isp$w_bus_semaphore, 0 ); 

    /* Dispatch on the mailbox status code
     */
    switch( mb_status ) {

    /* Normal command completion.
     */
    case  ISP$K_MB_STS_COMMAND_COMPLETE:
	status = SS$_NORMAL;
	break;

    /* Various mailbox error conditions.  Return an error
     * to the caller and let him decided what to do about it.
     */
    case ISP$K_MB_STS_INVALID_COMMAND:
    case ISP$K_MB_STS_HOST_INTFC_ERROR:
    case ISP$K_MB_STS_TEST_FAILED:
    case ISP$K_MB_STS_COMMAND_ERROR:
    case ISP$K_MB_STS_PARAMETER_ERROR:
	status = SS$_BADPARAM;
	break;

    case ISP$K_MB_STS_BUSY:
	status = SS$_TIMEOUT;
	break;

    default:	
	pkq_bugcheck();
    }

    WRITE_ISP( adp, handle, isp$w_bus_icr, icr_image );
    return( status );
}

/*
 *
 *   Name:	pkq_bugcheck - issue a bugcheck
 *
 *   Abstract:	Issue a bugcheck
 *
 *   Inputs:	None
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL		bugcheck succeeded
 *
 */
int pkq_bugcheck()
{
       bug_check( INCONSTATE, FATAL, COLD );
       return(SS$_NORMAL);
}

/*
 *
 *   Name:	pk$cmd_buffer_alloc - allocate a command buffer
 *
 *   Abstract:	Allocate a command/autosense buffer on behalf of
 *		the caller and store the class driver's idea of
 *		its address in the cell provided.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *		Requested command buffer size
 *		Address of cell to receive QBUF address
 *		
 *
 *   Outputs:	The address of the QBUF (kind of).
 *
 *   Return
 *   Values:	SS$_NORMAL		the allocation succeeded
 *		SS$_BADPARAM		requested size larger than QBUF
 *
 */
int pk$cmd_buffer_alloc( PKQ_SPDT	*spdt,		/* SCSI PDT address			*/
			SCDRP		*scdrp,		/* SCSI CDRP address			*/
			uint32		request_size,	/* Requested command buffer size	*/
			void		**ret_addr )	/* Cell to receive QBUF address		*/
{
    /* Allocate buffer space for the CDB and autosense data.  The CDB is preceded by
     * 8 bytes of overhead for use by the class driver.
     *  1.  Get the KPB address out of the SCDRP;
     *  2.  If the requested CDB size exceeds the maximum CDB size,
     *	    return SS$_BADPARAM;
     *  3.  Allocate a QBUF, which includes both the CDB and autosense buffer;
     *	4.  Fill in the SCDRP with buffer addresses and lengths;
     *  5.  Allocate a RSPID;
     *  6.  Return the address of the class driver overhead field to the caller.
     */
    /* ** TODO ** the command buffer addressing crock needs to be documented
     * 1.  This routine allocates enough space for both command and sense data
     *	   plus the command length and a status cell.
     * 2.  This routine returns a pointer to the status cell which must precede
     *	   the command length which must precede the command.
     * 3.  The class driver understands the relationship of these fields and autoincs
     *	   to pass the address of the cdb_size into the send command routine.
     */
    KPB		*kpb = scdrp->scdrp$ps_kpb;
    QBUF 	*qbuf;
    RSPID	rspid;
    if( request_size > QBUF$K_CDB_SIZE )
	return( SS$_BADPARAM );

    qbuf = pkq_get_qbuf( scdrp );
    if( qbuf == NULL )
	return( SS$_BADPARAM );
    qbuf->qbuf$ps_scdrp = scdrp;

    rspid = pkq_alloc_rspid( spdt, scdrp );
    if( rspid.seq_no == 0 )
	return( SS$_BADPARAM );
    qbuf->qbuf$r_rspid = rspid;
    qbuf->qbuf$l_port_status = 0;
    *ret_addr = &qbuf->qbuf$l_class_driver;
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pk$cmd_buffer_dealloc - deallocate a command buffer
 *
 *   Abstract:	Deallocates a command/autosense buffer (QBUF) on behalf
 *		of the caller and zeros its pointer in the SCDRP.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *
 *   Outputs:	SCDRP
 *		  scdrp$l_cmd_ptr	zeroed
 *
 *   Return
 *   Values:	SS$_NORMAL
 *
 */
int pk$cmd_buffer_dealloc( PKQ_SPDT	*spdt,		/* SCSI PDT address			*/
			  SCDRP		*scdrp )	/* SCSI CDRP address			*/
{
    QBUF	*qbuf;					/* Command/sense buffer			*/
    /* Deallocate buffer space for the CDB and autosense data.
     * Deallocate the RSPID.
     */
    qbuf = ( QBUF *)(( uint8 * )scdrp->scdrp$l_cmd_ptr -
		     offsetof( QBUF, qbuf$l_scsi_cdb_size ));
    pkq_dealloc_rspid( spdt, qbuf->qbuf$r_rspid );
    pkq_ret_qbuf( qbuf );
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pkq_get_qbuf - allocate a QBUF
 *
 *   Abstract:	Allocate a QBUF structure out of non-paged pool and
 *		return its address to the caller.
 *
 *   Inputs:	SCDRP address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	Address of allocated QBUF structure
 *
 */
QBUF *pkq_get_qbuf( SCDRP *scdrp )			/* Address of SCDRP for request		*/
{
    QBUF	*qbuf;					/* Address of allocated buffer		*/
    int32	alo_size;				/* Size of allocated buffer		*/
    
    /* Get a queue buffer to hold the SCSI command data block and the returned autosense data:
     *  -  Attempt to allocate a queue buffer (QBUF), doing a fork and wait
     *	   until the allocation succeeds - if the fork and wait itself fails
     *	   return 0 to the caller;
     *  -  Zero the QBUF;
     *  -  Fill in the standard header portion of the QBUF;
     *  -  Return the QBUF address;
     */
    while( ERROR( exe_std$alononpaged( sizeof( QBUF ), &alo_size, ( void ** )&qbuf ))) {
	if( ERROR( exe$kp_fork_wait( scdrp->scdrp$ps_kpb, ( FKB * )scdrp )))
	    return( 0 );
    }
    memset( qbuf, 0, sizeof( QBUF )); 
    qbuf->qbuf$w_size = alo_size;
    qbuf->qbuf$b_type = DYN$C_MISC;
    qbuf->qbuf$b_subtype = DYN$C_QBUF;
    return( qbuf );
}

/*
 *
 *   Name:	pkq_ret_qbuf - return a QBUF
 *
 *   Abstract:	Deallocates a QBUF structure to non-paged pool.
 *
 *   Inputs:	QBUF address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_ret_qbuf( QBUF *qbuf )				/* Buffer address			*/
{
    /* Return the queue buffer to the non-paged pool.
     */
    exe_std$deanonpaged( qbuf );
    return;
}

/*
 *
 *   Name:	pk$map_buffer - allocate mapping resources for transfer
 *
 *   Abstract:	Determine whether any of the pages invoved in the transfer
 *		fall outside the system direct DMA window.  If so, 
 *		allocate and map the transfer; otherwise, set up the SCDRP
 *		so that the send command routine can tell that no mapping
 *		is required.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *		Mapping request priority
 *
 *   Outputs:
 *
 *   Implicit
 *   outputs:	SCDRP fields, initialized appropriately **TODO** more
 *
 *   Return
 *   Values:	SS$_NORMAL		mapping succeeded
 *
 */
int pk$map_buffer( PKQ_SPDT	*spdt,			/* SCSI PDT address			*/
		  SCDRP		*scdrp,			/* SCSI CDRP address			*/
		  uint32	priority )		/* Request priority			*/
{
    uint32		use_map_registers;		/* Boolean: set if using map registers	*/
    uint64		pa;
    uint32		page_count;			/* Cell used for page or resource count	*/
    PTE			*svapte;			/* System VA of PTE			*/
    CRCTX		*crctx;				/* Counted resource context pointer	*/
    KPB			*kpb;				/* Kernel process block pointer		*/
    int			status;				/* VMS status				*/

    /* Compute the number of pages required by the transfer.  Assume
     * that mapping is NOT required.  Check the physical address of each
     * of the pages to see if it falls outside the direct DMA window;
     * if so, force the use of map registers.
     */
    svapte = scdrp->scdrp$l_svapte;
    page_count =
	( scdrp->scdrp$l_boff + scdrp->scdrp$l_bcnt + mmg$gl_bwp_mask ) >>
	    mmg$gl_vpn_to_va;
    for( use_map_registers = FALSE;  page_count; --page_count, svapte++ ) {
	pa = svapte->pte$v_pfn;
	if( !svapte->pte$v_valid ) {
	    pa = ioc_std$ptetopfn( svapte );
	    not_valid_counter++;
	}
	pa <<= mmg$gl_vpn_to_va;
	if( pa > spdt->spdt$l_direct_dma_size ) {
	    use_map_registers = TRUE;
	    break;
	}
    }
    /* Initialize the Counted Resource Context block embedded
     * in the SCDRP:
     *  - Standard structure header;
     *  - CRAB pointer and forklock index from the SPDT;
     *  - Priority from the input parameter;
     *  - Auxiliary context from KPB pointer in the SCDRP;
     *  - Item number, upper and lower bounds set to 0;
     *  - Callback routine;
     *  - Saved callback set to 0;
     *  - Item count (number of mapping resources required);
     *    computed as: 
     *      ( byte offset within mapping resource +
     *        byte count + bytes per mapping resource - 1 ) /
     *        bytes per mapping resource.
     *    A shift is used in place of division since bytes
     *    per mapping resource is constrained to be a power of 2.
     */
    if( use_map_registers == TRUE ) {
	crctx = ( CRCTX * )&scdrp->scdrp$r_crctx_base;
	crctx->crctx$w_size = CRCTX$K_LENGTH;
	crctx->crctx$b_type = DYN$C_MISC;
	crctx->crctx$b_subtype = DYN$C_CRCTX;
	crctx->crctx$l_crab = spdt->basespdt.spdt$ps_crab;
	crctx->crctx$b_flck = spdt->basespdt.spdt$is_flck;
	crctx->crctx$l_flags = priority;
	crctx->crctx$l_aux_context = scdrp->scdrp$ps_kpb;
	crctx->crctx$l_item_num = 0;
	crctx->crctx$l_up_bound = 0;
	crctx->crctx$l_low_bound = 0;
	crctx->crctx$l_callback = pkq_buffer_map_restart;
	crctx->crctx$l_saved_callback = 0;
	crctx->crctx$l_item_cnt = 
	    (((( scdrp->scdrp$l_boff & spdt->basespdt.spdt$is_crctx_bwp_mask ) +
	       scdrp->scdrp$l_bcnt + spdt->basespdt.spdt$is_crctx_bwp_mask ) >>
	      spdt->basespdt.spdt$is_crctx_shift ) +
	     spdt->basespdt.spdt$is_extmapreg );

	/* Allocate the mapping resources.  If the allocation fails and the
	 * request was high priority, convert it to normal priority and try
	 * the allocation again.  If allocation fails again, stall the thread
	 * until the map restart routine notifies us that the resources have
	 * been allocated for us.
	 */
	if( ERROR( status = ioc$alloc_cnt_res( spdt->basespdt.spdt$ps_crab, crctx, 0, 0, 0 ))) {
	    if( crctx->crctx$l_flags & CRCTX$M_HIGH_PRIO ) {
		crctx->crctx$l_flags &= ~CRCTX$M_HIGH_PRIO;
		status = ioc$alloc_cnt_res( spdt->basespdt.spdt$ps_crab, crctx, 0, 0, 0 );
	    }
	    if( ERROR( status )) {
		kpb = scdrp->scdrp$ps_kpb;
		kpb->kpb$ps_sch_stall_rtn = ioc$return;
		kpb->kpb$ps_sch_restrt_rtn = 0;
		exe$kp_stall_general( kpb );
	    }
	}

	/* The mapping resources are in hand; load the map.
	 */
	scdrp->scdrp$l_port_boff = scdrp->scdrp$l_boff;
	scdrp->scdrp$l_port_svapte = scdrp->scdrp$l_svapte;
	scdrp->scdrp$l_sva_spte = 0;
	status = ioc$load_map( spdt->basespdt.spdt$l_adp,
			      crctx,
			      scdrp->scdrp$l_svapte,
			      scdrp->scdrp$l_boff,
			      &scdrp->scdrp$l_sva_dma );

    /* The direct DMA window is being used.  Set the SCDRP up to remember
     * that this request requires no mapping resource.
     */
    } else {
	scdrp->scdrp$is_item_cnt = 0;
	scdrp->scdrp$is_item_num = 0;
	scdrp->scdrp$l_sva_dma = 0;
	scdrp->scdrp$l_sva_spte = 0;
	status = SS$_NORMAL;
    }
    return( status );
}

/*
 *
 *   Name:	pkq_buffer_map_restart - restart the map routine
 *
 *   Abstract:	Wake the KP thread when mapping resources become
 *		available.
 *
 *   Inputs:	CRCTX address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL
 *
 */
int pkq_buffer_map_restart( CRCTX *crctx )
{
    exe$kp_restart( ( KPB * )crctx->crctx$l_aux_context, SS$_NORMAL );
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pk$unmap_buffer - deallocate mapping resources
 *
 *   Abstract:	If the request required mapping resources, deallocate
 *		them and clear the appropriate SCDRP fields to indicate
 *		that they are no longer held.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *
 *   Outputs:	scdrp$is_item_cnt
 *		scdrp$is_item_num
 *		scdrp$l_sva_dma
 *		scdrp$l_svaspte		all zeroed
 *
 *   Return
 *   Values:	SS$_NORMAL		deallocate succeeded
 *		other			error returned from deallocate request
 *
 */
int pk$unmap_buffer( PKQ_SPDT *spdt,			/* SCSI Port Descriptor Table		*/
		    SCDRP *scdrp )			/* SCSI Class Driver Request Packet	*/
{
    CRCTX	*crctx;					/* Counted Resource Context Block	*/
    uint32	status;					/* VMS status code			*/
    status = SS$_NORMAL;
    if( scdrp->scdrp$l_sva_dma ) {
	crctx = ( CRCTX * )&scdrp->scdrp$r_crctx_base;
	if( SUCCESS( status = ioc$dealloc_cnt_res( spdt->basespdt.spdt$ps_crab, crctx ))) {
	    scdrp->scdrp$is_item_cnt = 0;
	    scdrp->scdrp$is_item_num = 0;
	    scdrp->scdrp$l_sva_dma = 0;
	    scdrp->scdrp$l_sva_spte = 0;
	}
    }
    return( status );
}

/*
 *
 *   Name:	pk$send_command - Queue a SCSI command to the adaptor
 *
 *   Abstract:  Set up the request queue entries required to describe
 *		the SCSI command and notify the ISP1020 RISC firmware
 *		that it has work to do.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *		SCDT address
 *		STDT address
 *		UCB address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	spdt$l_requestq_in	updated
 *
 *   Return
 *   Values:	SS$_NORMAL		command has been queued
 *		SS$_RETRY		no queue space available
 *
 */
int pk$send_command( PKQ_SPDT *spdt,		/* SCSI Port Descriptor Table			*/
		    SCDRP *scdrp,		/* SCSI Class Driver Request Packet		*/
		    SCDT *scdt,			/* SCSI Connection Descriptor Table		*/
		    STDT *stdt,			/* SCSI Target Descriptor Table			*/
		    UCB *ucb )			/* Unit Control Block				*/
{
#define MAX_ENTRIES_PER_REQUEST 3
    DECLARE_ISP();				/* ISP register file and temps			*/
    uint32		requestq_out;		/* Index of ISP entry chip is processing 	*/
    uint32		requestq_in;		/* Index of next available ISP entry		*/
    int32		avail_entries;		/* Number of available ISP entries		*/
    ISP_ENTRY		*isp_entry;		/* Current ISP entry				*/
    ISP_ENTRY		*first_entry;		/* Saved copy of first ISP entry		*/
    QBUF		*qbuf;			/* QBUF pointer					*/
    uint32		cdb_length;		/* SCSI command data block length		*/
    PTE			*svapte;		/* System VA of PTE				*/
    int32		i;			/* Data descriptor index			*/
    int32		pa;			/* Physical address				*/
    int32		byte_count;		/* Requested byte count				*/
    int32		pad_count;		/* request pad count */
    int32		byte_offset;		/* Offset w/in page of transfer start		*/
    int32		status;			/* VMS status					*/

    /* If this is the first command being sent after initialization
     * or bus reset, put a marker entry in the queue.
     * If this is the first marker after unit init we'll also send an enable_lun
     */
    if( spdt->spdt$l_pkq_flags & 
			(SPDT$M_PKQ_SEND_MARKER |
			 SPDT$M_PKQ_SEND_ENABLE_LUN) ) 
    {
	pkq_send_marker( spdt );
    }

    /* Put a command entry in the ISP1020 request queue.  Requestq_in is the
     * index of the next entry to be filled in by this routine.  Requestq_out
     * is the index of the next entry the ISP1020 firmware is going to empty.
     */
    /* Check to see if there is enough space available in the request queue
     * to accomodate a maximum size request (and still leave room for target
     * mode requests).  If not, retry several times to give the firmware a 
     * chance to empty the queue.
     * 
     * NOTE: This code always leaves MIN_FREE_ENTRIES request queue entries
     *	    free in the request queue for use by the target mode code.
     *	    this saves the complication of having to stall waiting from the
     *	    interrupt fork.
     */
    do {
	requestq_in = spdt->spdt$l_requestq_in;
#if PKQ_DEBUG
	spdt->spdt$l_requestq_out = 
#endif	
	requestq_out = ( uint32 )READ_ISP( spdt->basespdt.spdt$l_adp,
					  &spdt->spdt$q_iohandle_reg,
					  isp$w_mailbox_4 );
	if(( avail_entries = requestq_out - requestq_in ) <= 0 ) {
	    avail_entries += REQUEST_QUEUE_SIZE;
	}
	if( avail_entries <= MIN_FREE_ENTRIES )
	{
	    stall_qman( spdt );
	}

    } while(  avail_entries <= MIN_FREE_ENTRIES );

    /* If space is available, calculate the address of the next
     * entry to be filled and save the entry pointer for later use.
     */
    if( avail_entries > MAX_ENTRIES_PER_REQUEST ) {
	isp_entry = spdt->spdt$ps_requestq_va + requestq_in;
	first_entry = isp_entry;

	/* Fill in the header portion of the entry:
	 *  - Entry type is COMMAND;
	 *  - Entry count (number of entries that make up the command
	 *    including continuations) set to 1;
	 *  - Header Flags (flags set by the firmware when returning a
	 *    command in the response queue) set to 0;
	 *  - System (host) defined field, set to 0;
	 *  - Handle used to complete the request, set to the request's
	 *    QBUF address.
	 */
	isp_entry->isp$b_type = ISP$K_TY_COMMAND;
	isp_entry->isp$b_count = 1;
	isp_entry->isp$b_sysdef_1 = 0;
	isp_entry->isp$b_header_flags = 0;
	qbuf = ( QBUF *)(( uint8 * )scdrp->scdrp$l_cmd_ptr -
			 offsetof( QBUF, qbuf$l_scsi_cdb_size ));
	isp_entry->isp$l_handle = *( uint32 * )&qbuf->qbuf$r_rspid;

	/* Fill in the CDB portion of the entry:
	 *  - LUN from the SCDT;
	 *  - Target from the STDT;
	 *  - CDB length from the QBUF;
	 *  - Control flags:
	 *	o queuing characteristics based on SCDRP flags;
	 *	o No-disconnect flag based on the SCDT;
	 *	o Data direction (read/write/not-transfer) based
	 *	  on the byte count and status fields in the SCDRP;
	 *  - Reserved field set to 0;
	 *  - Timeout set to the sum of the SCDRP DMA and disconnect
	 *    timeouts;
	 *  - Total count of data segments used by the command.
	 *
	 * Also get the byte count and starting byte offset from
	 * the SCDRP.
	 */
	isp_entry->isp$b_lun = ( uint8 )scdt->scdt$l_scsi_lun;
	isp_entry->isp$b_target = stdt->stdt$is_scsi_id_num;
	cdb_length = qbuf->qbuf$l_scsi_cdb_size;
	isp_entry->isp$w_cdb_length = ( uint16 )cdb_length;
	isp_entry->isp$w_control_flags = 0;
	if( scdrp->scdrp$l_scsi_flags & SCDRP$M_FLAG_QUEUED_IO ) {
	    if( scdrp->scdrp$is_queue_char == SCDRP$K_QCHAR_UNORDERED )
		isp_entry->isp$v_cf_simple = 1;
	    else if( scdrp->scdrp$is_queue_char == SCDRP$K_QCHAR_ORDERED )
		isp_entry->isp$v_cf_ordered = 1;
	    else if( scdrp->scdrp$is_queue_char == SCDRP$K_QCHAR_HEAD )
		isp_entry->isp$v_cf_head = 1;
	}
	if(( scdt->scdt$is_con_flags & SCDT$M_CFLG_ENA_DISCON ) == 0 )
	    isp_entry->isp$v_cf_no_disconnect = 1;
	byte_count = scdrp->scdrp$l_bcnt;
	pad_count  = scdrp->scdrp$l_pad_bcnt;
	byte_offset = scdrp->scdrp$l_boff;
	isp_entry->isp$v_cf_data_direction = ISP$K_DD_NOT_XFR;
	if( byte_count != 0 ) {
	    if( scdrp->scdrp$is_sts & IRP$M_FUNC ) {
		isp_entry->isp$v_cf_data_direction = ISP$K_DD_READ;
	    } else {
		isp_entry->isp$v_cf_data_direction = ISP$K_DD_WRITE;
	    }
	}
	isp_entry->isp$w_reserved_0 = 0;
	isp_entry->isp$w_timeout =
	    scdrp->scdrp$l_dma_timeout + scdrp->scdrp$l_discon_timeout;
	isp_entry->isp$w_data_seg_count = 0;
	/*
	**  Set up duetime for timeout code.
	*/
	scdrp->scdrp$l_duetime = 
				exe$gl_abstim +
				scdrp->scdrp$l_dma_timeout + 
				scdrp->scdrp$l_discon_timeout+1;
	scdrp->scdrp$v_cnx_dscn = 1;

	/* Fill in the data segment portion of the entry.  First, get the
	 * starting SVAPTE.
	 */
	svapte = scdrp->scdrp$l_svapte;

	/* If the CDB is of "normal" length, move it into the entry with
	 * assignment statements.
	 */
	if( cdb_length <= 12 ) {
	    *( uint64 *)&isp_entry->isp$b_cdb[ 0 ] = *( uint64 *)&qbuf->qbuf$b_scsi_cdb[ 0 ];
	    *( uint32 *)&isp_entry->isp$b_cdb[ 8 ] = *( uint32 *)&qbuf->qbuf$b_scsi_cdb[ 8 ];

            /* Test the SCSI command in byte 0 of the CDB for an INQUIRE command 
            */
            if ( isp_entry->isp$b_cdb[0] == SCSI$K_INQUIRY ) {

            	/* An INQUIRE command is to be sent. 
             	 * Test for firmware version >=  2.00 
                 * (Only firmware >= V2.0 can force wide/synch re-negotiation)
                */

		if ( spdt->spdt$l_firmware_version >= 200) {

		   /* Before sending the INQUIRE:-
		    *  - If we think the target is talking wide we must force re-negotiation of wide 
             	    *  - If we think the target is talking synch we must force re-negotiation of synch 
            	   */
            
            	   /* Make sure that we have the current NVRAM parameters in the spdt
            	    * ( They definitely SHOULD be there - test can't hurt - costs little )
            	   */  
		   pkq_get_parameters( spdt );

                   /* Test if we think we are talking synch - and if so force synch renegotiation 
                    *      ( looking in the spdt seems the safest                                )
                    *      ( because we KNOW they are there after the call to pkq_get_parameters )
                   */
		   if ( spdt->spdt$r_tparams[stdt->stdt$is_scsi_id_num].spdt$v_synch_data_transfers) {
			
			isp_entry->isp$v_cf_init_sdtr_negot = 1;
                   }

                   /* Test if we think we are talking wide - and if so force wide renegotiation 
                   */
		   if ( spdt->spdt$r_tparams[stdt->stdt$is_scsi_id_num].spdt$v_wide_data_transfers ) {

			isp_entry->isp$v_cf_init_wdtr_negot = 1;

                   }          
		}
            }
            
	    /* A COMMAND entry has room for up to four data descriptors.  If the transfer
	     * requires the use of mapping registers (indicated by a nonzero scdrp$l_sva_dma),
	     * a single data descriptor covers the entire transfer.  Otherwise, we build
	     * a descriptor for each page involved in the transfer.
	     */
	    for(   i = 0;	
		   ( i < 4 ) &&( ( byte_count > 0 ) || (pad_count > 0) ); 
		    i++ 
	       ) 
	    {
	        if( byte_count>0 )
		{
		    if( scdrp->scdrp$l_sva_dma ) {
		    	isp_entry->isp$r_data_seg[ i ].isp$ps_base_address = 
							scdrp->scdrp$l_sva_dma;
		    	isp_entry->isp$r_data_seg[ i ].isp$l_byte_count = 
							byte_count;
		    	byte_count -= byte_count;
		    } else {
		    	pa = svapte->pte$v_pfn;
		    	if( !svapte->pte$v_valid ) {
			    pa = ioc_std$ptetopfn( svapte );
		    	}
		    	pa <<= mmg$gl_vpn_to_va;
		    	isp_entry->isp$r_data_seg[ i ].isp$ps_base_address =
			    ( void *)( spdt->spdt$q_direct_dma_base + pa + byte_offset );
		    	isp_entry->isp$r_data_seg[ i ].isp$l_byte_count =
			    (( byte_offset + byte_count ) > mmg$gl_page_size ) ?
			    	( mmg$gl_page_size - byte_offset ) :
					byte_count;
		    	byte_offset = 0;
		    	byte_count -= isp_entry->isp$r_data_seg[ i ].isp$l_byte_count;
		    	svapte++;
		    }
		    isp_entry->isp$w_data_seg_count++;
		}
		else
		{
		    /* here we must have more pad count to handle */

		    /* If we are doing a write, use the Erase Page. For a read, we must
		       use a different page. Otherwise we corrupt the EXE$GL_ERASEPB
		       buffer. In the case of a read, just use EXE$GL_BLAKHOLE since
		       we aren't interested in the data that is written to the pad
		       buffer anyways. MED 2/5/97                                   */
		    if (isp_entry->isp$v_cf_data_direction == ISP$K_DD_READ)
				pa = (exe$gl_blakhole) << mmg$gl_vpn_to_va;
			else
			    	pa = (exe$gl_eraseppt->pte$v_pfn) << mmg$gl_vpn_to_va;

		    isp_entry->isp$r_data_seg[ i ].isp$ps_base_address =
                        ( void *)( spdt->spdt$q_direct_dma_base + pa  );
		    isp_entry->isp$r_data_seg[ i ].isp$l_byte_count = 
				min( pad_count, mmg$gl_page_size);
		    pad_count -= min( pad_count, mmg$gl_page_size);
		    isp_entry->isp$w_data_seg_count++;

		    
		}
	    }

	/* The CDB is longer than 12 bytes, so it requires an EXTENDED command
	 * entry.  Change the entry type and copy the CDB into the entry.  Note
	 * that an EXTENDED command entry has no room for data segments, so at
	 * least one continuation entry is required to perform a data transfer.
	 */
	} else {
	    isp_entry->isp$b_type = ISP$K_TY_EXTENDED;
	    memcpy( isp_entry->isp$b_cdb, qbuf->qbuf$b_scsi_cdb, cdb_length );
	}

	/* Increment the request queue ring index and handle wrap-around.
	 */
	requestq_in++;
	if( requestq_in >= REQUEST_QUEUE_SIZE )
	    requestq_in = 0;

	/* Fill in a continuation entry if there is more data to transfer.
	 * In the header:
	 *  - Entry type set to CONTINUATION;
	 *  - Entry count (number of entries that make up the command
	 *    including continuations) updated in the first entry and copied
	 *    to the current entry (which means the count is correct only
	 *    in the first and last entries if there are more than 2);
	 *  - Header Flags (flags set by the firmware when returning a
	 *    command in the response queue) set to 0;
	 *  - System (host) defined field, set to 0;
	 */
	while( (byte_count > 0) || (pad_count >0)) {
	    isp_entry = spdt->spdt$ps_requestq_va + requestq_in;
	    isp_entry->isp$b_type = ISP$K_TY_CONTINUATION;
	    first_entry->isp$b_count++;
	    isp_entry->isp$b_count = first_entry->isp$b_count;
	    isp_entry->isp$b_sysdef_1 = 0;
	    isp_entry->isp$b_header_flags = 0;

	    /* A CONTINUATION entry has room for up to seven data descriptors.  If the transfer
	     * requires the use of mapping registers (indicated by a nonzero scdrp$l_sva_dma),
	     * a single data descriptor covers the entire transfer.  Otherwise, we build
	     * a descriptor for each page involved in the transfer.
	     */
	    for( i = 0;	( i < 7 ) && (( byte_count != 0 ) || (pad_count>0)); i++ )
	    {
		if( byte_count != 0)
		{	
		    if( scdrp->scdrp$l_sva_dma ) {
			isp_entry->isp$r_cont_seg[ i ].isp$ps_base_address = scdrp->scdrp$l_sva_dma;
			isp_entry->isp$r_cont_seg[ i ].isp$l_byte_count = byte_count;
			byte_count -= byte_count;
		    } else {
			pa = svapte->pte$v_pfn;
			if( !svapte->pte$v_valid ) {
			    pa = ioc_std$ptetopfn( svapte );
			}
			pa <<= mmg$gl_vpn_to_va;
			isp_entry->isp$r_cont_seg[ i ].isp$ps_base_address =
			    ( void *)( spdt->spdt$q_direct_dma_base + pa + byte_offset );
			isp_entry->isp$r_cont_seg[ i ].isp$l_byte_count =
			    (( byte_offset + byte_count ) > mmg$gl_page_size ) ?
				( mmg$gl_page_size - byte_offset ) :
				    byte_count;
			byte_offset = 0;
			byte_count -= isp_entry->isp$r_cont_seg[ i ].isp$l_byte_count;
			svapte++;
		    }
		}
		else
		{
		    /* here we must have more pad count to handle */

		    /* If we are doing a write, use the Erase Page. For a read, we must
		       use a different page. Otherwise we corrupt the EXE$GL_ERASEPB
		       buffer. In the case of a read, just use EXE$GL_BLAKHOLE since
		       we aren't interested in the data that is written to the pad
		       buffer anyways. MED 2/5/97                                   */
		    if (isp_entry->isp$v_cf_data_direction == ISP$K_DD_READ)
				pa = (exe$gl_blakhole) << mmg$gl_vpn_to_va;
			else
			    	pa = (exe$gl_eraseppt->pte$v_pfn) << mmg$gl_vpn_to_va;

		    isp_entry->isp$r_cont_seg[ i ].isp$ps_base_address =
                        ( void *)( spdt->spdt$q_direct_dma_base + pa  );
		    isp_entry->isp$r_cont_seg[ i ].isp$l_byte_count = 
				min( pad_count, mmg$gl_page_size);
		    pad_count -= min( pad_count, mmg$gl_page_size);
		    
		}
		first_entry->isp$w_data_seg_count++;
	    }

	    /* Increment the request queue ring index and handle wrap-around.
	     */
	    requestq_in++;
	    if( requestq_in >= REQUEST_QUEUE_SIZE )
		requestq_in = 0;
	}
	/* Update the requestq_in index in the SPDT and let the firmware
	 * know that it has more work by writing the updated index into
	 * mailbox register 4.  Exit with SS$_NORMAL.
	 */
	spdt->spdt$l_requestq_in = requestq_in;

#if PKQ_DEBUG	
trace_event(spdt,'send',
	       (((exe$gl_abstim&0xffff)<<16) + (requestq_in<<8) + requestq_out),
		(uint32)qbuf,(uint32)isp_entry);
#endif

	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_mailbox_4, requestq_in );
#if PKQ_DEBUG	
trace_event(spdt,'sdon',

	       (((exe$gl_abstim&0xffff)<<16) + (requestq_in<<8) + requestq_out),
		(uint32)scdrp,
		 ((qbuf->qbuf$r_rspid.index)+(qbuf->qbuf$r_rspid.seq_no<<16)));
#endif

	return( SS$_NORMAL );

    /* No room in the request queue after we waited for sufficient room. This
     * means either we're calulating the room wrong somewhere, we've misjudged
     * the maximum number of entries that can be used outside of send_command
     * or something else that we don't understand isn't working. Better crash.
     */
    } else {
	pkq_bugcheck();
    }
}

/*
 *
 *   Name:	pkq_send_marker - Queue a marker entry
 *
 *   Abstract:	Put a marker entry in the request queue.  Any time the
 *		bus is reset, all I/Os in progress or in queue will be
 *		returned as incomplete, marked accordingly.  The firmware
 *		needs a way to tell where new activity begins.  The marker
 *		entry precedes new activity. The target mode firmware also 
 *		requires a notify acknowledge message to be sent to clear the
 *		reset. This routine is also used to send the enable lun message
 *		after unit init.
 *
 *   Inputs:	SPDT address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	spdt$l_requestq_in	updated
 *
 *   Return
 *   Values:	SS$_NORMAL		send marker succeeded
 *		SS$_RETRY		no queue space available
 *
 */
int pkq_send_marker( PKQ_SPDT *spdt )		/* SCSI Port Descriptor Table			*/
{
    DECLARE_ISP();				/* Isp register file and temps			*/
    uint32		entries_needed;		/* # entries neede by this routine */
    uint32		requestq_out; 		/* Index of ISP entry chip is processing	*/
    uint32		requestq_in; 		/* Index of next available ISP entry		*/
    int32		avail_entries; 		/* Number of available ISP entries		*/
    ISP_ENTRY		*isp_entry; 		/* Current ISP entry				*/
    int			saved_ipl; 		/* IPL saved during device locking		*/

    /* Put a marker entry in the ISP1020 request queue.  Requestq_in is the
     * index of the next entry to be filled in by this routine.  Requestq_out
     * is the index of the next entry the ISP1020 firmware is going to empty.
     * Send marker requires 2 entries. One for the marker entry and one for the
     * Notify acknowledge entry.
     */
    entries_needed = 0;
    if((spdt->spdt$l_pkq_flags & SPDT$M_PKQ_SEND_MARKER)!= 0) 
							entries_needed += 2;
    if( target_mode( spdt ) )
    {
	if((spdt->spdt$l_pkq_flags & SPDT$M_PKQ_SEND_ENABLE_LUN)!= 0) 
							entries_needed++;
    }

    requestq_in = spdt->spdt$l_requestq_in;
    
    do {
#if PKQ_DEBUG
	spdt->spdt$l_requestq_out = 
#endif	
	requestq_out = ( uint32 )READ_ISP( spdt->basespdt.spdt$l_adp,
					  &spdt->spdt$q_iohandle_reg,
					  isp$w_mailbox_4 );
	if(( avail_entries = requestq_out - requestq_in ) <= 0 ) {
	    avail_entries += REQUEST_QUEUE_SIZE;
	}
	if( avail_entries <= entries_needed )
	{
	    stall_qman( spdt );
	}
    } while( avail_entries <= entries_needed );

    /* If there is space in the request ring for the marker
     * entry, get the entry address and build a marker entry.
     */
    if( ( avail_entries > 2 ) && 
		      ((spdt->spdt$l_pkq_flags & SPDT$M_PKQ_SEND_MARKER)!=0) ){

	if( target_mode( spdt ) )
	{
    	    reset_ack_notify(spdt);
	    avail_entries--;
    	    requestq_in = spdt->spdt$l_requestq_in;
	}

	isp_entry = spdt->spdt$ps_requestq_va + requestq_in;
	isp_entry->isp$b_type = ISP$K_TY_MARKER;
	isp_entry->isp$b_count = 1;
	isp_entry->isp$b_sysdef_1 = 0;
	isp_entry->isp$b_header_flags = 0;
	isp_entry->isp$l_handle = 0;
	isp_entry->isp$b_lun = 0;
	isp_entry->isp$b_target = 0;
	isp_entry->isp$b_modifier = 2; /* Need symbols for this */
	isp_entry->isp$b_marker_reserved_mbz = 0;

#if PKQ_DEBUG
	trace_event(spdt,'mrkr',requestq_in,exe$gl_abstim,0);
#endif
	

	/* Bump the request_in index modulo the queue size.
	 */

	requestq_in++;
	if( requestq_in >= REQUEST_QUEUE_SIZE )
	    requestq_in = 0;

	/* Update the requestq_in index in the SPDT and in mailbox
	 * register 4.  Then reset the send marker flag and return
	 * success to the caller.
	 */
	spdt->spdt$l_requestq_in = requestq_in;

#if PKQ_DEBUG	
trace_entry(spdt,isp_entry);
#endif

	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_mailbox_4, requestq_in );
	spdt->spdt$l_pkq_flags &= ~SPDT$M_PKQ_SEND_MARKER;
	avail_entries--;    
    } else {
    /*
     *  No room in the ring.  This shouldn't happen so crash.
     */
	pkq_bugcheck();
    }

    if( target_mode( spdt) && 
			 (spdt->spdt$l_pkq_flags & SPDT$M_PKQ_SEND_ENABLE_LUN) )
    {
	if( avail_entries > 1 ) 
	{
	    send_enable_lun( spdt );
	}	
        else 
	{
    	    /*
    	     *  No room in the ring.  This shouldn't happen so crash.
     	     */
	    pkq_bugcheck();
	}
    }
    return(SS$_NORMAL);
}

/*
 *
 *   Name:	pk$cmd_wait_completion - wait for command completion
 *
 *   Abstract:	Stall the thread until the hardware completes the
 *		command; when the thread is resumed, update the SCDRP
 *		to reflect the completion and return to the caller of
 *		pk$send_command.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *		SCDT address
 *		STDT address
 *
 *   Outputs:	TBS
 *
 *   Return
 *   Values:	port status
 *
 */
int pk$cmd_wait_completion( PKQ_SPDT *spdt,
			   SCDRP *scdrp,
			   SCDT *scdt,
			   STDT *stdt )
{
    KPB		*kpb;					/* Kernel Process Block pointer		*/
    QBUF	*qbuf;					/* QBUF pointer				*/
    uint32	status;					/* VMS status				*/

    /* Prepare for the stall:
     *  - Get the KPB address;
     *  - Set up the SPDT and SCDRP addresses in the KPB;
     *  - Set up the restart and stall routine pointers
     *    to do nothing;
     *  - Stall until this thread is resumed.
     */
    kpb = scdrp->scdrp$ps_kpb;
    kpb->kpb$ps_scsi_ptr1 = spdt;
    kpb->kpb$ps_scsi_scdrp = scdrp;
    kpb->kpb$ps_sch_restrt_rtn = NULL;
    kpb->kpb$ps_sch_stall_rtn = ioc$return;
    qbuf = ( QBUF *)(( uint8 * )scdrp->scdrp$l_cmd_ptr -
		     offsetof( QBUF, qbuf$l_scsi_cdb_size ));
#if PKQ_DEBUG	
    trace_event(spdt,'wait', (uint32)kpb , (uint32)scdrp , (uint32)qbuf );
#endif
    if(qbuf->qbuf$l_port_status == 0)
    {
	status = exe$kp_stall_general( kpb );
    }
    else
    {
	status = qbuf->qbuf$l_port_status;
#if PKQ_DEBUG	
    	trace_event(spdt,'nwai', (uint32)kpb , (uint32)scdrp , exe$gl_abstim );
#endif
    }	
#if PKQ_DEBUG	
    trace_event(spdt,'cont', (uint32)kpb , (uint32)scdrp , status);
#endif

    /* Resume after the stall:
     *  - Get the QBUF address;
     *  - Store the SCSI status in the SCDRP;
     *  - Compute the actual transfer count;
     *  - If there is valid autosense data, store its length
     *    and address in the SCDRP;
     *  - Return the port status to the caller of pk$send_command.
     */
    qbuf = ( QBUF *)(( uint8 * )scdrp->scdrp$l_cmd_ptr -
		     offsetof( QBUF, qbuf$l_scsi_cdb_size ));
    *scdrp->scdrp$l_sts_ptr = qbuf->qbuf$b_scsi_status;
    scdrp->scdrp$l_trans_cnt = scdrp->scdrp$l_bcnt +
	scdrp->scdrp$l_pad_bcnt -
	    qbuf->qbuf$l_bytes_not_xfrd;
    if( qbuf->qbuf$l_flags & QBUF$M_FL_AUTOSENSE_VALID ) {
	scdrp->scdrp$l_scsi_flags |= SCDRP$M_FLAG_ASENSE_VALID;
	scdrp->scdrp$is_sense_buffer_len = qbuf->qbuf$w_autosense_length;
	scdrp->scdrp$ps_sense_buffer = &qbuf->qbuf$b_autosense;
    }
    if( SUCCESS( status )) {
	status =  qbuf->qbuf$l_port_status;
    }
    return( status );
}

/*
 *
 *   Name:	pk$interrupt - field hardware interrupts
 *
 *   Abstract:	Process an interrupt from the ISP1020 RISC:
 *		if the interrupt was legitimate, disable
 *		diable interrupts and queue a fork process
 *		to do the real work.
 *
 *   Inputs:	IDB address
 *
 *   Implicit
 *   inputs:	Device IPL
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	Interrupt disabled
 *
 *   Return
 *   Values:	None
 *
 */
void pk$interrupt( IDB *idb )
{
    SCSI_UCB	*ucb;				/* Unit Control Block pointer			*/
    PKQ_SPDT	*spdt;				/* SCSI Port Descriptor Table pointer		*/
    int		saved_ipl;			/* IPL saved/restored by device_lock/unlock	*/
    DECLARE_ISP();				/* Define ISP_REGISTER structure and temps	*/

    /* Process ISP interrupts:
     *  1. Take out the device lock for the port;
     *  2. If there was an interrupt for this device:
     *    a. Disable port interrupts;
     *    b. Restore the device lock;
     *    c. Schedule a fork process to process the condition;
     *	3. Otherwise, the interrupt was spurious; ignore it by
     *     simply restoring the lock.
     */
    ucb = idb->idb$ps_auxstruc;
    spdt = ( PKQ_SPDT *)ucb->ucb$r_scsi_ucb.ucb$l_pdt;
#if PKQ_DEBUG	
    trace_event(spdt,'intr', (uint32)ucb ,(uint32)idb  , exe$gl_abstim );
#endif
    device_lock( spdt->basespdt.spdt$l_dlck, RAISE_IPL, &saved_ipl );
    if( READ_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		 isp$w_bus_isr) & ISP$M_ISR_RISC_INT ) {
	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_bus_icr, 0 );
/*
**	This was put in to ensure that the write to the icr gets done before
**	we return from the interrupt. It was replaced with the fork block
**	interlock code below. Now if we return with interrupts still enabled
**	we'll get another interrupt which will see the ucb fkb in use and
**	return after it reads the ICR register.
**
**	READ_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
**		isp$w_bus_icr);
**
*/
    	if( ucb->ucb$r_erlucb.ucb$r_ucb.ucb$l_fpc != 0 ) 
    	{
#if PKQ_DEBUG	
    	    trace_event(spdt,'reen', 'ter ' , 'intr'  , exe$gl_abstim );
#endif
	    READ_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		isp$w_bus_icr);
    	}
    	else
    	{    
	    fork( pkq_interrupt_fork, idb, spdt, ucb );
    	}
    }
    device_unlock( spdt->basespdt.spdt$l_dlck, saved_ipl, SMP_RESTORE );
}

/*
 *
 *   Name:	pkq_interrupt_fork - Process errors and response queue
 *
 *   Abstract:	Identify the cause the interrupt, process serious errors
 *		and then loop through the response queue completing requests.
 *
 *   Inputs:	IDB address
 *		SPDT address
 *		UCB address
 *
 *   Outputs:	SPDT
 *		    spdt$l_responseq_out	updated
 *		    
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_interrupt_fork( IDB *idb,	 	/* Interrupt Data Block				*/
			PKQ_SPDT *spdt,		/* SCSI Port Descriptor Table			*/
			SCSI_UCB *ucb )		/* Unit Control Block				*/
{
    int			status;			/* status value					*/
    DECLARE_ISP();				/* Declare ISP register file and temps		*/
    ADP			*adp;			/* Adaptor block pointer			*/
    uint64		*handle;		/* Pointer to ISP register file handle		*/
    uint32		mb_status;		/* Status returned in ISP Mailbox 0		*/
    uint32		responseq_in;		/* Next entry for driver to handle		*/
    uint32		responseq_out;		/* Last entry that ISP chip filled in		*/
    KPB			*kpb;			/* Kernel Process Block pointer		*/
    SCDRP		*scdrp;			/* SCSI class driver request packet		*/
    QBUF		*qbuf;			/* Queue buffer pointer				*/
    ISP_ENTRY		*isp_entry;		/* Current ISP entry				*/
    ISP_ENTRY		*saved_atio;		/* allocated atio block				*/
    RSPID		rspid;			/* Response ID					*/
    uint32		reset_flag;		/* Bus reset occurred if TRUE			*/
    int	       		saved_ipl;		/* IPL saved while acquiring device lock	*/
    ISP_ENTRY		*ctio_entry;		/* pointer to ctio entry to avoid ugly casting  */
    uint32		requestq_in;		/* request queue in index	 		*/

    /*
     *  Get the ADP and I/O handle addresses.  Set the bus reset detected flag
     * to FALSE.
     * Also check to see if queue manager needs to be restarted by calling
     * resume_qman.
     */
#if PKQ_DEBUG	
    trace_event(spdt,'ifrk', 0 , 0 , exe$gl_abstim );
#endif
    /*
    **	Reenable use of fork block by interrupt service routine.
    */
    adp	= spdt->basespdt.spdt$l_adp;
    handle = &spdt->spdt$q_iohandle_reg;
    reset_flag = FALSE;

    /* Main processing loop.  Determine the cause of the interrupt and process
     * accordingly.
     */
    do {

	/* The ISP1020 sets the bus semaphore lock to indicate the presence of
	 * a serious error or the completion of a requested mailbox operation.
	 * Read and dispatch on the status code presented in mailbox register 0.
	 */
	if( READ_ISP( adp, handle, isp$w_bus_semaphore ) & ISP$M_SEMAPHORE_LOCK ) {
	    mb_status = ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_0 );
#if PKQ_DEBUG	
	    {
		uint32 r1,r2,r3,r4,r5;
	    	trace_event(spdt,'ifrk', 'mbst' , mb_status , spdt->spdt$l_responseq_in );
	    	r1 = ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_1 );
	    	r2 = ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_2 );
	    	r3 = ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_3 );
	    	r4 = ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_4 );
	    	r5 = ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_5 );
		trace_event(spdt,' if1',r1,r2,r3);		
		trace_event(spdt,' if2',r4,r5,exe$gl_abstim);		
	    } 
#endif

	    switch( mb_status ) {

	    /* A SCSI bus reset has been detected.  Acquire the device lock, call
	     * the reset detected wait routine in scsi2common to prevent the queue
	     * manager from passing through any new requests.  Then restore the
	     * device lock, and fall through to process any I/Os that may have
	     * completed before the reset.
	     */
	    case ISP$K_MB_STS_SCSI_BUS_RESET:
	    case ISP$K_MB_STS_TIMEOUT_RESET:
		pkq_log_error( SCSI$C_BUS_RESET, 0, 0, spdt );
		reset_flag = TRUE;
		device_lock( spdt->basespdt.spdt$l_dlck, RAISE_IPL, &saved_ipl );
		spdt->basespdt.spdt$ps_rl_reset_detected_wait( spdt );
		device_unlock( spdt->basespdt.spdt$l_dlck, saved_ipl, SMP_RESTORE );
		resume_qman( spdt, TRUE ); 
		break;

	    /* The request queue has reached the threshhold specified in a
	     * WAKEUP mailbox command.  This driver doesn't currently issue
	     * the WAKEUP command, so ignore this condition.
	     */
	    case ISP$K_MB_STS_COMMAND_COMPLETE:
	    case ISP$K_MB_STS_ALIVE:
	    case ISP$K_MB_STS_REQUEST_Q_WAKEUP:
		break;
	    /*
	     * Here the isp1020 is notifying us that a data overrun has occured 
	     * on a data out command. This could be because a target has gone
	     * wacky or because of a failure in the isp1020. Most likely it is
	     * because of bad signal integrety on the bus causeing causing
	     * reqs/acks to get lost (or counted twice). In any case it is
	     * likely that data has been corrupted and we need to take serious
	     * action. We will reset the bus. This will cause disk IO's to be
	     * retried and tape IO's to go back to MTACP.
	     */

	    case ISP$K_MB_STS_OVERRUN_NO_RESET:

		pkq_log_error( SCSI$C_CTL_ERR, SCTL$K_MAPOVRR , 0, spdt );
		spdt->basespdt.spdt$ps_rl_queue_reset_fork( spdt , 0 );
		resume_qman(spdt, TRUE );
		break;

	    /* One of the following has occurred:
	     *  - the ISP1020 firmware has detected an internal error;
	     *  - the firmware was unable to transfer a request queue entry
	     *    into its internal store;
	     *  - the firmware was unable to transfer a response from its
	     *    internal store to the response queue;
	     *  - the status code was unrecognized.
	     * Log the error condition, then reinitialize the chip from the
	     * ground up before restarting.
	     */
	    case ISP$K_MB_STS_SYSTEM_ERROR:
		/* 
		** log an error and start kp to initialize the chip.
		*/ 
		pkq_log_error( SCSI$C_BUS_ERR, 0, 0, spdt );
		ucb->ucb$r_scsi_ucb.ucb$l_sts &= ~UCB$M_ONLINE;
		spdt->basespdt.spdt$l_sts &= ~SPDT$M_STS_ONLINE;
		kpb = spdt->spdt$ps_kpb;
		if(( kpb->kpb$is_flags & KPB$M_VALID ) == 0 ) 
		{
		    exe$kp_start( kpb, pkq_port_init, SA$K_KP_REGMSK );
		}
		break;			/* 
					** the kp will reset the bus so don't
					** fall through
					*/

	    case ISP$K_MB_STS_REQUEST_XFER_ERROR:
	    case ISP$K_MB_STS_RESPONSE_XFER_ERROR:
	    case ISP$K_MB_STS_INVALID_COMMAND:
	    case ISP$K_MB_STS_HOST_INTFC_ERROR:
	    case ISP$K_MB_STS_TEST_FAILED:
	    case ISP$K_MB_STS_COMMAND_ERROR:
	    case ISP$K_MB_STS_PARAMETER_ERROR:
	    case ISP$K_MB_STS_CHECKSUM_ERROR:
	    case ISP$K_MB_STS_SHADOW_LOAD_ERROR:
	    case ISP$K_MB_STS_BUSY:

	    default:
		pkq_log_error( SCSI$C_BUS_RESET, 0, 0, spdt );
		spdt->basespdt.spdt$ps_rl_queue_reset_fork( spdt , 0 );
		resume_qman( spdt, TRUE );
		break;
	    }

	    /* Clear the semaphore lock to allow the ISP1020 to update
	     * outgoing mailbox registers 0 - 4.
	     */
	    WRITE_ISP( adp, handle, isp$w_bus_semaphore, 0 );
	}

	/* Process the response queue ring:
	 *  Responseq_in is the index of the next entry to be filled by
	 *  the ISP1020 firmware, obtained by reading outgoing mailbox
	 *  register 5.  Responseq_out is the index of the next entry to
	 *  be emptied by this routine.  Processing continues until the
	 *  responseq_out is equal to responseq_in.
	 *  Note that mailbox register 5 may be updated as soon as the
	 *  RISC interrupt is cleared.
	 */
#if PKQ_DEBUG
	spdt->spdt$l_responseq_in = 
#endif
	responseq_in =
	    ( uint32 )READ_ISP( adp, handle, isp$w_mailbox_5 );
	WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_CLR_RISC_INT ); 
	responseq_out = spdt->spdt$l_responseq_out;
	while( responseq_out != responseq_in ) {

	    isp_entry = spdt->spdt$ps_responseq_va + responseq_out;

	    /* Check to see that the response is a status entry.
	     */
	    if( isp_entry->isp$b_type == ISP$K_TY_STATUS ) {

		/* Use the RSPID in the handle field of the response to locate the
		 * SCDRP and QBUF.  It is possible that the request may have already
		 * been completed by a bus reset thread; if so, there will not be a
		 * valid request corresponding to this status response.  In that case,
		 * discard the status response.
		 */
		rspid = *( RSPID * )&isp_entry->isp$l_handle;
		if( scdrp = pkq_rspid_to_scdrp( spdt, rspid  )) {
		    qbuf = ( QBUF *)(( uint8 * )scdrp->scdrp$l_cmd_ptr -
				     offsetof( QBUF, qbuf$l_scsi_cdb_size ));

		    qbuf->qbuf$l_bytes_not_xfrd = isp_entry->isp$l_bytes_not_xfrd;
#if PKQ_DEBUG	
		    trace_event(spdt,'iodn',rspid.index+(rspid.seq_no<<16) , 
		    			(((exe$gl_abstim & 0xffff ) << 16) +
				        		(responseq_in<<8)   +
				        		responseq_out   )  , 
		    			(uint32)qbuf );
#endif
		    /* If the port completion status and the SCSI status are both
		     * normal, continue processing in line; otherwise, process the
		     * error before continuing.
		     */
		    if(( isp_entry->isp$w_completion_status == ISP$K_CS_COMPLETE ) &&
		       ( isp_entry->isp$b_scsi_status == SCSI$C_GOOD )) {
			qbuf->qbuf$b_scsi_status = SCSI$C_GOOD;
			qbuf->qbuf$l_port_status = SS$_NORMAL;
		    } else {
			pkq_process_error( spdt, qbuf, isp_entry );
		    }

		    /* Restart the kernel process for the completed response.
		     */
		    exe$kp_restart( scdrp->scdrp$ps_kpb, SS$_NORMAL );
		} else {
		    spdt->spdt$is_pkq_rspid_stale_count++;
		}

	    /* The entry is not a status entry.  Figure out what's up.
	     */
	    } else {
#if PKQ_DEBUG	
		trace_entry(spdt,isp_entry);
#endif
		
	        /* If the entry has FULL set in the header flags, it is
		 * a request that was rejected because the ISP1020 internal
		 * queue is full.  Complete the request with SCSI$C_QUEUE_FULL
		 * and SS$_NORMAL.
		 *
		 *	Note: fix the uninitialized qbuf problem here.
		 */
		if( isp_entry->isp$b_header_flags & ISP$M_HF_FULL ) {
		    qbuf = NULL;
		    rspid = *( RSPID * )&isp_entry->isp$l_handle;
		    if( scdrp = pkq_rspid_to_scdrp( spdt, rspid  )) 
		    {
		    	qbuf = ( QBUF *)(( uint8 * )scdrp->scdrp$l_cmd_ptr -
				     offsetof( QBUF, qbuf$l_scsi_cdb_size ));
		    }
		    if( qbuf != NULL ) 
		    {
		    	qbuf->qbuf$b_scsi_status = SCSI$C_QUEUE_FULL;
		    	qbuf->qbuf$l_port_status = SS$_NORMAL;
		    	scdrp = ( SCDRP * )qbuf->qbuf$ps_scdrp;
		    	exe$kp_restart( scdrp->scdrp$ps_kpb, SS$_NORMAL );
#if PKQ_DEBUG	
		    	trace_event(spdt,'qful', 'rply' , (uint32)qbuf , (uint32)scdrp );
#endif
		    }
		/*
		 *  Note: Nothing below here is particularly performance
		 *	sensitive.
		 */

		/*
		 * If this is an ATIO type then we need to start a target
		 * mode operation.
		 *
		 * Build a ctio entry in the request queue.
		 *
		 */

		} else if( target_mode( spdt ) && 
				     isp_entry->isp$b_type == ISP$K_TY_ATIO  ) {
		    status = alloc_atio_buffer( spdt, isp_entry, &saved_atio);
		    if( status != SS$_NORMAL) pkq_bugcheck();
		    requestq_in = spdt->spdt$l_requestq_in;
		    ctio_entry = spdt->spdt$ps_requestq_va + requestq_in;
		    build_ctio( spdt, isp_entry, ctio_entry);
		    set_reqq_entry(spdt, requestq_in);		    
#if PKQ_DEBUG	
trace_entry(spdt,ctio_entry);
#endif

						
		/*
		 * It this is a CTIO type then we need to finish a target
		 * mode operation, Return the ATIO to the target mode firmware.
		 */

		} else if( target_mode( spdt ) &&
				    isp_entry->isp$b_type == ISP$K_TY_CTIO  ) {
		    /*
		     * Was the operation successful?
		     */
		     if(  isp_entry->isp$b_status  == 
						  ISP$K_CTIO_COMPLETED_WO_ERROR)
		     {
			return_atio(spdt, isp_entry);
		     }
		     else if( (isp_entry->isp$b_status == 
					ISP$K_CTIO_ABORTED_BY_HOST )||
			      (isp_entry->isp$b_status ==
					ISP$K_CTIO_UNACKED_EVENT_BY_HOST))
		     {
			/* 
			** here we should expect an immediate notify so don't
			** send the atio back yet.
			*/
			
		     }
		     else if( (isp_entry->isp$b_status == 
						  ISP$K_CTIO_SCSI_BUS_RESET ) )
		     {

			return_atio(spdt, isp_entry);

		     }
		     else
		     {
		     	pkq_log_error( SCSI$C_CMD_ABORT, 0 , 0, spdt );
			return_atio(spdt, isp_entry);
		     }; 

		/*
		 * It this is an immediate notify  type then we need to handle
		 * a target mode operation error
		 */

		} else if(target_mode( spdt) && 
				( isp_entry->isp$b_type == ISP$K_TY_NOTIFY)  ) {
		    ack_notify(spdt, isp_entry);
		    return_atio(spdt,isp_entry);
		/*
		 * All we need to do with enable lun is set the bit that says
		 * that we got it.
		 */

		} else if( target_mode( spdt) && 
			       isp_entry->isp$b_type == ISP$K_TY_ENABLE_LUN  ) {
		    spdt->spdt$l_pkq_flags |= SPDT$M_PKQ_GOT_ENABLE_LUN;


		/* If the entry has BADHEADER or BADPAYLOAD set in the header
		 * flags, the driver has an internal error.  Bugcheck.
		 */
		} else if( isp_entry->isp$b_header_flags &
			  ( ISP$M_HF_BADHEADER | ISP$M_HF_BADPAYLOAD )) {
		    pkq_bugcheck();
		}

		/* Otherwise, the entry is most likely a request being
		 * returned after a bus reset.  Ignore it and let the queue
		 * manager deal with restarting the request.
		 */
	    }

	    /* Bump to the next response queue entry modulo queue size.
	     */
	    responseq_out++;
	    if( responseq_out >= RESPONSE_QUEUE_SIZE ) {
		responseq_out = 0;
	    }
	    /*
	    ** conditionally resume the queue manager ( ie decrement count and
	    ** restart if zero.)
	    */
	    resume_qman( spdt, FALSE );
	}

	/* Emptied the response ring.  Notify the ISP1020 by writing the
	 * updated index into incoming mailbox 5 and store it in the SPDT.
	 */
	WRITE_ISP( adp, handle, isp$w_mailbox_5, responseq_out );
	spdt->spdt$l_responseq_out = responseq_out;

    /* If the ISP1020 RISC has another interrupt pending, continue.
     */
    } while( READ_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		      isp$w_bus_isr ) & ISP$M_ISR_RISC_INT );

    /* If a bus reset was detected above, call reset_detected_fork to complete
     * the reset clean-up.  The reset code is called directly rather than being
     * scheduled as a separate fork process as would happen if reset_detected_dipl
     * were being used.
     */
    if( reset_flag ) {
#if PKQ_DEBUG	
	trace_event(spdt,'rst ', 'dete', 'cted' , exe$gl_abstim );
#endif

	spdt->spdt$l_pkq_flags |= (SPDT$M_PKQ_SEND_MARKER ); 
	spdt->basespdt.spdt$ps_rl_reset_detected_fork( spdt );
    }
    
    /* Enable port interrupts.
     */
    ucb -> ucb$r_erlucb.ucb$r_ucb.ucb$l_fpc = 0;
    WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
	      isp$w_bus_icr, ( ISP$M_ICR_ALL_INT_ENB | ISP$M_ICR_RISC_INT_ENB ));
}

/*
 *
 *   Name:	pkq_process_error - process completions with error
 *
 *   Abstract:	Process responses that indicate either a SCSI or
 *		ISP1020 transport error.
 *
 *   Inputs:	QBUF address
 *		ISP_ENTRY address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:   QBUF
 *		    qbuf$b_autosense_length
 *		    qbuf$b_autosense
 *		    qbuf$l_flags
 *		    qbuf$l_port_status
 *		    qbuf$b_scsi_status
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_process_error( PKQ_SPDT *spdt,
			QBUF *qbuf,
		        ISP_ENTRY *isp_entry)
{
    /* Process error conditions:  this routine is entered when either
     * the SCSI status or the ISP response entry completion status
     * indicates an error.
     *
     * If the ISP response is COMPLETE, a SCSI error has occurred.
     * If a check condition has occurred and autosense is enabled,
     * move the autosense data and length into the QBUF.
     *
     * If the ISP response is other than COMPLETE, a transport error
     * of some sort has occurred.
     * - Use the error status map to convert the ISP completion status
     *   into a VMS status code.
     * - If required, use the ISP operation flags determine the VMS 
     *   status code.
     * - Log the error if appropriate.
     * - Return the VMS status code ... TODO
     *   
     */
    int		isp_status;
    int		vms_status;
    int		sense_data_length;
    isp_status = isp_entry->isp$w_completion_status;
    if( isp_status == ISP$K_CS_COMPLETE ) {
	if( isp_entry->isp$b_scsi_status == SCSI$C_CHECK_CONDITION ) {
	    if( isp_entry->isp$v_sf_got_sense ) {
		sense_data_length = isp_entry->isp$w_sense_data_length;
		if( sense_data_length > QBUF$K_AUTOSENSE_SIZE ) {
		    sense_data_length = QBUF$K_AUTOSENSE_SIZE;
		}
		qbuf->qbuf$w_autosense_length = sense_data_length;
		memcpy( qbuf->qbuf$b_autosense,
		       isp_entry->isp$b_sense_data,
		       sense_data_length );
		qbuf->qbuf$l_flags |= QBUF$M_FL_AUTOSENSE_VALID;
		vms_status = SS$_NORMAL;
	    }
	} else {
	    vms_status = SS$_NORMAL;
	}
    } else {
	if( isp_status < err_status_map_size ) {
  	    if( isp_status == ISP$K_CS_RESET )
 	    {
	    	spdt->spdt$l_pkq_flags |= SPDT$M_PKQ_SEND_MARKER;
#if PKQ_DEBUG
		trace_event(spdt,
				'rrtn',
				(uint32)isp_entry,
				(uint32)isp_entry->isp$l_handle,
				(uint32)isp_status);		
#endif

	    }
	    vms_status = err_status_map[ isp_status ].vms_status;
	    if( err_status_map[ isp_status ].flags & PKQ_M_ESM_USEOF ) {
		if( isp_entry->isp$w_bus_operation & ISP$M_OF_TIMEOUT )
		    vms_status = SS$_TIMEOUT;
		else if( isp_entry->isp$v_of_aborted )
		    vms_status = SS$_ABORT;
		else if( isp_entry->isp$v_of_device_reset )
		    vms_status = SS$_DEVOFFLINE;
		else if( isp_entry->isp$v_of_bus_reset )
		    vms_status = SS$_MEDOFL;
		else if( isp_entry->isp$v_of_parity_error )
		    vms_status = SS$_PARITY;
	    }
	    if( err_status_map[ isp_status ].flags & PKQ_M_ESM_LOG ) {
		/* Log the error */
	    } else if( err_status_map[ isp_status ].flags & PKQ_M_ESM_LOGI ) {
		/* Log the error if we're done initialization */
	    }
	} else {
	    vms_status = SS$_CTRLERR;
	    /* Log the error */
	}
    } 
    qbuf->qbuf$b_scsi_status = isp_entry->isp$b_scsi_status;
    qbuf->qbuf$l_port_status = vms_status;
}

/*
 *
 *   Name:	pk$negotiate_synch - (Re-)negotiate synchronous transfers
 *
 *   Abstract:	This routine is intended to cause renegotiation of
 *		synchronous transfers.  It doesn't cause any direct
 *		action at the port level.
 *
 *   Inputs:	SPDT address
 *		STDT address
 *
 *   Outputs:
 *
 *   Implicit
 *   outputs:	STDT
 *		   stdt->stdt$is_dipl_flags		renegotiate_synch set
 *
 *   Return
 *   Values:	SS$_NORMAL
 *
 */
int pk$negotiate_synch( PKQ_SPDT *spdt,			/* SCSI Port Descriptor Table		*/
		       STDT *stdt )			/* SCSI Target Descriptor Table		*/
{
    /* Turn on bit to cause synchronous negotiation.  This currently
     * doesn't do anything to the target at all.
     */
    stdt->stdt$is_dipl_flags |= STDT$M_DFLG_RENEGOTIATE_SYNC;
    return( SS$_NORMAL );
}

/*
 *
 *   Name:	pk$abort_command - abort a single command
 *
 *   Abstract:	This routine is intended to abort a single command:
 *		it is currently disabled.
 *
 *   Inputs:	SPDT address
 *		SCDRP address
 *		SCDT address
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	SS$_NORMAL
 *
 */
int pk$abort_command( PKQ_SPDT *spdt,
		     SCDRP *scdrp,
		     SCDT *scdt )
{
#if 0
    STDT	*stdt = scdt->scdt$ps_stdt;
    pkq_mailbox_io( spdt,
		   4, ISP$K_MB_ABORT, &spdt->spdt$w_mailbox_0,
		   (( stdt->stdt$ib_target_id << 8 ) |
		    ( scdt->scdt$l_scsi_lun & 0xff )),
		   (( uint32 )scdrp >> 16 ),
		   (( uint32 )scdrp & 0xffff ));
#endif
    return( SS$_NORMAL );
}


/*
 *
 *   Name:	pkq_log_error - Log an attention error
 *
 *   Abstract:	Log an attention error from the port.
 *
 *		
 *
 *   Inputs:	error type
 *		error subtype
 *		SCDT address or 0
 *		SPDT address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	TBS
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_log_error( uint8 type,
		   uint8 subtype,
		   SCDT *scdt,
		   PKQ_SPDT *spdt )
{
    SCSI_UCB *ucb = ( SCSI_UCB * )spdt->basespdt.spdt$l_port_ucb;
    spdt->basespdt.spdt$iw_erl_type = ( int16 )( type + ( subtype << 8 ));
    spdt->basespdt.spdt$ps_erl_scdt = scdt;
    erl_std$deviceattn( spdt, ucb );
}
    

/*
 *
 *   Name:	pkq_register_dump
 *
 *   Abstract:	Fill in the supplied error log buffer with device
 *		registers and other SCSI port information.
 *
 *   Inputs:	error log buffer address
 *		SPDT address
 *		UCB address
 *
 *   Outputs:	None
 *
 *   Implicit
 *   outputs:	Many error log buffer fields filled in.
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_register_dump( uint8 *buffer,
		  PKQ_SPDT *spdt,
		  SCSI_UCB *ucb )
{

    DECLARE_ISP();
    uint8	*curbuf;
    SCDT	*scdt;
    uint32	type;
    uint32	subtype;

    /* Routine-wide initialization:
     *  - initialize the current buffer pointer
     *  - get the SCDT address, if any
     *  - get the errlog SCSI entry type and subtype
     */
    curbuf = buffer;
    scdt = ( SCDT * )spdt->basespdt.spdt$ps_erl_scdt;
    type = spdt->basespdt.spdt$iw_erl_type & 0xff;
    subtype = ( spdt->basespdt.spdt$iw_erl_type >> 8 ) & 0xff;

    /* Fill in the header portion of the error log entry
     * except for the entry length in longwords which will
     * be filled in at the end of the routine.
     */
    {
	PKQ_ERL_HDR	*erl = ( PKQ_ERL_HDR * )curbuf;
	STDT		*stdt;
	erl->pkq_erl$b_rev = PKQ_ERL_REV;
	erl->pkq_erl$w_type = spdt->basespdt.spdt$iw_erl_type;
	erl->pkq_erl$b_scsi_id = 0xFF;
	if( scdt && ( stdt = scdt->scdt$ps_stdt )) {
	    erl->pkq_erl$b_scsi_id = stdt->stdt$is_scsi_id_num;
	}
	curbuf += sizeof( PKQ_ERL_HDR );
    }

    /* Fill in the SCSI Command Data Block if it can
     * be located; otherwise, provide a null field.
     */
    {
	PKQ_ERL_CMD	*erl = ( PKQ_ERL_CMD * )curbuf;
	KPB		*kpb;
	SCDRP		*scdrp;
	char		*cmdptr;
	uint8		cmdlth;
	uint32		i;

	/* The SCDT, KPB, SCDRP and its cmd_ptr field must all
	 * be present in order to locate the SCSI command data block.
	 */
	if( scdt && 
	   ( kpb = spdt->basespdt.spdt$ps_chip_kpb ) &&
	   ( scdrp = kpb->kpb$ps_scsi_scdrp ) &&
	   ( cmdptr = scdrp->scdrp$l_cmd_ptr )) {
	    cmdlth = *cmdptr++;
	    erl->pkq_erl$b_scsi_cdb_size = cmdlth;
	    memcpy( erl->pkq_erl$b_scsi_cdb, cmdptr, cmdlth );
	    curbuf += ( cmdlth + 1 );
	} else {
	    erl->pkq_erl$b_scsi_cdb_size = 0;
	    curbuf++;
	}
    }

    /* Fill in the request/response ISP queue entries 
     * for those error types in which they are relevant.
     */
    {
	PKQ_ERL_MSG	*erl = ( PKQ_ERL_MSG * )curbuf;
	ISP_ENTRY	*isp_entry;		/* ISP entry					*/ 
	uint32		requestq_in;
	uint32		responseq_out;
	int32		i;

	/* On this adaptor, "bus hung" means "chip hung" and
	 * "bus err" means "chip internal error".
	 */
	if(( type == SCSI$C_BUS_HUNG ) ||
	   ( type == SCSI$C_BUS_ERR )) {
	    requestq_in = spdt->spdt$l_requestq_in;

	    /* Look backward in the request ring for the most
	     * recent command entry.  If the message ring is
	     * well-formed, it will be found within the last
	     * MIN_FREE_ENTRIES entries; copy it to the
	     * error log buffer.  Otherwise, set the field to
	     * NULLs.
	     */
	    for( i = 0; i < MIN_FREE_ENTRIES; i++ ) {
		if( requestq_in == 0 ) {
		    requestq_in = REQUEST_QUEUE_SIZE - 1;
		}
		--requestq_in;
		isp_entry = spdt->spdt$ps_requestq_va + requestq_in;
		if( isp_entry->isp$b_type == ISP$K_TY_COMMAND ) {
		    break;
		} else {
		    isp_entry = 0;
		}
	    }
	    if( isp_entry ) {
		memcpy( erl->pkq_erl$b_request, isp_entry, ISP$K_ENTRY_LTH );
	    } else {
		memset( erl->pkq_erl$b_request, 0, ISP$K_ENTRY_LTH );
	    }

	    /* Copy the next response to be processed into the error log
	     * entry, then update the current buffer pointer.
	     */
	    responseq_out = spdt->spdt$l_responseq_out;
	    isp_entry = spdt->spdt$ps_responseq_va + responseq_out;
	    memcpy( erl->pkq_erl$b_response, isp_entry, ISP$K_ENTRY_LTH );
	    erl->pkq_erl$b_msg_lth = ( 2 * ISP$K_ENTRY_LTH );
	    curbuf += sizeof( PKQ_ERL_MSG );

	/* Provide a null msg segment for other error types.
	 */
	} else {
	    erl->pkq_erl$b_msg_lth = 0;
	    curbuf++;
	}
    }

    /* Store the error status byte if the containing structures
     * are all present.
     */
    {
	PKQ_ERL_STATUS	*erl = ( PKQ_ERL_STATUS * )curbuf;
	KPB		*kpb;
	SCDRP		*scdrp;
	int		*stsptr;

	/* Assume the status is not available and provide a
	 * dummy status byte.
	 */
	erl->pkq_erl$b_status = 0xff;

	/* The error log SCDT, KPB, SCDRP and its sts_ptr field must all
	 * be present in order to locate the SCSI status byte.
	 */
	if( scdt && 
	   ( kpb = spdt->basespdt.spdt$ps_chip_kpb ) &&
	   ( scdrp = kpb->kpb$ps_scsi_scdrp ) &&
	   ( stsptr = scdrp->scdrp$l_sts_ptr )) {
	    erl->pkq_erl$b_status = *stsptr;
	}
	curbuf += sizeof( PKQ_ERL_STATUS );
    }

    /* Pause the ISP RISC micro processor, so that RISC register
     * contents are frozen.
     */
    {
	ADP		*adp = spdt->basespdt.spdt$l_adp;
	uint64		*handle = &spdt->spdt$q_iohandle_reg;
	uint32		regnum;
	uint32		offset;
	uint32		temp;
	uint32		interval;

	/* Write a pause request into the HCCR and then wait
	 * a while for it to happen.  If we manage to exhaust
	 * the interval count fall through anyway to take whatever
	 * register values we can get.
	 */
	WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_PAUSE );
	interval = 10000;
	while( --interval &&
	      !( READ_ISP( adp, handle, isp$w_hccr )
		& ISP$M_HCCR_PAUSE_MODE  )) {}
    }

    /* Dump the device registers if they are relevant.
     */
    {
	PKQ_ERL_REGS	*erl = ( PKQ_ERL_REGS * )curbuf;
	ADP		*adp = spdt->basespdt.spdt$l_adp;
	uint64		*handle = &spdt->spdt$q_iohandle_reg;
	uint32		regnum;
	uint32		offset;
	uint32		temp;

	/* If the error being logged is a hung bus or bus error, dump
	 * all of the PCI, RISC and DMA registers in the ISP1020.
	 */
	if(( type == SCSI$C_BUS_HUNG ) ||
	   ( type == SCSI$C_BUS_ERR )) {
	    erl->pkq_erl$b_regs_lth = ISP$K_REG_FILE_SIZE;
	    for( regnum = 0; regnum < ISP$K_REG_FILE_CNT; regnum++ ) {
		offset = 2 * regnum;
		if( SUCCESS( ioc$read_io( adp, handle, offset, sizeof( uint16 ), &temp ))) {
		    if( offset & 2 ) {
			temp = (( temp >> 16 ) & 0xffff );
		    } else {
			temp  &= 0xffff;
		    }
		} else {
		    temp = 0;
		}
		erl->pkq_erl$w_regs[ regnum ] = temp;
	    }
	    curbuf += sizeof( PKQ_ERL_REGS );
	} else {
	    erl->pkq_erl$b_regs_lth = 0;
	    curbuf++;
	}
    }
    {
	PKQ_ERL_SXP_REGS	*erl = ( PKQ_ERL_SXP_REGS * )curbuf;
	ADP			*adp = spdt->basespdt.spdt$l_adp;
	uint64		*handle = &spdt->spdt$q_iohandle_reg;
	uint32		regnum;
	uint32		offset;
	uint32		temp;

	/* If the error being logged is a hung bus or bus error, dump
	 * all of the SXP (SCSI processor) registers in the ISP1020.
	 */
	if(( type == SCSI$C_BUS_HUNG ) ||
	   ( type == SCSI$C_BUS_ERR )) {
	    erl->pkq_erl$b_sxp_regs_lth = ISP$K_SXP_REG_SIZE;

	    /* Select the SXP register set while preserving the FIFO threshhold
	     * (and setting burst enable)
	     */
	    WRITE_ISP( adp, handle, isp$w_bus_config_1,
		      ( spdt->spdt$v_fifo_threshhold
		       | ISP$M_CONFIG_1_BURST_ENABLE 
		       | ISP$M_CONFIG_1_SEL_SXP_REGS ));

	    /* Dump the SXP registers into the error log buffer.
	     */
	    for( regnum = 0; regnum < ISP$K_SXP_REG_CNT; regnum++ ) {
		offset = 0x80 + ( 2 * regnum );  /* TODO need symbolic offset when doc arrives */
		if( SUCCESS( ioc$read_io( adp, handle, offset, sizeof( uint16 ), &temp ))) {
		    if( offset & 2 ) {
			temp = (( temp >> 16 ) & 0xffff );
		    } else {
			temp  &= 0xffff;
		    }
		} else {
		    temp = 0;
		}
		erl->pkq_erl$w_sxp_regs[ regnum ] = temp;
	    }

	    /* Bump the buffer pointer by the size of the SXP register file.
	     */
	    curbuf += sizeof( PKQ_ERL_SXP_REGS );

	    /* Now DE-Select the SXP register set while preserving the 
	     * FIFO threshhold and setting burst enable.
	     */
	    WRITE_ISP( adp, handle, isp$w_bus_config_1,
		      ( spdt->spdt$v_fifo_threshhold
		       | ISP$M_CONFIG_1_BURST_ENABLE ));

	/* Not logging registers.  Just supply a zero length field.
	 */
	} else {
	    curbuf++;
	    erl->pkq_erl$b_sxp_regs_lth = 0;
	}
    }

    /* Release the ISP RISC micro processor to resume processing.
     */
    {
	ADP		*adp = spdt->basespdt.spdt$l_adp;
	uint64		*handle = &spdt->spdt$q_iohandle_reg;
	uint32		regnum;
	uint32		offset;
	uint16		temp;
	WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_RELEASE );
    }

    /* Fill the entry length in longwords into the error log header.
     */
    {
	PKQ_ERL_HDR	*erl = ( PKQ_ERL_HDR * )buffer;
	erl->pkq_erl$l_lth_in_lw = 
	    ( curbuf - buffer + sizeof( long ) - 1 ) / sizeof( long );
    }
}

/*
 *
 *   Name:	pkq_watchdog - detect hung port
 *
 *   Abstract:	This routine is called periodically to make sure
 *		that the port is still alive.  It issues the
 *		GET FIRMWARE STATUS mailbox command and checks the
 *		return status to ensure that the command completed:
 *		if the command doesn't complete, the port is
 *		reinitialized.
 *
 *   Inputs:	crb	Channel request block address
 *
 *   Outputs:	None.
 *
 *   Return
 *   Values:	None.
 *
 */
void pkq_watchdog( CRB * crb )
{
    PKQ_SPDT		*spdt;			/* SCSI Port Descriptor Table address		*/
    SCSI_UCB		*ucb;			/* SCSI Unit Control Block address		*/
    KPB			*kpb;			/* Kernel Process Block address			*/
    int			status;			/* VMS status					*/

    /* Issue the mailbox command.  If it succeeds, start another timeout
     * and exit.
     */
    spdt = ( PKQ_SPDT * )crb->crb$l_scs_struc;
    if( SUCCESS( status = pkq_mailbox_io( spdt, 2,
					 ISP$K_MB_GET_FIRMWARE_STATUS,
					 0, 0, 0, 0, 0 ))) {
	crb->crb$l_duetime = exe$gl_abstim + PKQ_WATCHDOG_INTERVAL;

    /* The mailbox routine timed out.  Reset timeouts, reset the port
     * and UCB online bits and attempt to restart the port.
     */
    } else {
	ucb = ( SCSI_UCB * )spdt->basespdt.spdt$l_port_ucb;
	crb->crb$l_duetime = -1;
	ucb->ucb$r_scsi_ucb.ucb$l_sts &= ~UCB$M_ONLINE;
	spdt->basespdt.spdt$l_sts &= ~SPDT$M_STS_ONLINE;
	kpb = spdt->spdt$ps_kpb;
	if(( kpb->kpb$is_flags & KPB$M_VALID ) == 0 ) {
	    if( ERROR( exe$kp_start( kpb, pkq_port_init, SA$K_KP_REGMSK ))) {
		ucb->ucb$r_scsi_ucb.ucb$l_devdepend = PKQ$K_ERR_KPSTRT;
	    }
	}
    }
    return;
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
STDT *pkq_get_stdt( PKQ_SPDT *spdt, int scsi_id )
{
    STDT		*stdt;
    int			hash_id;
    
    /* Convert the SCSI ID into a hash table index by using
     * its low order bits.
     */
    hash_id = ( scsi_id & SPDT$K_STDT_HASH_BITMASK )
	<< SPDT$K_STDT_HASH_BITBASE;

    /* Get the address of the first STDT to which the SCSI ID
     * hashes.  If the SCSI ID in the STDT isn't the one desired,
     * chain down the hash list until the correct STDT is found
     * or the list is exhausted.
     */
    stdt = ( STDT * )(( STDT ** )spdt->basespdt.spdt$ps_stdt_hash_table + hash_id );
    while( stdt && ( stdt->stdt$is_scsi_id_num != scsi_id )) {
	stdt = stdt->stdt$ps_hash_flink;
    }

    /* Return the address of the STDT corresponding to the given
     * SCSI ID or if not found, NULL.
     */
    return( stdt );
}

/*
 *
 *   Name: pkq_set_speed
 *
 *   Abstract:
 *
 *	This routine looks at the nvram clock speed parameter and the psr_60mhz
 *	bit in the psr to determine speed. If they are the same then the speed
 *	is set as indicated (40 vs 60 mhz). IF they are different then the 
 *	speed is set based on the psr and a diagnostic message is output.
 *
 *   Inputs:
 *	spdt  	Pointer the the spdt for this port
 *
 *   Outputs:
 *	none
 *
 *   Return
 *   Values:
 *	Status returned from mailbox_io routine;
 *
 */
int pkq_set_speed( PKQ_SPDT *spdt)

{
    DECLARE_ISP();
    ADP			*adp = spdt->basespdt.spdt$l_adp;
    uint64		*handle = &spdt->spdt$q_iohandle_reg;
    uint16		data = 0;
    uint16		fast20 = 0;
    uint16		mask;
    uint16		clock_speed;
/*
 * get the clock designator from the board- have to stop risc processor first.
 */
    if ( ERROR ( pause_risc( spdt ) ) ) return SS$_CTRLERR;
     fast20=((READ_ISP(adp, handle, isp$w_risc_psr) & ISP$M_PSR_60MHZ_CLOCK)!=0);
    release_risc( spdt);

    if(fast20)
    {
	if (spdt->spdt$v_asynch_setup_time < 9 ) 
	{
	    spdt->spdt$v_asynch_setup_time = 9 ;
/*
**	    pkq_print_message( spdt,
**	      " WARNING! Nvram asynchronous setup time value set too low for fast20.",		STS$K_WARNING );
**	    pkq_print_message( spdt,
**		" Using larger value",
**		STS$K_WARNING );
*/
	}
	clock_speed=60;
	return(  pkq_mailbox_io( spdt, 2,
				       ISP$K_MB_SET_CLOCK_RATE,
				       clock_speed,
				       0, 0, 0, 0 ) );
    }

    if( ( fast20 != spdt->spdt$v_60mhz_enable) )
    {
	if( !fast20 )
	{
	    pkq_print_message( spdt, 
		" WARNING! nvram speed doesn't match board clock speed.",
		 STS$K_WARNING );
	    pkq_print_message( spdt, 
		" nvram set to 60 MHZ. Board set to 40MHZ",
		 STS$K_WARNING );
	}
    }
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
int pkq_nvram_read_array( PKQ_SPDT *spdt, ISP_NVRAM *nvram )
{
    int			i;
    uint16		*curbuf;
    uint8		*bytebuf;
    uint8		sum;
    int			status;
    for( i = 0, curbuf = ( uint16 *)nvram; i < ( ISP$K_NVRAM_SIZE / 2 ); i++ ) {
	*curbuf++ = pkq_nvram_read_word( spdt, i );
    }
    if(( nvram->isp$b_nvr_id[ 0 ] != 'I' ) ||
       ( nvram->isp$b_nvr_id[ 1 ] != 'S' ) ||
       ( nvram->isp$b_nvr_id[ 2 ] != 'P' ) ||
       ( nvram->isp$b_nvr_id[ 3 ] != ' ' ) ||
       ( nvram->isp$b_nvr_version < ISP$K_NVR_MIN_VERSION )) {
	return( SS$_BADPARAM );
    }
    for( i = 0, bytebuf = ( uint8 * )nvram, sum = 0; i < ISP$K_NVRAM_SIZE; i++ ) {
	sum += *bytebuf++;
    }
    if( sum ) {
	return( SS$_BADPARAM );
    }
    return( SS$_NORMAL );
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_nvram_write_word( PKQ_SPDT *spdt, uint16 nvaddr, uint16 data )
{
    DECLARE_ISP();
    ADP			*adp = spdt->basespdt.spdt$l_adp;
    uint64		*handle = &spdt->spdt$q_iohandle_reg;
    uint16		mask;
    uint16		data_out_val;
    pkq_nvram_on( adp, handle );
    pkq_nvram_command( adp, handle, ISP$K_NVRAM_WRITE_ENABLE );
    pkq_nvram_toggle( adp, handle );
    pkq_nvram_command( adp, handle, ( ISP$K_NVRAM_WRITE | nvaddr ));
    for( mask = ( 1 << 15 ); mask != 0; mask >>= 1 ) {
	data_out_val = ( data & mask ) ? ISP$M_NVRAM_DATA_OUT : 0;
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | data_out_val );
	pkq_nvram_delay( 2000 );
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | data_out_val | ISP$M_NVRAM_CLOCK );
	pkq_nvram_delay( 2000 );
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | data_out_val );
	pkq_nvram_delay( 2000 );
    }
    pkq_nvram_toggle( adp, handle );
    while( !( READ_ISP( adp, handle, isp$w_bus_nvram ) & ISP$M_NVRAM_DATA_IN )) {
	pkq_nvram_delay( 2000 );
    }
    pkq_nvram_off( adp, handle );
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
uint16 pkq_nvram_read_word( PKQ_SPDT *spdt, uint16 nvaddr )
{
    DECLARE_ISP();
    ADP			*adp = spdt->basespdt.spdt$l_adp;
    uint64		*handle = &spdt->spdt$q_iohandle_reg;
    uint16		data = 0;
    uint16		mask;
    pkq_nvram_on( adp, handle );
    pkq_nvram_command( adp, handle, ( ISP$K_NVRAM_READ | nvaddr ));
    for( mask = ( 1 << 15 ); mask != 0; mask >>= 1 ) {
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | ISP$M_NVRAM_CLOCK );
	pkq_nvram_delay( 2000 );
	data |= ( READ_ISP( adp, handle, isp$w_bus_nvram ) & ISP$M_NVRAM_DATA_IN ) ? mask : 0;
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT );
	pkq_nvram_delay( 2000 );
    }
    pkq_nvram_off( adp, handle );
    return( data );
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_nvram_command( ADP *adp, uint64 *handle, uint16 cmd )
{
    DECLARE_ISP();
    uint16		data_out_val;
    uint16		mask;
    for( mask = ( 1 << 8 ); mask != 0; mask >>= 1 ) {
	data_out_val = ( cmd & mask ) ? ISP$M_NVRAM_DATA_OUT : 0;
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | data_out_val );
	pkq_nvram_delay( 2000 );
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | data_out_val | ISP$M_NVRAM_CLOCK );
	pkq_nvram_delay( 2000 );
	WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | data_out_val );
	pkq_nvram_delay( 2000 );
    }
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_nvram_on( ADP *adp, uint64 *handle )
{
    DECLARE_ISP();
    WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT );
    pkq_nvram_delay( 2000 );
    WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_SELECT | ISP$M_NVRAM_CLOCK );
    pkq_nvram_delay( 2000 );
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_nvram_off( ADP *adp, uint64 *handle )
{
    DECLARE_ISP();
    /* Writing zero deselects the NVRAM chip.
     */
    WRITE_ISP( adp, handle, isp$w_bus_nvram, 0 );
    pkq_nvram_delay( 2000 );
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_nvram_toggle( ADP *adp, uint64 *handle  )
{
    DECLARE_ISP();
    pkq_nvram_off( adp, handle );
    WRITE_ISP( adp, handle, isp$w_bus_nvram, ISP$M_NVRAM_CLOCK );
    pkq_nvram_delay( 2000 );
    pkq_nvram_on( adp, handle );
}

/*
 *
 *   Name:
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_nvram_delay( int32 delta )
{
    /* Delay 'delta' nanoseconds:
     * convert the 32-bit delta value to 64 bits
     * and pass it by reference to exe$delay()
     */
    int64	big_delta = delta;
    exe$delay( &big_delta );
}

/*
 *
 *   Name:	pkq_print_firmware_version
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_print_firmware_ver( PKQ_SPDT *spdt, int severity )
{
    char	firmware_message[ 256 ];		/* Firmware version				*/

    strcpy( firmware_message, "loading firmware version " );
    pkq_convert_ver( spdt, firmware_message);
    strcat( firmware_message, " from " );
    if( spdt->spdt$l_pkq_flags & SPDT$M_PKQ_CONSOLE_FW ) {
	strcat( firmware_message, "console" );
    } else {
	strcat( firmware_message, "driver" );
    }
    pkq_print_message( spdt, firmware_message, severity );
    return;
}

/*
 *
 *   Name:	pkq_print_convert_ver
 *
 *   Abstract:
 *
 *   Inputs:
 *
 *   Outputs:
 *
 *   Return
 *   Values:
 *
 */
void pkq_convert_ver( PKQ_SPDT *spdt, char *message )
{
    char	version[ 8 ] = { 0 };		/* Firmware version number			*/
    int		units, tenths, hundredths;	/* Digits of version				*/

    hundredths = spdt->spdt$l_firmware_version % 10;
    tenths = ( spdt->spdt$l_firmware_version / 10 ) % 10;
    units = spdt->spdt$l_firmware_version / 100;
    version[ 0 ] = units + '0';
    version[ 1 ] = '.';
    version[ 2 ] = tenths + '0';
    version[ 3 ] = hundredths + '0';
    strcat( message, version );
    return;
}

/*
 *
 *   Name:	pkq_print_message
 *
 *   Abstract:	Print message text provided by the caller.
 *
 *   Inputs:	spdt			SPDT address
 *		message			pointer to message text
 *		severity		severity code
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_print_message( PKQ_SPDT *spdt, char *message, int severity )
{
    UCB		*ucb;				/* Unit Control Block pointer			*/
    DDB		*ddb;				/* Device Data Block pointer			*/

    ucb = spdt->basespdt.spdt$l_port_ucb;
    ddb = ucb->ucb$l_ddb;

    /* Ignore informational messages unless the SYSGEN UserD1
     * parameter is set.
     */
    if((( sgn$gl_userd1 & 1 ) == 0 ) &&
       ( severity == STS$K_INFO )) {
	return;
    }
    
    /* Print out a message prefix using the severity code
     * and device name from the DDB.
     */
    exe_std$outzstring( "\a\r\n%PKQDRIVER-" );
    switch( severity ) {
    case STS$K_INFO:
    default:
	exe_std$outzstring( "I- " );
	break;
    case STS$K_WARNING:
	exe_std$outzstring( "W- " );
	break;
    case STS$K_ERROR:
	exe_std$outzstring( "E- " );
	break;
    case STS$K_SEVERE:
	exe_std$outzstring( "F- " );
	break;
    }
    exe_std$outcstring( ddb->ddb$t_name );
    exe_std$outzstring( "0, " );

    /* Print the user-provided portion of the message followed
     * by CR-LF.
     */
    exe_std$outzstring( message );
    exe_std$outcrlf();
    return;
}

/*
 *
 *   Name:	pkq_init_rspid_table
 *
 *   Abstract:	Initialize the driver-wide RSPID table and
 *		chain its entries together into an free list.
 *
 *   Inputs:	None
 *
 *   Implicit
 *   Inputs:	rspid_listhead		free list head
 *		rspid_table		the RSPID table itself
 *
 *   Outputs:	p_rspid_table		pointer to the rspid table.
 *
 *   Implicit
 *   Outputs:	rspid_listhead		updated free list head
 *		rspid_table		initialized RSPID table
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_init_rspid_table( PKQ_SPDT *spdt, RTE *p_rspid_table ) {
    RTE		*rtp;
    int		i;


    rtp = spdt->spdt$ps_rspid_table;
    for( i = PKQ_NUM_RSPID - 1; i >= 0; --i ) {
	rtp[ i ].flink = spdt->spdt$ps_rspid_listhead;
	spdt->spdt$ps_rspid_listhead = &rtp[ i ] ;
	rtp[ i ].rspid.index = i;
	rtp[ i ].rspid.seq_no = 1;
	rtp[ i ].p1 = NULL;
	rtp[ i ].spare = NULL;
    }
}

/*
 *
 *   Name:	pkq_alloc_rspid
 *
 *   Abstract:	Allocate a RSPID and correlate it with an SCDRP
 *
 *   Inputs:	scdrp			SCSI class driver request packet
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	rtp->rspid		Response ID
 *
 */
RSPID pkq_alloc_rspid( PKQ_SPDT *spdt, SCDRP *scdrp ) {
    RTE		*rtp;
    RSPID	null_rspid = { 0,0 };

    /* If no RSPID is available, fork and wait until one frees up.
     * If the fork and wait operation fails, return a null RSPID
     * to the caller.
     */
    while(( rtp = spdt->spdt$ps_rspid_listhead ) == NULL ) {
	if( ERROR( exe$kp_fork_wait( scdrp->scdrp$ps_kpb, ( FKB * )scdrp )))
	    return( null_rspid );
	spdt->spdt$is_pkq_rspid_alloc_failures++;
    }
    
    /* Allocation succeeded.  Update the RSPID free list, store the
     * SCDRP address in the RSPID table, and return the RSPID to the
     * caller.
     */
    spdt->spdt$ps_rspid_listhead = rtp->flink;
    rtp->flink = NULL;
    rtp->p1 = ( void * )scdrp;
    return( rtp->rspid );
}

/*
 *
 *   Name:	pkq_dealloc_rspid
 *
 *   Abstract:	Deallocate a RSPID and update its sequence number.
 *
 *   Inputs:	rspid			Response Id
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	None
 *
 */
void pkq_dealloc_rspid( PKQ_SPDT *spdt,RSPID rspid ) {
    RTE		*rtp;
    RTE		*rtb;
    /* Point at the RSPID table entry specified by the RSPID.  If
     * the index and sequence numbers match, update the sequence
     * number and return the RSPID table entry to the free list.
     */
    rtb = spdt->spdt$ps_rspid_table;
    rtp = &rtb[ rspid.index ];
    if(( rtp->rspid.index == rspid.index ) &&
	( rtp->rspid.seq_no == rspid.seq_no )) {
	if( ++rtp->rspid.seq_no == 0 ) {
	    ++rtp->rspid.seq_no;
	}
	rtp->flink = spdt->spdt$ps_rspid_listhead;
	spdt->spdt$ps_rspid_listhead = ( RTE * )&rtp->flink;
	rtp->p1 = NULL;
    }
}

/*
 *
 *   Name:	pkq_rspid_to_scdrp
 *
 *   Abstract:	Return the SCDRP pointer corresponding to the
 *		given RSPID, if any.
 *
 *   Inputs:	rspid			Response ID
 *
 *   Outputs:	None
 *
 *   Return
 *   Values:	scdrp			Corresponding SCDRP or NULL
 *
 */
SCDRP *pkq_rspid_to_scdrp( PKQ_SPDT *spdt, RSPID rspid ) {
    RTE		*rtp;
    RTE		*rtb;
    /* Point at the RSPID table entry specified by the RSPID.  If
     * the index and sequence numbers match, return the SCDRP address
     * in the RTE.  Otherwise, return a NULL pointer.
     */
    rtb = spdt->spdt$ps_rspid_table;
    rtp = &rtb[ rspid.index ];
    if(( rtp->rspid.index == rspid.index ) &&
	( rtp->rspid.seq_no == rspid.seq_no )) {
	return(( SCDRP * )rtp->p1 );
    } else {
	return(( SCDRP * )NULL );
    }
}


/*
 *
 *   Name: stall_qman
 *
 *   Abstract:
 *
 *	This routine stalls the queue manager at least until 25% of current IO
 *	completes. It is meant to be used as a flow control mechanism to 
 *	be used when the request queue is full. This routine meerly sets a 
 *	count to indicate that the queue manager is stalled and does a KP_STALL.
 *	Any interrupt that sees the non zero count will decrement the count and
 * 	wake up the queue manager when the count becomes zero.
 *
 *	Note:	There is a more efficient way to allocate resources that
 *		involves calling a common routine. This routine will stall
 *		the entire io system while waiting for resources. Unfortunately
 *		this mechanism cannot be used directly since it only allows
 *		allocation of one resource at a time. This should be modified
 *		at the next opportunity.
 *   Inputs:
 *	    spdt    Address of the SPDT
 *
 *   Outputs:
 *	    none
 *
 *   Return
 *   Values:
 *	    none
 *
 */
void stall_qman(PKQ_SPDT *spdt)
{
    KPB *kpb;
        
    spdt->spdt$l_qman_stalled = REQUEST_QUEUE_SIZE >> 2; 
    kpb = spdt->basespdt.spdt$ps_qman_kpb;
    kpb->kpb$ps_sch_stall_rtn = ioc$return;
    kpb->kpb$ps_sch_restrt_rtn = 0;
#if PKQ_DEBUG	
trace_event(spdt,'stal',(uint32)kpb ,(uint32)spdt, exe$gl_abstim );
#endif
    exe$kp_stall_general( kpb );
#if PKQ_DEBUG	
trace_event(spdt,'resm',(uint32)kpb ,(uint32)spdt, exe$gl_abstim );
#endif
 
}

/*
 *
 *   Name: 
 *	    resume_qman
 *
 *   Abstract:
 *
 *	This routine conditionally resumes the queue manager KPB if it is 
 * 	waiting. This is indicated by the qkpb_stalled in the spdt. This routine
 *	must be called holding the fork lock.
 *
 *   Inputs:
 *	    spdt    	Address of the SPDT for this port.
 *	    uncond_flag TRUE means restart qman unconditioanlly.
 *			FALSE means decrement count and restart qman if <= 0
 *   Outputs:
 *	    none
 *
 *   Implicit Outputs:
 *	    spdt->spdt$l_restart_qman is decremented and set to zero if the 
 *	  			      queue manager is restarted.
 * 	    queue manager is conditioanlly restarted.
 *
 *   Return
 *   Values:
 *	    none
 *
 */
void resume_qman(PKQ_SPDT *spdt, uint32 uncond_flag)
{
    KPB *kpb;
    if( spdt->spdt$l_qman_stalled  )	    
    {
	if( ( (--(spdt->spdt$l_qman_stalled) <= 0) ) || uncond_flag )
	{
	    kpb = spdt->basespdt.spdt$ps_qman_kpb;		
#if PKQ_DEBUG	
	    trace_event(spdt,'rest',' kp ' ,(uint32)kpb, exe$gl_abstim );
#endif
	    spdt->spdt$l_qman_stalled = 0;
	    exe$kp_restart( kpb, SS$_NORMAL);
	}
    }
}

/*
**
**   Name: 
**	    dump_ram
**
**   Abstract:
**
** 	   This routine loads the firmware (normally left there by the console)
**	   into a buffer supplied as an input. It does this 16 bits at a time
**	   since we've had problems with the mailbox 'dump ram' command.
**
**   Inputs:
**	    spdt    	Address of the SPDT for this port.
**	    buffer	pointer to the destination buffer
**	    risc_addr	starting risc address (ie internal chip address)
**	    size	The number of 16 bit values to read
**	    
**   Outputs:
**	    buffer	Buffer is filled in with data
**
**   Implicit Outputs:
**
**   Return
**   Values:	status of last mailbox command
**
*/
int dump_ram( PKQ_SPDT *spdt, uint16 *buffer, uint32 risc_addr, uint32  size)
{
    int status ;
    int i;
    while(size)
    {
	status = pkq_mailbox_io( spdt, 2, ISP$K_MB_READ_RAM_WORD, risc_addr, 
								      0,0,0,0);
	if(SUCCESS(status))
	{
	    *buffer++ = spdt->spdt$w_mailbox_2;
	    risc_addr++;
	    size--;
	}
	else
	{
	    break;
	}
    }
    return (status);
}

/*
**
**   Name: 
**	    backoff_delay
**
**   Abstract:
**
**	This routine is used to generate a backoff delay loop. It will delay
**	up to a 'max' value that is passed is an input argument. The actual
**	delay argument is doubled until the max value is reached.
**
**   Inputs:
**	    delay	The address of an int32 # of nanoseconds to delay
**	    max		The maximum value that delay may be set to.
**	    
**	    
**   Outputs:
**
**   Implicit Outputs:
**
**   Return
**   Values: delay*2 or max whichever is smaller.
**
*/
uint32 backoff_delay( uint32 delay, uint32 maximum)
{
    pkq_nvram_delay( delay );
    return (min( (delay*2) , maximum) );
}

/*
**
**   Name: init_target_reply
**
**   Abstract:
**
**   	This routine does the unit init time initialization of the responses to
**   	target mode request sense and inquiry messages and the data structures
**	used to handle the requests.
**
**   Inputs:
**	spdt	address of the spdt for this device;
**	    
**   Outputs:
**	None
**
**   Implicit Outputs:
**	All of the fields in the two structures are inited.
**	The ATIO buffers are initialized.
**
**   Return
**	SS$_NORMAL
**
*/
void init_target_reply(PKQ_SPDT *spdt)
{
    int i;
    char *scan;
    INQ_RESP *inqr;
    REQ_SEN_RESP *reqsenser;

    inqr = &(spdt->spdt$s_inquiry_response);
    reqsenser = &(spdt->spdt$s_request_sense_response);


    inqr->scsi$inq$v_device_type = SCSI$C_CPU;
    inqr->scsi$inq$v_ansi_version = SCSI$C_ANSI_SCSI_2;
    inqr->scsi$inq$v_resp_data_format= SCSI$C_SCSI_2;
    inqr->scsi$inq$b_add_length= sizeof(INQ_RESP)-5;
    strcpy (inqr->scsi$inq$b_vendor_id, "DEC     ");


    scan = inqr->scsi$inq$b_product_id;
    strcpy (scan, "OpenVMS ");

    scan = scan+strlen(scan);
    *scan++ = (char)(spdt->basespdt.spdt$l_scsi_port_id+'A');
    *scan++ = (char)clu$gl_allocls;
    *scan++ = (char)clu$gl_tape_allocls;
    strncpy (scan, "     ", strlen("     "));

    strncpy (inqr->scsi$inq$b_product_revision, &sys$gq_version, 4);

/*
 *	If ioc$gl_nameing is set then we may need to overwrite the above data
 *	to support new device naming. If the ddb ddb$v_pac is set then the
 *	ddb contains a port allocation class. Use that instead of the port id,
 *	allocation class etc.
 */

    if( ioc$gl_naming & 1 )
    {
	UCB  *ucb;
        DDB  *ddb;
	uint32 *buff;

	ucb = spdt->basespdt.spdt$l_port_ucb;
	ddb = ucb->ucb$l_ddb;
	if( ddb->ddb$v_pac )
	{
	    buff = (uint32 *)( inqr->scsi$inq$b_product_id +8 );
	    *buff++ = scs$gb_systemid;
	    *buff   = ddb->ddb$l_allocls;
	}
    }


    reqsenser->scsi$sns$v_error_code = SCSI$SC1$C_CURRENT;
    reqsenser->scsi$sns$v_sense_key = SCSI$C_ILLEGAL_REQUEST;
    reqsenser->scsi$sns$b_add_sense_len = sizeof(REQ_SEN_RESP)-8; /* 8 is length of standard part om message */

    reqsenser->scsi$sns$b_add_sense_code = 0x25;	/* Lun not supported */

    for( i = 0; i<4; i++)
    {
	clear_entry( &spdt->spdt$s_atio_entries[i] );

    }    

}

/*
**
**   Name: Alloc_atio_buffer
**
**   Abstract:
**
**	This routine allocates one of 4 buffers available for accept target
**	IO processing. The ATIO is kept in the buffer until the IO is completed.
**
**   Inputs:
**	spdt	address of the SPDT
**	entry	isp_entry to copy into the buffer.
**	    
**   Outputs:
**	buffer	virtual address of the copied atio call
**
**   Implicit Outputs:
**	atio in *entry is copied into allocated block.
**
**   Return
**	SS$_NORMAL if the buffer is allocated and copied.
**	0	  otherwise.
*/
int alloc_atio_buffer(PKQ_SPDT *spdt, ISP_ENTRY *entry, ISP_ENTRY **buffer)
{
    int status = 0;
    int i;
    ISP_ENTRY *ent;
/*
**  First make sure that there isn't one already there.
*/
    for(i = 0; i<MAX_ATIO_ENTRIES ; i++)
    {
	ent = &spdt->spdt$s_atio_entries[i];
	if ( ( ent->isp$b_type == ISP$K_TY_ATIO ) &&
	     ( ent->isp$b_initiator_id == entry->isp$b_initiator_id) &&
	     ( ent->isp$b_tag_value == entry->isp$b_tag_value)
	   ) return status;
	
    }
/*
**	Now allocate one of the blocks. 
*/
    for(i = 0; i<MAX_ATIO_ENTRIES ; i++)
    {
	if ( spdt->spdt$s_atio_entries[i].isp$b_type == 0 )  
	{
	    *buffer = &(spdt->spdt$s_atio_entries[i]);
	    status = SS$_NORMAL;
	    spdt->spdt$s_atio_entries[i] = *entry;	    
	    break;
	}
    }
    return (status);
}



/*
**
**   Name: dealloc_atio_buffer
**
**   Abstract:
**  
**	This routine deallocates an atio buffer by clearing its type 
**
**   Inputs:
**	entry	address of the atio buffer
**   Outputs:
**	entry->isp$b_type is cleared
**   Implicit Outputs:
**	none
**   Return
**	SS$_NORMAL
*/
void dealloc_atio_buffer( ISP_ENTRY *entry)
{
    entry->isp$b_type = 0;
}

/*
**
**   Name: build_ctio
**
**   Abstract:
**  
**	This routine builds a ctio entry dirrectly into the request queue from
**	the information in an atio entry.
**
**   Inputs:
**	spdt	    pointer to the spdt
**	isp_entry   address of the atio entry
**	ctio_entry  address of buffer into which the ctio is to be built.
**	
**   Outputs:
**
**	none
**
**   Implicit Outputs:
**
**	contents of buffer *ctio are modified
**
**   Return
**	none
*/

void build_ctio(PKQ_SPDT *spdt,  ISP_ENTRY *isp_entry, ISP_ENTRY *ctio_entry)
{
    int xfer_len;			/* length of transfer */
    uint32 buffer_pa;			/* physical address or reply */
    uint32 bytes_in_page;		/* bytes of reply in first physical page*/

    /*
    ** first make a copy of the atio. That helps since some fields are the same.
    */
    *ctio_entry = *isp_entry;	    

			
    /*
    ** fill in the header section.
    */

    ctio_entry->isp$l_handle = 0;
    ctio_entry->isp$b_cdb_len = 0;
    
    ctio_entry->isp$b_type = ISP$K_TY_CTIO;
    ctio_entry->isp$b_count = 1;
    ctio_entry->isp$v_data_direction = ISP$K_DD_READ;
    ctio_entry->isp$b_ctio_status = 0;
    ctio_entry->isp$w_ctio_timeout = 5;
    ctio_entry->isp$l_residual_xfer_len = 0;
    ctio_entry->isp$v_send_scsi_status = 1;
    ctio_entry->isp$b_scsi_status = 0;
    if( isp_entry->isp$b_cdb[0] == SCSI$K_REQUEST_SENSE ) 
    {
	if( isp_entry->isp$b_cdb[ 4 ] < sizeof (REQ_SEN_RESP) )
	{
	    ctio_entry->isp$w_ctio_data_seg_count = 0;
	    ctio_entry->isp$l_flags = 0;
	    ctio_entry->isp$v_send_scsi_status = FALSE;
	}
	else
	{

	    /* send request sense reply */
	    ctio_entry->isp$l_xfer_length = sizeof (REQ_SEN_RESP);

/*
**	    This assumes that spdt$q_direct_dma_base has all 0's in the ms longword
**	    (this is enforced with a bugcheck in the unit init code)
*/

	    buffer_pa = ioc$sva_to_pa( &spdt->spdt$s_request_sense_response ,0,0,0)+
				    (uint32)spdt->spdt$q_direct_dma_base;

	    bytes_in_page = (buffer_pa&mmg$gl_bwp_mask)+mmg$gl_page_size-buffer_pa;
/*
**	Assume the reply is all in one page for now.
*/
	    ctio_entry->isp$w_ctio_data_seg_count = 1;
	    ctio_entry->isp$r_ctio_data_seg[0].isp$ps_base_address = (void *)buffer_pa;

	    if (sizeof (REQ_SEN_RESP) <= bytes_in_page)	/* does reply cross a */
							/* page boundry ?     */
	    {
	        ctio_entry->isp$r_ctio_data_seg[0].isp$l_byte_count = sizeof (REQ_SEN_RESP);
	    }
	    else
	    {
		ctio_entry->isp$r_ctio_data_seg[0].isp$l_byte_count = bytes_in_page;
		ctio_entry->isp$r_ctio_data_seg[1].isp$l_byte_count  = 
					sizeof (REQ_SEN_RESP)-bytes_in_page;
		ctio_entry->isp$r_ctio_data_seg[1].isp$ps_base_address = 
		    (void*)(ioc$sva_to_pa( ( char *)&spdt->spdt$s_request_sense_response
						    + mmg$gl_page_size, 0, 0, 0)
			   + (uint32)spdt->spdt$q_direct_dma_base);

		ctio_entry->isp$w_ctio_data_seg_count = 2;
	    
	    }   
	
	}
    }
    else if ( isp_entry->isp$b_cdb[0] == SCSI$K_INQUIRY ) 
    {
	if( isp_entry->isp$b_cdb[ 4 ] < sizeof (INQ_RESP) )
	{
	    if(isp_entry->isp$b_cdb[ 4 ] >0 )
	    {
		ctio_entry->isp$l_xfer_length = 1;
		spdt->spdt$b_short_inquiry_response = 0x7f;  /* send lun not supported */
		buffer_pa = 
		    ioc$sva_to_pa( &spdt->spdt$b_short_inquiry_response , 0, 0, 0) + 
		    (uint32)spdt->spdt$q_direct_dma_base;
		ctio_entry->isp$w_ctio_data_seg_count = 1;
		ctio_entry->isp$r_ctio_data_seg[0].isp$ps_base_address = 
								(void *)buffer_pa;
		ctio_entry->isp$r_ctio_data_seg[0].isp$l_byte_count  = 1;
	    }
	    else
	    {
		ctio_entry->isp$w_ctio_data_seg_count = 0;
		ctio_entry->isp$l_flags = 0;
		ctio_entry->isp$v_send_scsi_status = FALSE;
	    }
	}
	else
	{
	    /* send inquiry reply */
	    ctio_entry->isp$l_xfer_length = sizeof (INQ_RESP);
	    buffer_pa = 
		ioc$sva_to_pa( &spdt->spdt$s_inquiry_response , 0, 0, 0) + 
		(uint32)spdt->spdt$q_direct_dma_base;
	    bytes_in_page = (buffer_pa&mmg$gl_bwp_mask)+mmg$gl_page_size-buffer_pa;

	    ctio_entry->isp$w_ctio_data_seg_count = 1;
	    ctio_entry->isp$r_ctio_data_seg[0].isp$ps_base_address = (void *)buffer_pa;
	    if (sizeof ( INQ_RESP) <= bytes_in_page)
	    {
		ctio_entry->isp$r_ctio_data_seg[0].isp$l_byte_count = 
							  sizeof (INQ_RESP);
	    }
	    else
	    {
		ctio_entry->isp$r_ctio_data_seg[0].isp$l_byte_count  = bytes_in_page;
		ctio_entry->isp$r_ctio_data_seg[1].isp$l_byte_count  = 
					    sizeof (INQ_RESP)-bytes_in_page;
		ctio_entry->isp$r_ctio_data_seg[1].isp$ps_base_address = 
		    (void *)(ioc$sva_to_pa( ( char *)&spdt->spdt$s_inquiry_response+
					    mmg$gl_page_size, 0, 0, 0) +
			(uint32)spdt->spdt$q_direct_dma_base);

		ctio_entry->isp$w_ctio_data_seg_count = 2;
	    }
	}
    }
    else
    {
	/* here return check condition */
	ctio_entry->isp$w_ctio_data_seg_count = 0;
	ctio_entry->isp$b_scsi_status = SCSI$C_CHECK_CONDITION;
	ctio_entry->isp$l_flags = 0;
	ctio_entry->isp$v_send_scsi_status = TRUE;
	
    }
}


/*
**  find_atio	search for an atio in the spdt atio buffers given a ctio entry.
**
**  Abstract:
**
**	This routine takes as input a ctio entry received in the response queue
**	and finds the atio that is associated with that ctio ( or 0 if not
**	found ). The initiator id and tag must match in the two messages.
**
**  input:
**		spdt	address of spdt
**		ctio_entry	address of the ctio entry for the requested atio
**  Outputs:
**		none
**
**  return:
**		ATIO_ENTRY address (or 0 if not found)
**
**
**
*/

ISP_ENTRY *find_atio(PKQ_SPDT *spdt, ISP_ENTRY *ctio_entry)
{
    int i;
    ISP_ENTRY *atio_entry = 0, *entry;
    for (i = 0;i < MAX_ATIO_ENTRIES;i++)
    {
	entry = &spdt->spdt$s_atio_entries[i];
	if ( ( entry->isp$b_type == ISP$K_TY_ATIO ) &&
	     ( entry->isp$b_initiator_id == ctio_entry->isp$b_initiator_id ) &&
	     ( entry->isp$b_tag_value == ctio_entry->isp$b_tag_value)
	   )
	{    
	    atio_entry=entry;
	    break;
	}
    }
    return atio_entry;
}

/*
**
**   Name: set_reqq_entry
**
**   Abstract:
**	This routine updates the request queue after an entry has been placed
**	into it. it updates spdt$l_requestq_in and writes the new value to the
**	device register to notify the isp_fw that there is a new entry in the queue.
**	The mailbox (hardware) register is updated with the new value in effect 
**	sending the entry to the chip firmware.
**
**   Inputs:
**	spdt		address of the spdt
**	requestq_in	current value of spdt$l_requestq_in
**
**   Outputs:
**	none
**
**   Implicit Outputs:
**	spdt$l_requestq_in is incremented and, if necessary wraps.
**	 that value is also written to the mailbox 4 register.
**
**   Return
**	none
**
*/
void set_reqq_entry(PKQ_SPDT *spdt, int requestq_in)		    
{
	DECLARE_ISP();
	requestq_in++;
	if( requestq_in >= REQUEST_QUEUE_SIZE )
	    requestq_in = 0;
	spdt->spdt$l_requestq_in = requestq_in;
	WRITE_ISP( spdt->basespdt.spdt$l_adp, &spdt->spdt$q_iohandle_reg,
		  isp$w_mailbox_4, requestq_in );
}

/*
**
**   Name: ack_notify
**
**   Abstract:
**  
**	This routine sends a notify_ack message to the isp_fw in response to
**	an immediate notify.
**
**   Inputs:
**	spdt	address of SPDT
**	isp_entry   address of the isp_entry for the immediate notify message
**
**   Outputs:
**	none
**
**   Implicit Outputs:
**	A Notify acknowledge message is put into the request queue.
**
**   Return
**	none
**
*/
void ack_notify(PKQ_SPDT *spdt, ISP_ENTRY *isp_entry)
{
    int requestq_in;
    ISP_ENTRY *ack_entry;
/*
**  get a request queue entry.
*/    
    requestq_in = spdt->spdt$l_requestq_in;
    ack_entry = spdt->spdt$ps_requestq_va+requestq_in;
/*
**  copy the immediate notify message and turn it into an acknowledge
*/
    *ack_entry = *isp_entry;
    ack_entry->isp$b_type = ISP$K_TY_NOTIFY_ACK;
    ack_entry->isp$b_event = 0;
#if PKQ_DEBUG	
trace_entry(spdt,ack_entry);
#endif

    set_reqq_entry(spdt, requestq_in);		    
    
}

/*
**  reset_ack_notify	send an Notify Acknowledge message to clear an interrupt
**
**  Abstract:
**
**	This routine sends an ack notify message to tell the pkqdriver target
**	mode firmware that a reset has been cleared. This performs the same 
**	funtion as the marker entry for initiator mode.
**
**  input:
**	spdt	address of the spdt
**
**  Outputs:
**		none
**
**  return:
**		none
**
**  Implicit Outputs:
**	A notify acknowlege message is put in the requestq woth the
**	'reset cleared' bit set.
**
*/

void reset_ack_notify( PKQ_SPDT *spdt)
{
    ISP_ENTRY *ack;
    int	requestq_in;
    
    requestq_in = spdt->spdt$l_requestq_in;
    ack = spdt->spdt$ps_requestq_va + requestq_in;
    clear_entry(ack);
    ack->isp$b_type = ISP$K_TY_NOTIFY_ACK ;
    ack->isp$b_count  = 1 ; 
    ack->isp$b_target_id  = (char )spdt->basespdt.spdt$is_scsi_id_num ;
    ack->isp$v_reset_cleared  = 1 ;
    set_reqq_entry(spdt, requestq_in);		    

#if PKQ_DEBUG	
trace_entry(spdt,ack);
#endif
}

/*
**  clear_entry	clear an isp_entry structure.
**
**  input:
**	entry	address of isp entry to clear
**
**  Outputs:
**		none
**
**  return:
**		none
**  Implicit Outputs:
**  	The buffer at entry address is cleared (8 quadwords)
**
** Note: there should be an assume in here that the length of an entry is 64.
**
*/
void clear_entry(ISP_ENTRY *entry)
{
    uint64 *scan;

#if PKQ_DEBUG
    if( sizeof(ISP_ENTRY) != 64) pkq_bugcheck();
#endif
    scan = (uint64 *)entry;
    *scan = 0;
    *(scan+1) = 0;
    *(scan+2) = 0;
    *(scan+3) = 0;
    *(scan+4) = 0;
    *(scan+5) = 0;
    *(scan+6) = 0;
    *(scan+7) = 0;
}

/*
**
**   Name: return_atio
**
**   Abstract:
**	This routine returns and deallocates an atio that is being held in the
**	spdt while a target message is happening.
**
**   Inputs:
**	spdt		address of the spdt
**	isp_entry	address of the atio being held 
**
**   Outputs:
**	None:
**
**   Implicit Outputs:
**	isp_entry is copied into the request queue
**	spdt$l_requestq_in is updated
**   Return
**	None
**
**
*/
void return_atio(PKQ_SPDT *spdt,ISP_ENTRY *isp_entry)
{
    int requestq_in;
    ISP_ENTRY *atio_entry, *requestq_entry;

    requestq_in = spdt->spdt$l_requestq_in;
    requestq_entry = spdt->spdt$ps_requestq_va + requestq_in;
    atio_entry = find_atio(spdt, isp_entry);
    if(atio_entry != NULL)
    {
	*requestq_entry = *atio_entry;
	dealloc_atio_buffer( atio_entry );
	set_reqq_entry(spdt, requestq_in);		    
    }
#if PKQ_DEBUG	
trace_entry(spdt,requestq_entry);
#endif

        
}

/*
**
**   Name: send_enable_lun
**
**   Abstract:
**	This routine puts an enable lun command in the request queue to enable
**	target mode.
**
**   Inputs:
**	spdt	address of the spdt
**
**   Outputs:
**	none
**
**   Implicit Outputs:
**	request queue is updated to send the enable lun command.
**
**
**   Return
**	none
**
*/
void send_enable_lun(PKQ_SPDT *spdt)
{
    ISP_ENTRY *lun_entry;
    int i;
    lun_entry = spdt->spdt$ps_requestq_va + spdt->spdt$l_requestq_in;
    clear_entry(lun_entry);
    lun_entry->isp$b_type = ISP$K_TY_ENABLE_LUN;
    lun_entry->isp$b_count = 1;
    lun_entry->isp$b_lun = 0;
    lun_entry->isp$l_flags =	0;	/* clear ISP$M_TAGGED_QUEUE_ENABLE */
    lun_entry->isp$b_command_count = MAX_ATIO_ENTRIES;
    lun_entry->isp$b_notify_count = MAX_NOTIFY_ENTRIES;
    lun_entry->isp$b_group_6_count = 6;
    lun_entry->isp$b_group_7_count = 6;
    lun_entry->isp$w_timeout = 6;

#if PKQ_DEBUG	
trace_entry(spdt,lun_entry);
#endif
    set_reqq_entry(spdt, spdt->spdt$l_requestq_in );		    
    spdt->spdt$l_pkq_flags &= ~SPDT$M_PKQ_SEND_ENABLE_LUN;
    wait_enable_lun(spdt);
}

void wait_enable_lun( PKQ_SPDT *spdt)
{
    int64 delay = 10000;
    int count = 10000;
    while(((spdt->spdt$l_pkq_flags & SPDT$M_PKQ_GOT_ENABLE_LUN)==0) && 
								   (--count)>0 )
    {
        exe$kp_tqe_wait( spdt->basespdt.spdt$ps_qman_kpb,
                        &delay, spdt->basespdt.spdt$is_flck  );
    }
}



/*
**
**   Name: pause_risc
**
**   Abstract:
**	This routine pauses risc processor and wait for the indication that it
**	is paused.
**
**   Inputs:
**	spdt	address of the spdt
**
**   Outputs:
**	none
**
**   Implicit Outputs:
**	The risc firmware will not be running upon return from this routine
**
**
**   Return
**	none
**
*/
uint32 pause_risc( PKQ_SPDT *spdt)
{
	DECLARE_ISP();
	ADP		*adp = spdt->basespdt.spdt$l_adp;
	uint32 interval;
	uint64		*handle = &spdt->spdt$q_iohandle_reg;
    
	/* Write a pause request into the HCCR and then wait
	 * a while for it to happen.  If we manage to exhaust
	 * the interval count fall through anyway to take whatever
	 * register values we can get.
	 */
	WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_PAUSE );
	
	interval = 100000;
	while( --interval &&
	      !( READ_ISP( adp, handle, isp$w_hccr )
		& ISP$M_HCCR_PAUSE_MODE  )) {}
	return(interval != 0);
}

/*
**
**   Name: release_risc
**
**   Abstract:
**	This routine releases risc processor that presumably is paused.
**
**   Inputs:
**	spdt	address of the spdt
**
**   Outputs:
**	none
**
**   Implicit Outputs:
**	The risc firmware will be running upon return from this routine
**
**
**   Return
**	none
**
*/
void release_risc( PKQ_SPDT *spdt)
{
	DECLARE_ISP();
	ADP		*adp = spdt->basespdt.spdt$l_adp;
	uint64		*handle = &spdt->spdt$q_iohandle_reg;
    
/*
**	Reenable the risc processor
*/

	WRITE_ISP( adp, handle, isp$w_hccr, ISP$K_HCCR_RELEASE );
}

/*
**
**   Name: target_mode
**
**   Abstract:
**	This routine returnes a true value iff the loaded firmware supports 
**	target mode. Currently this routine only compares the
**	spdt$l_firmware_version field to 300. If a more complex algoritm is
**	required ( after 4.x or whatever ) or if the format of
**      spdt$l_firmware_version changes this routine will need to change.
**
**	qLogic may decide to use odd major revision numbers  for target mode and
**	even major revisions for non-target mode code. In that case this routine
**	will look something like:
**
**	return ( (spdt->spdt$l_firmware_version/100) & 1 );
**
**   Inputs:
**	spdt	address of the spdt
**
**   Outputs:
**	none
**
**   Implicit Outputs:
**	The risc firmware will be running upon return from this routine
**
**
**   Return
**	FALSE = firmware doesn't support target mode.
**	TRUE = firmware does support target mode.
**
*/

unsigned int target_mode( PKQ_SPDT *spdt )
{
    return ( spdt->spdt$l_firmware_version >= 300 );
}


/*
**
**	Debug only code starts here.
**
*/
#if PKQ_DEBUG
/*
** Allocate a trace buffer
*/
int pkq_alloc_trace( PKQ_SPDT *spdt )
{
    int status;
    int32 retlen;	
    char *buffer;

    status = exe_std$alononpaged( TRACE_SIZE  , &retlen,  ( void ** )&buffer );
    spdt->spdt$ps_next_log = 
	spdt->spdt$ps_log_buffer = 
	       (struct pkq_trace_entry *)(buffer+sizeof(struct pkq_trace_entry));
    spdt->spdt$l_log_size = TRACE_ENTRIES-1;
    
    return status;
}
/*
** trace an event.
*/
void pkq_trace_event(PKQ_SPDT *spdt, uint32 type, uint32 p1, uint32 p2, uint32 p3)
{
    struct pkq_trace_entry *next;
    int saved_ipl;
    device_lock( spdt->basespdt.spdt$l_dlck, RAISE_IPL, &saved_ipl );

    next = spdt->spdt$ps_next_log;
    next->trace_type = type;
    next->trace_p1 = p1;
    next->trace_p2 = p2;
    next->trace_p3 = p3;
    next = next+1;
    if ( ( next - spdt->spdt$ps_log_buffer ) >= (spdt->spdt$l_log_size) ) 
    {
	next = spdt->spdt$ps_log_buffer;
    }
    spdt->spdt$ps_next_log = next;
    next->trace_type = '****';
    next->trace_p1 = '****';
    device_unlock( spdt->basespdt.spdt$l_dlck, saved_ipl, SMP_RESTORE );
}
/*
**  print64	Print a 64 bit value with a description.
**
**  input:
**		str	pointer to an asciz descriptor string
**		value	a 64bit value
**  Outputs:
**		none
**
**  return:
**		none
**
**
**  This routine writes out a string to the console of the form:
**
**	descriptive string = 01234567 89ABCDEF(cr)  
**
**	The call to output that line would be...
** 	
**	print64("descriptive string", 0x123456789abcdef);
**
**	This routine is meant for debugging purposes only!
**
*/
void print64( char *str, int64 value)
{
    char ostr[32],*scan;
    int i;
    scan = &ostr[16+2];
    *scan-- = 0;
    exe_std$outzstring( str );
    exe_std$outzstring( " =" );
    for ( i = 0;i<16;i++)
    {
	*scan = value&0xf;
	if ((*scan) >9)
	{
	    *scan += 'A'-10;
	}
	else
	{
	    *scan += '0';
	}
	scan--;
	if ( ( i == 7) || (i == 15))
	{
	    *scan-- = ' ';
	}
	value = value>>4;
    }
    exe_std$outzstring( ostr );
    exe_std$outcrlf();

}

/*
** trace_entry  add a request-response queue entry to the trace buffer.
**
**  input:
**		spdt	pointer to spdt 
**		entry	pointer to isp entry in request queue or response queue
**  Outputs:
**		none
**
**  Return:
**		none
**
**  Implicit outputs:
**
**	Trace buffer is updated including spdt$ps_next_log
**	
**  Description:
**
**	This routine adds 5 lines (0x50 bytes) to the trace buffer. 
**	The first line contains either 'sent' or 'got' for entries in the
**	request and response queue respectivly, the in and out pointers for the
**	queues the entry address and exe$gl_abstime. The next 4 lines are a copy
**	of the actual entry from the buffer.
**	
**	Note that trace_event synchronizes access to the trace buffer at dipl so
**	it is possible to be interrupted between lines of trace. 
**	
**	
*/
void pkq_trace_entry(PKQ_SPDT *spdt,ISP_ENTRY *entry)
{
    int i;
    uint32 *buff;

    buff = (uint32 *)entry;
    if(entry >= spdt->spdt$ps_responseq_va)
    {
	trace_event(spdt, 'got ',   ((char)spdt->spdt$l_responseq_out<<24) +
				    ((char)spdt->spdt$l_responseq_in<<16)  +
				    ((char)spdt->spdt$l_requestq_in<<8)    +
				    ((char)spdt->spdt$l_requestq_out),
				    (uint32)entry,
				    exe$gl_abstim);
    }
    else
    {
	trace_event(spdt, 'sent',   ((char)spdt->spdt$l_responseq_out<<24) +
				    ((char)spdt->spdt$l_responseq_in<<16)  +
				    ((char)spdt->spdt$l_requestq_in<<8)    +
				    ((char)spdt->spdt$l_requestq_out),
				    (uint32)entry,
				    exe$gl_abstim);
    }
    for( i = 0; i<16; i+=4)
    {
    
	trace_event(spdt,
		       *(buff+(i+0)),
		       *(buff+(i+1)),
		       *(buff+(i+2)),
		       *(buff+(i+3)));
    }
}

#define out_field( a ,b) print_field((int)nvram->a, b ,&line_count)
void pkq_dump_nvram( PKQ_SPDT *spdt, ISP_NVRAM *nvram)
{
    char buffer[80];
    int line_count = 0;
    int i;
    print_field( ( spdt->basespdt.spdt$l_scsi_port_id + 0xa) , " port_id", &line_count);
    exe_std$outcrlf();
  
    exe_std$outzstring( " NvRAM Parameters " );
    exe_std$outzstring( " Header =  " );

    strncpy (buffer, nvram->isp$b_nvr_id , 4);
    buffer[4] = 0;
    exe_std$outzstring( buffer );
    exe_std$outcrlf();
    line_count = 0;
    out_field( isp$v_nvr_fifo_threshhold,  " fifo_threshhold" );
    out_field( isp$v_nvr_adaptor_enable , " adaptor_enable ");
    out_field( isp$v_nvr_initiator_scsi_id , " initiator_scsi_id ");
    out_field( isp$b_nvr_bus_reset_delay , " bus_reset_delay ");
    out_field( isp$b_nvr_retry_count , " retry_count ");
    out_field( isp$b_nvr_retry_delay , " retry_delay ");
    out_field( isp$v_nvr_asynch_setup_time , " asynch_setup_time ");
    out_field( isp$v_nvr_req_ack_active_negation , " req_ack_active_negation ");
    out_field( isp$v_nvr_data_active_negation , " data_active_negation ");
    out_field( isp$v_nvr_data_dma_burst_enable , " data_dma_burst_enable ");
    out_field( isp$v_nvr_command_dma_burst_enable , " command_dma_burst_enable ");
    out_field( isp$b_nvr_tag_age_limit , " tag_age_limit ");
    out_field( isp$v_nvr_termination_low_enable , " termination_low_enable ");
    out_field( isp$v_nvr_termination_high_enable , " termination_high_enable ");
    out_field( isp$v_nvr_pcmc_burst_enable , " pcmc_burst_enable ");
    out_field( isp$v_nvr_60mhz_enable , " 60mhz_enable ");
    out_field( isp$w_nvr_max_queue_depth , " max_queue_depth ");
        exe_std$outcrlf();
        exe_std$outcrlf();
    line_count = 0;
    for( i = 0; i<16; i++)
    {
	print_field(i," id #",&line_count);
	exe_std$outcrlf();
	line_count = 0;
	out_field( isp$r_nvr_target[i].isp$b_nvr_capabilites , " capabilites ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_renegotiate_on_error , " renegotiate_on_error ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_stop_queue_on_check , " stop_queue_on_check ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_auto_request_sense , " auto_request_sense ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_tagged_queuing , " tagged_queuing ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_synch_data_transfers , " synch_data_transfers ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_wide_data_transfers , " wide_data_transfers ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_parity_checking , " parity_checking ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_disconnect_allowed , " disconnect_allowed ");
	out_field( isp$r_nvr_target[i].isp$b_nvr_execution_throttle , " execution_throttle ");
	out_field( isp$r_nvr_target[i].isp$b_nvr_synch_period , " synch_period ");
	out_field( isp$r_nvr_target[i].isp$v_nvr_synch_offset , " reqack_offset ");
	exe_std$outcrlf();
	exe_std$outcrlf();
	line_count = 0;

    }

}
void print_field(int value, char *str, int *count)
{
    char *equals = " = ";
    int count_needed;
    if(value > 255)
	count_needed = 4;
    else
	count_needed = 2;

    count_needed += strlen( str ) + strlen( equals )+1;
    if( (count_needed + *count) >80)
    {

	*count = count_needed;
	exe_std$outcrlf();

    }
    else
    {
	*count += count_needed;
    }
    exe_std$outzstring( str );
    exe_std$outzstring( equals );
    out_value(value);
    exe_std$outzstring( " " );
}
void out_value(int value)
{
    char buff[8],*scan;
    scan = buff;
    if(value >255 )
    {
	*scan++= hex_digit(value>>12);
	*scan++= hex_digit(value>>8);
    }    
    *scan++ = hex_digit(value>>4);
    *scan++ = hex_digit(value>>0);
    *scan++ = 0;
    exe_std$outzstring( buff );
    
}
char hex_digit(int value)
{
    value = value&0x0f;
    if (value >9)
	return ( value +'A'-10);
    else 
	return ( value +'0');
}
#endif
