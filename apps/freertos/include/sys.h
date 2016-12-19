/***************************************************************************/
/* Module Name: sys.h                                                      */
/*                                                                         */
/* Description: This file defines the condtant and declearation ot the     */
/*              system                                                     */
/*                                                                         */
/***************************************************************************/
#ifndef _SYS_H
#define _SYS_H

#define MEM_BIG_SIZE    2048L
#define MEM_BIG2_SIZE   2048L

#define MEM_LARGE_SIZE  1024L
#define MEM_MID2_SIZE   512
#define MEM_MID_SIZE    128
#define MEM_SMALL_SIZE  64

#define POOL_SIZE		4096

#define NIPQUAD(xaddr) \
        ((unsigned char *)&xaddr)[0], \
        ((unsigned char *)&xaddr)[1], \
        ((unsigned char *)&xaddr)[2], \
        ((unsigned char *)&xaddr)[3]

/*Ssytem time tick is 10ms */
#define SECOND_BASE     OS_TICKS_PER_SEC
#define HOUR_BASE       OS_TICKS_PER_SEC * 60 * 60L
#define SYS_NAME_LEN    16
#define MAXNAMELEN	48  /* max length of hostname or name for auth */
#define MAXSECRETLEN	30  /* max length of password or secret */

#define MAX_IF_NAME     15  /*interface name length */

#define SYS_PKT_STACK_SIZE  100

typedef struct net_buf      NET_BUF;
typedef struct sys_device   SYS_DEVICE;
typedef struct net_if       NET_IF;

#define MAX_NO_OF_LOG   40
#define MAX_LOG_SIZE    50

/*
 * Packet buffer struct
 * struct size == 64 <= (MEM_SMALL_SIZE)
 */

/* fixed size is qeual 64 bytes */
struct net_buf
{
    INT16U      cmd;       /* the cmd fiels is a common area, it will used for bridge module*/
    INT16U      proto;     /* proto type (input), protocol & TTL (for output) */
    INT8U       *start;    /* point to start of data area */
    INT8U       *pdata;    /* point to data start */

    NET_BUF     *next;     /* next buffer, for upper layer */
    //INT8U	refcnt;	   /* packet duplicate counter*/

    INT16U      len;       /* real data length */
    INT16U      ip_tlen;   /* ip packet's len filed, host order (assigne in IP_Input */
    INT16U      ip_hlen;   /* ip header length, (byte)*/
    INT16U      type;      /* packet type */
    SYS_DEVICE  *pdev;     /* point to received device */
    NET_IF      *pif;      /* for NAT to record the output interface, 0 is not init */
    NET_IF      *src_pif;  /* Temporary solution to specify the interface of  */
                           /* incoming packets, it is not work when multiple interface */
                           /* connect to same physical device */
    INT32U      destip ;   /* Destination IP */
    INT32U      segseq ;   /* 1. Seqno for TCP out-of-sequence process, */
                           /* 2. for UDP identification */
                           /* 3. for TCP output segment sequence*/

	/* the above size is 44 bytes */
    INT16U      fragoffset;/* fragment and offset */
    INT8U       segflags ; /* 1. CODE bits for TCP out-of-sequence process (bit 0 - 5) */
                           /* 2. CODE bits for retransmit (TCP output)*/
                           /* and Fragment control (bit 6,7)*/
    INT16U      rport;     /* for TCP/UDP receive port */
    INT32U      rip;       /* for TCP/UDP receive IP */
};

#define MEM_BIG_NO      800
#define MEM_BIG2_NO     100    // non-cache buffer
#define MEM_LARGE_NO    100
#define MEM_MID2_NO     120
#define MEM_MID_NO      100

#define MEM_SMALL1_NO    (200+MEM_BIG_NO)
#define MEM_SMALL2_NO    (600)

/*
 * temp buffer for printf
 */
struct SYS_IO_BUF
{
    INT16S  putc;
    INT16S  getc;
    INT16S  flag; // 1 full, 0 not full
    INT16S  used; // 1 used, 0 unused for critical section protection
    UCHAR   buf[POOL_SIZE];
};

#define SYS_MAX_USER 3

struct SYS_USERS
{
    INT8U num;
    CHAR  name[SYS_MAX_USER][MAXNAMELEN];
    CHAR  pass[SYS_MAX_USER][MAXSECRETLEN];
};

// move the VENDOR_MAGIC to syscfg.h
#define VENDOR_CONFIG_OFFSET    256

/*The size of SYS_VENDOR_DEFINE should not more then 240 bytes reserve 16 bytes for reset code*/
struct SYS_VENDOR_DEFINE
{
    INT32U  magic;              /*magic string for version control*/
    INT8U   mac[2][6];          /*MAC 0/1 address */
};

/*#define SEC         0                       // RTC data offsets in the buffer
#define MIN         1                       // note that RTC data is stored in
#define HOUR        2                       // Binary Coded Decimal (BCD) format
#define DATE        3
#define MONTH       4
#define DAY         5
#define YEAR        6
#define RTC_MAX_REC 7 */

/*define year 00-99, mon 0-11, day: depend on mon_day[], */
/*hour 0 - 23, min 0 -59 sec 0 - 59*/
typedef struct sys_time
{
    INT16U  tm_year;
    INT8U   tm_mon, tm_day, tm_hour, tm_min, tm_sec;
    INT8U   am_pm, h12_24, weekday;    // am == 0, pm == 1, h24==0, h12 == 1
	INT16S	timezone; /* -12 to +12 */
	INT16S	daylight; /* daylight saving time */
} SYS_TIME;

/*for display log setting*/
struct LOG_CTRL
{
    INT16S state;   /*on/off*/
    INT32S file_len;
    CHAR   filename[32];
};

#define SYS_MAX_LOGLEN  0x2000  // 8K
typedef struct _sys_cmdlog
{
	INT32U uart_pos;
	INT32U buf_pos;
	INT32U count;
	INT8S data[SYS_MAX_LOGLEN];

} SYS_CMDLOG;

typedef struct {                       /* MEMORY CONTROL BLOCK                                         */
    void   *OSMemAddr;                 /* Pointer to beginning of memory partition                     */
    void   *OSMemFreeList;             /* Pointer to list of free memory blocks                        */
    INT32U  OSMemBlkSize;              /* Size (in bytes) of each block of memory                      */
    INT32U  OSMemNBlks;                /* Total number of blocks in this partition                     */
    INT32U  OSMemNFree;                /* Number of memory blocks remaining in this partition          */
#ifdef OS_TEST
    void    *OSMemStartAddr;    /* Pointer to start of memory partition     */
    void    *OSMemEndAddr;      /* Pointer to end of memory partition     */
    INT32U  OSMemUtilize;       /* to measure the maximun utilization of memory block*/
    INT32U  OSMemError;         /* to record the times of fail of memory allocation*/
#endif
} OS_MEM;

/*
 * public external declearation
 */
extern OS_MEM  *pSYS_MemBig;
extern OS_MEM  *pSYS_MemLarge;
extern OS_MEM  *pSYS_MemMid;
extern OS_MEM  *pSYS_MemMid2;
extern OS_MEM  *pSYS_MemSmall1;
extern OS_MEM  *pSYS_MemSmall2;
extern INT8U	DBG_SysInfo;
extern INT32U	SYS_Configuration;
extern const CHAR *SYS_ConfigFile;         /* SYSTEM configuration file name */
extern struct	SYS_USERS SYS_Users;
extern INT16U   SYS_HwVersion;
extern INT16U   SYS_FreePktNum;
//extern INT16U   SYS_FreePktNum(void);
extern INT32U   SYS_TimeSec;
extern INT32U  SYS_TimeHalfSec;
extern INT32U   SYS_LastTimeSec;
extern struct	SYS_VENDOR_DEFINE *pVendorCfg;
extern INT16U   SYS_CPUId;
extern INT32U   SYS_CPUSubId;
extern INT32U   SYS_FlashUId;
extern INT16U   SYS_CPUSpeed;
extern CHAR	SYS_HostName[MAXNAMELEN+1];
extern CHAR SYS_DomainName[255+1];
extern INT8U	SYS_PrimaryDNS[4];
extern INT8U SYS_SecondaryDNS[4];
extern INT32U  SYS_KernelStart,SYS_KernelSize,SYS_WebStart,SYS_WebSize,SYS_FlhBlkStart;
extern INT16S SYS_SetTime(SYS_TIME *t);
extern INT16S SYS_GetTime(SYS_TIME *t);
extern char *SYS_GetPass( const char * prompt );
extern INT32S SYS_Install_Handler(SYS_DEVICE *dev, INT32U location, void (*handler)(SYS_DEVICE *dev));
extern VOID SYS_Register_IRQ(INT32U type, INT32U index, INT32U priority, INT32U level, INT32U trigger, INT32U handler);
extern INT32S OS_TimerInit(void);
/*
*===========================================================================
* Macro    : LINEAR_TO_REAL  and REAL_TO_LINEAR
*   This functoin translate linear address to real address or
*   from real address to linear address
*
* Input :
*   linear_addr : linear address
*   real_addr   : real address
*
*===========================================================================
*/
#define LINEAR_TO_REAL(linear_addr, real_addr)   \
        real_addr = linear_addr
#define REAL_TO_LINEAR(real_addr,linear_addr)   \
        linear_addr = real_addr

#define SYS_RESET_WDT()

/* Define functions called by other modules */
extern INT16S SYS_PktAllocate(NET_BUF **) ;   /* Allocate pbuf */
extern INT16S SYS_PktFree(NET_BUF *) ;        /* Release pbuf */
extern INT16S SYS_Pkt2Allocate(NET_BUF **) ;   /* Allocate pbuf */
extern INT16S SYS_Pkt2Free(NET_BUF *) ;        /* Release pbuf */
extern INT16S SYS_PktCopy(NET_BUF *pbuf, NET_BUF **n);
extern INT16S SYS_PktClone(NET_BUF *pbuf, NET_BUF **n);
extern INT16S SYS_FreePktChain(NET_BUF *pbuf, INT16U all);

extern INT16S   SYS_Null(VOID);
extern VOID SYS_AtoMac(UCHAR *cp, UCHAR *mac);
extern VOID SYS_AtoIP(const CHAR *cp, UCHAR *dest);
extern VOID *SYS_Malloc(INT32U size);
extern VOID *SYS_Realloc(VOID *block, INT32U size);
extern INT32U SYS_AtoH(const CHAR *cp);
extern INT16S SYS_SaveConfig(const CHAR *file,INT16S offset);
extern INT16S SYS_ReloadConfig(const CHAR *file, INT16S offset);
extern INT16S SYS_PktStackSize(VOID);
extern INT16S SYS_Delay(INT16U count);
extern INT16S SYS_FormatTime(INT32U time, SYS_TIME *t);
extern void SYS_ASCII2Hex(INT8S *string,const INT8S *HexArray,const INT16U len);
extern VOID SYS_DisplayDebugLevel(INT8U dbg);
extern VOID SYS_ChangeDebugLevel(INT8U id, INT8U *dbg);
extern INT16S LOG_Get(INT8S *log);
extern INT16S LOG_Printf(const CHAR *fmt, ...);

#ifndef strlwr
char *strlwr (char *a);
#endif

char *itoa(int value, char *string, int radix);
char *ultoa(unsigned long value, char *string, int radix);
int stricmp(const char *s1, const char *s2);
char *strupr (char *a);

#define random(num)	(rand()%(num))
int memicmp(const void *memptr1, const void *memptr2, int count); /* string.h */

//#define printf sys_printf
extern int test_printf(void);
//extern int sys_printf(const char *format, ...);

#define udelay(time)        { INT32U z; for(z=0; z<time; z++) __asm volatile ("nop"); }

#define OS_ENTER_CRITICAL()		vPortEnterCritical();
#define OS_EXIT_CRITICAL()		vPortExitCritical();

/*
 * Task stack size, priority and related configuration
 */

#define SYS_ISR_STK_SIZE 	    512

/*System Event task, handle device's event from ISR */
#define EVENT_TASK_PRIO      	6
#define EVENT_TASK_STK_SIZE  	512
#define EVENT_Q_MSG          	5

/*System task */
#define SYS_TASK_PRIO      	    1
#define SYS_TASK_STK_SIZE  	    2048

/*CAN bus task */
#define CAN_TASK_PRIO      	    2
#define CAN_IRQ_PRIO            6
#define CAN_TASK_STK_SIZE  	    2048


#define ___swab16(x) \
                    ((unsigned int)( \
                                (((unsigned int)(x) & (unsigned int)0x00ffU) << 8) | \
                                (((unsigned int)(x) & (unsigned int)0xff00U) >> 8) ))
#define ___swab32(x) \
                    ((unsigned long)( \
                                (((unsigned long)(x) & (unsigned long)0x000000ffUL) << 24) | \
                                (((unsigned long)(x) & (unsigned long)0x0000ff00UL) <<  8) | \
                                (((unsigned long)(x) & (unsigned long)0x00ff0000UL) >>  8) | \
                                (((unsigned long)(x) & (unsigned long)0xff000000UL) >> 24) ))

#define htonl(x) ___swab32(x)
#define ntohl(x) ___swab32(x)
#define htons(x) ___swab16(x)
#define ntohs(x) ___swab16(x)

#endif /* #ifndef _SYS_H */
