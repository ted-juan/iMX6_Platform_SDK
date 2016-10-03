/***************************************************************************/
/* Module Name: dev.h 	                                                   */
/*                                                                         */
/* Description: Define the data structure of device control block and      */
/*              related definition                                         */
/*                                                                         */
/***************************************************************************/
#ifndef _DEV_H
#define _DEV_H
/*
 * If you add a new Device, please add a new record for initialization
 * the physical device. The record is defined in "dev.c"
 */
enum
{
    LOOP,       /* 0 local device */
    ETH0,       /* 1 ethernet device 1*/
    ETH1,       /* 2 ethernet device 2*/
    UART0,      /* 3 uart device 1*/
    UART1,      /* 4 uart device 2*/
    MAX_DEV
} ;

/*
*  virtual device for bridge module
*/
#define BR_DEV  MAX_DEV
#define MAX_ETH_DEV ETH1
/*
 *  type of device
 */
#define DEV_LOOP    0
#define DEV_UART    1
#define DEV_ETHER   2
#define DEV_USB     3
#define DEV_VIRTUAL 4       // for slow bus, special device
#define DEV_BULK    5       // bulk device, espacial for IDE, memory card
#define DEV_OTHER   6       // other device

/*
 *  State of device
 */
#define DEV_UNINIT      0
#define DEV_DOWN        1
#define DEV_NEGOTIATE   2
#define DEV_UP          3

/*
 *  IO control of the DEVICE
 */
#define	DEV_OPEN		0x0000          /* open a device */
#define	DEV_SET	        0x0001          /* set device's configuration */
#define	DEV_GET	        0x0002          /* get device's configuration and status */
#define	DEV_CLOSE		0x0003          /* close a device */
#define	DEV_WRITE		0x0004          /* dirct write data to device */
#define	DEV_READ 		0x0005          /* direct read data from device */
#define DEV_LINK_UP     0x0006          /* device link connected */
#define DEV_LINK_DOWN   0x0007          /* device link disconnection */
#define DEV_SYS_RESET   0x0009			/* device reset event */
#define DEV_SYS_HALT    0x0010			/* System halt event */
#define DEV_IDE_DMA     0x0013          /* IDE DMA complete event */

/*
 * Defines for the device mode field.
 */
#define DV_DEBUG        0x1    /* turn on debugging                */
#define DV_BROADCAST    0x2    /* broadcast address valid          */
#define DV_NOTRAILERS   0x4    /* avoid use of trailers            */
#define DV_PROMISC      0x8    /* receive all packets              */
#define DV_ALLMULTI     0x10   /* receive all multicast packets    */
#define DV_SIMPLEX      0x20   /* can't hear own transmissions     */
#define DV_MULTICAST    0x40   /* supports multicast               */

/*
 * System only define the UART and Ethernet for system device
 * Other devices (i.e. flash, GPIO) are not defined.
 */
struct sys_device
{
	/* Device name  */
	CHAR    name[MAX_IF_NAME+1];
	/* index of the device */
    INT8U   num;
	/* device type(ex.DEV_ETHERNET or DEV_UART ...etc.) */
	INT8U   type;
	/* device state(ex.INITED, UP, DPWN) */
    INT8U   state;
	/* device's configuration structure pointer, this field will be */
	/* initialize by device initial function */
	VOID    *devcfg;
	/* device's statistic structure pointer, this field will be */
	/* allocate memory and initialize by device initial function */
	VOID    *devstat;
    /* for NET device link list, associate to NET interface */
    NET_IF  *list;
    /* for speed up processing, the filed will stored MAC address of a*/
    /* Ethernet device when it was initialized, the UART device is useless*/
    INT8U   mac[6];
    /* entry function for initializ */
    INT16S (*init)(SYS_DEVICE *dev);
	/* entry function for IO control   */
    INT16S (*ioctl)(SYS_DEVICE *dev, INT16U optname, INT8U *optval, INT16U len);
    /*entry for event handler*/
    INT16S (*event)(SYS_DEVICE *dev, INT16U event, VOID *msg);
	/* entry function for transmit/write  */
    INT16S (*output)(SYS_DEVICE *dev, NET_BUF *pbuf);
	/* entry function for receive/read */
    INT16S (*input)(SYS_DEVICE *dev, NET_BUF *pbuf);
	/* entry function to close the driver */
    INT16S (*close)(SYS_DEVICE *dev);
};

/*Uart statistics */
struct UART_STAT
{
    INT32U  tx_bcnt;    /* Transmission byte counter*/
    INT32U  rx_bcnt;    /* receive vyte counter */
    INT32U  tx_pcnt;    /* Transmission packet counter */
    INT32U  rx_pcnt;    /* receive packet counter */
    INT32U  tx_err;     /* transmit error */
    INT32U  rx_err;     /* receive error */
    INT16U  framing;    /* RX framing error */
    INT16U  parity;     /* RX parity Error */
    INT16U  overrun;    /* RX overrun error */
    INT16U  full;       /* RX buffer full */
};

/*Ethernet statistics */
struct ETH_STAT
{
    INT32U  tx_bcnt;    /* Transmission byte counter*/
    INT32U  rx_bcnt;    /* receive vyte counter */
    INT32U  tx_pcnt;    /* Transmission packet counter */
    INT32U  rx_pcnt;    /* receive packet counter */
    INT32U  tx_err;     /* transmit error */
    INT32U  rx_err;     /* receive error */
    INT16U  tx_full;    /* TX buffer full */
    INT16U  collision;  /* TX collision */
    INT16U  under_run;  /* TX under run */
    INT16U  rx_full;    /* RX buffer full */
    INT16U  phy_err;    /* RX PHY error */
    INT16U  crc_err;    /* RX crc error */
    INT16U  runt;       /* RX runt packet*/
    INT16U  long_pkt;   /* RX long packet */
    INT16U  over_len;   /* RX over buffer length */
    INT16U  dribble;    /* RX dribble packet */
};

typedef union
{
    struct ETH_STAT eth;
    struct UART_STAT uart;
} DEV_STAT;

typedef struct _DEV_CFG_
{
    INT8U num;
    INT8U type;
    union
    {
        struct
        {
            INT8U clone[6];
            INT8U mode;
        }eth;
        struct
        {
            INT8U   baud;
            INT8U   parity;
            INT8U   data;
            INT8U   stop;
        }uart;
    }c;
}DEV_CFG;

typedef struct _DEV_EVENT_
{
    INT16U event;
    INT16U arg;
    SYS_DEVICE *pdev;
}EVENT_MSG;


typedef union _CACHE32_REG
{
    struct
    {
       INT32U size:4;
       INT32U reserve:6;
       INT32U enable:1;
       INT32U base:21;
    } reg;
    INT32U value;
} CACHE32_REG;


extern INT16S SYS_DeviceInit(VOID);
extern INT16S DEV_UartInit(SYS_DEVICE *pdev);
extern INT16S DEV_UartIoctl(SYS_DEVICE *pdev, INT16U optname, INT8U *optval, INT16U id);
extern INT16S DEV_UartEvent(SYS_DEVICE *pdev, INT16U event, VOID *msg);
extern INT16S DEV_UartOutput(SYS_DEVICE *pdev,NET_BUF *pbuf);
extern INT16S DEV_EthInit(SYS_DEVICE *pdev);
extern INT16S DEV_EthIoctl(SYS_DEVICE *pdev, INT16U optname, INT8U *optval, INT16U id);
extern INT16S DEV_EthEvent(SYS_DEVICE *pdev, INT16U event, VOID *msg);
extern INT16S DEV_EthOutput(SYS_DEVICE *pdev,NET_BUF *pbuf);
extern INT16S GPIO_Init(VOID);
extern INT16S GPIO_Read(INT16U bits);
extern INT16S GPIO_Write(INT16U bits, INT32U data);
extern INT16S GPIO_Ctrl(INT16U bits, INT32U data);

extern SYS_DEVICE *SYS_GetDevByName(CHAR *name);
extern SYS_DEVICE *SYS_GetDevById(INT16U id);

#if (CACHE == 1)
    extern VOID DEV_NoneCacheRegion(INT16U region, CHAR *addr, INT32U size);
    extern VOID DEV_CacheCtrl(INT16U ctrl);
    extern INT16S SYS_NoneCacheRegionInit(VOID);
    extern CHAR *SYS_AllocNoneCacheRegion(INT32U size);
    extern VOID SYS_FreeNoneCacheRegion(INT32U size, CHAR *buf);

    #define SYS_Malloc_Consistent(size,align) (void *)((((INT32U)(SYS_AllocNoneCacheRegion(((INT32U)size)+align))-1) |(align-1) )+ 1)
#else
	/*cache not enable*/
	#define SYS_AllocNoneCacheRegion(a) 	SYS_Malloc(a)
	#define SYS_FreeNoneCacheRegion(a,b)	SYS_Free(b)
#endif

#endif
/*end define _DEV_H */
