#ifndef _TYPE_H
#define _TYPE_H

typedef unsigned char       BOOLEAN;
typedef unsigned char       INT8U;
typedef char                INT8S;
typedef unsigned short      INT16U;
typedef short               INT16S;
typedef unsigned int        INT32U;
typedef int                 INT32S;
typedef float               FP32;
typedef double              FP64;
typedef void                VOID;
typedef INT32U              OS_STK;
typedef  long long       INT64S;
typedef unsigned long long  INT64U;

#define u32 INT32U

#define BYTE                INT8S        
#define UBYTE               INT8U       
#define WORD                INT16S     
#define UWORD               INT16U
#define LONG                INT32S
#define ULONG               INT32U
#define CHAR                INT8S
#define UCHAR               INT8U
#define USHORT              INT16U
#define BOOL                INT32U
            
#define FALSE 0
#define TRUE 1

#endif
