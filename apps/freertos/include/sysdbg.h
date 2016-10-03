/*************************************************************************/
/*                                                                       */
/* Module Name:     sysdbg.h                                             */
/*                                                                       */
/* DESCRIPTION:                                                          */
/*                                                                       */
/*      This file contains the public debug macro and declearation.      */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
#ifndef SYSDBG_H
#define SYSDBG_H


extern void SYS_PrintFrame(INT8U *, INT16S) ;
extern VOID SYS_DumpMem(INT8U *pbuff, INT32S len);

#define SYS_ASSERT(fmt,y) 				\
{										\
	if (!(y))							\
	{									\
		printf("Assertion failed at %s:%d\n", __FILE__, __LINE__); \
        printf fmt; for(;;);			\
	}									\
}

// define the error message structure          
typedef struct
{
    const INT16S err_code;
    const CHAR **err_msg;
}ERR_MSG;


// define debug macro and utility          
#ifdef DEBUG
                                    
#define SYS_DEBUGP(debug, mask, fmt)   if(debug & mask){ printf fmt; }

#define SYS_FRAMEP(debug_len, fmt, ptr, max_len) if (debug_len) \
        { printf fmt; \
          SYS_PrintFrame(ptr, ((debug_len > max_len) ? max_len : debug_len)); }
          
#define PROB_SIZE 256
struct prob
{
    INT8U step;
    INT16U count;
};
extern struct prob Prob[PROB_SIZE];
extern struct prob *pProb;

#else
#define SYS_DEBUGP(debug, mask, fmt)    {}
#define SYS_FRAMEP(debug_len, fmt, ptr, max_len)    {}
#define SYS_PROB(id)    {}
#endif


/* Define mask values used by debugging */
#define DBG_MASK1           0xFF
#define DBG_MASK2           0xFE
#define DBG_MASK3           0xFC
#define DBG_MASK4           0xF8
#define DBG_MASK5           0xF0


/* Define debugging levels */
#define DBG_LEVEL0          0x00        /* None */
#define DBG_LEVEL1          0x01        /* Show MASK 1 */
#define DBG_LEVEL2          0x03        /* Show MASK 1, 2 */
#define DBG_LEVEL3          0x07        /* Show MASK 1, 2, 3 */
#define DBG_LEVEL4          0x0F        /* Show MASK 1, 2, 3, 4 */
#define DBG_LEVEL5          0x1F        /* Show MASK 1, 2, 3, 4, 5 */

        
#endif  /* SYSDBG_H */
