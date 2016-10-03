/***************************************************************************/
/* Module Name: error.h 	                                               */
/*                                                                         */
/* Description: This file defines the error types of the system            */
/*                                                                         */
/* History:                                                                */
/*                                                                         */
/***************************************************************************/
#ifndef _ERR_H_
#define _ERR_H_

#define	SYSOK			    0		/* Successful return value */
#define	SYSERR			    -1		/* Errored return value */

#define MEM_NO_FREE_BUF     -2      /* return value for pbuffer allocation */
#define MEM_TOO_MANY_BUF    -3

#define ERR_DEV_QUEUE_FULL      -100


#endif
