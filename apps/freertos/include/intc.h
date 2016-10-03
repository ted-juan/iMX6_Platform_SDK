/***************************************************************************/
/* Module Name: irq.h                                                      */
/*                                                                         */
/* Description: system interrupt controler defind. it is HW dependent.     */
/*                                                                         */
/***************************************************************************/
#ifndef IRQ_H
#define IRQ_H

#define MAX_IRQ_NUM 128

extern INT32S SYS_Install_Handler(SYS_DEVICE *dev, INT32U location, void (*handler)(SYS_DEVICE *));
extern INT32S SYS_IRQ_Init(void);

#endif
/*end of define IRQ*/
