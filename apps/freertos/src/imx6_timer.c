/***************************************************************************/
/* Module Name: imx6_timer.c 		                                       */
/*                                                                         */
/* Description: This module implement the main C entry function for        */
/*          timer interrupt service routing                                */
/*                                                                         */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
#include <stdio.h>
#include "timer/epit.h"
#include "timer/timer.h"
#include "registers/regsepit.h"
#include "FreeRTOSConfig.h"

extern INT32S SYS_Install_Handler(SYS_DEVICE *dev, INT32U location, void (*handler)(SYS_DEVICE *dev));

/* epit instance for system tick */
static uint32_t epit_instance = HW_EPIT2;
static volatile uint8_t g_wait_for_irq;

/*
*============================================================================
*   Public Function: OS_TimerInit
*
*      The ISR used for the scheduler tick.
*
*   Interface parameter:
*		Input:
*			none
*
*		Output:
*			none
*
*============================================================================
*/

/*
 * The ISR used for the scheduler tick.
 */
void vTickISR(void)
{
    g_wait_for_irq = 0;
	 /* clear the compare event flag */
	epit_get_compare_event(epit_instance);

#if 0
    /* Increment the RTOS tick count, then look for the highest priority
	task that is ready to run. */
	__asm volatile("bl xTaskIncrementTick");

	#if configUSE_PREEMPTION == 1
		__asm volatile("bl vTaskSwitchContext");
	#endif
#endif
}

/*
*============================================================================
*   Public Function: OS_TimerInit
*
*       System Timer init function
*
*   Interface parameter:
*		Input:
*			none
*
*		Output:
*			SYSOK	- successful
*			other	- fail
*
*============================================================================
*/
INT32S OS_TimerInit( void )
{
    uint32_t counter = 0;
    /* stops after xx seconds */
    uint32_t max_duration = 10;
    uint32_t freq = 0;
    uint32_t g_system_timer_port = HW_EPIT1;

#if 0
    freq = get_main_clock(IPG_CLK);
    epit_init(g_system_timer_port, CLKSRC_IPG_CLK, freq / 1000000,
              SET_AND_FORGET, 1000, WAIT_MODE_EN | STOP_MODE_EN);
#endif

    /* Initialize the EPIT timer used for tick timer. An interrupt
       is generated every 10ms */
    /* typical IPG_CLK is in MHz, so divide it to get a reference
       clock of 1MHz => 1us per count */
    freq = get_main_clock(IPG_CLK);
    epit_init(epit_instance, CLKSRC_IPG_CLK, freq/1000000,
              SET_AND_FORGET, 10000, WAIT_MODE_EN | STOP_MODE_EN);
    epit_setup_interrupt(epit_instance, vTickISR, TRUE);
    epit_counter_enable(epit_instance, 10000, IRQ_MODE); // 10ms

    while ((counter/100) != max_duration) {
        g_wait_for_irq = 1;
        while (g_wait_for_irq == 1);
        counter++;

        if (!(counter%100))
            printf("Elapsed time %d seconds <=> %d ticks.\n", counter/100, counter);
    };

	return SYSOK;
}
