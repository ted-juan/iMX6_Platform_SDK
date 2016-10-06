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
#include "core/interrupt.h"
#include "core/gic.h"
#include "FreeRTOS.h"

extern INT32S SYS_Install_Handler(SYS_DEVICE *dev, INT32U location, void (*handler)(SYS_DEVICE *dev));

/* epit instance for system tick */
static uint32_t epit_instance = HW_EPIT2;
static volatile uint8_t g_wait_for_irq;
extern irq_hdlr_t g_interrupt_handlers[IMX_INTERRUPT_COUNT];
extern volatile uint32_t g_vectNum[4];
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

#if 1
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
*   Public Function: vConfigureTickInterrupt
*
*       System Timer init function
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
void vConfigureTickInterrupt( void )
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
    epit_setup_interrupt(epit_instance, FreeRTOS_Tick_Handler, TRUE);
    epit_get_compare_event(epit_instance);
    epit_counter_enable(epit_instance, 10000, IRQ_MODE); // 10ms

#if 0
    while ((counter/100) != max_duration) {
        g_wait_for_irq = 1;
        while (g_wait_for_irq == 1);
        counter++;

        if (!(counter%100))
            printf("Elapsed time %d seconds <=> %d ticks.\n", counter/100, counter);
    };
#endif

}

void vClearTickInterrupt(void)
{
	 /* clear the compare event flag */
	epit_get_compare_event(epit_instance);

}

/* This is the callback function which is called by the FreeRTOS Cortex-A port
layer in response to an interrupt.  If the function is called
vApplicationFPUSafeIRQHandler() then it is called after the floating point
registers have been saved.  If the function is called vApplicationIRQHandler()
then it will be called without first having saved the FPU registers.  See
http://www.freertos.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html for
more information */
void vApplicationFPUSafeIRQHandler( uint32_t ulICCIAR )
{
    // vectNum = RESERVED[31:13] | CPUID[12:10] | INTERRUPT_ID[9:0]
    // send ack and get ID source
    uint32_t vectNum = gic_read_irq_ack();

    /* Re-enable interrupts. */
    __asm ( "cpsie i" );

    // Check that INT_ID isn't 1023 or 1022 (spurious interrupt)
    if (vectNum & 0x0200)
    {
        gic_write_end_of_irq(vectNum);  // send end of irq
    }
    else
    {
        // copy the local value to the global image of CPUID
        unsigned cpu = (vectNum >> 10) & 0x7;
        unsigned irq = vectNum & 0x1FF;

        // Store the current interrupt number.
        g_vectNum[cpu] = irq;

        // Call the service routine stored in the handlers array. If there isn't
        // one for this IRQ, then call the default handler.
        irq_hdlr_t isr = g_interrupt_handlers[irq];
        if (isr)
        {
            isr();
        }
        else
        {
            default_interrupt_routine();
        }

        // Clear current interrupt number.
        g_vectNum[cpu] = 0;

        // Signal the end of the irq.
        gic_write_end_of_irq(vectNum);
    }
}

