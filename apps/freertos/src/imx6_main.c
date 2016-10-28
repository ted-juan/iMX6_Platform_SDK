/*
 * Copyright (c) 2010-2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "sdk.h"
#include "platform_init.h"
#include "cortex_a9.h"
#include "mmu.h"
#include "sdk_version.h"
#include "print_clock_info.h"
#include "print_version.h"
#include "cpu_utility/cpu_utility.h"
#include "FreeRTOS.h"
#include "task.h"

extern int SDK_TEST(void);

INT8U   DBG_SysInfo = DBG_LEVEL1;
struct SYS_IO_BUF	SYS_OutBuf;
extern void gpio_buzzer(int);
extern void gpio_beep(void);
extern void multicore_test(void);

static void SYS_Task1(void *pvParameters)
{
	const uint32_t xFrequency = 100;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	pvParameters= pvParameters; /* avoid compile warning */

	while(1)
	{
		printf("TASK1\n");
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		//vTaskDelay(1);
	}
}

static void SYS_Task2(void *pvParameters)
{
	const uint32_t xFrequency = 100;
	TickType_t xLastWakeTime = xTaskGetTickCount();
    int *gpio_addr = (int *)0x020ac000;
    int data=0;

	pvParameters= pvParameters; /* avoid compile warning */

    gpio_beep();
    gpio_beep();

#if 0
    __asm volatile (
    "ldr     r10,=_start \n"
    "blx     r10  \n");
#endif


	while(1)
	{
		printf("TASK2\n");
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
        gpio_beep();

#if 0
        data = *gpio_addr;
        *gpio_addr |= 0x1000;
#endif
		//vTaskDelay(1);
	}
}

/*
*============================================================================
*   Public Function: SYS_Init
*       Create the processes of the system. The function will get the
*       information of process and create them.
*
*   Interface parameter:
*       Input: void pointer
*
*       Return value: SYSOK - successful
*                     other - fail
*
*============================================================================
*/
INT32S SYS_Init(void)
{
    INT8U err;
	INT32S ret;
	uint32_t xLastWakeTime;

    /*this function should be done before driver using the none cache region*/
//    SYS_NoneCacheRegionInit();
//    SYS_MEM_Init();

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	printf("start timer= %d\n", xLastWakeTime);

    /*Create the system task*/
	err = xTaskCreate(SYS_Task1, "sys_task1", SYS_TASK_STK_SIZE, NULL, SYS_TASK_PRIO, ( xTaskHandle * ) NULL);
	err |= xTaskCreate(SYS_Task2, "sys_task2", SYS_TASK_STK_SIZE, NULL, SYS_TASK_PRIO, ( xTaskHandle * ) NULL);

	if (err == pdPASS)
        return SYSOK;
    else
        return SYSERR;
}

/*!
 * main function that decides which tests to run and prompts the user before
 * running test.
 * @return      should never return because ARM is at WFI (low power mode in the end)
 */
INT32S main(void)
{
	INT32S ret;
    uint32_t cpu_id = cpu_get_current();

    while (cpu_id != 0)
        gpio_beep();

    /* hardware initialize */
    platform_init();

    print_version();

    show_freq();
    show_ddr_config();

//    multicore_test();

    /*clear output buffer memory*/
    memset((CHAR*)&SYS_OutBuf,0,sizeof(struct SYS_IO_BUF));

    printf("\n\n");
    printf("==========================================\n");
    printf("%s - FreeRTOS\n",configSYS_PLATFORM_NAME);
    printf("==========================================\n");
    printf("System running!....\n");

    // Run the unit test function.
    //SDK_TEST();

    /* Create system tasks */
    ret = SYS_Init();
    SYS_ASSERT(("<1> Cannot initial system\n"),ret==SYSOK);

	vTaskStartScheduler();

	while(1);

    return 0;
}
