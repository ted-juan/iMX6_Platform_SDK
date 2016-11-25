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
#include "queue.h"
#include "semphr.h"

//#define MULTICORE
#define MAX_MSG_QUEUE_DEPTH 20

INT8U DBG_SysInfo = DBG_LEVEL1;
struct SYS_IO_BUF	SYS_OutBuf;

/* external function */
extern void gpio_buzzer(int);
extern void gpio_beep(void);
extern void multicore_test(void);
extern void flexcan_test(void);


xQueueHandle xMsgQueue;
SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t xSemaphoreTx = NULL;
SemaphoreHandle_t xSemaphoreRx = NULL;

struct SYS_Message
{
    INT32U ucMessageID;
    portCHAR ucData[ 20 ];
};


static void SYS_Task1(void *pvParameters)
{
	INT32U count = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	struct SYS_Message *pTxMessage;

	pvParameters= pvParameters; /* avoid compile warning */

	while(1)
	{
		if( xSemaphoreTake(xSemaphoreTx, portMAX_DELAY) == pdTRUE )
		{
			/* use mutex to protect data */
			if( xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE )
			{
				pTxMessage = malloc(sizeof(struct SYS_Message));
				if (pTxMessage != NULL)
				{
					pTxMessage->ucMessageID = count++;
					xQueueSend( xMsgQueue, (void *)&pTxMessage, 0 );
					printf("TASK1 tx id=%d\n", count);
				}
				xSemaphoreGive( xSemaphore );
			}
			xSemaphoreGive( xSemaphoreRx );
		}
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

static void SYS_Task2(void *pvParameters)
{
	const INT32U xFrequency = 100;
	struct SYS_Message *pRxMessage;

	pvParameters= pvParameters; /* avoid compile warning */

	while(1)
	{
		if( xSemaphoreTake(xSemaphoreRx, portMAX_DELAY) == pdTRUE )
		{
			/* use mutex to protect data */
			if( xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE )
			{
				if(xQueueReceive(xMsgQueue, &pRxMessage, 10/portTICK_RATE_MS) == pdPASS)
				{
						printf("TASK2 rx id=%d\n", pRxMessage->ucMessageID);
						free(pRxMessage);
				}
				xSemaphoreGive( xSemaphore );
			}
			xSemaphoreGive( xSemaphoreTx );
		}
		vTaskDelay(300/portTICK_RATE_MS);
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
	INT32U xLastWakeTime;

    /*this function should be done before driver using the none cache region*/
//    SYS_NoneCacheRegionInit();
//    SYS_MEM_Init();

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	printf("start timer= %d\n", xLastWakeTime);

	/* Create message queue */
	xMsgQueue = xQueueCreate(MAX_MSG_QUEUE_DEPTH , sizeof(struct SYS_Message *));
    if (xMsgQueue == NULL)
	    printf("Create queue fail\n");

	/* Create mutex */
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL)
	    printf("Create mutex fail\n");

    /* Create Semaphore , max 10,  init 10*/
    xSemaphoreTx = xSemaphoreCreateCounting( 10, 10 );
    if (xSemaphore == NULL)
	    printf("Create sem Tx fail\n");

    /* Create Semaphore , max 10,  init 0*/
    xSemaphoreRx = xSemaphoreCreateCounting( 10, 0 );
    if (xSemaphoreRx == NULL)
	    printf("Create sem Rx fail\n");
    /*Create the system task*/
    /* message queue test */
	err = xTaskCreate(SYS_Task1, "sys_task1", SYS_TASK_STK_SIZE, NULL, SYS_TASK_PRIO, ( xTaskHandle * ) NULL);
	err |= xTaskCreate(SYS_Task2, "sys_task2", SYS_TASK_STK_SIZE, NULL, SYS_TASK_PRIO, ( xTaskHandle * ) NULL);

	if (err == pdPASS)
	{
		vTaskStartScheduler();
        return SYSOK;
	}
    else
    {
        printf("Create task fail\n");
        return SYSERR;
    }
}

#ifdef MULTICORE

void cpu1_entry(void * arg)
{
    INT32U cpu_id = cpu_get_current();
    INT32S cpuCount = cpu_get_cores();
    INT32S ret;

    gpio_beep();

    if (cpuCount == 1)
    {
        printf("This chip only has one CPU!\n");
        return;
    }

    printf("I am CPU %d!\n", cpu_id);

    if (cpu_id != 0)
    {
        // Enable interrupts on secondary CPUs.
        gic_init_cpu();
    }

    /* Create system tasks */
    ret = SYS_Init();

	while(1);
}

#endif


/*!
 * main function that decides which tests to run and prompts the user before
 * running test.
 * @return      should never return because ARM is at WFI (low power mode in the end)
 */
INT32S main(void)
{
	INT32S ret;
    INT32U cpu_id = cpu_get_current();

    /* hardware initialize */
    platform_init();

    print_version();

    show_freq();
    show_ddr_config();

    /*clear output buffer memory*/
    memset((CHAR*)&SYS_OutBuf,0,sizeof(struct SYS_IO_BUF));

    printf("\n\n");
    printf("==========================================\n");
    printf("%s - FreeRTOS\n",configSYS_PLATFORM_NAME);
    printf("==========================================\n");
    printf("System running!....\n");


#ifdef MULTICORE
    // start second cpu
    cpu_start_secondary(1, &cpu1_entry, 0);

  	while(1);
#endif

    //Run the unit test function.
    //multicore_test();
    //epit_test();
    //flexcan_test();
    imx6_can_test();

    /* Create system tasks */
    ret = SYS_Init();

	while(1);

    return 0;
}


