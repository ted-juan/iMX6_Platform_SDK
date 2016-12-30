/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
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

#include "sdk.h"
#include "flexcan/flexcan.h"
#include "registers/regsflexcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "assert.h"
#include "imx6_can.h"
#include "flexcan/flexcan.h"

extern struct can_mb *get_can_mb(uint32_t instance, uint32_t mbID);

/* CAN module data structures */
extern uint32_t can1_port;
extern uint32_t can2_port;
extern uint32_t can_test_count;

static void CAN_IRQ_handler(void);

struct can_packet send_packet;
xQueueHandle xCANQueue;

/*!
 * Can1 receive ISR function
 */
void CAN_IRQ_handler(void)
{
    uint64_t iflag;
    struct can_packet *packet;
    volatile struct can_mb *can_mb;
    BaseType_t xHigherPriorityTaskWoken;

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;
    can_test_count++;
    iflag = can_mb_int_flag(can1_port);

	//printf("\tCAN1 MB: count:%d, flag=0x%x, prio=0x%x\n", can_test_count, iflag, portICCRPR_RUNNING_PRIORITY_REGISTER);

    if(iflag & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE)
    {
		packet = malloc(sizeof(struct can_packet));
		if (packet == NULL)
			printf("can pkt alloc fail");
		else
		{
			can_mb = get_can_mb( can1_port, FLEXCAN_RX_BUF_ID);

			if (can_mb->cs & FLEXCAN_MB_CNT_IDE)
				packet->can_id = (can_mb->id & CAN_EFF_MASK) | CAN_EFF_FLAG;
			else
				packet->can_id = (can_mb->id >> 18) & CAN_SFF_MASK;

			if (can_mb->cs & FLEXCAN_MB_CNT_RTR)
				packet->can_id |= CAN_RTR_FLAG;

			packet->can_dlc = (can_mb->cs >> 16) & 0x0f;
			*(INT32U *)&packet->data[0] = htonl(can_mb->data0);
			*(INT32U *)&packet->data[4] = htonl(can_mb->data1);
			xQueueSendFromISR( xCANQueue, (void *)&packet, &xHigherPriorityTaskWoken );
			//print_can_mb(can1_port, FLEXCAN_RX_BUF_ID);
    	}
		freertos_can_int_ack(can1_port, FLEXCAN_IFLAG_RX_FIFO_AVAILABLE);
    }

    /* transmission complete interrupt */
    if (iflag & (1 << FLEXCAN_TX_BUF_ID)) {
            /* after sending a RTR frame mailbox is in RX mode */
			can_mb = get_can_mb( can1_port, FLEXCAN_TX_BUF_ID);
			can_mb->cs = FLEXCAN_MB_CODE_TX_INACTIVE;

			freertos_can_int_ack(can1_port, (1 << FLEXCAN_TX_BUF_ID));
    }

    /* Now the buffer is empty we can switch context if necessary. */
    if( xHigherPriorityTaskWoken )
    {
        /* Actual macro used here is port specific. */
        taskYIELD();
    }
}
void CAN_send( struct can_packet *cf)
{
    volatile struct can_mb *can_mb;
	u32 can_id;
    u32 ctrl = FLEXCAN_MB_CNT_CODE(0xc) | (cf->can_dlc << 16);
    u32 data[2];

    if (cf->can_id & CAN_EFF_FLAG) {
            can_id = cf->can_id & CAN_EFF_MASK;
            ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
    } else {
            can_id = (cf->can_id & CAN_SFF_MASK) << 18;
    }

    if (cf->can_id & CAN_RTR_FLAG)
            ctrl |= FLEXCAN_MB_CNT_RTR;

    data[0] = htonl(*(INT32U *)&cf->data[0]);
    data[1] = htonl(*(INT32U *)&cf->data[4]);

    set_can_mb(can1_port, FLEXCAN_TX_BUF_ID, ctrl,
               can_id, data[0], data[1]);

    /* Errata ERR005829 step8:
     * Write twice INACTIVE(0x8) code to first MB.
     */
	can_mb = get_can_mb( can1_port, FLEXCAN_TX_BUF_RESERVED);
    can_mb->cs = FLEXCAN_MB_CODE_TX_INACTIVE;
    can_mb->cs = FLEXCAN_MB_CODE_TX_INACTIVE;
}

//! periodically send CAN packet ID 0x152
static void CAN_Send_Task( void *x)
{
    struct can_packet *packet = malloc(sizeof(struct can_packet));
    (void)x;

    packet->can_id = 0x100;
    packet->can_dlc = 8;
    *(INT32U *)&packet->data[0] = 0x11111111;
    *(INT32U *)&packet->data[4] = 0x22222222;

    for( ;; )
    {
        CAN_send(packet);
        packet->can_id++;
        (*(INT32U *)&packet->data[0])++;
        (*(INT32U *)&packet->data[4])++;
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}


static void CAN_Rcv_Task( void *x)
{
    struct can_packet *packet;
    (void)x;

    while(1)
    {
        if(xQueueReceive(xCANQueue, &packet, 10/portTICK_RATE_MS) == pdPASS)
        {
            /* handlie the receive packet here*/
            printf("CAN_Rcv_Task\n");
            printf("\t.id   = 0x%x\n", packet->can_id);
            printf("\t.len  = 0x%x\n", packet->can_dlc);
            printf("\tdata0 = 0x%x\n", *(INT32U *)&packet->data[0]);
            printf("\tdata1 = 0x%x\n", *(INT32U *)&packet->data[4]);

            /* after handling the rx packet, free it */
            free(packet);
        }
        vTaskDelay(100/portTICK_RATE_MS);
    }
}


void CAN_probe(void)
{
	int i;

    can_imx6_init(can1_port, CAN_LAST_MB);

    printf("update CAN1 rate KBS_125\n");
    can_update_bitrate(can1_port, KBPS_125);

    can_imx6_start(can1_port, FLEXCAN_TX_BUF_ID);

    //enable CAN1 interrupt
    freertos_can_setup_interrupt(can1_port, CAN_IRQ_handler, true, CAN_IRQ_PRIO << portPRIORITY_SHIFT );

    /* enable FIFO interrupt */
    can_enable_fifo_interrupt(can1_port);

    // start
    can_exit_freeze(can1_port);


}

//! system initialisation
int CAN_Init( void )
{
    /* Create message queue */
    xCANQueue = xQueueCreate(MAX_CAN_QUEUE_DEPTH , sizeof(struct can_packet *));
    if (xCANQueue == NULL)
        printf("Create queue fail\n");

    xTaskCreate(CAN_Rcv_Task, "can_rcv_task", SYS_TASK_STK_SIZE, 0, SYS_TASK_PRIO , NULL );
    xTaskCreate(CAN_Send_Task, "can_send_task", SYS_TASK_STK_SIZE, 0, SYS_TASK_PRIO , NULL );

    CAN_probe();
    //imx6_can_test();

    return 0;
}

