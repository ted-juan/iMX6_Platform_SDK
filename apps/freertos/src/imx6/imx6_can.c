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
uint32_t can1_port = HW_FLEXCAN1;
uint32_t can2_port = HW_FLEXCAN2;

static void CAN_IRQ_handler(void);

struct can_packet send_packet;
xQueueHandle xCANQueue;

//! possible time segment settings for propseg, pseg1, pseg2
static struct time_segment time_segments[18] = {
    {1, 2, 1},  /* 0: total 8 timequanta */
    {1, 2, 2},  /* 1: total 9 timequanta */
    {2, 2, 2},  /* 2: total 10 timequanta */
    {2, 2, 3},  /* 3: total 11 timequanta */
    {2, 3, 3},  /* 4: total 12 timequanta */
    {3, 3, 3},  /* 5: total 13 timequanta */
    {3, 3, 4},  /* 6: total 14 timequanta */
    {3, 4, 4}, /* 7: total 15 timequanta */
    {4, 4, 4}, /* 8: total 16 timequanta */
    {4, 4, 5}, /* 9: total 17 timequanta */
    {4, 5, 5}, /* 10: total 18 timequanta */
    {5, 5, 5}, /* 11: total 19 timequanta */
    {5, 5, 6}, /* 12: total 20 timequanta */
    {5, 6, 6}, /* 13: total 21 timequanta */
    {6, 6, 6}, /* 14: total 22 timequanta */
    {6, 6, 7}, /* 15: total 23 timequanta */
    {6, 7, 7}, /* 16: total 24 timequanta */
    {7, 7, 7} /* 17: total 25 timequanta */
};

void can_imx6_init(uint32_t instance, uint32_t max_mb)
{
    uint32_t ctl;
    int i;

    /* configure the I/O for the port */
    hw_can_iomux_config(instance);

    can_sw_reset(instance);         //software reset

    ctl = HW_FLEXCAN_MCR_RD(instance);
    ctl &= ~BM_FLEXCAN_MCR_MAXMB;    	//clear MAXMB field
    ctl |= BF_FLEXCAN_MCR_MAXMB(max_mb);     	// set MAXMB field (0-63)
    HW_FLEXCAN_MCR_WR(instance, ctl);
    ctl &= ~BM_FLEXCAN_MCR_MDIS;
    HW_FLEXCAN_MCR_WR(instance, ctl);

    // Programming the bit timing segments"
    HW_FLEXCAN_CTRL_WR(instance, (CAN_TIMING_PARAMETERS << 16));

    // initialize MBs to zero
    for (i = 0; i < CAN_NUMBER_OF_BUFFERS; i++) {
        set_can_mb(instance, i, 0, 0, 0, 0);
    }

    // disable all MB interrupts
    HW_FLEXCAN_IMASK1_WR(instance, 0);
    HW_FLEXCAN_IMASK2_WR(instance, 0);
}

/* reference to linux driver flexcan.c*/
void can_imx6_start(uint32_t instance, uint32_t max_mb)
{
    uint32_t ctrl, i;

    ctrl = HW_FLEXCAN_MCR_RD(instance);
    ctrl &= ~BM_FLEXCAN_MCR_MAXMB;    	//clear MAXMB field

    ctrl |= BM_FLEXCAN_MCR_FRZ | BM_FLEXCAN_MCR_FEN | BM_FLEXCAN_MCR_HALT |
    		BM_FLEXCAN_MCR_SUPV | BM_FLEXCAN_MCR_WRN_EN |
			BM_FLEXCAN_MCR_IDAM_C | BM_FLEXCAN_MCR_SRX_DIS |
			BM_FLEXCAN_MCR_WAK_MSK | BM_FLEXCAN_MCR_SLF_WAK |
			BF_FLEXCAN_MCR_MAXMB(max_mb);

    HW_FLEXCAN_MCR_WR(instance, ctrl);

    ctrl = HW_FLEXCAN_CTRL_RD(instance);
    ctrl &= ~BM_FLEXCAN_CTRL_TSYN;
    ctrl |= BM_FLEXCAN_CTRL_BOFF_REC | BM_FLEXCAN_CTRL_LBUF |
    		BM_FLEXCAN_CTRL_ERR_MSK |
            BM_FLEXCAN_CTRL_TWRN_MSK | BM_FLEXCAN_CTRL_RWRN_MSK |
			BM_FLEXCAN_CTRL_BOFF_MSK;

    HW_FLEXCAN_CTRL_WR(instance, ctrl);

    /* clear and invalidate all mailboxes first */
    for (i = FLEXCAN_TX_BUF_ID; i < CAN_NUMBER_OF_BUFFERS; i++) {
        set_can_mb(instance, i, FLEXCAN_MB_CODE_RX_INACTIVE, 0, 0, 0);
    }

    /* Errata ERR005829: mark first TX mailbox as INACTIVE */
    set_can_mb(instance, FLEXCAN_TX_BUF_RESERVED, FLEXCAN_MB_CODE_TX_INACTIVE, 0, 0, 0);

    /* mark TX mailbox as INACTIVE */
    set_can_mb(instance, FLEXCAN_TX_BUF_ID, FLEXCAN_MB_CODE_TX_INACTIVE, 0, 0, 0);

    /* acceptance mask/acceptance code (accept everything) */
    HW_FLEXCAN_RXGMASK_WR(instance, 0);
    HW_FLEXCAN_RX14MASK_WR(instance, 0);
    HW_FLEXCAN_RX15MASK_WR(instance, 0);
    HW_FLEXCAN_RXFGMASK_WR(instance, 0);

    /* unfreeze */
    ctrl = HW_FLEXCAN_MCR_RD(instance);
    ctrl &= ~BM_FLEXCAN_MCR_HALT;
    HW_FLEXCAN_MCR_WR(instance, ctrl);

    while (HW_FLEXCAN_MCR_RD(instance) & BM_FLEXCAN_MCR_FRZ_ACK) ;  // poll until complete

}

void can_update_bitrate(uint32_t instance, enum can_bitrate bitrate)
{
    unsigned int can_PE_CLK = get_peri_clock(CAN_CLK);  //can protocol engine clock, input from CCM
    unsigned int temp;
    uint32_t presdiv;	// Clock pre-divider
    struct time_segment ts;

    if (can_PE_CLK == 30000000){
        switch (bitrate){
            case MBPS_1: //
                presdiv = 1; // sets sclk ftq to 15MHz = PEclk/2
                ts = time_segments[7]; // 15 time quanta (15-8=7 for array ID)
                break;
            case KBPS_800:
                presdiv = 1; // sets sclk ftq to 15MHz = PEclk/2
                ts = time_segments[11]; // 19 time quanta (19-8=11 for array ID)
                break;
            case KBPS_500:
                presdiv = 2; // sets sclk ftq to 10MHz = PEclk/3
                ts = time_segments[12]; // 20 time quanta (20-8=12 for array ID)
                break;
            case KBPS_250:
                presdiv = 4; // sets sclk ftq to 6MHz = PEclk/5
                ts = time_segments[16]; // 24 time quanta (24-8=16 for array ID)
                break;
            case KBPS_125:
                presdiv = 9; // sets sclk ftq to 3MHz = PEclk/10
                ts = time_segments[16]; // 24 time quanta (24-8=16 for array ID)
                break;
            case KBPS_62:  //62.5kps
                presdiv = 19; // sets sclk ftq to 1.5MHz = PEclk/20
                ts = time_segments[16]; // 24 time quanta (24-8=16 for array ID)
                break;
            case KBPS_20:
                presdiv = 59; // sets sclk ftq to 500KHz = PEclk/60
                ts = time_segments[17]; // 25 time quanta (25-8=17 for array ID)
                break;
            case KBPS_10:
                presdiv = 119; // sets sclk ftq to 500KHz = PEclk/60
                ts = time_segments[17]; // 25 time quanta (25-8=17 for array ID)
                break;
            default:
                printf("CAN bitrate not supported\n");
                break;
        }
    }
    else { printf("CAN PE_CLK input to CAN module speed not supported\n");}

    // Update the the bit timing segments
    temp = HW_FLEXCAN_CTRL_RD(instance) & CAN_TIMING_MASK;
    temp += (presdiv <<24) + (ts.pseg1 <<19) + (ts.pseg2 <<16) + (ts.propseg);
    HW_FLEXCAN_CTRL_WR(instance, temp);
}

void can_sw_reset(uint32_t instance)
{
    uint32_t val;

    val = HW_FLEXCAN_MCR_RD(instance);
    val |= BF_FLEXCAN_MCR_SOFT_RST(1);
    HW_FLEXCAN_MCR_WR(instance, val);
    while (HW_FLEXCAN_MCR_RD(instance) & BF_FLEXCAN_MCR_SOFT_RST(1)) ;  // poll until complete
}

void set_can_mb(uint32_t instance, uint32_t mbID, uint32_t cs, uint32_t id, uint32_t data0, uint32_t data1)
{
    uint32_t base = REGS_FLEXCAN_BASE(instance);
    volatile struct can_message_buffers *can_mb =
        (volatile struct can_message_buffers *)(base + CAN_MB_OFFSET);

    can_mb->MB[mbID].cs = cs;
    can_mb->MB[mbID].id = id;
    can_mb->MB[mbID].data0 = data0;
    can_mb->MB[mbID].data1 = data1;
}

void print_can_mb(uint32_t instance, uint32_t mbID)
{
    uint32_t base = REGS_FLEXCAN_BASE(instance);
    volatile struct can_message_buffers *can_mb =
        (volatile struct can_message_buffers *)(base + CAN_MB_OFFSET);
    printf("\tMB[%d].cs    = 0x%x\n", mbID, can_mb->MB[mbID].cs);
    printf("\tMB[%d].id    = 0x%x\n", mbID, can_mb->MB[mbID].id);
    printf("\tMB[%d].data0 = 0x%x\n", mbID, can_mb->MB[mbID].data0);
    printf("\tMB[%d].data1 = 0x%x\n\n", mbID, can_mb->MB[mbID].data1);
}

struct can_mb *get_can_mb(uint32_t instance, uint32_t mbID)
{
    uint32_t base = REGS_FLEXCAN_BASE(instance);
    volatile struct can_message_buffers *can_mb =
        (volatile struct can_message_buffers *)(base + CAN_MB_OFFSET);

    return &can_mb->MB[mbID];
}

void can_enable_fifo_interrupt(uint32_t instance)
{
	HW_FLEXCAN_IMASK1_WR(instance, FLEXCAN_IFLAG_DEFAULT);
}

void can_enable_mb_interrupt(uint32_t instance, uint32_t mbID)
{
    uint32_t val;

    if (mbID < 32) {
	val = HW_FLEXCAN_IMASK1_RD(instance);
 	val |= (1 << mbID);
	HW_FLEXCAN_IMASK1_WR(instance, val);
    } else if (mbID < 64) {
	val = HW_FLEXCAN_IMASK2_RD(instance);
 	val |= (1 << (mbID - 32));
	HW_FLEXCAN_IMASK1_WR(instance, val);
    }
}

void can_disable_mb_interrupt(uint32_t instance, uint32_t mbID)
{
    uint32_t val;

    if (mbID < 32) {
	val = HW_FLEXCAN_IMASK1_RD(instance);
 	val &= ~(1 << mbID);
	HW_FLEXCAN_IMASK1_WR(instance, val);
    } else if (mbID < 64) {
	val = HW_FLEXCAN_IMASK2_RD(instance);
 	val &= ~(1 << (mbID - 32));
	HW_FLEXCAN_IMASK1_WR(instance, val);
    }
}

void can_exit_freeze(uint32_t instance)
{
    uint32_t val;

    val = HW_FLEXCAN_MCR_RD(instance);
    val &= ~BM_FLEXCAN_MCR_FRZ;
    HW_FLEXCAN_MCR_WR(instance, val);
}

void can_freeze(uint32_t instance)
{
    uint32_t val;

    val = HW_FLEXCAN_MCR_RD(instance);
    val |= BM_FLEXCAN_MCR_FRZ;
    HW_FLEXCAN_MCR_WR(instance, val);
}

uint64_t can_mb_int_flag(uint32_t instance)
{
    uint64_t val;

    val = (uint64_t)HW_FLEXCAN_IFLAG2_RD(instance);
    val <<= 32;
    val |=(uint64_t)HW_FLEXCAN_IFLAG1_RD(instance);

    return val;
}

void freertos_can_int_ack(uint32_t instance, uint32_t val)
{
	HW_FLEXCAN_IFLAG1_WR(instance, val);
}


void freertos_can_setup_interrupt(uint32_t instance, void (*irq_subroutine)(void), bool enableIt, uint32_t priority)
{
    uint32_t irq_id = CAN_IRQS(instance);

    if (enableIt)
    {
	// register the IRQ sub-routine
        freertos_register_interrupt_routine(irq_id, irq_subroutine);

        // enable the IRQ
        enable_interrupt(irq_id, CPU_0, priority);
    }
    else
    {
        // disable the IRQ
        disable_interrupt(irq_id, CPU_0);
    }
}
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
    iflag = can_mb_int_flag(can1_port);

	//printf("\tCAN1 MB: flag=0x%x, prio=0x%x\n", iflag, portICCRPR_RUNNING_PRIORITY_REGISTER);

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

