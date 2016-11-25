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

#define MAX_RX_MB 8

void can2_rx_handler(void);
void imx6_can1_rx_handler(void);

/* CAN module data structures */
extern uint32_t can1_port;
extern uint32_t can2_port;

extern uint32_t can_test_count;

/*! ------------------------------------------------------------
 * CAN Test (loopback can1/can2 ports)
 *  ------------------------------------------------------------
 */
void imx6_can_test(void)
{
    int i;
	uint8_t ch;

    can_test_count = 0;
    can_init(can1_port, CAN_LAST_MB);  // max 64 MB 0-63
    printf("update CAN1 rate KBS_125\n");
    can_update_bitrate(can1_port, KBPS_125);

    // set-up 8 MBs, and id 0 ~ id 7  for the RX
    for (i = 0; i < MAX_RX_MB; i++) {
        set_can_mb(can1_port, i, 0x04000000 + (i << 16), (i << 18), 0, 0);
        can_enable_mb_interrupt(can1_port, i); // enable MB interrupt for idMB=i
    }

    // set-up 8 MBs, and id 8 ~ id 15  for the TX
    for (i = MAX_RX_MB; i < (MAX_RX_MB + 8); i++) {
    	set_can_mb(can1_port, i, 0x0c000000 + ((i-7) << 16), (i << 18), 0x12345678, 0x87654321);
    }

    //enable CAN1 interrupt
    can_setup_interrupt(can1_port, imx6_can1_rx_handler, true);

    // init CAN1 MB0
    can_exit_freeze(can1_port);

    while (1) ;

    can_freeze(can1_port);

    printf("%d MBs were transmitted \n", can_test_count);
    printf("---- FLEX CAN test complete ----\n");
}

/*!
 * Can2 receive ISR function
 */
void imx6_can1_rx_handler(void)
{
    int i = 0;
    uint64_t iflag;

    iflag = can_mb_int_flag(can1_port);
    if(iflag !=0L){
        for (i = 0; i < MAX_RX_MB; i++) {
            if (iflag & (1L << i)) {
        		can_mb_int_ack(can1_port, i);
                printf("\tCAN1 MB:%d Recieved:\n", i);
                print_can_mb(can1_port, i);
                can_test_count++;
            }
        }
    }
}
