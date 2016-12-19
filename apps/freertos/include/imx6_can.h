
//! can packet structure

//! used to describe a CAN20a (standard 11 bit) packet.
//! a packet length of 0 indicates an RTR packet
struct can_packet
{
    unsigned int can_id;	//!< CAN identifier
    unsigned int can_dlc;	//!< CAN data length
    unsigned char data[8];	//!< CAN data array [8]
};

#define MAX_CAN_QUEUE_DEPTH 20

void CAN_send( struct can_packet *cf);
