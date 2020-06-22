#include "mh_j1939.h"
#include "flash_utils.h"
/**
 ***********************************************************************************************
 @brief SAE J1939-21 CAN support
 @details Functions for using J1939 together with CAN-protocol
 @file mh_j1939.c
 @author Magnus Roos and Henrik Sellén Wikström
 @version 1.0
 @date 29-May-2020
 ***********************************************************************************************
 */

void create_cts(union j1939_pdu* message, const tp_cm_session_t* tp_session, const uint8_t node_address)
{
    uint8_t new_destination_address = message->j1939.source_address;
    message->can.can_id = TP_CM_ID | CAN_EFF_FLAG;
    message->j1939.data[0] = TP_CM_CTS;
    message->j1939.data[1] = CTS_LIMIT;
    message->j1939.data[2] = tp_session->sequence_number;
    message->j1939.data[3] = SAE_RESERVED;
    message->j1939.data[4] = SAE_RESERVED;
    message->j1939.pdu_specific = new_destination_address;
    message->j1939.source_address = node_address;
}

void create_ack(union j1939_pdu* message, const uint8_t ack_status, const uint8_t node_address)
{
    message->can.data[4] = message->j1939.source_address;
    message->can.can_id = ACK_ID | CAN_EFF_FLAG;
    message->j1939.source_address = node_address;
    message->can.can_dlc = 8;
    message->can.data[2] = SAE_RESERVED;
    message->can.data[3] = SAE_RESERVED;
    message->can.data[5] = LEAST_BITS(PGN_PROP_A);
    message->can.data[6] = MID_BITS(PGN_PROP_A);
    message->can.data[7] = MOST_BITS(PGN_PROP_A);

    message->can.data[1] = message->can.data[0];
    message->can.data[0] = ack_status;

}
