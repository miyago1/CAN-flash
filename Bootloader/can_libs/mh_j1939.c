#include "mh_j1939.h"
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

void create_cts(j1939_pdu_t* message, const tp_cm_session_t* tp_session, const uint8_t node_address)
{
    uint8_t new_destination_address = message->source_address;
    message->priority = 7;
    message->extended_data_page = 0;
    message->data_page = 0;
    message->pdu_format = PDU_FORMAT_TP_CM;
    message->data[0] = TP_CM_CTS;
    message->data[1] = 1; //send limit
    message->data[2] = tp_session->sequence_number;
    message->data[3] = 0xFF;
    message->data[4] = 0xFF;
    message->pdu_specific = new_destination_address;
    message->source_address = node_address;
}

void create_ack(j1939_pdu_t* message, const uint8_t ack_status, const uint8_t node_address)
{
    message->data[4] = message->source_address;
    message->priority = 6;
    message->extended_data_page = 0;
    message->data_page = 0;
    message->pdu_format = PDU_FORMAT_ACK;
    message->source_address = node_address;
    message->pdu_specific = 0xFF;
    message->dlc = 8;
    message->data[2] = 0xFF;
    message->data[3] = 0xFF;
    message->data[5] = LEAST_BITS(PGN_PROP_A);
    message->data[6] = MID_BITS(PGN_PROP_A);
    message->data[7] = MOST_BITS(PGN_PROP_A);

    message->data[1] = message->data[0];
    message->data[0] = ack_status;

}
