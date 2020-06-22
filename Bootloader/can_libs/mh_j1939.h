#ifndef _MH_J1939_H
#define _MH_J1939_H

#include <avr/io.h>
#include "can.h"

#define ADDRESS_GLOBAL              0xFF

#define CB_ACK                      0x00
#define CB_NACK                     0x01
#define CB_ACCESS_DENIED            0x02
#define CB_CANNOT_RESPOND           0x03

#define TP_CM_RTS                   0x10
#define TP_CM_CTS                   0x11
#define TP_CM_EndOfMsgACK           0x13
#define TP_CM_CA                    0xFF
#define TP_CM_BAM                   0x20

#define TP_CM_ID                    0x1CEC0000
#define TP_DT_ID                    0x1CEB0000
#define ACK_ID                      0x18E8FF00
#define REQUEST_ID                  0x18EA0000
#define PDU_FORMAT_TP_CM            236
#define PDU_FORMAT_TP_DT            235
#define PDU_FORMAT_FLASH            239

#define CTS_LIMIT                   1

#define PGN_PROP_A                  61184
#define PGN_TP_CM                   60416
#define PGN_TP_DT                   60160
#define PGN_REQUEST                 59904

#define MASK_EFF_MCU                0x1BF8FFFF
#define FILTER_EFF_MCU              0x18E80000
#define MASK_SFF_MCU                0x7FF
#define FILTER_SFF_MCU              0x000

#define SAE_RESERVED                0XFF


struct j1939_pdu_fields {
    unsigned int source_address:8;
    unsigned int pdu_specific:8;
    unsigned int pdu_format:8;
    unsigned int data_page:1;
    unsigned int extended_data_page:1;
    unsigned int priority:3;
    unsigned int is_error_frame:1;
    unsigned int is_rtr_frame:1;
    unsigned int is_extended_id:1;
    unsigned int dlc:8;
    uint8_t data[8] __attribute__((aligned(8)));
};

union j1939_pdu {
    struct can_frame can;
    struct j1939_pdu_fields j1939;
};

typedef struct {
    uint16_t page_address;
    uint8_t packet_count;
    uint16_t message_size;
    uint8_t bytes_receieved;
    uint8_t sequence_number;
} tp_cm_session_t;


// FUNCTIONS

void create_cts(union j1939_pdu *message, const tp_cm_session_t *tp_session, const uint8_t node_address);
void create_ack(union j1939_pdu *message, const uint8_t ack_status, const uint8_t node_address);

#endif // _MH_J1939_H

