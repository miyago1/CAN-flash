#ifndef _MH_J1939_H
#define _MH_J1939_H

#include <avr/io.h>

#define ADDRESS_GLOBAL              0xFF

#define CB_ACK                      0x00
#define CB_NACK                     0x01
#define CB_ACCESS_DENIED            0x02
#define CB_CANNOT_RESPOND           0x03

#define PA_FLASH_REQUEST            0x00
#define PA_FLASH_INFO               0x01
#define PA_FLASH_VERIFY             0x02
#define PA_FLASH_DONE               0x03

#define TP_CM_RTS                   0x10
#define TP_CM_CTS                   0x11
#define TP_CM_EndOfMsgACK           0x13
#define TP_CM_CA                    0xFF
#define TP_CM_BAM                   0x20

#define PDU_FORMAT_ACK              232
#define PDU_FORMAT_TP_CM            236
#define PDU_FORMAT_TP_DT            235
#define PDU_FORMAT_FLASH            239

#define LEAST_BITS(n)               ( n & 0x0000FF)
#define MID_BITS(n)                 (( n & 0x00FF00 ) >> 8)
#define MOST_BITS(n)                (( n & 0xFF0000 ) >> 16)

#define THIS_NODE_CTS_LIMIT 2

#define PGN_PROP_A                  61184
#define PGN_TP_CM                   60416
#define PGN_TP_DT                   60160
#define PGN_REQUEST                 59904

#define MASK_EFF_SIDL               0xF8
#define MASK_EFF_SIDH               0x1B
#define FILTER_EFF_SIDL             0xE8
#define FILTER_EFF_SIDH             0x18

#define MASK_SFF_MCU                0x7FF
#define FILTER_SFF_MCU              0x000


typedef struct {
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
} j1939_pdu_t;

typedef struct {
    uint16_t page_address;
    uint8_t packet_count;
    uint16_t message_size;
    uint8_t bytes_receieved;
    uint8_t sequence_number;
} tp_cm_session_t;


// FUNCTIONS
void create_cts(j1939_pdu_t *message, const tp_cm_session_t *tp_session, const uint8_t node_address);
void create_ack(j1939_pdu_t *message, const uint8_t ack_status, const uint8_t node_address);

#endif // _MH_J1939_H

