#ifndef MCP2515_C_H
#define MCP2515_C_H

#include <avr/io.h>
#include <stdbool.h>
#include "mh_spi.h"
#include "can.h"

// ENDLESS DEFINES
#define MCP_8MHz_1000kBPS_CFG1 (0x00)
#define MCP_8MHz_1000kBPS_CFG2 (0x80)
#define MCP_8MHz_1000kBPS_CFG3 (0x80)

#define MCP_8MHz_500kBPS_CFG1 (0x00)
#define MCP_8MHz_500kBPS_CFG2 (0x90)
#define MCP_8MHz_500kBPS_CFG3 (0x82)

#define MCP_8MHz_250kBPS_CFG1 (0x00)
#define MCP_8MHz_250kBPS_CFG2 (0xB1)
#define MCP_8MHz_250kBPS_CFG3 (0x85)

#define MCP_8MHz_200kBPS_CFG1 (0x00)
#define MCP_8MHz_200kBPS_CFG2 (0xB4)
#define MCP_8MHz_200kBPS_CFG3 (0x86)

#define MCP_8MHz_125kBPS_CFG1 (0x01)
#define MCP_8MHz_125kBPS_CFG2 (0xB1)
#define MCP_8MHz_125kBPS_CFG3 (0x85)

#define MCP_8MHz_100kBPS_CFG1 (0x01)
#define MCP_8MHz_100kBPS_CFG2 (0xB4)
#define MCP_8MHz_100kBPS_CFG3 (0x86)

#define MCP_8MHz_80kBPS_CFG1 (0x01)
#define MCP_8MHz_80kBPS_CFG2 (0xBF)
#define MCP_8MHz_80kBPS_CFG3 (0x87)

#define MCP_8MHz_50kBPS_CFG1 (0x03)
#define MCP_8MHz_50kBPS_CFG2 (0xB4)
#define MCP_8MHz_50kBPS_CFG3 (0x86)

#define MCP_8MHz_40kBPS_CFG1 (0x03)
#define MCP_8MHz_40kBPS_CFG2 (0xBF)
#define MCP_8MHz_40kBPS_CFG3 (0x87)

#define MCP_8MHz_33k3BPS_CFG1 (0x47)
#define MCP_8MHz_33k3BPS_CFG2 (0xE2)
#define MCP_8MHz_33k3BPS_CFG3 (0x85)

#define MCP_8MHz_31k25BPS_CFG1 (0x07)
#define MCP_8MHz_31k25BPS_CFG2 (0xA4)
#define MCP_8MHz_31k25BPS_CFG3 (0x84)

#define MCP_8MHz_20kBPS_CFG1 (0x07)
#define MCP_8MHz_20kBPS_CFG2 (0xBF)
#define MCP_8MHz_20kBPS_CFG3 (0x87)

#define MCP_8MHz_10kBPS_CFG1 (0x0F)
#define MCP_8MHz_10kBPS_CFG2 (0xBF)
#define MCP_8MHz_10kBPS_CFG3 (0x87)

#define MCP_8MHz_5kBPS_CFG1 (0x1F)
#define MCP_8MHz_5kBPS_CFG2 (0xBF)
#define MCP_8MHz_5kBPS_CFG3 (0x87)

#define INSTRUCTION_RESET 0xC0
#define INSTRUCTION_WRITE 0x02
#define INSTRUCTION_READ 0x03
#define INSTRUCTION_BITMOD 0x05
#define INSTRUCTION_LOAD_TX0 0x40
#define INSTRUCTION_LOAD_TX1 0x42
#define INSTRUCTION_LOAD_TX2 0x44
#define INSTRUCTION_RTS_TX0 0x81
#define INSTRUCTION_RTS_TX1 0x82
#define INSTRUCTION_RTS_TX2 0x84
#define INSTRUCTION_RTS_ALL 0x87
#define INSTRUCTION_READ_RX0 0x90
#define INSTRUCTION_READ_RX1 0x94
#define INSTRUCTION_READ_STATUS 0xA0
#define INSTRUCTION_RX_STATUS 0xB0
#define INSTRUCTION_RESET 0xC0

#define RXBnCTRL_RXM_STD 0x20
#define RXBnCTRL_RXM_EXT 0x40
#define RXBnCTRL_RXM_STDEXT 0x00
#define RXBnCTRL_RXM_MASK 0x60
#define RXBnCTRL_RTR 0x08
#define RXB0CTRL_BUKT 0x04
#define RXB0CTRL_FILHIT_MASK 0x03
#define RXB1CTRL_FILHIT_MASK 0x07
#define RXB0CTRL_FILHIT 0x00
#define RXB1CTRL_FILHIT 0x01

#define CANCTRL_REQOP 0xE0
#define CANCTRL_ABAT 0x10
#define CANCTRL_OSM 0x08
#define CANCTRL_CLKEN 0x04
#define CANCTRL_CLKPRE 0x03

#define CANSTAT_OPMOD 0xE0

#define MCP_SIDH 0
#define MCP_SIDL 1
#define MCP_EID8 2
#define MCP_EID0 3
#define MCP_DLC 4
#define MCP_DATA 5

#define TXB_EXIDE_MASK 0x08

#define N_TXBUFFERS 3
#define N_RXBUFFERS 2

#define RTR_MASK 0x40

#define MCP_RXF0SIDH 0x00
#define MCP_RXF0SIDL 0x01
#define MCP_RXF0EID8 0x02
#define MCP_RXF0EID0 0x03
#define MCP_RXF1SIDH 0x04
#define MCP_RXF1SIDL 0x05
#define MCP_RXF1EID8 0x06
#define MCP_RXF1EID0 0x07
#define MCP_RXF2SIDH 0x08
#define MCP_RXF2SIDL 0x09
#define MCP_RXF2EID8 0x0A
#define MCP_RXF2EID0 0x0B
#define MCP_CANSTAT 0x0E
#define MCP_CANCTRL 0x0F
#define MCP_RXF3SIDH 0x10
#define MCP_RXF3SIDL 0x11
#define MCP_RXF3EID8 0x12
#define MCP_RXF3EID0 0x13
#define MCP_RXF4SIDH 0x14
#define MCP_RXF4SIDL 0x15
#define MCP_RXF4EID8 0x16
#define MCP_RXF4EID0 0x17
#define MCP_RXF5SIDH 0x18
#define MCP_RXF5SIDL 0x19
#define MCP_RXF5EID8 0x1A
#define MCP_RXF5EID0 0x1B
#define MCP_TEC 0x1C
#define MCP_REC 0x1D
#define MCP_RXM0SIDH 0x20
#define MCP_RXM0SIDL 0x21
#define MCP_RXM0EID8 0x22
#define MCP_RXM0EID0 0x23
#define MCP_RXM1SIDH 0x24
#define MCP_RXM1SIDL 0x25
#define MCP_RXM1EID8 0x26
#define MCP_RXM1EID0 0x27
#define MCP_CNF3 0x28
#define MCP_CNF2 0x29
#define MCP_CNF1 0x2A
#define MCP_CANINTE 0x2B
#define MCP_CANINTF 0x2C
#define MCP_EFLG 0x2D
#define MCP_TXB0CTRL 0x30
#define MCP_TXB0SIDH 0x31
#define MCP_TXB0SIDL 0x32
#define MCP_TXB0EID8 0x33
#define MCP_TXB0EID0 0x34
#define MCP_TXB0DLC 0x35
#define MCP_TXB0DATA 0x36
#define MCP_TXB1CTRL 0x40
#define MCP_TXB1SIDH 0x41
#define MCP_TXB1SIDL 0x42
#define MCP_TXB1EID8 0x43
#define MCP_TXB1EID0 0x44
#define MCP_TXB1DLC 0x45
#define MCP_TXB1DATA 0x46
#define MCP_TXB2CTRL 0x50
#define MCP_TXB2SIDH 0x51
#define MCP_TXB2SIDL 0x52
#define MCP_TXB2EID8 0x53
#define MCP_TXB2EID0 0x54
#define MCP_TXB2DLC 0x55
#define MCP_TXB2DATA 0x56
#define MCP_RXB0CTRL 0x60
#define MCP_RXB0SIDH 0x61
#define MCP_RXB0SIDL 0x62
#define MCP_RXB0EID8 0x63
#define MCP_RXB0EID0 0x64
#define MCP_RXB0DLC 0x65
#define MCP_RXB0DATA 0x66
#define MCP_RXB1CTRL 0x70
#define MCP_RXB1SIDH 0x71
#define MCP_RXB1SIDL 0x72
#define MCP_RXB1EID8 0x73
#define MCP_RXB1EID0 0x74
#define MCP_RXB1DLC 0x75
#define MCP_RXB1DATA 0x76


#define ERROR_OK 0
#define ERROR_FAIL 1
#define ERROR_ALLTXBUSY 2
#define ERROR_FAILINIT 3
#define ERROR_FAILTX 4
#define ERROR_NOMSG 5

#define MASK0 0
#define MASK1 1

#define CANCTRL_REQOP_NORMAL 0x00
#define CANCTRL_REQOP_SLEEP 0x20
#define CANCTRL_REQOP_LOOPBACK 0x40
#define CANCTRL_REQOP_LISTENONLY 0x60
#define CANCTRL_REQOP_CONFIG 0x80
#define CANCTRL_REQOP_POWERUP 0xE0

#define RXF0 0
#define RXF1 1
#define RXF2 2
#define RXF3 3
#define RXF4 4
#define RXF5 5

#define TXB0 0
#define TXB1 1
#define TXB2 2


#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02
#define CANINTF_TX0IF 0x04
#define CANINTF_TX1IF 0x08
#define CANINTF_TX2IF 0x10
#define CANINTF_ERRIF 0x20
#define CANINTF_WAKIF 0x40
#define CANINTF_MERRF 0x80


#define TXB_ABTF 0x40
#define TXB_MLOA 0x20
#define TXB_TXERR 0x10
#define TXB_TXREQ 0x08
#define TXB_TXIE 0x04
#define TXB_TXP 0x03

#define RXBSIDL_IDE 3

static const uint8_t DLC_MASK       = 0x0F;

// MCP2515 Initializition
void mcp2515_initialize_module(const uint8_t node_address, const uint8_t baudrate);
void mcp2515_set_bitrate_8mhz(uint8_t choice);
void mcp2515_set_operation_mode ( uint8_t operation_mode );

// MCP2515 Register manipulation
uint8_t mcp2515_read_status ( void );
uint8_t mcp2515_read_rx_status ( void );
void mcp2515_write_register ( const uint8_t reg, const uint8_t value );
void mcp2515_write_registers ( const uint8_t reg, const uint8_t values[], const uint8_t n );
void mcp2515_bitmodify_register ( const uint8_t reg, const uint8_t mask, const uint8_t data );
uint8_t mcp2515_read_register ( const uint8_t reg );
void mcp2515_read_registers ( const uint8_t reg, uint8_t values[], const uint8_t n );

// CAN functions
uint8_t mcp2515_set_acceptance_filter(const uint8_t num, const bool ext, const uint32_t acceptance_filter);
uint8_t mcp2515_set_acceptance_mask(const uint8_t mask, const bool ext, const uint32_t acceptance_mask);
uint8_t mcp2515_send_message(const struct can_frame *frame);
uint8_t mcp2515_read_message(struct can_frame* frame);



#endif
