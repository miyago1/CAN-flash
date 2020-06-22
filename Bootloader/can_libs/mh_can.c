/**
 ***********************************************************************************************
 @brief MCP2515 CAN module library
 @details MCP2515 API for mainpulating registers and sending/receiving can messages.
 @file mh_can.c
 @author Magnus Roos and Henrik Sellén Wikström
 @version 1.0
 @date 29-May-2020
 ***********************************************************************************************
 */

#include <util/delay.h> // Delay Functions
#include <string.h>

#include "mh_can.h"
#include "mh_j1939.h"
#include "flash_utils.h"


void mcp2515_write_register ( const uint8_t reg, const uint8_t value )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_WRITE );
    spi_master_transmit ( reg );
    spi_master_transmit ( value );

    PORTD |= 1 << PIN7;  // SS high

}

void mcp2515_write_registers ( const uint8_t reg, const uint8_t values[], const uint8_t n )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_WRITE );
    spi_master_transmit ( reg );
    for ( uint8_t i = 0; i < n; i++ ) {
        spi_master_transmit ( values[i] );
    }

    PORTD |= 1 << PIN7;  // SS high
}

void mcp2515_bitmodify_register ( const uint8_t reg, const uint8_t mask, const uint8_t data )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_BITMOD );
    spi_master_transmit ( reg );
    spi_master_transmit ( mask );
    spi_master_transmit ( data );

    PORTD |= 1 << PIN7;  // SS high
}


uint8_t mcp2515_read_register ( const uint8_t reg )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_READ );
    spi_master_transmit ( reg );
    uint8_t ret = spi_master_transmit ( 0x00 );

    PORTD |= 1 << PIN7;  // SS high

    return ret;
}

void mcp2515_read_registers ( const uint8_t reg, uint8_t values[], const uint8_t n )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit(INSTRUCTION_READ);
    spi_master_transmit(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i=0; i<n; i++) {
        values[i] = spi_master_transmit(0x00);
    }

    PORTD |= 1 << PIN7;  // SS high
}

static void mcp2515_load_id_in_buffer ( uint8_t *buffer, const bool ext, const uint32_t id )
{
    uint16_t canid = ( uint16_t ) ( id & 0x0FFFF );

    if ( ext ) {
        buffer[MCP_EID0] = ( uint8_t ) ( canid & 0xFF );
        buffer[MCP_EID8] = ( uint8_t ) ( canid >> 8 );
        canid = ( uint16_t ) ( id >> 16 );
        buffer[MCP_SIDL] = ( uint8_t ) ( canid & 0x03 );
        buffer[MCP_SIDL] += ( uint8_t ) ( ( canid & 0x1C ) << 3 );
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = ( uint8_t ) ( canid >> 5 );
    } else {
        buffer[MCP_SIDH] = ( uint8_t ) ( canid >> 3 );
        buffer[MCP_SIDL] = ( uint8_t ) ( ( canid & 0x07 ) << 5 );
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

uint8_t mcp2515_read_status ( void )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_READ_STATUS );
    uint8_t i = spi_master_transmit ( 0x00 );

    PORTD |= 1 << PIN7;  // SS high

    return i;
}

uint8_t mcp2515_read_rx_status ( void )
{
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_RX_STATUS );
    uint8_t i = spi_master_transmit ( 0x00 );

    PORTD |= 1 << PIN7;  // SS high

    return i;
}

void mcp2515_initialize_module(const uint8_t node_address, const uint8_t baudrate)
{

    /* NANO    MCP2515
        D7      CS
        D8      INT
        D11     MOSI
        D12     MISO
        D13     SCK   */

    spi_init();

    // Reset function
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit ( INSTRUCTION_RESET );

    PORTD |= 1 << PIN7;  // SS high

    _delay_ms ( 5 );


    // Clearing transmitbuffers
    uint8_t zeros[14] = { 0 };
    mcp2515_write_registers( MCP_TXB0CTRL, zeros, 14 );
    mcp2515_write_registers( MCP_TXB1CTRL, zeros, 14 );
    mcp2515_write_registers( MCP_TXB2CTRL, zeros, 14 );

    mcp2515_write_register( MCP_RXB0CTRL, 0x60 );   // Turn off filter and masks for RXB0
    mcp2515_write_register( MCP_RXB1CTRL, 0 );

    mcp2515_write_register( MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF );

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1

    mcp2515_bitmodify_register( MCP_RXB1CTRL,
                                RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                                RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT );


    mcp2515_set_acceptance_filter( RXF1, true, 0 );    // Not used

    // Filter and mask for RXB1
    mcp2515_set_acceptance_mask( MASK1, false, MASK_SFF_MCU );      // Filter all standard frames
    mcp2515_set_acceptance_filter( RXF2, false, FILTER_SFF_MCU );   // Filter all standard frames

    mcp2515_set_acceptance_filter( RXF3, false, 0 );       // Not used
    mcp2515_set_acceptance_filter( RXF4, false, 0 );       // Not used
    mcp2515_set_acceptance_filter( RXF5, false, 0 );       // Not used
    
    mcp2515_set_bitrate_8mhz(baudrate);    // 2 = 20, 3 = 125,4 = 200, 5=250, 6 = 500, 7 = 1k
    mcp2515_set_operation_mode( CANCTRL_REQOP_NORMAL );

}


uint8_t mcp2515_set_acceptance_mask ( const uint8_t mask, const bool ext, const uint32_t acceptance_mask )
{
    // setMode function for config mode
    mcp2515_set_operation_mode (CANCTRL_REQOP_CONFIG);

    uint8_t id_buffer[4];
    mcp2515_load_id_in_buffer ( id_buffer, ext, acceptance_mask );

    uint8_t reg;
    switch ( mask ) {
    case MASK0:
        reg = MCP_RXM0SIDH;
        break;
    case MASK1:
        reg = MCP_RXM1SIDH;
        break;
    default:
        return ERROR_FAIL;
    }

    mcp2515_write_registers ( reg, id_buffer, 4 );

    return ERROR_OK;
}

uint8_t mcp2515_set_acceptance_filter ( const uint8_t num, const bool ext, const uint32_t acceptance_filter )
{
    // setMode function for config mode
    mcp2515_set_operation_mode (CANCTRL_REQOP_CONFIG);

    uint8_t reg;

    switch ( num ) {
    case RXF0:
        reg = MCP_RXF0SIDH;
        break;
    case RXF1:
        reg = MCP_RXF1SIDH;
        break;
    case RXF2:
        reg = MCP_RXF2SIDH;
        break;
    case RXF3:
        reg = MCP_RXF3SIDH;
        break;
    case RXF4:
        reg = MCP_RXF4SIDH;
        break;
    case RXF5:
        reg = MCP_RXF5SIDH;
        break;
    default:
        return ERROR_FAIL;
    }

    uint8_t id_buffer[4];
    mcp2515_load_id_in_buffer ( id_buffer, ext, acceptance_filter );
    mcp2515_write_registers ( reg, id_buffer, 4 );

    return ERROR_OK;
}




void mcp2515_set_bitrate_8mhz(uint8_t choice)
{
    uint8_t cfg1 = 0, cfg2 = 0, cfg3 = 0;

    switch(choice) {
    case 0:
        cfg1 = MCP_8MHz_5kBPS_CFG1;
        cfg2 = MCP_8MHz_5kBPS_CFG2;
        cfg3 = MCP_8MHz_5kBPS_CFG3;
        break;
    case 1:
        cfg1 = MCP_8MHz_10kBPS_CFG1;
        cfg2 = MCP_8MHz_10kBPS_CFG2;
        cfg3 = MCP_8MHz_10kBPS_CFG3;
        break;
    case 2:
        cfg1 = MCP_8MHz_20kBPS_CFG1;
        cfg2 = MCP_8MHz_20kBPS_CFG2;
        cfg3 = MCP_8MHz_20kBPS_CFG3;
        break;
    case 3:
        cfg1 = MCP_8MHz_125kBPS_CFG1;
        cfg2 = MCP_8MHz_125kBPS_CFG2;
        cfg3 = MCP_8MHz_125kBPS_CFG3;
        break;
    case 4:
        cfg1 = MCP_8MHz_200kBPS_CFG1;
        cfg2 = MCP_8MHz_200kBPS_CFG2;
        cfg3 = MCP_8MHz_200kBPS_CFG3;
        break;
    case 5:
        cfg1 = MCP_8MHz_250kBPS_CFG1;
        cfg2 = MCP_8MHz_250kBPS_CFG2;
        cfg3 = MCP_8MHz_250kBPS_CFG3;
        break;
    case 6:
        cfg1 = MCP_8MHz_500kBPS_CFG1;
        cfg2 = MCP_8MHz_500kBPS_CFG2;
        cfg3 = MCP_8MHz_500kBPS_CFG3;
        break;
    case 7:
        cfg1 = MCP_8MHz_1000kBPS_CFG1;
        cfg2 = MCP_8MHz_1000kBPS_CFG2;
        cfg3 = MCP_8MHz_1000kBPS_CFG3;
        break;
    default:
        break;
    }
    mcp2515_write_register ( MCP_CNF1, cfg1 );
    mcp2515_write_register ( MCP_CNF2, cfg2 );
    mcp2515_write_register ( MCP_CNF3, cfg3 );

}
void mcp2515_set_operation_mode ( uint8_t operation_mode)
{
    mcp2515_bitmodify_register ( MCP_CANCTRL, CANCTRL_REQOP, operation_mode );
    uint8_t counter = 0;
    bool modeMatch = false;
    while ( counter < 10 ) {
        uint8_t newmode = mcp2515_read_register ( MCP_CANSTAT );
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == CANCTRL_REQOP_NORMAL;

        if ( modeMatch ) {
            break;
        }
        _delay_ms ( 1 );
        counter++;
    }
}

uint8_t mcp2515_send_message ( const struct can_frame* frame )
{
    uint8_t ctrl, sidh, status, address;
    uint8_t data[13];

    // Check if nay transmitbuffers are available
    status = mcp2515_read_status();
    if ( !CHECK_BIT(status,2) ) {
        address = 0x81;
        ctrl = MCP_TXB0CTRL;
        sidh = MCP_TXB0SIDH;
    } else if ( !CHECK_BIT(status,4) ) {
        address = 0x82;
        ctrl = MCP_TXB1CTRL;
        sidh = MCP_TXB1SIDH;
    } else if ( !CHECK_BIT(status,6) ) {
        address = 0x84;
        ctrl = MCP_TXB2CTRL;
        sidh = MCP_TXB2SIDH;
    } else {
        return 0;
    }

    bool ext = ( frame->can_id & CAN_EFF_FLAG );
    bool rtr = ( frame->can_id & CAN_RTR_FLAG );
    uint32_t id = ( frame->can_id & ( ext ? CAN_EFF_MASK : CAN_SFF_MASK ) );

    mcp2515_load_id_in_buffer ( data, ext, id );

    data[MCP_DLC] = rtr ? ( frame->can_dlc | RTR_MASK ) : frame->can_dlc;

    memcpy ( &data[MCP_DATA], frame->data, frame->can_dlc );

    mcp2515_write_registers ( sidh, data, 5 + frame->can_dlc );

    mcp2515_bitmodify_register ( ctrl, TXB_TXREQ, TXB_TXREQ );

    // Message loaded, transmitting

    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit(address);

    PORTD |= 1 << PIN7;  // Slave select to HIGH



    status = mcp2515_read_register ( ctrl );
    if ( ( status & ( TXB_ABTF | TXB_MLOA | TXB_TXERR ) ) != 0 ) {
        return ERROR_FAILTX;
    }
    return ERROR_OK;


}

uint8_t mcp2515_read_message(struct can_frame *frame)
{

    uint8_t status = mcp2515_read_status();
    uint8_t sidh, data, ctrl, rx_if;

    if ( status & CANINTF_RX0IF) {
        ctrl = MCP_RXB0CTRL;
        sidh = MCP_RXB0SIDH;
        data = MCP_RXB0DATA;
        rx_if = CANINTF_RX0IF;

    } else if ( status & CANINTF_RX1IF ) {
        ctrl = MCP_RXB1CTRL;
        sidh = MCP_RXB1SIDH;
        data = MCP_RXB1DATA;
        rx_if = CANINTF_RX1IF;
    }
    else
    {
        return ERROR_NOMSG;
    }

    uint8_t reg_buffer[5];

    mcp2515_read_registers(sidh, reg_buffer, 5);

    uint32_t id = (reg_buffer[MCP_SIDH]<<3) | (reg_buffer[MCP_SIDL]>>5);

    if(CHECK_BIT(reg_buffer[MCP_SIDL],RXBSIDL_IDE)) {
        id = (id<<2) + (reg_buffer[MCP_SIDL] & 0x03);
        id = (id<<8) + reg_buffer[MCP_EID8];
        id = (id<<8) + reg_buffer[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (reg_buffer[MCP_DLC] & DLC_MASK);

    if(mcp2515_read_register(ctrl) == RXBnCTRL_RTR){
        id |= CAN_RTR_FLAG;
        }
    
    frame->can_id = id;
    frame->can_dlc = dlc;

    mcp2515_read_registers(data, frame->data, dlc);

    mcp2515_bitmodify_register(MCP_CANINTF, rx_if, 0);

    return ERROR_OK;


}
