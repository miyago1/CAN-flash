#include "mh_can.h"
#include "flash_utils.h"
#include <util/delay.h> // Delay Functions
#include <string.h>

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
        cfg1 = MCP_8MHz_125kBPS_CFG1;
        cfg2 = MCP_8MHz_125kBPS_CFG2;
        cfg3 = MCP_8MHz_125kBPS_CFG3;
        break;
    }
    mcp2515_write_register ( MCP_CNF1, cfg1 );
    mcp2515_write_register ( MCP_CNF2, cfg2 );
    mcp2515_write_register ( MCP_CNF3, cfg3 );

}

uint8_t mcp2515_send_message (j1939_pdu_t* pdu)
{
    uint8_t ctrl, sidh, status, address;
    uint8_t data[13];

    // Check if any transmitbuffers are available
    status = mcp2515_read_status();
    if ( !CHECK_BIT(status,2) ) {
        address = INSTRUCTION_RTS_TX0;
        ctrl = MCP_TXB0CTRL;
        sidh = MCP_TXB0SIDH;
    } else if ( !CHECK_BIT(status,4) ) {
        address = INSTRUCTION_RTS_TX1;
        ctrl = MCP_TXB1CTRL;
        sidh = MCP_TXB1SIDH;
    } else if ( !CHECK_BIT(status,6) ) {
        address = INSTRUCTION_RTS_TX2;
        
        ctrl = MCP_TXB2CTRL;
        sidh = MCP_TXB2SIDH;
    } else {
        return 0;
    }
   
    data[MCP_EID0] = pdu->source_address;
    data[MCP_EID8] = pdu->pdu_specific;
    data[MCP_SIDL] = ((pdu->pdu_format & 0x1C) << 3) | SIDL_EID_FLAG | (pdu->pdu_format & 0x03);
    data[MCP_SIDH] = pdu->priority << 5 | pdu->extended_data_page << 4 | pdu->data_page << 3 | (pdu->pdu_format & 0xE0) >> 5;
    
    data[MCP_DLC] = pdu->dlc; // RTR flag finns i samma reg som DLC

    memcpy ( &data[MCP_DATA], pdu->data, pdu->dlc );

    mcp2515_write_registers ( sidh, data, 5 + pdu->dlc );

    // Message loaded, transmitting
    SPCR = SPI_SPCR;
    SPSR = SPI_SPSR;
    PORTD &= ~(1 << PIN7); // SS low

    spi_master_transmit(address);

    PORTD |= 1 << PIN7;  // SS high

    // Check if transmit was succesful
    status = mcp2515_read_register ( ctrl );
    if ( ( status & ( TXB_ABTF | TXB_MLOA | TXB_TXERR ) ) != 0 ) {
        return ERROR_FAILTX;
    }
    return ERROR_OK;

}
uint8_t mcp2515_read_message(j1939_pdu_t* pdu){

    uint8_t status = mcp2515_read_status();
    uint8_t data, ctrl, rx_if;
    uint8_t reg_buffer[5];

    if ( status & CANINTF_RX0IF) {
        ctrl = MCP_RXB0CTRL;
        mcp2515_read_registers(MCP_RXB0SIDH, reg_buffer, 5);
        data = MCP_RXB0DATA;
        rx_if = CANINTF_RX0IF;

    } else if ( status & CANINTF_RX1IF ) {
        ctrl = MCP_RXB1CTRL;
        mcp2515_read_registers(MCP_RXB1SIDH, reg_buffer, 5);
        data = MCP_RXB1DATA;
        rx_if = CANINTF_RX1IF;
    }
    else
    {
        return ERROR_NOMSG;
    }

    if(CHECK_BIT(reg_buffer[MCP_SIDL],RXBSIDL_IDE)) {
        pdu->source_address = reg_buffer[MCP_EID0];
        pdu->pdu_specific = reg_buffer[MCP_EID8];
        pdu->pdu_format = reg_buffer[MCP_SIDL] & 0x03;
        pdu->pdu_format |= (reg_buffer[MCP_SIDL] & 0xE0) >> 3 | (reg_buffer[MCP_SIDH] & 0x07) << 5; 
        pdu->data_page = 0;
        pdu->extended_data_page = 0;
        pdu->priority = (reg_buffer[MCP_SIDH] & 0xE0);        
    } else {
        return ERROR_NOMSG;
    }

    pdu->dlc = (reg_buffer[MCP_DLC] & DLC_MASK);

    if(mcp2515_read_register(ctrl) == RXBnCTRL_RTR){
        pdu->is_rtr_frame = 1;
        }
    
    mcp2515_read_registers(data, pdu->data, pdu->dlc);

    mcp2515_bitmodify_register(MCP_CANINTF, rx_if, 0);

    return ERROR_OK;


}


