#ifndef __MCP2515_H__
#define __MCP2515_H__

#include <stdlib.h>
#include "hardware/spi.h"
#include "spi.h"

/** MCP2515 SPI connected CAN controller definitions */

#define MCP2515_BFPCTRL                     0x0C
#define MCP2515_B1BFS                       ( 1 << 5 )
#define MCP2515_B0BFS                       ( 1 << 4 )
#define MCP2515_B1BFE                       ( 1 << 3 )
#define MCP2515_B0BFE                       ( 1 << 2 )
#define MCP2515_B1BFM                       ( 1 << 1 )
#define MCP2515_B0BFM                       ( 1 << 0 )

/** The Tranmission Error Count Register */
#define MCP2515_TEC                         0x1C

/** The Receive Error Count Register */
#define MCP2515_REC                         0x1D

/** Configuration Register 3 */
#define MCP2515_CNF3                        0x28

/** Configuration Register 2 */
#define MCP2515_CNF2                        0x29

/** Configuration Register 1 */
#define MCP2515_CNF1                        0x2A

/** Interrupt Enable Register */
#define MCP2515_CANINTE                     0x2B
#define MCP2515_CANINTE_MERRE               ( 1 << 7 )
#define MCP2515_CANINTE_WAKIE               ( 1 << 6 )
#define MCP2515_CANINTE_ERRIE               ( 1 << 5 )
#define MCP2515_CANINTE_TX2IE               ( 1 << 4 )
#define MCP2515_CANINTE_TX1IE               ( 1 << 3 )
#define MCP2515_CANINTE_TX0IE               ( 1 << 2 )
#define MCP2515_CANINTE_RX1IE               ( 1 << 1 )
#define MCP2515_CANINTE_RX0IE               ( 1 << 0 )

/** Interrupt Flag Register */
#define MCP2515_CANINTF                     0x2C
#define MCP2515_CANINTF_MERRF               ( 1 << 7 )
#define MCP2515_CANINTF_WAKIF               ( 1 << 6 )
#define MCP2515_CANINTF_ERRIF               ( 1 << 5 )
#define MCP2515_CANINTF_TX2IF               ( 1 << 4 )
#define MCP2515_CANINTF_TX1IF               ( 1 << 3 )
#define MCP2515_CANINTF_TX0IF               ( 1 << 2 )
#define MCP2515_CANINTF_RX1IF               ( 1 << 1 )
#define MCP2515_CANINTF_RX0IF               ( 1 << 0 )

/** Error Flag Register */
#define MCP2515_EFLG                        0x2D
#define MCP2515_EFLG_RX1OVR                 ( 1 << 7 )
#define MCP2515_EFLG_RX0OVR                 ( 1 << 6 )
#define MCP2515_EFLG_TXBO                   ( 1 << 5 )
#define MCP2515_EFLG_TXEP                   ( 1 << 4 )
#define MCP2515_EFLG_RXEP                   ( 1 << 3 )
#define MCP2515_EFLG_TXWAR                  ( 1 << 2 )
#define MCP2515_EFLG_RXWAR                  ( 1 << 1 )
#define MCP2515_EFLG_EWARN                  ( 1 << 0 )


/** TXBnCTRL: TRANSMIT BUFFER n CONTROL REGISTER */
#define MCP2515_TXB0CTRL                    0x30
#define MCP2515_TXB1CTRL                    0x40
#define MCP2515_TXB2CTRL                    0x50

#define MCP2515_TXBnCTRL_ABTF               ( 1 << 6 )
#define MCP2515_TXBnCTRL_MLOA               ( 1 << 5 )
#define MCP2515_TXBnCTRL_TXERR              ( 1 << 4 )
#define MCP2515_TXBnCTRL_TXREQ              ( 1 << 3 )
#define MCP2515_TXBnCTRL_TXP1               ( 1 << 1 )
#define MCP2515_TXBnCTRL_TXP0               ( 1 << 0 )

#define MCP2515_TXBnCTRL_TXP                ( 3 << 0 )

#define MCP2515_TRANSMIT_PRIORITY_HIGHEST   3
#define MCP2515_TRANSMIT_PRIORITY_HIGH      2
#define MCP2515_TRANSMIT_PRIORITY_LOW       1
#define MCP2515_TRANSMIT_PRIORITY_LOWEST    0

/** TXRTSCTRL: TXnRTS PIN CONTROL AND STATUS REGISTER */
#define MCP2515_TXRTSCTRL                   0x0D

#define MCP2515_TXRTSCTRL_B2RTS             ( 1 << 5 )
#define MCP2515_TXRTSCTRL_B1RTS             ( 1 << 4 )
#define MCP2515_TXRTSCTRL_B0RTS             ( 1 << 3 )
#define MCP2515_TXRTSCTRL_B2RTSM            ( 1 << 2 )
#define MCP2515_TXRTSCTRL_B1RTSM            ( 1 << 1 )
#define MCP2515_TXRTSCTRL_B0RTSM            ( 1 << 0 )

/** TXBnSIDH: TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER HIGH */
#define MCP2515_TXB0SIDH                    0x31
#define MCP2515_TXB1SIDH                    0x41
#define MCP2515_TXB2SIDH                    0x51

#define MCP2515_TXB0SIDL                    0x32
#define MCP2515_TXB1SIDL                    0x42
#define MCP2515_TXB2SIDL                    0x52

#define MCP2515_TXBSIDL_MASK                ( 0xE << 5 )
#define MCP2515_TXBSIDL_BIT                 5

/** Extended Identifier Enable bit */
#define MCP2515_TXBSIDL_EXIDE               ( 1 << 3 )

#define MCP2515_TXBSIDL_EID17               ( 1 << 1 )
#define MCP2515_TXBSIDL_EID16               ( 1 << 0 )
#define MCP2515_TXBSIDL_EID                 ( 3 << 0 )

/** TXBnEID8: TRANSMIT BUFFER n EXTENDED IDENTIFIER 8 REGISTER HIGH */
#define MCP2515_TXB0EID8                    0x33
#define MCP2515_TXB1EID8                    0x43
#define MCP2515_TXB2EID8                    0x53

/** TXBnEID0: TRANSMIT BUFFER n EXTENDED IDENTIFIER 0 REGISTER LOW */
#define MCP2515_TXB0EID0                    0x34
#define MCP2515_TXB1EID0                    0x44
#define MCP2515_TXB2EID0                    0x54

/** TXBnDLC: TRANSMIT BUFFER n DATA LENGTH CODE REGISTER */
#define MCP2515_TXB0DLC                     0x35
#define MCP2515_TXB1DLC                     0x45
#define MCP2515_TXB2DLC                     0x55

#define MCP2515_TXBDLC_RTR                  ( 1 << 6 )
#define MCP2515_TXBDLC_DLC                  ( 0xF << 0 )
#define MCP2515_TXBDLC_DLC_MASK             ( 0xF << 0 )
#define MCP2515_TXBDLC_DLC_BIT              ( 0xF << 0 )

/** TXBnDm: TRANSMIT BUFFER n DATA BYTE m REGISTER */
#define MCP2515_TXB0D0                      0x36
#define MCP2515_TXB0D1                      0x37
#define MCP2515_TXB0D2                      0x38
#define MCP2515_TXB0D3                      0x39
#define MCP2515_TXB0D4                      0x3A
#define MCP2515_TXB0D5                      0x3B
#define MCP2515_TXB0D6                      0x3C
#define MCP2515_TXB0D7                      0x3D

#define MCP2515_TXB1D0                      0x46
#define MCP2515_TXB1D1                      0x47
#define MCP2515_TXB1D2                      0x48
#define MCP2515_TXB1D3                      0x49
#define MCP2515_TXB1D4                      0x4A
#define MCP2515_TXB1D5                      0x4B
#define MCP2515_TXB1D6                      0x4C
#define MCP2515_TXB1D7                      0x4D

#define MCP2515_TXB2D0                      0x56
#define MCP2515_TXB2D1                      0x57
#define MCP2515_TXB2D2                      0x58
#define MCP2515_TXB2D3                      0x59
#define MCP2515_TXB2D4                      0x5A
#define MCP2515_TXB2D5                      0x5B
#define MCP2515_TXB2D6                      0x5C
#define MCP2515_TXB2D7                      0x5D

/** RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER (ADDRESS: 60h) */
#define MCP2515_RXB0CTRL                    0x60
#define MCP2515_RXB0CTRL_RXM_BIT            ( 5 )
#define MCP2515_RXB0CTRL_RXM_MASK           ( 0x3 << MCP2515_RXB0CTRL_RXM_BIT )
#define MCP2515_RXB0CTRL_RXM_SET(reg, val)  ( ( reg ^ MCP2515_RXB0CTRL_RXM_MASK ) | ( val << MCP2515_RXB0CTRL_RXM_BIT ) )
#define MCP2515_RXB0CTRL_RXM_GET(x)         ( ( ( x ) >> MCP2515_RXB0CTRL_RXM_BIT ) & MCP2515_RXB0CTRL_RXM_MASK )
#define MCP2515_RXB0CTRL_RXRTR              ( 1 << 3 )
#define MCP2515_RXB0CTRL_BUKT               ( 1 << 2 )
#define MCP2515_RXB0CTRL_BUKT1_RO           ( 1 << 1 )
#define MCP2515_RXB0CTRL_FILHIT0            ( 1 << 0 )

#define MCP2515_RXB0SIDL                    0x62
#define MCP2515_RXB1SIDL                    0x72
#define MCP2515_RXBnSIDL_SID0_2_BIT         ( 4 )
#define MCP2515_RXBnSIDL_SID0_2_MASK        ( 0x7 << MCP2515_RXBnSIDL_SID0_2_BIT )
#define MCP2515_RXBnSIDL_SID0_2_SET(x, val) ( ( x & ~MCP2515_RXBnSIDL_SID0_2_MASK ) | ( val << MCP2515_RXBnSIDL_SID0_2_BIT ) )
#define MCP2515_RXBnSIDL_SID0_2_VALUE(x)    ( ( x & MCP2515_RXBnSIDL_SID0_2_MASK ) >> MCP2515_RXBnSIDL_SID0_2_BIT )
#define MCP2515_RXBnSIDL_SRR                ( 1 << 4 )
#define MCP2515_RXBnSIDL_IDE                ( 1 << 3 )
#define MCP2515_RXBnSIDL_EID17_16_BIT         ( 0 )
#define MCP2515_RXBnSIDL_EID17_16_MASK        ( 0x3 << MCP2515_RXBnSIDL_EID17_16_BIT )
#define MCP2515_RXBnSIDL_EID17_16_SET(x, val) ( ( x & ~MCP2515_RXBnSIDL_EID17_16_MASK ) | ( val << MCP2515_RXBnSIDL_EID17_16_BIT ) )
#define MCP2515_RXBnSIDL_EID17_16_VALUE(x)    ( ( x & MCP2515_RXBnSIDL_EID17_16_MASK ) >> MCP2515_RXBnSIDL_EID17_16_BIT )

#define MCP2515_RXB0DLC                     0x65
#define MCP2515_RXB1DLC                     0x75
#define MCP2515_RXBnDLC_DLC3_0_BIT          ( 0 )
#define MCP2515_RXBnDLC_DLC3_0_MASK        ( 0x3 << MCP2515_RXBnDLC_DLC3_0_BIT )
#define MCP2515_RXBnDLC_DLC3_0_SET(x, val) ( ( x & ~MCP2515_RXBnDLC_DLC3_0_MASK ) | ( val << MCP2515_RXBnDLC_DLC3_0_BIT ) )
#define MCP2515_RXBnDLC_DLC3_0_VALUE(x)    ( ( x & MCP2515_RXBnDLC_DLC3_0_MASK ) >> MCP2515_RXBnDLC_DLC3_0_BIT )


/**  RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER (ADDRESS: 70h)*/
#define MCP2515_RXB1CTRL                    0x70
#define MCP2515_RXB1CTRL_RXM_BIT            ( 5 )
#define MCP2515_RXB1CTRL_RXM_MASK           ( 0x3 << MCP2515_RXB1CTRL_RXM_BIT )
#define MCP2515_RXB1CTRL_RXM_SET(x)         ( ( ( x ) & MCP2515_RXB1CTRL_RXM_MASK ) << MCP2515_RXB1CTRL_RXM_BIT )
#define MCP2515_RXB1CTRL_RXM_GET(x)         ( ( ( x ) >> MCP2515_RXB1CTRL_RXM_BIT ) & MCP2515_RXB1CTRL_RXM_MASK )
#define MCP2515_RXB1CTRL_RXRTR              ( 1 << 3 )
#define MCP2515_FILHIT_MASK                 ( 0x7 )
#define MCP2515_FILHIT_BIT                  ( 0 )
#define MCP2515_FILHIT_SET(x)               ( ( ( x ) & MCP2515_FILHIT_MASK ) << MCP2515_FILHIT_BIT )
#define MCP2515_FILHIT_GET(x)               ( ( ( x ) >> MCP2515_FILHIT_BIT ) & MCP2515_FILHIT_MASK )

#define MCP2515_INST_RESET                  ( 0xC0 )
#define MCP2515_INST_READ                   ( 0x03 )
#define MCP2515_INST_READ_RX_BUFFER         ( 0x90 ) // TODO: Fix this incomplete value
#define MCP2515_INST_WRITE                  ( 0x02 )
#define MCP2515_INST_LOAD_TX_BUFFER         ( 0x40 ) // TODO: Fix this incomplete value
#define MCP2515_INST_RTS                    ( 0x80 ) // TODO: Fix this incomplete value
#define MCP2515_INST_READ_STATUS            ( 0xA0 )
#define MCP2515_INST_RX_STATUS              ( 0xB0 )
#define MCP2515_INST_BIT_MODIFY             ( 0x05 )

#define MCP2515_CANCTRL                     ( 0xF )
#define MCP2515_CANCTRL_REQOP_BIT           (5)
#define MCP2515_CANCTRL_REQOP_MASK          ( 0x7 << MCP2515_CANCTRL_REQOP_BIT )
#define MCP2515_CANCTRL_REQOP_SET(x, val)   ( ( x & ~MCP2515_CANCTRL_REQOP_MASK ) | ( val << MCP2515_CANCTRL_REQOP_BIT ) )
#define MCP2515_CANCTRL_REQOP_VALUE(x)      ( ( x & MCP2515_CANCTRL_REQOP_MASK ) >> MCP2515_CANCTRL_REQOP_BIT )

#define MCP2515_CANSTAT                     ( 0xE )

typedef enum {
    MCP2515_CANCTRL_MODE_NORMAL = 0x0,
    MCP2515_CANCTRL_MODE_SLEEP,
    MCP2515_CANCTRL_MODE_LOOPBACK,
    MCP2515_CANCTRL_MODE_LISTEN_ONLY,
    MCP2515_CANCTRL_MODE_CONFIGURATION
} mcp2515_canctrl_reqop_t;

extern uint8_t MCP2515_read_reg(
        spidrv_t* spi,
        uint8_t reg );

extern mcp2515_canctrl_reqop_t MCP2515_get_mode(
        spidrv_t* spi );

extern void MCP2515_reset(
        spidrv_t* spi );

extern void MCP2515_write_reg(
        spidrv_t* spi,
        const uint8_t reg,
        uint8_t value );

extern void MCP2515_set_config(
        spidrv_t* spi,
        const uint8_t cnf1,
        const uint8_t cnf2,
        const uint8_t cnf3 );

extern void MCP2515_set_mode(
        spidrv_t* spi,
        const mcp2515_canctrl_reqop_t mode );

extern void MCP2515_read_rx_buffer(
        spidrv_t* spi,
        const uint8_t id,
        uint8_t* buffer );

extern void MCP2515_bit_modify(
        spidrv_t* spi,
        const uint8_t reg,
        const uint8_t mask,
        const uint8_t value );

extern void MCP2515_write_tx_buffer(
        spidrv_t* spi,
        uint16_t id,
        const uint8_t* data,
        const uint8_t nbytes
    );

#endif
