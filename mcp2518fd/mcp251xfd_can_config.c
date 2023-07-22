#include "mcp251xfd_can_config.h"

MCP251XFD_BitTimeStats  MCP2518FD_Ext1_BTStats;
uint32_t                SYSCLK_Ext1;

MCP251XFD_Config MCP2518FD_Ext1_Config =
{
    //--- Controller clocks ---
    .XtalFreq       = 0, 		// CLKIN is not a crystal
    .OscFreq        = 4000000, 	// CLKIN is a 4MHz oscillator
    .SysclkConfig   = MCP251XFD_SYSCLK_IS_CLKIN_MUL_10,
    .ClkoPinConfig  = MCP251XFD_CLKO_SOF,
    .SYSCLK_Result  = &SYSCLK_Ext1,
    //--- CAN configuration ---
    .NominalBitrate = 1000000, // Nominal Bitrate to 1Mbps
    .DataBitrate    = 4000000, // Data Bitrate to 4Mbps
    .BitTimeStats   = &MCP2518FD_Ext1_BTStats,
    .Bandwidth      = MCP251XFD_NO_DELAY,
    .ControlFlags   = MCP251XFD_CAN_RESTRICTED_MODE_ON_ERROR
                    | MCP251XFD_CAN_ESI_REFLECTS_ERROR_STATUS
                    | MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS
                    | MCP251XFD_CANFD_BITRATE_SWITCHING_ENABLE
                    | MCP251XFD_CAN_PROTOCOL_EXCEPT_AS_FORM_ERROR
                    | MCP251XFD_CANFD_USE_ISO_CRC
                    | MCP251XFD_CANFD_DONT_USE_RRS_BIT_AS_SID11,
    //--- GPIOs and Interrupts pins ---
    .GPIO0PinMode   = MCP251XFD_PIN_AS_INT0_TX,
    .GPIO1PinMode   = MCP251XFD_PIN_AS_INT1_RX,
    .INTsOutMode    = MCP251XFD_PINS_PUSHPULL_OUT,
    .TXCANOutMode   = MCP251XFD_PINS_OPENDRAIN_OUT,
    //--- Interrupts ---
    .SysInterruptFlags = MCP251XFD_INT_ENABLE_ALL_EVENTS,
};
