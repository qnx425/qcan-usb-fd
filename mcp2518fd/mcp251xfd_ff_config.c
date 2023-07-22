#include "mcp251xfd_ff_config.h"

MCP251XFD_RAMInfos Ext1_TEF_RAMInfos;
//MCP251XFD_RAMInfos Ext1_TXQ_RAMInfos;
MCP251XFD_RAMInfos Ext1_FIFOs_RAMInfos[MCP2518FD_EXT1_FIFO_COUNT - 1];
/*
 * MCP2518FD_EXT1_FIFO_COUNT - 1 -> потому что есть только TEF, но нет TXQ
 */

MCP251XFD_FIFO MCP2518FD_Ext1_FIFOlist[MCP2518FD_EXT1_FIFO_COUNT] =
{
	{
			.Name 			= MCP251XFD_TEF,
			.Size 			= MCP251XFD_FIFO_10_MESSAGE_DEEP,
			.ControlFlags 	= MCP251XFD_FIFO_ADD_TIMESTAMP_ON_OBJ,
			.InterruptFlags = MCP251XFD_FIFO_NO_INTERRUPT_FLAGS,
			.RAMInfos 		= &Ext1_TEF_RAMInfos,
	},

	{
			.Name 			= TXFIFO,
			.Size 			= MCP251XFD_FIFO_12_MESSAGE_DEEP,
			.Payload 		= MCP251XFD_PAYLOAD_64BYTE,
			.Direction 		= MCP251XFD_TRANSMIT_FIFO,
			.Attempts 		= MCP251XFD_THREE_ATTEMPTS,
			.Priority 		= MCP251XFD_MESSAGE_TX_PRIORITY16,
			.ControlFlags	= MCP251XFD_FIFO_NO_RTR_RESPONSE,
			.InterruptFlags = MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
			.RAMInfos 		= &Ext1_FIFOs_RAMInfos[0],
	},

	{
			.Name 			= RXFIFO,
			.Size 			= MCP251XFD_FIFO_14_MESSAGE_DEEP,
			.Payload 		= MCP251XFD_PAYLOAD_64BYTE,
			.Direction 		= MCP251XFD_RECEIVE_FIFO,
			.ControlFlags 	= MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
			.InterruptFlags = MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
			.RAMInfos 		= &Ext1_FIFOs_RAMInfos[1],
	},
};

// 10 * 12 + 12 * 72 + 14 * 76 = 2048

MCP251XFD_Filter MCP2518FD_Ext1_FilterList[MCP2518FD_EXT1_FILTER_COUNT] =
{
	{
			.Filter 		= MCP251XFD_FILTER0,
			.EnableFilter 	= true,
			.Match 			= MCP251XFD_MATCH_SID_EID,
			.AcceptanceID 	= MCP251XFD_ACCEPT_ALL_MESSAGES,
			.AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES,
			.PointTo 		= RXFIFO,
	},
};
