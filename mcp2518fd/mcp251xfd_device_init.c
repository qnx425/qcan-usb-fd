#include "mcp251xfd_driver_config.h"
#include "mcp251xfd_can_config.h"
#include "mcp251xfd_ff_config.h"

#include "system_stm32f1xx.h"

//=============================================================================
// Configure the MCP251XFD device on EXT1
//=============================================================================
eERRORRESULT ConfigureMCP251XFDDeviceOnEXT1(void)
{
    //--- Configure module on Ext1 ---
    eERRORRESULT ErrorExt1 = ERR__NO_DEVICE_DETECTED;
    
    ErrorExt1 = Init_MCP251XFD(CANEXT1, &MCP2518FD_Ext1_Config);
    
    if (ErrorExt1 != ERR_OK) return ErrorExt1;

	uint16_t tsPrescaler = (uint16_t)( *MCP2518FD_Ext1_Config.SYSCLK_Result / 1000000 );

    ErrorExt1 = MCP251XFD_ConfigureTimeStamp(CANEXT1, true, MCP251XFD_TS_CAN20_SOF_CANFD_SOF,
    		 tsPrescaler, false);
    if (ErrorExt1 != ERR_OK) return ErrorExt1;

    ErrorExt1 = MCP251XFD_ConfigureFIFOList(CANEXT1, &MCP2518FD_Ext1_FIFOlist[0], MCP2518FD_EXT1_FIFO_COUNT);
    if (ErrorExt1 != ERR_OK) return ErrorExt1;

    ErrorExt1 = MCP251XFD_ConfigureFilterList(CANEXT1, MCP251XFD_D_NET_FILTER_DISABLE,
    		 &MCP2518FD_Ext1_FilterList[0], MCP2518FD_EXT1_FILTER_COUNT);
    if (ErrorExt1 != ERR_OK) return ErrorExt1;

    const bool waitOperationChange = true;
	MCP251XFD_RequestOperationMode( CANEXT1, MCP251XFD_CONFIGURATION_MODE, waitOperationChange );
    
    return ErrorExt1;
}
