#include "mcp251xfd_driver_config.h"
#include "mcp251xfd_driver_interface.h"
#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi1;

MCP251XFD_BitTimeConfig	MCP2518FD_Ext1_BTConfig = { 0 };

MCP251XFD MCP251XFD_Ext1 =
{
    .UserDriverData 	= &MCP2518FD_Ext1_BTConfig,
    //--- Driver configuration ---
    .DriverConfig       = MCP251XFD_DRIVER_USE_READ_WRITE_CRC
                        | MCP251XFD_DRIVER_USE_SAFE_WRITE
                        | MCP251XFD_DRIVER_ENABLE_ECC
                        | MCP251XFD_DRIVER_INIT_SET_RAM_AT_0
						| MCP251XFD_DRIVER_INIT_CHECK_RAM
						| MCP251XFD_DRIVER_SAFE_RESET,
    //--- IO configuration ---
    .GPIOsOutState      = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,
    //--- Interface driver call functions ---
    .SPI_ChipSelect     = SPI_CS_EXT1, 	// Here the chip select of the EXT1 interface is 1
    .InterfaceDevice    = &hspi1, 		// Here this point to the address memory of the peripheral SPI0
    .fnSPI_Init         = MCP251XFD_InterfaceInit,
    .fnSPI_Transfer     = MCP251XFD_InterfaceTransfer,
    //--- Time call function ---
    .fnGetCurrentms     = GetCurrentms,
    //--- CRC16-CMS call function ---
    .fnComputeCRC16     = ComputeCRC16,
    //--- Interface clocks ---
    .SPIClockSpeed      = 12000000,
};
