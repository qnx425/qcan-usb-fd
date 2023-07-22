#ifndef DRIVER_CONFIG_H_INC
#define DRIVER_CONFIG_H_INC

#include "MCP251XFD.h"

#define SPI_CS_EXT1 ( 0 )

extern MCP251XFD MCP251XFD_Ext1;
#define CANEXT1 &MCP251XFD_Ext1

#endif
