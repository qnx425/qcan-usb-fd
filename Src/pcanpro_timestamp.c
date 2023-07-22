#include <assert.h>
#include <stm32f1xx_hal.h>
#include "pcanpro_timestamp.h"

#include "mcp251xfd_driver_config.h"

void pcan_timestamp_init( void )
{
  //HAL_GetTick must not return 0 to use LED events in early stage.
  //Let's increment the value here by one.
  HAL_IncTick();
}

uint32_t pcan_timestamp_millis( void )
{
  return HAL_GetTick();
}

uint32_t pcan_timestamp_us( void )
{
//  uint32_t timestamp_us = ( TIM3->CNT << 16 ) | TIM2->CNT;

  uint32_t timestamp_us = 0;
  MCP251XFD_GetTimeStamp( CANEXT1, &timestamp_us);

  return timestamp_us;
}
