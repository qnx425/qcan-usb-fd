#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "usb_device.h"

#include "main.h"

extern void SystemClock_Config(void);

int __io_putchar( int ch ) {
#ifdef DEBUG
	return ITM_SendChar( ch );
#else
	return ch;
#endif
}

/*
 * https://github.com/rogerclarkmelbourne/STM32duino-bootloader
 * On "generic" boards, the USB reset (to force re-enumeration by the host),
 * is triggered by reconfiguring the USB D+ line (PA12) into GPIO mode,
 * and driving PA12 low for a short period,
 * before setting the pin back to its USB operational mode.
 */
static void USBD_ForceDisconnect(void)
{
  RCC->APB2ENR |= 4;          // I/O port A clock enable
  GPIOA->CRH   &= 0xFFF0FFFF; // General purpose output push-pull
  GPIOA->CRH   |= 0x00020000; // Output mode, max speed 2 MHz
  GPIOA->BRR   |= 1uL << 12;  // Reset the corresponding ODRx bit

  HAL_Delay( 3000 );

  GPIOA->CRH   &= 0xFFF0FFFF; // Floating input (reset state)
  GPIOA->CRH   |= 0x00040000; // Input mode (reset state)
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  USBD_ForceDisconnect();

  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_GPIO_Init();

  ITM->TCR |= 1;
  ITM->TER |= 1;

  pcan_can_init();
  pcan_protocol_init();
  pcan_usb_device_init();

  pcan_led_init();
  pcan_led_set_mode( LED_STAT, LED_MODE_OFF, 0xFFFFFFFF );

  for(;;)
  {
    pcan_usb_device_poll();
    pcan_protocol_poll();
    pcan_led_poll();
  }
}
