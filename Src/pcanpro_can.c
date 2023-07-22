#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_variant.h"
#include "pcanfd_ucan.h"

#include "main.h"
#include "mcp251xfd_driver_config.h"
#include "mcp251xfd_ff_config.h"

/*
 * 1. GCC does not inline any functions when not optimizing - https://gcc.gnu.org/onlinedocs/gcc/Inline.html
 * 2. you must have the symbols defined somewhere - https://stackoverflow.com/questions/16245521/c99-inline-function-in-c-file
 * 3. https://stackoverflow.com/questions/19068705/undefined-reference-when-calling-inline-function
 * 4. https://stackoverflow.com/questions/18635665/c99-referring-to-inline-function-undefined-reference-to-xxx-and-why-should-i-pu
 *
 * function defined inline will never emit an externally visible function
 * with -O0 optimization flag we get a linker error 'undefined reference to'
 * So to fix this error, just add extern inline for this function
 */
extern inline eERRORRESULT MCP251XFD_WriteRAM32(MCP251XFD *pComp, uint16_t address, uint32_t data);
extern inline eERRORRESULT MCP251XFD_ReadRAM32(MCP251XFD *pComp, uint16_t address, uint32_t* data);
extern inline eERRORRESULT MCP251XFD_WriteSFR8(MCP251XFD *pComp, uint16_t address, const uint8_t data);
extern inline eERRORRESULT MCP251XFD_WriteSFR16(MCP251XFD *pComp, uint16_t address, const uint16_t data);
extern inline eERRORRESULT MCP251XFD_WriteSFR32(MCP251XFD *pComp, uint16_t address, const uint32_t data);
extern inline eERRORRESULT MCP251XFD_ReadSFR8(MCP251XFD *pComp, uint16_t address, uint8_t* data);
extern inline eERRORRESULT MCP251XFD_ReadSFR16(MCP251XFD *pComp, uint16_t address, uint16_t* data);
extern inline eERRORRESULT MCP251XFD_ReadSFR32(MCP251XFD *pComp, uint16_t address, uint32_t* data);
extern inline eERRORRESULT MCP251XFD_StartCAN20(MCP251XFD *pComp);
extern inline eERRORRESULT MCP251XFD_StartCANFD(MCP251XFD *pComp);
extern inline eERRORRESULT MCP251XFD_StartCANListenOnly(MCP251XFD *pComp);

extern eERRORRESULT ConfigureMCP251XFDDeviceOnEXT1(void);

#define CAN_TX_FIFO_SIZE (64)
struct t_can_dev
{
  void *dev;
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  uint32_t esr_reg;
  int (*rx_isr)( uint8_t, struct  t_can_msg* );
  int (*tx_isr)( uint8_t, struct  t_can_msg* );
  void (*err_handler)( int bus, uint32_t esr );
};

__attribute__((section(".noinit"))) static struct t_can_dev can_dev_array[CAN_BUS_TOTAL];

#define CAN_WITHOUT_ISR 1

void pcan_can_init( void )
{
	eMCP251XFD_Devices device = eMPC251XFD_DEVICE_COUNT;

	if( ERR_OK == ConfigureMCP251XFDDeviceOnEXT1() ) {
	  MCP251XFD_GetDeviceID( CANEXT1, &device, NULL, NULL );
#ifdef DEBUG
	  printf( "  device : %s \r\n", MCP251XFD_DevicesNames[device] );
#endif
	}

	if( MCP2518FD != device ) {
	  Error_Handler();
	}

	memset( &can_dev_array[CAN_BUS_1], 0, sizeof( struct t_can_dev ) );

	can_dev_array[CAN_BUS_1].dev = CANEXT1;
}

uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt )
{
  const uint32_t data_bits = pmsg->size<<3;
  const uint32_t control_bits = ( pmsg->flags & MSG_FLAG_EXT ) ? 67:47;
 
  if( pmsg->flags & MSG_FLAG_BRS )
    return (control_bits*nt) + (data_bits*dt);
  else
    return (control_bits+data_bits)*nt;
}

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask )
{
  return 0;
}

static int _can_send( MCP251XFD *p_can, struct t_can_msg *p_msg )
{
    eERRORRESULT 			ErrorExt1 	= ERR_OK;
    eMCP251XFD_FIFOstatus 	FIFOstatus 	= 0;

    ErrorExt1 = MCP251XFD_GetFIFOStatus(CANEXT1, TXFIFO, &FIFOstatus);
    if (ErrorExt1 != ERR_OK) return ErrorExt1;

    if( FIFOstatus & MCP251XFD_TX_FIFO_NOT_FULL )
    {
        MCP251XFD_CANMessage 		TansmitMessage;
        eMCP251XFD_MessageCtrlFlags	controlFlags 	= MCP251XFD_NO_MESSAGE_CTRL_FLAGS;
        const bool 					isCANFD 		= p_msg->flags & MSG_FLAG_FD  ? true : false;
        const bool 					rtr      		= p_msg->flags & MSG_FLAG_RTR ? true : false;
        const bool 					brs      		= p_msg->flags & MSG_FLAG_BRS ? true : false;

        if( p_msg->flags & MSG_FLAG_EXT ) {
        	TansmitMessage.MessageID = p_msg->id & 0x1FFFFFFF;
        	controlFlags |= MCP251XFD_EXTENDED_MESSAGE_ID;
        }
        else {
        	TansmitMessage.MessageID = p_msg->id & 0x7FF;
        	controlFlags |= MCP251XFD_STANDARD_MESSAGE_ID;
        }

        if( isCANFD ) {
			controlFlags |= MCP251XFD_CANFD_FRAME;
			if( brs ) {
				controlFlags |= MCP251XFD_SWITCH_BITRATE;
			}
        }
        else {
        	if( rtr ) {
				controlFlags |= MCP251XFD_REMOTE_TRANSMISSION_REQUEST;
        	}
        }

        TansmitMessage.ControlFlags	= controlFlags;
        TansmitMessage.DLC 			= ( eMCP251XFD_DataLength )p_msg->size;
        TansmitMessage.PayloadData 	= p_msg->data;

        ErrorExt1 = MCP251XFD_TransmitMessageToFIFO(CANEXT1, &TansmitMessage, TXFIFO, true);

        return ErrorExt1;
    }

  return 0;
}

static void pcan_can_flush_tx( int bus )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  struct t_can_msg *p_msg;

  /* empty fifo */
  if( p_dev->tx_head == p_dev->tx_tail )
	return;

  if( !p_dev->dev )
    return;

  p_msg = &p_dev->tx_fifo[p_dev->tx_tail];
  if( _can_send( p_dev->dev, p_msg ) < 0 )
    return;

  if( p_dev->tx_isr )
  {
    (void)p_dev->tx_isr( bus, p_msg );
  }

  /* update fifo index */
  p_dev->tx_tail = (p_dev->tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
}

int pcan_can_write( int bus, struct t_can_msg *p_msg )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];

  if( !p_dev )
    return 0;

  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (p_dev->tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == p_dev->tx_tail )
  {
	++p_dev->tx_ovfs;
    return -1;
  }

  p_dev->tx_fifo[p_dev->tx_head] = *p_msg;
  p_dev->tx_head = tx_head_next;

  return 0;
}

void pcan_can_install_rx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->rx_isr = cb;
}

void pcan_can_install_tx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->tx_isr = cb;
}
#if 0
void pcan_can_install_err_callback( int bus, void (*cb)( int , uint32_t ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->err_handler = cb;
}
#endif
static bool isoMode 		= true;
static bool loopbackMode 	= false;
static bool silentMode 		= false;

eMCP251XFD_OperationMode opMode = MCP251XFD_CONFIGURATION_MODE;

void apply_settings( int bus, uint16_t opt_mask )
{
	MCP251XFD *p_can = can_dev_array[bus].dev;

	if( !p_can ) return;

	extern MCP251XFD_Config MCP2518FD_Ext1_Config;

	isoMode = 	opt_mask & UCAN_OPTION_ISO_MODE ?
				( MCP2518FD_Ext1_Config.ControlFlags |=  MCP251XFD_CANFD_USE_ISO_CRC ) :
				( MCP2518FD_Ext1_Config.ControlFlags &= ~MCP251XFD_CANFD_USE_ISO_CRC );

	MCP251XFD_ConfigureCANController( p_can, MCP2518FD_Ext1_Config.ControlFlags, 0 );

	opMode  = 	opt_mask & UCAN_OPTION_20AB_MODE ?
				MCP251XFD_NORMAL_CAN20_MODE :
				MCP251XFD_NORMAL_CANFD_MODE;

	bool can20only = MCP251XFD_NORMAL_CAN20_MODE == opMode ? true : false;

	MCP251XFD_BitTimeConfig *p_bt = p_can->UserDriverData;

	uint32_t dataBitRate = CAN_CLOCK_MHZ / ( p_bt->DBRP + 1 ) / ( 3 + p_bt->DTSEG1 + p_bt->DTSEG2 );

	if( !can20only && dataBitRate >= 4 ) {
		static const uint32_t tdc_const = 25;
		p_bt->TDCO = tdc_const / dataBitRate;
		p_bt->TDCMOD = MCP251XFD_AUTO_MODE;
	}
	else {
		p_bt->TDCMOD = MCP251XFD_TDC_DISABLED;
	}

    MCP251XFD_SetBitTimeConfiguration( p_can, p_bt, can20only );
}

void pcan_can_set_silent( int bus, uint8_t silent_mode )
{
  MCP251XFD *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  silentMode = silent_mode ? true : false;
}

void pcan_can_set_iso_mode( int bus, uint8_t iso_mode )
{
	MCP251XFD *p_can = can_dev_array[bus].dev;

  if( !p_can )
	return;

  isoMode = iso_mode ? true : false;
}

void pcan_can_set_loopback( int bus, uint8_t loopback )
{
	MCP251XFD *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;
  
  loopbackMode = loopback ? true : false;
}

void pcan_can_set_bus_active( int bus, uint16_t mode )
{
	MCP251XFD *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  if( mode ) {
	  if( silentMode ) {
		  MCP251XFD_StartCANListenOnly( p_can );
	  }
	  else {
#ifndef LOOPBACK_MODE
		  if( MCP251XFD_NORMAL_CANFD_MODE == opMode ) {
			  MCP251XFD_StartCANFD( p_can );
		  }
		  else {
			  MCP251XFD_StartCAN20( p_can );
		  }
#else
		  const bool waitOperationChange = true;
		  MCP251XFD_RequestOperationMode( p_can, MCP251XFD_EXTERNAL_LOOPBACK_MODE, waitOperationChange );
#endif
	  }
  }
  else {
	  const bool waitOperationChange = true;
	  MCP251XFD_RequestOperationMode( p_can, MCP251XFD_CONFIGURATION_MODE, waitOperationChange );
  }
}

void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate )
{
}

void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, int is_data_bitrate )
{
  MCP251XFD *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  MCP251XFD_BitTimeConfig *p_bt = p_can->UserDriverData;

  if( is_data_bitrate ) {
	  p_bt->DBRP	= brp;
	  p_bt->DTSEG1	= tseg1;
	  p_bt->DTSEG2	= tseg2;
	  p_bt->DSJW	= sjw;
  }
  else {
	  p_bt->NBRP	= brp;
	  p_bt->NTSEG1	= tseg1;
	  p_bt->NTSEG2	= tseg2;
	  p_bt->NSJW	= sjw;
  }
}
#if 0
static void pcan_can_tx_complete( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_msgs;
}

static void pcan_can_tx_err( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_errs;
}
#endif
int pcan_can_stats( int bus, struct t_can_stats *p_stats )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  
  p_stats->tx_msgs = p_dev->tx_msgs;
  p_stats->tx_errs = p_dev->tx_errs;
  p_stats->rx_msgs = p_dev->rx_msgs;
  p_stats->rx_errs = p_dev->rx_errs;
  p_stats->rx_ovfs = p_dev->rx_ovfs;

  return sizeof( struct t_can_stats );
}

void checkForReceiveMessage( MCP251XFD * );

void pcan_can_poll( void )
{
#if ( CAN_WITHOUT_ISR == 1 )
	checkForReceiveMessage( CANEXT1 );
#endif
	pcan_can_flush_tx( CAN_BUS_1 );
}

static void pcan_can_isr_frame( MCP251XFD *hfdcan, eMCP251XFD_FIFO fifo )
{
  const int bus = CAN_BUS_1;
  struct t_can_dev * const p_dev = &can_dev_array[bus];
  struct t_can_msg  msg = { 0 };
  
	eERRORRESULT 			ErrorExt1 			= ERR_OK;
	uint32_t 				MessageTimeStamp 	= 0;
	uint8_t 				RxPayloadData[64];
	MCP251XFD_CANMessage	ReceivedMessage;

	ReceivedMessage.PayloadData = &RxPayloadData[0];
	// that will be received
	ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO( hfdcan, &ReceivedMessage, MCP251XFD_PAYLOAD_64BYTE,
				&MessageTimeStamp, fifo);
	if (ErrorExt1 == ERR_OK)
	{
	//***** Do what you want with the message *****
	  msg.id = ReceivedMessage.MessageID;

	  if( ReceivedMessage.ControlFlags & MCP251XFD_EXTENDED_MESSAGE_ID )
	  {
		msg.flags |= MSG_FLAG_EXT;
	  }

	  if( ReceivedMessage.ControlFlags & MCP251XFD_REMOTE_TRANSMISSION_REQUEST )
	  {
		msg.flags |= MSG_FLAG_RTR;
	  }

	  if( ReceivedMessage.ControlFlags & MCP251XFD_CANFD_FRAME )
	  {
		msg.flags |= MSG_FLAG_FD;
	  }

	  if( ReceivedMessage.ControlFlags & MCP251XFD_SWITCH_BITRATE )
	  {
		msg.flags |= MSG_FLAG_BRS;
	  }

	  msg.size = ReceivedMessage.DLC;

	  msg.timestamp = MessageTimeStamp;

	  memcpy( msg.data, RxPayloadData, MCP251XFD_DLCToByte( msg.size, true ) );

	  if( p_dev->rx_isr )
	  {
		if( p_dev->rx_isr( bus, &msg ) < 0 )
		{
		  ++p_dev->rx_ovfs;
		  return;
		}
	  }
	  ++p_dev->rx_msgs;
	}
}

void checkForReceiveMessage( MCP251XFD *hfdcan )
{
	eERRORRESULT 			ErrorExt1 	= ERR_OK;
	eMCP251XFD_FIFOstatus 	FIFOstatus 	= 0;

	ErrorExt1 = MCP251XFD_GetFIFOStatus( hfdcan, RXFIFO, &FIFOstatus );

	if (ErrorExt1 == ERR_OK) {
		if( FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY ) {
			pcan_can_isr_frame( hfdcan, RXFIFO );
		}
	}
}
