# !ACHTUNG! READ FIRST [COPYRIGHT ISSUE](https://github.com/moonglow/pcan_pro_x/issues/16)

## QPCAN USB FD firmware for Blue Pill board

Target hardware: Blue Pill board

Pinout:  
|PIN/PINS|DESCRIPTION|  
| ------ | ------ |  
|PA11/PA12|USB FS DM/DP|  
|PC13|Status LED|  

Works with [PEAK PCAN-View][pvw] in Windows

Before connecting the device to the USB port, the following must be done in the PCAN-View window:  
1. All transmissions must be disabled.  
2. PCAN-View program must be disconnected from PCAN hardware.  

![](/images/1.png)  
  
![](/images/2.png)  

Otherwise for unknown reasons there is a failure in USB connection.  
Inside pcan_protocol_process_data() function fields pmsg->size and pmsg->type are zero.

Toolchain: GNU Arm Embedded Toolchain

It is possible to build firmware with STM32CubeIDE project (I used version 1.12.1) or make.

---

Limitations.  
1. Clock Frequency must be 40 MHz.  
2. Error Generator does not work.  

---

It is possible to use the CANHacker program with the QPCAN-USB FD device.

Certainly, in classic CAN mode only.

To do this, it is need to configure the settings in the Settings window of the CANHacker program.

![](/images/3.png)  

CAN Device   - PEAKUSB  
CAN Baudrate - User Def. depending on the speed. The algorithm is below.  

BTR0 - upper byte Baudrate Reg field  
BTR1 - lower byte Baudrate Reg field  

BRP = 10 * ( BTR0 & 0x3F ) + 9  

tseg1 = ( BTR1 & 0xF )  
tseg2 = ( BTR1 >> 4  ) & 0x07  

SYSCLK = 80000000 (80 MHz)  

BitRate = SYSCLK / ( 1 + BRP ) / ( 3 + tseg1 + tseg2 )  

Examples.  

  1 MBit/s -> Baudrate Reg = 1  
500 kBit/s -> Baudrate Reg = 14  
250 kBit/s -> Baudrate Reg = 114  
125 kBit/s -> Baudrate Reg = 314  
100 kBit/s -> Baudrate Reg = 325  

License
----

WTFPL

[pvw]: <https://www.peak-system.com/PCAN-View.242.0.html?&L=1>
