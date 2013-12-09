/*
 *    Author: Frans-Willem
 *      Minor tweak for Tivaware compatibility by Richard Aplin, Dec2013
 *
 *    This version runs on TI Tiva C series Launchpad - part # TM4C123G , costs USD$12 at time of writing, CSR dongle costs ~$250
 *
 *    Connect:
 *
 *
 */

//two new includes for Tivaware types
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_structs.h"
#include "csrspi.h"


#define SYSCTL_PERIPH_LEDS	SYSCTL_PERIPH_GPIOF
#define GPIO_PORT_LEDS_BASE	GPIO_PORTF_BASE
#define GPIO_PIN_LED_R		GPIO_PIN_1
#define GPIO_PIN_LED_G		GPIO_PIN_3
#define GPIO_PIN_LED_B		GPIO_PIN_2

#define BUTTONS_GPIO_PERIPH     SYSCTL_PERIPH_GPIOF
#define BUTTONS_GPIO_BASE       GPIO_PORTF_BASE
#define NUM_BUTTONS             2
#define LEFT_BUTTON             GPIO_PIN_4
#define RIGHT_BUTTON            GPIO_PIN_0
#define ALL_BUTTONS             (LEFT_BUTTON | RIGHT_BUTTON)

#define BUFFER_SIZE	1024

#define CPU_CLOCK_100MHZ	//else 80mhz, both work for me

//#define PRINT_PROTOCOL 1	//debug info

//typedef unsigned int size_t;	//removed for tivaware

unsigned char pReceiveBuffer[BUFFER_SIZE];
volatile size_t nReceiveLength;
volatile int bReceiving = 1;

unsigned char pTransmitBuffer[BUFFER_SIZE];
volatile size_t nTransmitLength;
volatile size_t nTransmitOffset;
volatile int bTransmitting = 0;

#define MODE_SPI	0
#define MODE_JTAG	0xFFFF;
unsigned short g_nMode = MODE_SPI;

#define CMD_READ		0x0100
#define CMD_WRITE		0x0200
#define CMD_SETSPEED	0x0300
#define CMD_GETSTOPPED	0x0400
#define CMD_GETSPEED	0x0500
#define CMD_UPDATE		0x0600
#define CMD_GETSERIAL	0x0700
#define CMD_GETVERSION	0x0800
#define CMD_SETMODE		0x0900
#define CMD_SETBITS		0x0F00
#define CMD_BCCMDINIT	0x4000
#define CMD_BCCMD		0x4100

void WriteWord(unsigned char *szOffset, unsigned short n) {
	szOffset[1] = n & 0xFF;
	szOffset[0] = (n >> 8) & 0xFF;
}

unsigned short ReadWord(unsigned char *szOffset) {
	return (szOffset[0] << 8) | szOffset[1];
}

void TransmitWord(unsigned short n) {
	if (nTransmitLength + 2 < BUFFER_SIZE) {
		WriteWord(&pTransmitBuffer[nTransmitLength], n);
		nTransmitLength += 2;
	}
}

void TransmitDWord(unsigned int n) {
	TransmitWord((n >> 16) & 0xFFFF);
	TransmitWord(n & 0xFFFF);
}

int CmdRead(unsigned short nAddress, unsigned short nLength);
int CmdWrite(unsigned short nAddress, unsigned short nLength, unsigned char *pData);
int CmdSetSpeed(unsigned short nSpeed);
int CmdGetStopped();
int CmdGetSpeed();
int CmdUpdate();
int CmdGetSerial();
int CmdGetVersion();
int CmdSetMode(unsigned short nMode);
int CmdSetBits(unsigned short nWhich, unsigned short nValue);
void CmdBcCmdInit(unsigned short nA, unsigned short nB);
int CmdBcCmd(unsigned short nLength, unsigned char *pData);

void TransmitCallback() {
	size_t nPacket;
	if (bTransmitting) {
		nPacket = nTransmitLength - nTransmitOffset;
		if (nPacket > 64)
			nPacket = 64;
		USBDBulkPacketWrite((void *)&g_sBulkDevice, &pTransmitBuffer[nTransmitOffset], nPacket, true);
		nTransmitOffset += nPacket;
		if (nPacket < 64)
			bTransmitting = 0;
	}
}
void StartTransmit() {
	nTransmitOffset = 0;
	bTransmitting = 1;
	if (USBDBulkTxPacketAvailable((void *)&g_sBulkDevice))
		TransmitCallback();
}
void WaitTransmit() {
	while (bTransmitting);
	nTransmitLength = nTransmitOffset = 0;
}


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
// (Tivaware tweak)
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void InitButtons(void)
{
	// Set each of the button GPIO pins as an input with a pull-up.
	ROM_GPIODirModeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
	ROM_GPIOPadConfigSet(BUTTONS_GPIO_BASE, ALL_BUTTONS,
	                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

/*
 * main.c
 */
void main(void) {
	size_t nOffset;
	unsigned short nCommand, nArgs[4];
	uint32_t buttons;

#ifndef CPU_CLOCK_100MHZ
	//Set Clock at 80MHz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
#else
	//Hey what the hell, let's run at 100Mhz, why not?
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
#endif
	//Enable UART on Port A
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//Do some output!
	ConfigureUART(); //UARTStdioInit(0);	tivaware tweak
	UARTprintf("CSR USB-SPI Emulator for Tiva Launchpad by Frans-Willem 2012,\n tweaked by Richard Aplin 2013\n");

	UARTprintf("CPU Clock %dMhz\n",SysCtlClockGet()/1000000);

	//Csr SPI init
	CsrInit();
	UARTprintf("CSR initialized\n");

	//Status LEDs Init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_LEDS);
	InitButtons();
	GPIOPinTypeGPIOOutput(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_R|GPIO_PIN_LED_G|GPIO_PIN_LED_B);
	GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_R|GPIO_PIN_LED_G|GPIO_PIN_LED_B, 0);
	UARTprintf("LEDs initialized\n");

	//check for left button (NOT) being held down on boot for FAST mode
	buttons=ROM_GPIOPinRead(BUTTONS_GPIO_BASE, LEFT_BUTTON);
	if (!buttons){
		UARTprintf("FAST SPI mode enabled - test reliability with BlueFlash before using!\n");
		CsrSpiEnableFastMode();
	}else{
		UARTprintf("Regular (slow) SPI mode enabled\n");
	}

	//USB init?
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	USBStackModeSet(0, eUSBModeForceDevice, 0);	//tivaware tweak
    USBDBulkInit(0, (tUSBDBulkDevice *)&g_sBulkDevice);
    UARTprintf("Waiting for host...\n");

	while (1) {
		while (bReceiving); //Wait until we're ready receiving data
		if (nReceiveLength == 0) {
			UARTprintf("Empty packet...\n");
		} else {
			WaitTransmit();
			if (pReceiveBuffer[0] != '\0') {
				UARTprintf("Invalid start byte\n");
			} else {
				nOffset = 1;
				while (nReceiveLength >= nOffset + 2) {
					nCommand = ReadWord(&pReceiveBuffer[nOffset]);
					nOffset += 2;
					switch (nCommand) {
					case CMD_READ:
						if (nReceiveLength < nOffset + 4) {
							UARTprintf("Too few arguments to read\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
#if PRINT_PROTOCOL
							UARTprintf("CMD_READ %04x %04x\n",nArgs[0],nArgs[1]);
#endif
							CmdRead(nArgs[0], nArgs[1]);
						}
						break;
					case CMD_WRITE:
						if (nReceiveLength < nOffset + 4) {
							UARTprintf("Too few arguments to write\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							if (nReceiveLength < nOffset + (nArgs[1] * 2)) {
								UARTprintf("Too few arguments to write\n");
							} else {
#if PRINT_PROTOCOL
								UARTprintf("CMD_WRITE %04x %04x\n",nArgs[0],nArgs[1]);
#endif
								CmdWrite(nArgs[0], nArgs[1], &pReceiveBuffer[nOffset]);
								nOffset+=nArgs[1] * 2;
							}
						}
						break;
					case CMD_SETSPEED:
						if (nReceiveLength < nOffset + 2) {
							UARTprintf("Too few arguments to set speed\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
#if PRINT_PROTOCOL
							UARTprintf("CMD_SETSPEED %04x\n",nArgs[0]);
#endif
							CmdSetSpeed(nArgs[0]);
						}
						break;
					case CMD_GETSTOPPED:
						CmdGetStopped();
						break;
					case CMD_GETSPEED:
						CmdGetSpeed();
						break;
					case CMD_UPDATE:
#if PRINT_PROTOCOL
						UARTprintf("CMD_UPDATE\n");
#endif
						CmdUpdate();
						break;
					case CMD_GETSERIAL:
						CmdGetSerial();
						break;
					case CMD_GETVERSION:
						CmdGetVersion();
						break;
					case CMD_SETMODE:
						if (nReceiveLength < nOffset + 2) {
							UARTprintf("Too few arguments to set mode\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
#if PRINT_PROTOCOL
							UARTprintf("CMD_SETMODE %04x\n",nArgs[0]);
#endif
							CmdSetMode(nArgs[0]);
						}
						break;
					case CMD_SETBITS:
						if (nReceiveLength < nOffset + 4) {
							UARTprintf("Too few arguments to set bits\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
#if PRINT_PROTOCOL
							UARTprintf("CMD_SETBITS %04x %04x\n",nArgs[0],nArgs[1]);
#endif
							CmdSetBits(nArgs[0], nArgs[1]);
						}
						break;
					case CMD_BCCMDINIT:
						if (nReceiveLength < nOffset + 4) {
							UARTprintf("Too few arguments to init bccmd\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
#if PRINT_PROTOCOL
							UARTprintf("CMD_BCCMDINIT %04x %04x\n",nArgs[0],nArgs[1]);
#endif
							CmdBcCmdInit(nArgs[0], nArgs[1]);
						}
						break;
					case CMD_BCCMD:
						if (nReceiveLength < nOffset + 2) {
							UARTprintf("Too few arguments to bccmd\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							if (nReceiveLength < nOffset + (nArgs[0] * 2)) {
								UARTprintf("Too few arguments to bccmd\n");
							} else {
#if PRINT_PROTOCOL
								UARTprintf("CMD_BCCMD %04x\n",nArgs[0]);
#endif
								CmdBcCmd(nArgs[0], &pReceiveBuffer[nOffset]);
								nOffset+=nArgs[0] * 2;
							}
						}
						break;
					default:
						UARTprintf("Unknown command: %04X", nCommand);
						nOffset = nReceiveLength;
						continue;
					}
					nOffset += 2; //Read the two inbetween bytes
				}
			}
			if (nTransmitLength)
				StartTransmit();
		}

		//Get ready for the next packet
		nReceiveLength = 0;
		bReceiving = 1;
	}
}

uint32_t TxHandler(void *pvi32CBData, uint32_t ulEvent,
                          uint32_t ulMsgValue, void *pvMsgData)
/*unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)*/
{
    if(ulEvent == USB_EVENT_TX_COMPLETE)
    {
    	TransmitCallback();
    }
    return(0);
}

unsigned short pCsrBuffer[1024];


int CmdRead(unsigned short nAddress, unsigned short nLength) {
	unsigned short *pCurrent;
	GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_G, GPIO_PIN_LED_G);
	if (g_nMode == MODE_SPI && nLength < 1024 && CsrSpiRead(nAddress,nLength,pCsrBuffer)) {
		GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_G, 0);
		TransmitWord(CMD_READ);
		TransmitWord(nAddress);
		TransmitWord(nLength);
		pCurrent = pCsrBuffer;
		while (nLength--) {
			TransmitWord(*(pCurrent++));
		}
	} else {
		GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_G, 0);
		UARTprintf("Read Failed\n");
		TransmitWord(CMD_READ + 1);
		TransmitWord(nAddress);
		TransmitWord(nLength);
		while (nLength--)
			TransmitWord(0);
	}
	return 1;
}
int CmdWrite(unsigned short nAddress, unsigned short nLength, unsigned char *pData) {
	if (nLength > 1024 || g_nMode != MODE_SPI)
		return 0;
	unsigned short i;
	for (i = 0; i < nLength; i++) {
		pCsrBuffer[i] = ReadWord(&pData[i * 2]);
	}
	GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_R, GPIO_PIN_LED_R);
	CsrSpiWrite(nAddress, nLength, pCsrBuffer);
	GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_R, 0);
	return 1;
}
int CmdSetSpeed(unsigned short nSpeed) {
	g_nSpeed = nSpeed;
	return 1;
}
int CmdGetStopped() {
	TransmitWord(CMD_GETSTOPPED);
	TransmitWord(g_nMode != MODE_SPI || CsrSpiIsStopped()); //TODO
	return 1;
}
int CmdGetSpeed() {
	TransmitWord(CMD_GETSPEED);
	TransmitWord(g_nSpeed);
	return 1;
}
int CmdUpdate() {
	return 1;
}
int CmdGetSerial() {
	TransmitWord(CMD_GETSERIAL);
	TransmitDWord(31337);
	return 1;
}
int CmdGetVersion() {
	TransmitWord(CMD_GETVERSION);
	TransmitWord(0x119);
	return 1;
}
int CmdSetMode(unsigned short nMode) {
	g_nMode = nMode;
	return 1;
}
int CmdSetBits(unsigned short nWhich, unsigned short nValue) {
	if (nWhich) g_nWriteBits = nValue;
	else g_nReadBits = nValue;
	return 1;
}

void CmdBcCmdInit(unsigned short nA, unsigned short nB) {
	UARTprintf("BCCMD Init: %04X %04X\n", nA, nB);
	g_nBcA = nA;
	g_nBcB = nB;
}

int CmdBcCmd(unsigned short nLength, unsigned char *pData) {
	unsigned short i;
	//UARTprintf("BCCMD input:\n");
	for (i = 0; i < nLength; i++) {
		pCsrBuffer[i] = ReadWord(&pData[i*2]);
	}
	GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_B, GPIO_PIN_LED_B);
	if (CsrSpiBcCmd(nLength, pCsrBuffer)) {
		TransmitWord(CMD_BCCMD);
	} else {
		TransmitWord(CMD_BCCMD + 1);
	}
	GPIOPinWrite(GPIO_PORT_LEDS_BASE, GPIO_PIN_LED_B, 0);
	TransmitWord(nLength);
	for (i=0; i<nLength; i++) {
		TransmitWord(pCsrBuffer[i]);
	}
	return 1;
}

uint32_t RxHandler(void *pvCBData, uint32_t ulEvent,
                          uint32_t ulMsgValue, void *pvMsgData)
//unsigned long RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
    switch(ulEvent)
    {
        case USB_EVENT_CONNECTED:
        {
        	if (bReceiving) {
        		nReceiveLength = 0;
        	}
            UARTprintf("Connected...\n");
            break;
        }
        case USB_EVENT_DISCONNECTED:
        {
        	UARTprintf("Disconnected...\n");
            break;
        }
        case USB_EVENT_RX_AVAILABLE:
        {
        	if (bReceiving) {
        		if (nReceiveLength + ulMsgValue > BUFFER_SIZE) {
        			UARTprintf("Buffer overflow\n");
        			return 0;
        		}
        		nReceiveLength += USBDBulkPacketRead((void *)&g_sBulkDevice, &pReceiveBuffer[nReceiveLength], ulMsgValue, true);
        		if (ulMsgValue < 64) {
        			bReceiving = 0;
        		}
        		return ulMsgValue;
        	}
        	return 0;
        }
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }
        default:
        {
            break;
        }
    }

    return(0);
}
