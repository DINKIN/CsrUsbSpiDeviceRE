/*
 * usb_structs.h
 *
 *  Created on: 31 dec. 2012
 *      Author: Frans-Willem
 *      Minor tweak for Tivaware compatibility by Richard Aplin, Dec2013
 */

#ifndef USB_STRUCTS_H_
#define USB_STRUCTS_H_

#define BULK_BUFFER_SIZE 256

extern uint32_t RxHandler(void *pvCBData, uint32_t ui32Event,
                          uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t TxHandler(void *pvi32CBData, uint32_t ui32Event,
                          uint32_t ui32MsgValue, void *pvMsgData);

extern tUSBDBulkDevice g_sBulkDevice;


#endif /* USB_STRUCTS_H_ */
