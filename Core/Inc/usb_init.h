/*
 * usb_init.h
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */

#ifndef INC_USB_INIT_H_
#define INC_USB_INIT_H_

void usb_initialize(void);
void usbhost_process(void);
uint8_t IsUSBHostConnected(void);

#endif /* INC_USB_INIT_H_ */
