/*
 * usb_init.h
 *
 *  Created on: Sep 29, 2020
 *      Author: ngx-vivek
 */

#ifndef INC_USB_INIT_H_
#define INC_USB_INIT_H_

void usb_host_initialize(void);
void usb_device_initialize(void);
void usbhost_process(void);
uint8_t IsUSBHostConnected(void);
void usb_host_deinitialize(void);

#endif /* INC_USB_INIT_H_ */
