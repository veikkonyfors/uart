/*
 * uart.h
 *
 *  Created on: Jan 6, 2026
 *      Author: pappa
 */

#ifndef UART_H_
#define UART_H_

int uart_init(const char *port, int baudrate);
int uart_send(const uint8_t *data, size_t len);

#endif /* UART_H_ */
