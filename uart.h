/*
 * uart.h
 *
 *  Created on: Jan 6, 2026
 *      Author: pappa
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stddef.h>

int uart_init(const char *port, int baudrate);
int uart_send(const uint8_t *data, size_t len);

/**
 *
 * Created on: Jan 6, 2026
 * Author: pappa
 *
 * @brief Reads a frame from uart port previously initialized with uart_init().
 * Frame starts when header byte 0xC8 encountered.
 * @param *frame  returned frame data, space provided by the caller.
 * @param len On input: max space allocated for frame, on output: length of frame returned.
 * @return size of the frame returned or -1 in case of error.
 * @note To be run on RPI connected to drone's FC UART.
 * @note Never ending loop to be terminated with SIGINT or SIGKILL.
 * @note Receiver accepts UDP connection from any IP address.
 */
int uart_read_frame(const uint8_t *frame, size_t len);

#endif /* UART_H_ */
