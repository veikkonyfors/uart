/**
 * @file uart.h
 * @author pappa
 * @date Jan 6, 2026
 * @brief Universal Asynchronous Receiver-Transmitter methods.
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stddef.h>

/**
 *
 * Created on: Jan 6, 2026
 * Author: pappa
 *
 * @brief Initiates given uart port.
 * File descriptor for the initiated uart port is maintained and used in global variable uart_fd.
 * @param *port uart to be initialized.
 * @param baudrate speed.
 * @return 0 on success, -1 on failure.
 */
int uart_init(const char *port, int baudrate);

/**
 *
 * Created on: Jan 6, 2026
 * Author: pappa
 *
 * @brief Sends data to uart port previously initialized with uart_init().
 * @param *data  Data to be sent.
 * @param len Length of data to be sent.
 * @return 0 on success, -1 on failure.
 */
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
 */
int uart_read_frame(const uint8_t *frame, size_t len);

#endif /* UART_H_ */
