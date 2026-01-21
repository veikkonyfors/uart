/*
 * uart.c
 *
 *  Created on: Jan 6, 2026
 *      Author: pappa
 */

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdint.h>


static int uart_fd = -1;

int uart_init(const char *port, int baudrate) {
    struct termios options;

    printf("Open UART: %s baudrate: %d\n", port, baudrate);

    uart_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("UART open failed");
        return -1;
    }

    // Prepare fd
    if (fcntl(uart_fd, F_SETFL, 0) == -1) {
        perror("fcntl:  F_SETFL failed");
        close(uart_fd);
        return -1;
    }

    // Get current settings
    if (tcgetattr(uart_fd, &options) != 0) {
        perror("tcgetattr failed");
        close(uart_fd);
        return -1;
    }

    // Set baud rate
    speed_t speed;
    switch(baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 921600: speed = B921600; break;
        case 1000000: speed = B1000000; break;
        default:
            // Try special baud rate
            printf("Use special baud rate: %d\n", baudrate);
            speed = B115200; // Oletus
            break;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // UART settins
    options.c_cflag |= (CLOCAL | CREAD);    // No modem control and read mode
    options.c_cflag &= ~PARENB;             // No parity
    options.c_cflag &= ~CSTOPB;             // 1 stop-bit
    options.c_cflag &= ~CSIZE;              // clear data bits
    options.c_cflag |= CS8;                 // 8 data bits

    // Input options
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Flow control off
    options.c_iflag &= ~(INLCR | ICRNL);        // Retain crlf

    // Local options
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode

    // Output options
    options.c_oflag &= ~OPOST; // Raw output

    // Set timeout
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0; // No timeout

    // Clear and set
    tcflush(uart_fd, TCIOFLUSH);
    if (tcsetattr(uart_fd, TCSANOW, &options) != 0) {
        perror("tcsetattr failed");
        close(uart_fd);
        return -1;
    }

    printf("UART initialized\n");
    return 0;
}


int uart_send(const uint8_t *data, size_t len) {
    if (uart_fd < 0) {
        fprintf(stderr, "UART uninitialized\n");
        return -1;
    }

    ssize_t bytes_written = write(uart_fd, data, len);
    if (bytes_written != (ssize_t)len) {
        fprintf(stderr, "Only %ld/%zu bytes sent\n",
                bytes_written, len);
        return -1;
    }

    // Wait for all data sent
    tcdrain(uart_fd);
    return 0;
}

/**
 *
 * Created on: Jan 6, 2026
 * Author: pappa
 *
 * @brief Reads a frame from uart port previously initialized with uart_init().
 * frame starts with header byte 0xC8
 * @param *data frame.
 * @param len length of frame returned.
 * @return 0.
 * @note To be run on RPI connected to drone's FC UART.
 * @note Never ending loop to be terminated with SIGINT or SIGKILL.
 * @note Receiver accepts UDP connection from any IP address.
 */
int uart_read_frame(const uint8_t *data, size_t len) {


    unsigned char header[2];
    unsigned char payload[64];

    if (uart_fd < 0) {
        fprintf(stderr, "UART uninitialized\n");
        return -1;
    }

    while (1) {
        // 1. Find sync (0xC8)
        if (read(uart_fd, &header[0], 1) <= 0) continue;
        if (header[0] != 0xC8) continue;

        // 2. Read length
        if (read(uart_fd, &header[1], 1) <= 0) continue;

        int bytes_remaining = header[1];

        if (bytes_remaining > sizeof(payload)) {
            fprintf(stderr, "uart_read_crsf: frame too big: %d\n", bytes_remaining);
            return -1;
            continue;
        }

        // Read rest of the frame
        int n = read(uart_fd, payload, bytes_remaining);

        if (n == bytes_remaining) {

            printf("uart_read_crsf: Read frame type: 0x%02x, Payload size: %d\n", payload[0], n);
            return bytes_remaining + 2;
        }

    }

    return -1;
}

