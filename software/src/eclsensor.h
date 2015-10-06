/*
 * Serial interface to Eclipse sensor FPGA
 */

/*
 * Copyright (c) 2015 Micah Elizabeth Scott <micah@misc.name>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/time.h>


class EclSensor
{
public:
    // Matching constants in Verilog
    static const unsigned kTxCount = 12;
    static const unsigned kRxCount = 20;
    static const unsigned kRxTimerNybbles = 4;

    static const int kNumRetries = 10;
    static const int kPacketTimeoutMillisec = 100;

    struct Packet {
        unsigned tx_id;
        uint32_t rx_timers[kRxCount];
    };

    bool init(const char *rbf_path, const char *serial_path);
    const Packet *poll();

private:
    int serial_fd;
    char block_buffer[512];
    unsigned block_offset, block_len;
    Packet line_buffer;
    unsigned line_column;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline bool EclSensor::init(const char *rbf_path, const char *serial_path)
{
    struct termios options;
    FILE *rbf_file;

    serial_fd = open(serial_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        perror("Failed to open serial port");
        return false;
    }

    rbf_file = fopen(rbf_path, "rb");
    if (!rbf_file) {
        perror("Failed to open FPGA bitstream (.rbf) file");
        return false;
    }

    tcgetattr(serial_fd, &options);

    // Turn off posix TTY features we don't want, set up 8-N-1
    options.c_cflag &= (tcflag_t) ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
    options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD | CS8);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK | IXON | IXOFF);
#ifdef IUCLC
    options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
    options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

    // Polling read, don't wait for data to arrive
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    for (int tries = 0; tries <= kNumRetries; tries++) {

        // Start out assuming the FPGA is configured.
        // It sends back data at 921600 baud (wow, such fast)
        cfsetispeed(&options, B921600);
        cfsetospeed(&options, B921600);
        tcsetattr(serial_fd, TCSANOW, &options);

        // Reset receiver state

        block_offset = 0;
        block_len = 0;
        line_column = 0;
        memset(&line_buffer, 0, sizeof line_buffer);

        // Make sure we are receiving good data

        fprintf(stderr, "Checking FPGA... ");
        struct timeval start_time;
        gettimeofday(&start_time, 0);
        while (1) {
            if (poll()) {
                // Success, we got a good packet back.
                fprintf(stderr, "ok\n");
                return true;
            }

            usleep(1);

            struct timeval now;
            gettimeofday(&now, 0);
            long millis = (now.tv_sec - start_time.tv_sec) * 1000 +
                          (now.tv_usec - start_time.tv_usec) / 1000;
            if (millis >= kPacketTimeoutMillisec) {
                fprintf(stderr, "timeout\n");
                break;
            }
        }

        // Nope. Assume it isn't configured.
        fprintf(stderr, "Configuring FPGA... ");
        rewind(rbf_file);

        // Start at 115200 baud for FPGA configuration
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        tcsetattr(serial_fd, TCSANOW, &options);

        // Send a Break for at least 50ms to reset the FPGA.
        // This is detected by a discrete circuit on the Pluto board.
        // On Linux, the tcsendbreak duration unit is defined as between .25 and .5 sec.

        tcsendbreak(serial_fd, 1);

        // The FPGA is configured with bits from a .rbf file, LSB-first.
        // The Pluto FPGA board has a circuit that will read a short pulse
        // as 0 and a longer pulse as 1. We can send 3 pulses per UART byte.
        // Operate on groups of 64 UART bytes / 192 pulses / 24 RBF bytes,
        // padded out to the nearest block. (After the FPGA is configured,
        // extra input is ignored.)

        uint8_t rbf_block[24];
        uint8_t uart_block[64];

        while (1) {
            memset(rbf_block, 0, sizeof rbf_block);
            memset(uart_block, 0, sizeof rbf_block);

            if (fread(rbf_block, 1, sizeof rbf_block, rbf_file) <= 0) {
                break;
            }

            for (unsigned byte = 0; byte < sizeof uart_block; byte++) {
                unsigned bit[3];
                for (unsigned i = 0; i < 3; i++) {
                    unsigned index = byte * 3 + i;
                    bit[i] = 1 & (rbf_block[index >> 3] >> (index & 7));
                }
                // [start 0]x1 0x1 0x1[1 stop]
                uart_block[byte] = 0x92 | bit[0] | (bit[1] << 3) | (bit[2] << 6);
            }

            unsigned offset = 0;
            while (offset < sizeof uart_block) {
                int result = write(serial_fd, uart_block + offset, sizeof uart_block - offset);
                if (result < 0) {
                    if (errno == EAGAIN) {
                        tcdrain(serial_fd);
                    } else {
                        perror("Failed while writing to configure the FPGA");
                    }
                } else {
                    offset += result;
                }
            }
        }

        // Finish sending the last block
        tcdrain(serial_fd);
        fprintf(stderr, "done\n");

        // Try again...
    }

    // Out of tries
    return false;
}

inline const EclSensor::Packet* EclSensor::poll()
{
    // Poll for available data, returning when we get a complete packet

    while (1) {
        // Fill the block buffer if needed
        if (!block_len) {
            ssize_t retval = read(serial_fd, block_buffer, sizeof block_buffer);
            if (retval > 0) {
                block_len = retval;
                block_offset = 0;
            } else {
                // Not ready
                if (retval < 0 && errno != EAGAIN) {
                    perror("Error reading from FPGA serial port");
                }
                return 0;
            }
        }

        // Parse each character into the correct part of the packet buffer,
        // yielding if we see a complete packet. The remainder of the block
        // will be parsed next time.

        while (block_offset < block_len) {
            char c = block_buffer[block_offset++];
            unsigned c_upper = c - 'A';
            unsigned c_lower = c - 'a';
            unsigned c_digit = c - '0';

            if (c == '\n' || c == '\r') {
                // Line endings, return a packet if we have one.
                if (line_column == 2 + kRxTimerNybbles * kRxCount) {
                    line_column = 0;
                    return &line_buffer;
                } else {
                    // Redundant line ending, or we received a malformed line
                    line_column = 0;
                }
            } else if (c_upper < kTxCount && line_column == 0) {
                // First character, transmitter ID as an uppercase letter
                line_buffer.tx_id = c_upper;
                line_column++;
            } else {
                // Hex digit, one rx timer nybble
                int rx_id = (line_column - 1) / kRxTimerNybbles;
                int shift = 4 * (kRxTimerNybbles - 1 - ((line_column - 1) % kRxTimerNybbles));
                uint32_t mask = 0xF << shift;
                uint32_t bits = (0xF & ((c_digit < 10) ? c_digit : c_lower)) << shift;
                line_buffer.rx_timers[rx_id] = (line_buffer.rx_timers[rx_id] & (~mask)) | bits;
                line_column++;
            }
        }

        // Finished the block
        block_len = 0;
    }
}
