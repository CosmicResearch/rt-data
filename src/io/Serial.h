/**
 * rt-data
 * Copyright (C) 2019 Guillem Castro
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <termios.h>
#include <vector>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <stdio.h>

enum Parity {
    NONE,
    EVEN,
    ODD,
    MARK, // Parity bit present but not used, always set to 1
    SPACE // Parity bit present but not used, always set to 0
};

class Serial {

public:

    /**
     * Full constructor
     * @param file The file that represents the serial stream to open
     * @param baudRate The baud rate of the communication
     * @param bits The lenght of a Character. 5, 6, 7 or 8 bits
     * @param parity The type of parity of the characters
     */
    Serial(const std::string& file, int baudRate, int bits, Parity parity) : file(file), baud_rate(convert_baud_rate(baudRate)), bits(convert_bits(bits)), parity(convert_parity(parity)), isopen(false) {
        start();
    }

    /**
     * Constructor with file and baud rate
     * @param file The file that represents the serial stream to open
     * @param baudRate The baud rate of the communication
     */
    Serial(const std::string& file, int baudRate) : Serial(file, baudRate, 8, NONE) {

    }

    /**
     * Constructor with filename
     * @param file The file that represents the serial stream to open
     */
    explicit Serial(const std::string& file) : Serial(file, 9600, 8, NONE) {

    }

    ~Serial() {
        try {
            stop();
        }
        catch (...) {
            // Nothing to do here
        }
    }

    /**
     * Open the serial port and configure it
     */
    void start();

    /**
     * Close the serial port. No more actions can be performed on the object.
     */
    void stop();

    /**
     * Flush all the data received and not read, and all data written but not transmitted
     */
    void flush();

    /**
     * Send a buffer of bytes to through the serial port
     * @param buffer The bytes to send
     */
    void send(const std::vector<uint8_t>& buffer);

    /**
     * Receive a single byte from the serial port
     * @returns The received byte
     */
    uint8_t receive();

    /**
     * Receive `size` bytes from the serial port
     * @param out An output vector where the bytes will be stored
     * @param size The ammount of bytes to be received
     */
    void receive(std::vector<uint8_t>& out, size_t size);

    /**
     * Is the Serial port open?
     * @returns Whether or not the Serial port is open
     */
    bool is_open();

    /**
     * Is the Serial port closed?
     * @returns Whether or not the Serial port is closed
     */
    bool is_closed();
    
    /**
     * How many bytes are available to read?
     * @returns int: the number of bytes available to read.
     */
    int available();
    
    /**
     * Prints data to the serial port as human-readable ASCII text.
     */
    void print(const char* format);

private:

    /** Helper methods for conversions **/

    static speed_t convert_baud_rate(int baudRate);

    static uint32_t convert_bits(int bits);

    static uint32_t convert_parity(Parity parity);

    /** End helper methods **/

    std::string file;

    speed_t baud_rate;

    uint32_t bits, parity;

    termios tty;

    int tty_fd;

    bool isopen;

};
