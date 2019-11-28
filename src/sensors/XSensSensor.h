
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

#include "../Sensor.h"
#include "../Log.h"
#include "../xbus/xbusdef.h"
#include "../xbus/xbusparser.h"
#include "../xbus/xbusmessage.h"
#include "../xbus/xsdeviceid.h"

#define TIME_AFTER_RESET (1000)


/*!
 * \brief The size of the queue used for device responses.
 * This is set to one as in typical Xbus operation each command receives a
 * response before the next command is sent.
 */
//#define RESPONSE_QUEUE_SIZE (1)
/*!
 * \brief The size of the queue used for data messages.
 * This is set to two to allow some overlap between printing received data to
 * the PC serial port and the reception of the subsequent data packet. In
 * more complex applications it might be necessary to increase this if
 * message processing might occasionally require more time than normal.
 */
//#define DATA_QUEUE_SIZE (5)
/*!
 * \brief The maximum size of an xbus message supported by the application.
 * This is the size of the message buffers in the message data memory pool.
 */
#define MAX_XBUS_DATA_SIZE (128)

// Forward declaration
class XSensData;

/**
 * XSens sensor
 * Can read from a XSens MTi connected to a I2C port.
 */
class XSensSensor : public Sensor {

public:

    /**
     * Default constructor
     */
     XSensSensor() : Sensor(), address(0x6B) {}

    XSensSensor(const std::string& name, const std::string& topic,
        uint64_t rate, const uint8_t& address = 0x6B) : Sensor(name, topic, rate), address(address) {}

    /**
     * Constructor with all configurable parameters
     * @param name Name of the sensor. Used to identify it.
     * @param topic The topic where this sensor will publish its data to
     * @param rate The rate in nanoseconds at which the sensor will be read
     * @param host The hostname of the host where the gpsd daemon is running. By default "localhost"
     * @param port The port that gpsd is listening to. By default DEFAULT_GPSD_PORT
     */
  /*  GPSSensor(const std::string& name, const std::string& topic,
        uint64_t rate, const std::string& host="localhost", const std::string& port=DEFAULT_GPSD_PORT) : Sensor(name, topic, rate), host(host), port(port), gps(host.c_str(), port.c_str()) {

    }
*/
    /**
     * Constructor with a Configuration object
     * @param config The node containing the configuration for this sensor
     */
    /*explicit GPSSensor(Configuration& config) : Sensor(config),
        host(config["host"].get<std::string>()), port(config["port"].get<std::string>()), gps(host.c_str(), port.c_str()) {

    }*/

    explicit XSensSensor(Configuration& config) : Sensor(config) {}

    /**
     * Starts the sensor.
     * @throws std::runtime_error if the sensor has been already started or if it was stopped.
     * @throws std::runtime_error if the gpsd daemon is not running
     */
    virtual void start() override;

    /**
     * Stops the sensor.
     * @throws std::runtime_error if the sensor has been already stopped or if it was never started.
     */
    virtual void stop() override;

    uint8_t getAddr() const;

    void setAddr(const uint8_t& addr);

protected:

    /**
     * Internal read method
     */
    virtual void read();

private:

  uint8_t address;

  XbusParser* xbusParser;

  void i2cWrite(const uint8_t* message, size_t len);
  void i2cReadWithOpCode(uint8_t opcode, uint16_t len);
  void i2cReadWithOpCode(uint8_t opcode, uint8_t* dest, uint16_t len);
  void update();

  bool waitFor(uint32_t timeout, enum XsMessageId mid);
  bool waitFor(uint32_t timeout, enum XsMessageId mid, XbusMessage const* response);
  void sendMessage(XbusMessage const* m);
  void sendWakeupAck();
  bool wakeupMotionTracker();

  XbusMessage const* doTransaction(XbusMessage const* m);
  uint32_t readDeviceId();
  void dumpResponse(XbusMessage const* response);
  bool setOutputConfiguration(OutputConfiguration const* conf, uint8_t elements);
  bool configureMotionTracker();

};

struct xsens_data_t {
  uint16_t* packet_counter;
  uint32_t* status_word;
  float* delta_q;
  float* magnetic_field;
};

class XSensData : public Data {

public:

    /**
     * Default constructor
     */
    XSensData() : Data() {

    }



    explicit XSensData(xsens_data_t* xsens_data) :
      packet_counter(xsens_data->packet_counter),
      status_word(xsens_data->status_word),
      delta_q(xsens_data->delta_q),
      magnetic_field(xsens_data->magnetic_field)
     {

    }

    uint16_t* getPacketCounter() const;
    uint32_t* getStatusWord() const;
    float* getDeltaQ() const;
    float* getMagneticField() const;

    /**
     * Serialize the AnalogData. Do not call directly.
     * @param object The resulting SerializedObject where the data must be saved.
     */
    virtual void serialize(SerializedObject* object) override;

    /**
     * Deserialize the AnalogData. Do not call directly.
     * @param object The SerializedObject to load the data from.
     */
    virtual void deserialize(SerializedObject* object) override;

private:

  uint16_t* packet_counter;
  uint32_t* status_word;
  float* delta_q;
  float* magnetic_field;

};
