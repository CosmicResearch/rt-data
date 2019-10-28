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

#include "XSensSensor.h"
#include "io/I2C.h"
#include "io/GPIO.h"

#include <queue>
#include <time.h>

std::queue<XbusMessage*> dataQ, responseQ;

clock_t bf;

I2C i2c(I2C::I2C_1);
// GPIO nRST(4); // Active low reset pin
GPIO DRDY(5); // Active DRDY pin


void* allocateMessageData(size_t bufSize) {
  return bufSize < MAX_XBUS_DATA_SIZE ? malloc(bufSize) : NULL;
}

void deallocateMessageData(void const* buffer) {
  free((void*)buffer);
}

/*!
 * \brief XbusParser callback function to handle received messages.
 * \param message Pointer to the last received message.
 *
 * In this example received messages are copied into one of two message
 * queues for later handling by the main thread. Data messages are put
 * in one queue, while all other responses are placed in the second queue.
 * This is done so that data and other messages can be handled separately
 * by the application code.
 */
void mtMessageHandler(struct XbusMessage const* message) {
  XbusMessage* m = (XbusMessage*)malloc(sizeof(message));
  if (m)
  {
  	memcpy(m, message, sizeof(XbusMessage));
  	if (message->mid == XMID_MtData2)
  	{
  		dataQ.push(m);
  	}
  	else
  	{
  		responseQ.push(m);
  	}
  }
  else if (message->data)
  {
  	deallocateMessageData(message->data);
  }
}

void XSensSensor::i2cWrite(const uint8_t* message, size_t len) {

  i2c.open();
  std::vector<uint8_t> buffer; buffer.assign(message, message + len); // Oju que potser peta

  i2c.write(address, buffer);
  i2c.close();
}

void XSensSensor::i2cReadWithOpCode(uint8_t opcode, uint16_t len)
{
  const int preambleLength = 2;
  uint8_t* buf = (uint8_t*)allocateMessageData((int)len+preambleLength);
  if (buf)
	{
		buf[0] = XBUS_PREAMBLE;
		buf[1] = XBUS_MASTERDEVICE;

    i2c.open();
    i2c.write(address, opcode);

    std::vector<uint8_t> received;
    i2c.read(address, received, len);

    memcpy(&buf[2], received.data(), received.size());

		XbusParser_parseBuffer(xbusParser, buf, len+preambleLength);
		deallocateMessageData(buf);
	}
}

void XSensSensor::i2cReadWithOpCode(uint8_t opcode, uint8_t* dest, uint16_t len)
{
  //Do a I2C write transfer for the opcode, do not generate a stop condition
  i2c.open();
  i2c.write(address, opcode);

  //Generate a repeated start condition (?)
  //Do a I2C read transfer for size bytes; store the received bytes to the destination buffer
  std::vector<uint8_t> received(len);
  i2c.read(address, received, len);

  dest = received.data();

  i2c.close();
}

void XSensSensor::update() {
  uint8_t pipe_status[4];
  i2cReadWithOpCode(XBUS_PIPE_STATUS, pipe_status, 4);

  uint16_t notificationSize = pipe_status[0] | (pipe_status[1] << 8);
  uint16_t measurementSize = pipe_status[2] | (pipe_status[3] << 8);

  if (notificationSize)
  {
      i2cReadWithOpCode(XBUS_NOTIFICATION_PIPE, notificationSize);
  }

  if (measurementSize)
  {
      i2cReadWithOpCode(XBUS_MEASUREMENT_PIPE, measurementSize);
  }
}

bool XSensSensor::waitFor(uint32_t timeout, enum XsMessageId mid, XbusMessage const* response)
{
  clock_t deltaT = clock() - bf;
  double time_taken = (((double)deltaT)/CLOCKS_PER_SEC) * 1000; // in miliseconds
  while (time_taken < (double)timeout) {
    // Assegurar-se que la línia DRDY funciona per defecte
    if (DRDY.read() == HIGH)
    {
      update();
      if (!responseQ.empty())
      {
        response = (XbusMessage const*)responseQ.front(); // Quan tot compili: probar si funciona sense castejar
        responseQ.pop();
        return response->mid == mid;
      }
      else
      {
        Log::log(DEBUG) << "[" << name << "] responseQ is empty";
      }
    }
  }
  Log::log(DEBUG) << "[" << name << "] no response under timeout";
  return false;
}

/*!
 * \brief Wait for a wakeup message from the MTi.
 * \param timeout Time to wait to receive the wakeup message.
 * \return true if wakeup received within timeout, else false.
 *
 * The MTi sends an XMID_Wakeup message once it has completed its bootup
 * procedure. If this is acknowledged by an XMID_WakeupAck message then the MTi
 * will stay in configuration mode. Otherwise it will automatically enter
 * measurement mode with the stored output configuration.

 * Nota: has d'actualitzar (clock_t bf) abans de cridar aquesta funcio.
 */
bool XSensSensor::waitFor(uint32_t timeout, enum XsMessageId mid)
{
  clock_t deltaT = clock() - bf;
  double time_taken = (((double)deltaT)/CLOCKS_PER_SEC) * 1000; // in miliseconds
  while (time_taken < (double)timeout) {
    // Assegurar-se que la línia DRDY funciona per defecte
    if (DRDY.read() == HIGH)
    {
      update();
      if (!responseQ.empty())
      {
        XbusMessage const* m = responseQ.front();
        responseQ.pop();
        return m->mid == mid;
      }
      else
      {
        Log::log(DEBUG) << "[" << name << "] responseQ is empty";
      }
    }
  }
  Log::log(DEBUG) << "[" << name << "] no response under timeout";
  return false;
}

/*!
 * \brief Send a message to the MT
 *
 * This function formats the message data and writes this to the MT I2C
 * interface. It does not wait for any response.
 */
void XSensSensor::sendMessage(XbusMessage const* m)
{
	uint8_t buf[64];
	size_t rawLength = XbusMessage_format(buf, m, XLLF_I2c);
  i2cWrite(buf, rawLength);
}

/*!
 * \brief Send wakeup acknowledge message to MTi.
 *
 * Sending a wakeup acknowledge will cause the device to stay in configuration
 * mode instead of automatically transitioning to measurement mode with the
 * stored output configuration.
 */
void XSensSensor::sendWakeupAck()
{
	XbusMessage ack = {XMID_WakeupAck};
	sendMessage(&ack);
	Log::log(DEBUG) << "[" << name << "] Device ready for operation.";
}

/*!
 * \brief Releases the MTi reset line and waits for a wakeup message.
 *
 * If no wakeup message is received within TIME_AFTER_RESET miliseconds the restore communications
 * procedure is done to reset the MTi to default baudrate and output configuration.
 */
bool XSensSensor::wakeupMotionTracker() {
  //nRST.write((enum PinStatus)HIGH); // Release MT from reset.
  // enviar un missatge RESET
  // sens a RESET message
  XbusMessage reset = {XMID_Reset};
  sendMessage(&reset);
  bf = clock();
	if (waitFor(TIME_AFTER_RESET, XMID_Wakeup))
	{
		sendWakeupAck();
	}
	else
	{
    Log::log(DEBUG) << "[" << name <<  "] WakeUp message not received.";
    return false;
  }
  return true;
}

/*!
 * \brief Send a message to the MT and wait for a response.
 * \returns Response message from the MT, or NULL is no response received
 * within 500ms.
 *
 * Blocking behaviour is implemented by waiting for a response to be written
 * to the response queue by the XbusParser.
 */
XbusMessage const* XSensSensor::doTransaction(XbusMessage const* m) {
  sendMessage(m);
  enum XsMessageId mAck = m->mid;
  mAck = (XsMessageId)((int)mAck + 1);
  XbusMessage const* response = NULL;
  bf = clock();
  return waitFor(500, mAck, response) ? response : NULL;
}

/*!
 * \brief Read the device ID of the motion tracker.
 */
uint32_t XSensSensor::readDeviceId()
{
	XbusMessage reqDid = {XMID_ReqDid};
	XbusMessage const* didRsp = doTransaction(&reqDid);
	uint32_t deviceId = 0;
	if (didRsp)
	{
		if (didRsp->mid == XMID_DeviceId)
		{
			deviceId = *(uint32_t*)didRsp->data;
		}
    else
    {
      Log::log(DEBUG) << "[" << name <<  "] DeviceId not received.";
    }
	}
	return deviceId;
}

/*!
 * \brief Writes information from a message to the LOG file as a debug message.
 */
void XSensSensor::dumpResponse(XbusMessage const* response)
{
	switch (response->mid)
	{
		case XMID_GotoConfigAck:
      Log::log(DEBUG) << "[" << name <<  "] Device went to config mode.";
			break;

		case XMID_Error:
      Log::log(DEBUG) << "[" << name << "] Device error!";
			break;

		default:
      Log::log(DEBUG) << "[" << name << "] Received response MID= " << response->mid << ", length= " << response->length;
			break;
	}
}

/*!
 * \brief Sets MT output configuration.
 * \param conf Pointer to an array of OutputConfiguration elements.
 * \param elements The number of elements in the configuration array.
 *
 * The response from the device indicates the actual values that will
 * be used by the motion tracker. These may differ from the requested
 * parameters as the motion tracker validates the requested parameters
 * before applying them.
 */
bool XSensSensor::setOutputConfiguration(OutputConfiguration const* conf, uint8_t elements)
{
	XbusMessage outputConfMsg = {XMID_SetOutputConfig, elements, (void*)conf};
	XbusMessage const* outputConfRsp = doTransaction(&outputConfMsg);
	if (outputConfRsp)
	{
		if (outputConfRsp->mid == XMID_OutputConfig)
		{
      Log::log(INFO) << "[" << name <<  "] Output configuration set to:";
			OutputConfiguration* conf = (OutputConfiguration*)outputConfRsp->data;
			for (int i = 0; i < outputConfRsp->length; ++i)
			{
        Log::log(INFO) << "[" << name <<  "] " << XbusMessage_dataDescription(conf->dtype) << ": " << conf->freq << " Hz.";
				++conf;
			}
			return true;
		}
		else
		{
			dumpResponse(outputConfRsp);
		}
	}
	else
	{
    Log::log(DEBUG) << "[" << name <<  "] Failed to set output configuration.";
	}
	return false;
}

/*!
 * \brief Sets the motion tracker output configuration based on the function
 * of the attached device.
 *
 * The output configuration depends on the type of MTi-1 device connected.
 * An MTI-1 (IMU) device does not have an onboard orientation filter so
 * cannot output quaternion data, only inertial and magnetic measurement
 * data.
 * MTi-2 and MTi-3 devices have an onboard filter so can send quaternions.
 */
bool XSensSensor::configureMotionTracker(void)
{
	uint32_t deviceId = readDeviceId();

	if (deviceId)
	{
    Log::log(DEBUG) << "[" << name <<  "] Found device with ID: " << deviceId;

		if (!XsDeviceId_isMtMk4_X(deviceId))
		{
      Log::log(DEBUG) << "[" << name <<  "] Device is not an MTi-1 series.";
			return false;
		}

		DeviceFunction function = XsDeviceId_getFunction(deviceId);
    Log::log(DEBUG) << "[" << name <<  "] Device is an MTi-" << function << ": " << XsDeviceId_functionDescription(function);

		if (function == DF_IMU)
		{
			OutputConfiguration conf[] = {
				{XDI_PacketCounter, 65535},
				{XDI_SampleTimeFine, 65535},
				{XDI_Acceleration, 100},
				{XDI_RateOfTurn, 100},
				{XDI_MagneticField, 100}
			};
			return setOutputConfiguration(conf,
					sizeof(conf) / sizeof(OutputConfiguration));
		}

		else
		{
			OutputConfiguration conf[] = {
				{XDI_PacketCounter, 65535},
				{XDI_SampleTimeFine, 65535},
				{XDI_Quaternion, 100},
				{XDI_StatusWord, 65535}
			};
			return setOutputConfiguration(conf,
					sizeof(conf) / sizeof(OutputConfiguration));
		}
	}
	return false;
}

void XSensSensor::start() {

  XbusParserCallback xbusCallback = {};
	xbusCallback.allocateBuffer = allocateMessageData;
	xbusCallback.deallocateBuffer = deallocateMessageData;
	xbusCallback.handleMessage = mtMessageHandler;

  xbusParser = XbusParser_create(&xbusCallback);

//  nRST.open();
//  nRST.set_mode((enum Mode)OUTPUT);
  DRDY.open();
  DRDY.set_mode((enum Mode)INPUT);

  if (wakeupMotionTracker()) {
    if (configureMotionTracker()) {
      Sensor::start();
      Log::log(INFO) << "[" << name << "] XSens sensor started.";
    }
    else
    {
      Log::log(FATAL) << "[" << name <<  "] Failed to configure MTi device.";
    }
  }
  else {
    Log::log(FATAL) << "[" << name <<  "] Failed to communicate with MTi device.";
  }
}

void XSensSensor::stop() {
  // Put XSens in configuration mode
  XbusMessage m = {XMID_GotoConfig};
  XbusMessage const* response = doTransaction(&m);
  if (response)
  {
    dumpResponse(response);
    Sensor::stop();
  }
}

void XSensSensor::read() {
  /*
  1- Mirar el pin DRDY
    - Si esta high, llegir les pipes
    - Si no, no hi ha cap pipe amb missatges pendents de llegir
  2- Si hi ha missatges
    - Buidar les queues i gestionar els missatges
      - Mirar com els gestiona el guille amb la classe GPSData
  */
  if (DRDY.read() == HIGH)
  {
    update();
    if (!dataQ.empty())
    {
      // It's a measurement message
      XbusMessage const* measure = (XbusMessage const*)dataQ.front(); // Quan tot compili: probar si funciona sense castejar
      dataQ.pop();
      xsens_data_t* xsensd_data;
      // Mirem si hi ha packet_counter
      if (XbusMessage_getDataItem(xsensd_data->packet_counter, XDI_PacketCounter, measure))
      {
        Log::log(DEBUG) << "[" << name << "] Packet Counter received";
      }
      if (XbusMessage_getDataItem(xsensd_data->status_word, XDI_StatusWord, measure))
      {
        Log::log(DEBUG) << "[" << name << "] Status Word received";
      }
      if (XbusMessage_getDataItem(xsensd_data->delta_q, XDI_DeltaQ, measure))
      {
        Log::log(DEBUG) << "[" << name << "] DeltaQ received";
      }
      if (XbusMessage_getDataItem(xsensd_data->magnetic_field, XDI_MagneticField, measure))
      {
        Log::log(DEBUG) << "[" << name << "] Magnetic Field received";
      }
      std::shared_ptr<XSensData> xsens_data = std::make_shared<XSensData>(xsensd_data);
      xsens_data->set_origin(name);
      queue.push(xsens_data);
    }
    else
    {
      Log::log(DEBUG) << "[" << name << "] dataQ is empty";
    }
    if (!responseQ.empty())
    {
      // It's a notification message
      XbusMessage const* response = (XbusMessage const*)responseQ.front(); // Quan tot compili: probar si funciona sense castejar
      responseQ.pop();
      dumpResponse(response);
    }
    else
    {
      Log::log(DEBUG) << "[" << name << "] responseQ is empty";
    }
  }
  else
  {
    Log::log(DEBUG) << "[" << name << "] DRDY pin is LOW.";
  }
}

uint8_t XSensSensor::getAddr() const {
  return address;
}

void XSensSensor::setAddr(const uint8_t& addr) {
  address = addr;
}
uint16_t* XSensData::getPacketCounter() const {
  return packet_counter;
}

uint32_t* XSensData::getStatusWord() const {
  return status_word;
}
float* XSensData::getDeltaQ() const {
  return delta_q;
}
float* XSensData::getMagneticField() const {
  return magnetic_field;
}

void XSensData::serialize(SerializedObject* object) {
    Data::serialize(object);
    object->put("packet_counter", (unsigned int)*packet_counter);
    object->put("status_word", (int) *status_word);
    object->put("delta_q[0]", delta_q[0]);
    object->put("delta_q[1]", delta_q[1]);
    object->put("delta_q[2]", delta_q[2]);
    object->put("delta_q[3]", delta_q[3]);
    object->put("magnetic_field-x", magnetic_field[0]);
    object->put("magnetic_field-y", magnetic_field[1]);
    object->put("magnetic_field-z", magnetic_field[2]);
}

void XSensData::deserialize(SerializedObject* object) {
    Data::deserialize(object);
    uint16_t pc = (uint16_t)object->get_uint("packet_counter");
    packet_counter = &pc;
    uint32_t sw = (uint32_t)object->get_int("status_word");
    status_word = &sw;
    float deltaQ [4];
    deltaQ[0] = object->get_float("delta_q[0]");
    deltaQ[1] = object->get_float("delta_q[1]");
    deltaQ[2] = object->get_float("delta_q[2]");
    deltaQ[3] = object->get_float("delta_q[3]");
    delta_q = &deltaQ[0];
    float magF [3];
    magF[0] = object->get_float("magnetic_field-x");
    magF[1] = object->get_float("magnetic_field-y");
    magF[2] = object->get_float("magnetic_field-z");
    magnetic_field = &magF[0];
}
