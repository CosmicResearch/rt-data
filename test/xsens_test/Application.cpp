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

#include <rtdata/Application.h>
//#include <rtdata/sensors/AnalogSensor.h>
#include <rtdata/io/SQLiteWriter.h>
#include <rtdata/utils/LambdaListener.h>
#include <rtdata/Log.h>
#include <rtdata/sensors/XSensSensor.h>
#include <rtdata/sensors/xbus/xbusparser.h>

SQLiteWriter writer("database.db");

void Application::setup() {
    Log::init();
    Log::log(INFO) << "Acceptance test config started";
    std::cout << "Acceptance test config started" << std::endl;
    writer.open();
    /*std::shared_ptr<Sensor> sensor = std::make_shared<AnalogSensor>(
        "./a.txt", // file
        "test", // topic
        "stubsensor", // sensor name
        10000000, // sampling rate = 10 ms
        0.0, // sensor min voltage = 0V
        3.3, // sensor max voltage = 3.3V
        100.0, // scale
        10, // ADC quantization bits = 10 bits
        0, // ADC min voltage = 0V
        3.3 // ADC max voltage = 3.3V
    );*/
    std::shared_ptr<Sensor> sensor = std::make_shared<XSensSensor>(
      "xsens", // sensor name
      "xsens-test", // topic
      10000000, // sampling rate
      0x6B // i2c address
    );
    manager.add_sensor(sensor);
    broker.subscribe("xsens-test", std::make_shared<LambdaListener>([](std::string topic, std::shared_ptr<Data> data) {
        /*std::shared_ptr<AnalogData> analog_data = std::static_pointer_cast<AnalogData>(data);
        Log::log(DEBUG) << "Received data with time " << analog_data->getTimestamp().toNanos();
        Log::log(DEBUG) << "Received data with value " << analog_data->getValue();
        Log::log(DEBUG) << "Received data with origin " << data->getOrigin().c_str();*/
        std::shared_ptr<XSensData> xsens_data = std::static_pointer_cast<XSensData>(data);
        std::cout << "Received data with time " << data->get_timestamp().to_nanos() << std::endl;
        // He d'anar amb compte perque les dades son tot punters
        std::cout << "Received status word with value " << xsens_data->getStatusWord() << std::endl; // Falta mirar lo dels punters
        std::cout << "Received data with origin " << data->get_origin().c_str() << std::endl;
        writer.write(topic, data);
        writer.flush();
    }));
    broker.start();
    manager.start();
    std::cout << "Acceptance test config ended" << std::endl;
}

void Application::loop() {

}
