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
#include <rtdata/sensors/XSensSensor.h>
#include <rtdata/io/SQLiteWriter.h>
#include <rtdata/utils/LambdaListener.h>
#include <rtdata/Log.h>

SQLiteWriter writer("database.db");

void Application::setup() {
    Log::init();
    Log::log(INFO) << "Acceptance test config started";
    std::shared_ptr<Sensor> sensor = std::make_shared<XSensSensor>(
      "xsens", // sensor name
      "xsens-test", // topic
      10000000, // sampling rate
      0x6B // i2c address
    );
    //writer.open();
   
    manager.add_sensor(sensor);
    broker.subscribe("xsens-test", std::make_shared<LambdaListener>([](std::string topic, std::shared_ptr<Data> data) {
        std::shared_ptr<XSensData> xsens_data = std::static_pointer_cast<XSensData>(data);
        Log::log(DEBUG) << "Received data with time " << data->get_timestamp().to_nanos();
        // He d'anar amb compte perque les dades son tot punters
	Log::log(DEBUG) << "Received packet counter with value " << *(xsens_data->getPacketCounter());
        Log::log(DEBUG) << "Received status word with value " << *(xsens_data->getStatusWord());
	Log::log(DEBUG) << "Received deltaQ with value " << *(xsens_data->getDeltaQ());
        Log::log(DEBUG) << "Received magnetic field with value " << *(xsens_data->getMagneticField());
	Log::log(DEBUG) << "Received data with origin " << data->get_origin().c_str();
        writer.write(topic, data);
        writer.flush();
    }));
    broker.start();
    manager.start();
    Log::log(DEBUG) << "Acceptance test config ended";
}

void Application::loop() {

}
