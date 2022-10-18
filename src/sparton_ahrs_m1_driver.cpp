#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <iostream>

#include <chrono>
#include <thread>

bool SpartonAHRSM1Driver::writeAck(std::string s) {
    std::string data(s.size() + 2, '\0');
    serial_->write(s.size(), (const uint8_t*)s.c_str());
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    return (s.substr(0, s.size() - 2) + "OK\r\n" == data);
}


bool SpartonAHRSM1Driver::reset() {
    bool ret = true;

    // Reseting ahrs
    std::string msg = "reset\r\n";
    serial_->write(msg.size(), (const uint8_t*)msg.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    serial_->flush();

    // Setting up accelerometer gathering function in (m/s^2)
    ret &= writeAck("accelpScaleFactor f0.01 set\r\n");
    ret &= writeAck(": accel 8 8 accelpScaled di@ @ ff. .\"  \" 8 8 accelpScaled di@ 4 + @ ff. .\"  \" 8 8 accelpScaled di@ 8 + @ ff. cr ;\r\n");

    // Setting up gyroscope gathering function in (rad/s)
    ret &= writeAck(": gyro 8 8 gyropScaled di@ @ ff. .\"  \" 8 8 gyropScaled di@ 4 + @ ff. .\"  \" 8 8 gyropScaled di@ 8 + @ ff. cr ;\r\n");

    // Setting up magnetometer gathering function in (rad/s)
    ret &= writeAck("magpScaleFactor f0.0001 set\r\n");
    ret &= writeAck(": mag 8 8 magpScaled di@ @ ff. .\"  \" 8 8 magpScaled di@ 4 + @ ff. .\"  \" 8 8 magpScaled di@ 8 + @ ff. cr ;\r\n");

    // Disabling input echo from the ahrs
    msg = "0 echo!\r\n";
    serial_->write(msg.size(), (const uint8_t*)msg.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    serial_->flush();

    return ret;
}


    

std::vector<float> SpartonAHRSM1Driver::read_accelerometer() {
    // Requesting accelerations
    serial_->write(7, (uint8_t*)"accel\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<float> accelerometer;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(accelerometer));

    return accelerometer;
}

std::vector<float> SpartonAHRSM1Driver::read_magnetometer() {
    // Requesting accelerations
    serial_->write(5, (uint8_t*)"mag\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<float> magnetometer;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(magnetometer));

    return magnetometer;
}

std::vector<float> SpartonAHRSM1Driver::read_gyroscope() {
    // Requesting accelerations
    serial_->write(6, (uint8_t*)"gyro\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<float> gyroscope;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(gyroscope));

    return gyroscope;
}