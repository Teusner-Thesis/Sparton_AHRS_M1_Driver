#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <iomanip>
#include <iostream>

#include <chrono>
#include <thread>

bool SpartonAHRSM1Driver::writeAck(std::string s) {
    // Write command
    serial_->write(s.size(), (const uint8_t*)s.c_str());

    // Read acknowledgement
    std::string data(s.size() + 2, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    return (s.substr(0, s.size() - 2) + "OK\r\n" == data);
}

bool SpartonAHRSM1Driver::forgetFunction(std::string function_name) {
    // Write commnad
    std::string command = "forget " + function_name + "\r\n";
    serial_->write(command.size(), reinterpret_cast<const uint8_t*>(const_cast<char *>(command.c_str())));

    // Read acknowledgement
    std::string data(command.size() + 18, '\0');
    serial_->read_until(data.size(), (uint8_t*)(data.c_str()), '\n', 1000);

    // Return aknowledgement
    return (data.find(command.substr(0, command.size() - 2) + "OK\r\n") == 0) or (data.find(command.substr(0, command.size() - 2) + "Can't find it \r\nOK\r\n") == 0);
}


void SpartonAHRSM1Driver::reset() {
    // Reseting ahrs
    std::string msg = "reset\r\n";
    serial_->write(msg.size(), (const uint8_t*)msg.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    serial_->flush();
}

void SpartonAHRSM1Driver::enable_echo() {
    // Disabling input echo from the ahrs
    std::string msg = "1 echo!\r\n";
    serial_->write(msg.size(), (const uint8_t*)msg.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flush();
}

void SpartonAHRSM1Driver::disable_echo() {
    // Disabling input echo from the ahrs
    std::string msg = "0 echo!\r\n";
    serial_->write(msg.size(), (const uint8_t*)msg.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flush();
}

bool SpartonAHRSM1Driver::init() {
    // Enable echo
    enable_echo();

    // Return variable
    bool ret = true;

    // Setting up accelerometer gathering function in (m/s^2)
    ret &= writeAck("accelpScaleFactor f0.01 set\r\n");
    serial_->flush();
    ret &= forgetFunction("accel");
    serial_->flush();
    ret &= writeAck(": accel 8 8 accelpScaled di@ @ ff. .\"  \" 8 8 accelpScaled di@ 4 + @ ff. .\"  \" 8 8 accelpScaled di@ 8 + @ ff. cr ;\r\n");

    // Setting up quaternion gathering function
    ret &= forgetFunction("accelVar");
    serial_->flush();
    ret &= writeAck(": accelVar 8 8 accelVariance di@ ff. cr ;\r\n");
    serial_->flush();

    // Setting up gyroscope gathering function in (rad/s)
    ret &= forgetFunction("gyro");
    serial_->flush();
    ret &= writeAck(": gyro 8 8 gyropScaled di@ @ ff. .\"  \" 8 8 gyropScaled di@ 4 + @ ff. .\"  \" 8 8 gyropScaled di@ 8 + @ ff. cr ;\r\n");
    serial_->flush();

    // Setting up quaternion gathering function
    ret &= forgetFunction("gyroVar");
    serial_->flush();
    ret &= writeAck(": gyroVar 8 8 gyroVariance di@ ff. cr ;\r\n");
    serial_->flush();

    // Setting up magnetometer gathering function in (rad/s)
    ret &= writeAck("magpScaleFactor f0.0001 set\r\n");
    serial_->flush();
    ret &= forgetFunction("mag");
    serial_->flush();
    ret &= writeAck(": mag 8 8 magpScaled di@ @ ff. .\"  \" 8 8 magpScaled di@ 4 + @ ff. .\"  \" 8 8 magpScaled di@ 8 + @ ff. cr ;\r\n");
    serial_->flush();

    // Setting up quaternion gathering function
    ret &= forgetFunction("q");
    serial_->flush();
    ret &= writeAck(": q 8 8 quaternion di@ @ ff. .\"  \" 8 8 quaternion di@ 4 + @ ff. .\"  \" 8 8 quaternion di@ 8 + @ ff. .\"  \" 8 8 quaternion di@ 12 + @ ff. cr ;\r\n");
    serial_->flush();

    // Disable echo
    disable_echo();

    return ret;
}

std::vector<double> SpartonAHRSM1Driver::read_accelerometer() {
    // Requesting accelerations
    serial_->write(7, (uint8_t*)"accel\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<double> accelerometer;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<double>(iss),
        std::istream_iterator<double>(),
        std::back_inserter(accelerometer));

    return accelerometer;
}

double SpartonAHRSM1Driver::read_accelerometer_variance() {
    // Requesting accelerations
    serial_->write(11, (uint8_t*)"accelVar\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    return stod(data.substr(0, data.find("\n")));
}

std::vector<double> SpartonAHRSM1Driver::read_magnetometer() {
    // Requesting accelerations
    serial_->write(6, (uint8_t*)"mag\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);

    std::cout <<  data.substr(0, data.find("\n")) << std::endl;
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<double> magnetometer;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<double>(iss),
        std::istream_iterator<double>(),
        std::back_inserter(magnetometer));

    return magnetometer;
}

std::vector<double> SpartonAHRSM1Driver::read_gyroscope() {
    // Requesting accelerations
    serial_->write(7, (uint8_t*)"gyro\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<double> gyroscope;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<double>(iss),
        std::istream_iterator<double>(),
        std::back_inserter(gyroscope));

    return gyroscope;
}

double SpartonAHRSM1Driver::read_gyroscope_variance() {
    // Requesting accelerations
    serial_->write(10, (uint8_t*)"gyroVar\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    return stod(data.substr(0, data.find("\n")));
}

std::vector<double> SpartonAHRSM1Driver::read_quaternion() {
    // Requesting accelerations
    serial_->write(4, (uint8_t*)"q\r\n");

    // Reading data
    std::string data(1024, '\0');
    serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n', 1000);
    std::istringstream iss(data.substr(0, data.find("\n")));

    // Preparing return
    std::vector<double> quaternion;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<double>(iss),
        std::istream_iterator<double>(),
        std::back_inserter(quaternion));

    return quaternion;
}