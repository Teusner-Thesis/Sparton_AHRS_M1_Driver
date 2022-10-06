#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <iostream>

#include <chrono>
#include <thread>


void SpartonAHRSM1Driver::reset() {
    sync_serial_.writeString("reset\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Disabling input echo from the ahrs
    sync_serial_.writeString("0 echo!\r\n");

    // Setting up accelerometer gathering function in (m/s^2)
    sync_serial_.writeString("accelpScaleFactor f0.01 set\r\n");
    sync_serial_.writeString(": accel 8 8 accelpScaled di@ @ ff.\r\n.\"  \"\r\n8 8 accelpScaled di@ 4 + @ ff.\r\n.\"  \"\r\n8 8 accelpScaled di@ 8 + @ ff.\r\ncr\r\n;\r\n");

    // Setting up gyroscope gathering function in (rad/s)
    sync_serial_.writeString(": gyro 8 8 gyropScaled di@ @ ff.\r\n.\"  \"\r\n8 8 gyropScaled di@ 4 + @ ff.\r\n.\"  \"\r\n8 8 gyropScaled di@ 8 + @ ff.\r\ncr\r\n;\r\n");

    // Setting up magnetometer gathering function in (rad/s)
    sync_serial_.writeString("magpScaleFactor f0.0001 set\r\n");
    sync_serial_.writeString(": mag 8 8 magpScaled di@ @ ff.\r\n.\"  \"\r\n8 8 magpScaled di@ 4 + @ ff.\r\n.\"  \"\r\n8 8 magpScaled di@ 8 + @ ff.\r\ncr\r\n;\r\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

std::vector<float> SpartonAHRSM1Driver::read_accelerometer() {
    // Requesting accelerations
    sync_serial_.flush(SyncSerial::FlushType::FlushBoth);
    sync_serial_.writeString("accel\r\n");
    std::istringstream iss(sync_serial_.readLine());

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
    sync_serial_.flush(SyncSerial::FlushType::FlushBoth);
    sync_serial_.writeString("mag\r\n");
    std::istringstream iss(sync_serial_.readLine());

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
    sync_serial_.flush(SyncSerial::FlushType::FlushBoth);
    sync_serial_.writeString("gyro\r\n");
    std::istringstream iss(sync_serial_.readLine());

    // Preparing return
    std::vector<float> gyroscope;

    // Getting values from the stream using >> operator on istream
    std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(gyroscope));

    return gyroscope;
}