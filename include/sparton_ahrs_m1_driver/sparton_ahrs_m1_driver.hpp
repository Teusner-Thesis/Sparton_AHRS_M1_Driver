#pragma once

#include "sparton_ahrs_m1_driver/SyncSerial.hpp"

#include <boost/asio.hpp>
#include <string>
#include <iostream>


class SpartonAHRSM1Driver {

    public:
        /**
        * Constructor.
        * \param port device name, example "/dev/ttyUSB0" or "COM4"
        * \param baud_rate communication speed, example 9600 or 115200
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        SpartonAHRSM1Driver(std::string port, unsigned int baud_rate) : sync_serial_(port, baud_rate) {
            reset();
        };

        /**
        * Reset and initialize the AHRS with value getters
        * \throws boost::system::system_error on failure
        */
        void reset();

        /**
        * Read values from accelerometer.
        * \return std::vector<float> accelerations along x, y, z axis
        * the result is in (m/s^2)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<float> read_accelerometer();

        /**
        * Read values from magnetometer.
        * \return std::vector<float> magnetic flux along x, y, z axis
        * the result is in (T)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<float> read_magnetometer();

        /**
        * Read values from gyroscope.
        * \return std::vector<float> angular velocity along x, y, z axis
        * the result is in (rad/s)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<float> read_gyroscope();

    private:
        SyncSerial sync_serial_;        
};