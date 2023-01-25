#pragma once

#include <string>
#include <iostream>

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>


class SpartonAHRSM1Driver {

    public:
        /**
        * Constructor.
        * \param port device name, example "/dev/ttyUSB0" or "COM4"
        * \param baud_rate communication speed, example 9600 or 115200
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        SpartonAHRSM1Driver(std::string port, unsigned int baud_rate, bool reset_on_init=false) : serial_(rtac::asio::Stream::CreateSerial(port, baud_rate)) {
            serial_->start();
            if (reset_on_init)
                reset();
        };

        /**
        * Reset and initialize the AHRS with value getters
        * \return bool reset successful
        * \throws boost::system::system_error on failure
        */
        bool reset();

        /**
        * Write a std::string and check the acknowledgment
        * \return bool the acknowledgment
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        bool writeAck(std::string s);

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
        rtac::asio::Stream::Ptr serial_;       
};