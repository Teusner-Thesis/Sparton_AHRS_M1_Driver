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
        SpartonAHRSM1Driver(std::string port, unsigned int baud_rate, bool call_init=false) : serial_(rtac::asio::Stream::CreateSerial(port, baud_rate)) {
            serial_->start();
            serial_->enable_io_dump("./rx.log", "./tx.log");
            if (call_init) {
                init();
            }
        };

        /**
        * Reset the AHRS (use with precautions)
        * \return bool reset successful
        * \throws boost::system::system_error on failure
        */
        void reset();

        /**
        * Initialize the AHRS with value getters
        * \return bool initialize successful
        * \throws boost::system::system_error on failure
        */
        bool init();

        /**
        * Enables AHRS echo
        * \return bool initialize successful
        * \throws boost::system::system_error on failure
        */
        void enable_echo();

        /**
        * Disables AHRS echo
        * \return bool initialize successful
        * \throws boost::system::system_error on failure
        */
        void disable_echo();

        /**
        * Read values from accelerometer.
        * \return std::vector<double> accelerations along x, y, z axis
        * the result is in (m/s^2)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<double> read_accelerometer();

        /**
        * Read accelerometer variance.
        * \return double accelerometer variance
        * the result is in (rad/s)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        double read_accelerometer_variance();

        /**
        * Read values from magnetometer.
        * \return std::vector<double> magnetic flux along x, y, z axis
        * the result is in (T)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<double> read_magnetometer();

        /**
        * Read values from gyroscope.
        * \return std::vector<double> angular velocity along x, y, z axis
        * the result is in (rad/s)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<double> read_gyroscope();
        
        /**
        * Read gyroscope variance.
        * \return double gyroscope variance
        * the result is in (rad/s)
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        double read_gyroscope_variance();

        /**
        * Read atitude values from imu.
        * \return std::vector<double> quaternion along x, y, z, w coordinates
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        std::vector<double> read_quaternion();

    private:
        rtac::asio::Stream::Ptr serial_;

        /**
        * Write a std::string and check the acknowledgment
        * \return bool the acknowledgment
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        bool writeAck(std::vector<std::string> s);

        /**
        * Forget a defined function in init and check the acknowledgment
        * \return bool the acknowledgment
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        bool forgetFunction(std::string function_name);
};