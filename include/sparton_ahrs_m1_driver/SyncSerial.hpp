#pragma once

#include <boost/asio.hpp>
#include <string>

class SyncSerial {
    public:

        enum FlushType {
            FlushReceive = TCIFLUSH,
            FlushSend    = TCOFLUSH,
            FlushBoth    = TCIOFLUSH
        };

        /**
        * Constructor.
        * \param port device name, example "/dev/ttyUSB0" or "COM4"
        * \param baud_rate communication speed, example 9600 or 115200
        * \throws boost::system::system_error if cannot open the
        * serial device
        */
        SyncSerial(std::string port, unsigned int baud_rate) : io(), serial_(io, port) {
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        };

        /**
        * Write a string to the serial device.
        * \param s string to write
        * \throws boost::system::system_error on failure
        */
        void writeString(std::string s) {
            boost::asio::write(serial_, boost::asio::buffer(s.c_str(), s.size()));
        };

        /**
        * Blocks until a line is received from the serial device.
        * Eventual '\n' or '\r\n' characters at the end of the string are removed.
        * \return a string containing the received line
        * \throws boost::system::system_error on failure
        */
        std::string readLine() {
            //Reading data char by char, code is optimized for simplicity, not speed
            using namespace boost;
            char c;
            std::string result;
            for(;;)
            {   
                asio::read(serial_, asio::buffer(&c,1));
                switch(c)
                {
                    case '\r':
                        break;
                    case '\n':
                        return result;
                    default:
                        result+=c;
                }
            }
        }

        boost::system::error_code flush(FlushType flushType = FlushBoth) {
            if(::tcflush(serial_.lowest_layer().native_handle(), flushType) == 0) {
                return boost::system::error_code();
            }
            else {
                return boost::system::error_code(errno, boost::asio::error::get_system_category());
            }
        }

    private:
        boost::asio::io_service io;
        boost::asio::serial_port serial_;
        boost::asio::streambuf read_buffer_;
};