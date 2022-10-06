#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <string>
#include <iostream>

int main() {
    try {
        SpartonAHRSM1Driver s("/dev/ttyUSB0", 115200);

        for (int i=0; i<1000; ++i) {
            std::vector<float> v = s.read_gyroscope();
            std::vector<float> w = s.read_magnetometer();
            std::vector<float> x = s.read_accelerometer();

            std::copy(v.begin(), v.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::copy(w.begin(), w.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::copy(x.begin(), x.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
        }
    }
    catch(boost::system::system_error& e) {
        std::cout<<"Error: "<<e.what()<<std::endl;
        return 1;
    }
}