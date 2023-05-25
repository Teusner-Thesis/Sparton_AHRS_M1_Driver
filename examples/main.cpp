#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <string>
#include <iostream>

int main() {
    SpartonAHRSM1Driver s("/dev/ttyUSB0", 115200);

    bool ret = s.reset();

    if (!ret) {
        std::cerr << "Reset not successfull" << std::endl;
    }
    else {
        std::cout << "Reset successfull" << std::endl;
        for (int i=0; i<1000; ++i) {
            std::vector<float> v = s.read_gyroscope();
            std::vector<float> w = s.read_magnetometer();
            std::vector<float> x = s.read_accelerometer();
            std::vector<float> z = s.read_quaternion();

            std::copy(v.begin(), v.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::copy(w.begin(), w.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::copy(x.begin(), x.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::copy(z.begin(), z.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
        }
    }


    getchar();
}