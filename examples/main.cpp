#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <string>
#include <iostream>

int main() {
    SpartonAHRSM1Driver s("/dev/ttyUSB0", 115200);

    bool ret = s.init();

    if (!ret) {
        std::cerr << "Init not successfull" << std::endl;
    }
    else {
        std::cout << "Init successfull" << std::endl;
        while (true) {
            double a = s.read_accelerometer_variance();
            double b = s.read_gyroscope_variance();
            std::vector<double> w = s.read_magnetometer();
            std::vector<double> x = s.read_accelerometer();
            std::vector<double> v = s.read_gyroscope();
            std::vector<double> z = s.read_quaternion();

            std::cout << "Acceleration ";
            std::copy(v.begin(), v.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::cout << "Accelerometer variance " << a << std::endl;
            std::cout << "Magnetometer ";
            std::copy(w.begin(), w.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::cout << "Gyroscope ";
            std::copy(x.begin(), x.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
            std::cout << "Gyroscope variance " << b << std::endl;
            std::cout << "Quaternion ";
            std::copy(z.begin(), z.end(), std::ostream_iterator<float>(std::cout, ", "));
            std::cout << std::endl;
        }
    }

    getchar();
}