#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>


inline double sqr(double x)
{
	return x*x;
}

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif // !constrain

inline void quaternion2euler(double qw, double qx, double qy, double qz, double* pRoll, double* pPitch, double* pYaw)
{
	*pRoll = atan2(2*qy*qz+2*qw*qx, 2*sqr(qw)+2*sqr(qz)-1);
	*pPitch = -asin(constrain(2*qx*qz-2*qw*qy, -1, 1)); // Attempt to avoid potential NAN...
	*pYaw = atan2(2*qx*qy+2*qw*qz, 2*sqr(qw)+2*sqr(qx)-1);
}


int main() {
    SpartonAHRSM1Driver s("/dev/ttyUSB0", 115200);

    bool ret = s.init();

    if (!ret) {
        std::cerr << "Reset not successfull" << std::endl;
    }
    else {
        std::cout << "Reset successfull" << std::endl;
        while (true) {
            std::vector<double> z = s.read_quaternion();

            // std::copy(z.begin(), z.end(), std::ostream_iterator<float>(std::cout, ", "));
            // std::cout << std::endl;

            Eigen::Quaterniond q;
            q.w() = z[0];
            q.x() = z[1];
            q.y() = z[2];
            q.z() = z[3];

            q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * q * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());

            std::cout << q << std::endl;

            q.normalize();

            // std::cout << std::endl;
            // auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

            double yaw, pitch, roll;
            quaternion2euler(q.w(), q.x(), q.y(), q.z(), &roll, &pitch, &yaw);

            std::cout << "yaw: \t"  << yaw * 180. / M_PI << std::endl;
            std::cout << "pitch: \t" << pitch * 180. / M_PI <<  std::endl;
            std::cout << "roll: \t"  << roll * 180. / M_PI <<  std::endl;
            std::cout << std::endl;
        }
    }


    getchar();
}