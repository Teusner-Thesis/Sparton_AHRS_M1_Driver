#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>

#include <sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp>
#include <eigen3/Eigen/Core>


int main(int argc, char **argv) {
    SpartonAHRSM1Driver driver("/dev/IMU", 115200);

    bool ret = driver.init();
    if (!ret) {
        std::cerr << "Driver initialization unsuccessfull";
        return -1;
    }

    std::ofstream file;
    file.open("raw_data.csv");
    file << "#time,mx,my,mz,ax,ay,az\n";

    auto t0 = std::chrono::system_clock::now();

    // desired frame rate
    typedef std::chrono::duration<int, std::ratio<1, 50>> frame_duration;

    while (true) {
        auto t1 = std::chrono::system_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        std::vector<double> m;
        m = driver.read_magnetometer();
        std::vector<double> a;
        a = driver.read_accelerometer();
        std::vector<double> g;
        g = driver.read_gyroscope();
        std::vector<double> q;
        q = driver.read_quaternion();
        file << time / 1000. << ","  << m[0] << "," << m[1] << "," << m[2] << " " << a[0] << "," << a[1] << "," << a[2] << "," << g[0] << "," << g[1] << "," << g[2] << "," << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "\n";

        auto end_time = t1 + frame_duration(1);
        std::this_thread::sleep_until(end_time);
    }
}