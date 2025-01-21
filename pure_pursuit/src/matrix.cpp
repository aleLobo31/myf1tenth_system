#include <iostream>
#include <eigen3/Eigen/Dense>

int main()
{
    Eigen::Matrix3d R;
    Eigen::Vector3d v;
    Eigen::Vector3d s;

    R << 1.5, 2.0, 3.1, 4.2, 5.1, 6.1, 7.1, 8.1, 9.1;
    v << 1.1, 2.2, 3.3;

    v << 0.0, 0.0, 0.0;

    s = R * v;

    int mod = 125%100;

    // std::cout << "Solution: " << s << std::endl;

    std::cout << "Rtdo: " << mod <<std::endl;

    return 0;
}