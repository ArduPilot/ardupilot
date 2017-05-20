// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// scan I2C and SPI buses for expected devices
//
#include <iostream>
#include <AP_Eigen/AP_Eigen.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

using namespace Eigen;

void setup(void)
{
    Matrix2d a;
    a << 1, 2,
        3, 4;
    MatrixXd b(2,2);
    b << 2, 3,
        1, 4;
    std::cout << "a + b =\n" << a + b << std::endl;
    std::cout << "a - b =\n" << a - b << std::endl;
    std::cout << "Doing a += b;" << std::endl;
    a += b;
    std::cout << "Now a =\n" << a << std::endl;
    Vector3d v(1,2,3);
    Vector3d w(1,0,0);
    std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
}

void loop(void)
{

}

AP_HAL_MAIN();
