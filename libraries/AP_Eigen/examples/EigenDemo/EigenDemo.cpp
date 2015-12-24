/*
 * author: Daniel Frenzel <dgdanielf@gmail.com>
 */
#include <iostream>
#include <stdio.h>

#include <AP_Eigen/AP_Algebra.h>
#include <AP_Eigen/AP_Eigen.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


Eigen::Matrix3f rotate_a_to_b_i (const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
    Eigen::Vector3f a_norm = a.normalized();
    Eigen::Vector3f b_norm = b.normalized();

    // Find axis and angle using cross product and ..
    Eigen::Vector3f x = a_norm.cross(b_norm);
    
    // .. the dot product
    float theta = acosf( a_norm.dot(b_norm) );

    // A is a skew-symmetric matrix corresponding to x
    Eigen::Matrix3f A;
    A << 0, -x.z(),  x.y(),  x.z(),  0.f,  -x.x(), -x.y(), x.x(),  0.f;

    // Find rotation matrix using exponential map
    return Eigen::Matrix3f::Identity() + sinf(theta) * A + ( (1 - cosf(theta) ) * (A*A) );
}

Matrix3f rotate_a_to_b_ii (const Vector3f &a, const Vector3f &b) {
    Vector3f a_norm = a.normalized();
    Vector3f b_norm = b.normalized();

    // Find axis and angle using cross product and ..
    Vector3f x = a_norm % b_norm;

    // .. the dot product
    float theta = acosf( a_norm * b_norm );

    // A is a skew-symmetric matrix corresponding to x
    Matrix3f A(0.f,    -x.z,   x.y,
               x.z,    0.f,    -x.x,
               -x.y,   x.x,    0.f);
    
    Matrix3f I; I.identity();
    
    // Find rotation matrix using exponential map
    return I + A * sinf(theta) + ( (A*A) * (1 - cosf(theta) ) );
}

// Memory
void setup(void) {
    float fTest = 0.5;
    printf("save_sin: %f\n", safe_asin(fTest));
    
    
    printf("eucl_distance: %f\n", squared_distance(2.f,2.f));
    printf("eucl_distance: %f\n", squared_distance(3.f,3.f));
    printf("eucl_distance: %f\n", squared_distance(1.f,2.f,3.f,4.f,5.f));

    
    /*
     * Test type size
     */
    int PV = sizeof(Eigen::Vector3f);
    int PM = sizeof(Eigen::Matrix3f);
  
    int EV = sizeof(Eigen::Vector3f);
    int EM = sizeof(Eigen::Matrix3f);
    
    int AV = sizeof(Vector3f);
    int AM = sizeof(Matrix3f);
    
    printf("Eigen:\t vector: %d\tmatrix: %d\n",  PV, PM);
    printf("Intf:\t vector: %d\tmatrix: %d\n",  EV, EM);
    printf("AP:\t vector: %d\tmatrix: %d\n",  AV, AM);
    
    /*
     * Test type operators
     */
    Eigen::Vector3f v1(0,0,0);
    Eigen::Vector3f v2(1,1,1);
    Eigen::Vector3f v3(2,2,2);
    
    std::cout << "v3 length: " << v3.norm() << std::endl;
    std::cout << "v3 length: " << v3.is_nan() << std::endl;
    std::cout << "v3 length: " << v3.is_inf() << std::endl;
    

    Matrix3f m_ap(8,7,6,
                  5,4,3,
                  2,1.6,0);
    
    /*
     * Test float to int conversion
     */
    Eigen::Vector3f v_float(1.4,2.5,3.7);
    Eigen::Vector3i v_int1 = Eigen::Vector3f::get_nearest_integral<int>(v_float);
    Eigen::Vector3i v_int2 = Eigen::Vector3i::get_integral<int>(v_float);
    
    std::cout << "v_int1: " << v_int1 <<std::endl;
    std::cout << "v_int2: " << v_int2 <<std::endl;
    std::cout << "v_int2: " << v_int2 <<std::endl;   
    
    Vector3f ap_vec(1,2,3);    
    Eigen::Vector3f eigen_vec = from_vector(ap_vec);
    std::cout << "conversion: " << eigen_vec <<std::endl; 

    /*
     * Test euler constrains
     */

    Eigen::Vector3f v_euler(360,359,362);
    v_euler.constrain_180_deg();
    std::cout << "euler constr: " << v_euler <<std::endl;

    Eigen::Vector3f v_euler2(360,359,362);
    v_euler2.constrain_360_deg();
    std::cout << "euler constr: " << v_euler2 <<std::endl;
    
    Eigen::Vector3f dummy = v_euler.constrained_360_deg();
    std::cout << "euler constr: " << dummy <<std::endl;
    dummy = v_euler.constrained_180_deg();
    std::cout << "euler constr: " << dummy <<std::endl;
    /*
     * Test floating numbers
     */
    is_zero(4.5);
}

// Benchmark
void loop(void)
{
    long timer1 = AP_HAL::millis();
    Eigen::Vector3f a(1,0,0);
    Eigen::Vector3f b(0,1,99);
    
    Eigen::Matrix3f m_i = rotate_a_to_b_i(a, b);
    Eigen::Vector3f first = (m_i * a);
    Eigen::Vector3f diff(0,0,0);
    
    for(int i = 0; i < 10000000; i++) {
        m_i = rotate_a_to_b_i(a, b);
        diff += (first - (m_i * a));
    }
    
    printf("Eigen:\t %f\t%f\t%f: %d\n",  diff.x(), diff.y(), diff.z(), (int)(AP_HAL::millis() - timer1));

    long timer2 = AP_HAL::millis();;
    Vector3f c(1,0,0);
    Vector3f d(0,1,99);
    
    Matrix3f m_ii = rotate_a_to_b_ii(c, d);
    Vector3f first_ii = (m_ii * c);
    Vector3f diff_ii(0,0,0);
    
    for(int i = 0; i < 10000000; i++) {
        m_ii = rotate_a_to_b_ii(c, d);
        diff_ii += (first_ii - (m_ii * c));
    }
    
    printf("AP:\t %f\t%f\t%f: %d\n",  diff_ii.x, diff_ii.y, diff_ii.z, (int)(AP_HAL::millis() - timer2));
}

AP_HAL_MAIN();
