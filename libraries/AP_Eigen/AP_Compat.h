#ifndef COMPAT_H
#define COMPAT_H

#include <AP_Math/AP_Math.h>
#include <AP_Eigen/AP_Eigen.h>


template <class T>
Eigen::Matrix<T, 3, 3> from_matrix(const Matrix3<T> &other) {
    Eigen::Matrix<T, 3, 3> tmp;
  
    tmp(0,0) = other.a.x;
    tmp(0,1) = other.a.y;
    tmp(0,2) = other.a.z;
    
    tmp(1,0) = other.b.x;
    tmp(1,1) = other.b.y;
    tmp(1,2) = other.b.z;
    
    tmp(2,0) = other.c.x;
    tmp(2,1) = other.c.y;
    tmp(2,2) = other.c.z;

    return tmp;
}

template <class T>
Eigen::Matrix<T, 2, 1> from_vector(const Vector2<T> &other) {
    Eigen::Matrix<T, 2, 1> tmp;
  
    tmp[0] = other.x;
    tmp[1] = other.y;
 
    return tmp;
}

template <class T>
Eigen::Matrix<T, 3, 1> from_vector(const Vector3<T> &other) {
    Eigen::Matrix<T, 3, 1> tmp;
  
    tmp[0] = other.x;
    tmp[1] = other.y;
    tmp[2] = other.z;
 
    return tmp;
}

#endif // COMPAT_H