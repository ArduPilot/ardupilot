#ifndef AP_INTF_H
#define AP_INTF_H

/*
 * The PX4 stack creates a lot of drama if just using C99 math features :D
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && CONFIG_HAL_BOARD != HAL_BOARD_LINUX // maybe more?!
    #pragma once
    #pragma GCC diagnostic push
        #define RAND_MAX __RAND_MAX
        #pragma GCC diagnostic ignored "-Wshadow"
        #pragma GCC diagnostic ignored "-Wfloat-equal"
        #define _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC 1

        #include <AP_Eigen/eigen/Eigen/Core>
        #include <AP_Eigen/eigen/Eigen/Dense>
    #pragma GCC diagnostic pop
#else // Native: Linux, SITL, ..
    #include <AP_Eigen/eigen/Eigen/Core>
    #include <AP_Eigen/eigen/Eigen/Dense>
#endif
    
#include <AP_Eigen/AP_Extensions.h>
#include <AP_Math/AP_Math.h>

/*
 * ArduPilot compatibility typedefs using the Eigen library
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */
namespace IntfDef {
    template <class T>
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    template <class T>
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    template <class T>
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
}

namespace IntfImp {
    template <class T>
    class VectorE2 : public IntfDef::Vector2<T> {
    public:
        const T &x() const { return (*this)[0]; }
        const T &y() const { return (*this)[1]; }
        // CTORs
        VectorE2()                                              : IntfDef::Vector2<T>(VectorE2::Zero(2)) { }
        template <class OtherDerived>
        VectorE2(const Eigen::MatrixBase<OtherDerived>& other)  : IntfDef::Vector2<T>(other) { }
        template <class OtherDerived>
        VectorE2(const Vector2<OtherDerived> &v)                : IntfDef::Vector2<T>(v.x, v.y) { }
        VectorE2(const T &sx, const T &sy)                      : IntfDef::Vector2<T>(sx, sy) { }

        // type operators
        template <class OtherDerived>
        operator Vector2<OtherDerived>() const {
            return Vector2<OtherDerived>((*this)[0], (*this)[1] );
        }
        VectorE2<T> &operator=(const VectorE2<T> &other) {
            if(this != &other) {
                (*this)[0] = other[0];
                (*this)[1] = other[1];
            }
            return *this;
        }
        VectorE2<T> &operator=(const Vector2<T> &other) {
            if(this != &other) {
                (*this)[0] = other.x;
                (*this)[1] = other.y;
            }
            return *this;
        }
        
        /*
         * @brief If <T> is a floating point type, 
         * this functions rounds to the closest integer and returns it as <T>.
         */
        template <class IntType, class FloatType>
        static VectorE2<IntType> get_nearest_integral(const VectorE2<FloatType> &other) {
            // just a warning that the rounding is useless
            if(!std::is_integral<IntType>::value || !std::is_floating_point<FloatType>::value) {
                printf("VectorE2::get_nearest_integral(): template parameters do not make sense\n");
                //std::cerr << "VectorE2::get_nearest_integral(): template parameters do not make sense" << std::endl;
            }
            
            IntType vx = safeFloatToInt<IntType>(round(other[0]));
            IntType vy = safeFloatToInt<IntType>(round(other[1]));
            return VectorE2<IntType>(vx, vy);
        }
        /*
         * @brief If <T> is a floating point type, 
         * this functions rounds always down and returns it as <T>.
         */
        template <class IntType, class FloatType>
        static VectorE2<IntType> get_integral(const VectorE2<FloatType> &other) {
            // just a warning that the rounding is useless
            if(!std::is_integral<IntType>::value || !std::is_floating_point<FloatType>::value) {
                printf("VectorE2::get_integral(): template parameters do not make sense\n");
                //std::cerr << "VectorE2::get_integral(): template parameters do not make sense" << std::endl;
            }
            
            IntType vx = safeFloatToInt<IntType>(other[0]);
            IntType vy = safeFloatToInt<IntType>(other[1]);
            return VectorE2<IntType>(vx, vy);
        }
        
        /*
         * @brief constrains the values of the matrix to fit within a given range: low and high
         */
        void constrain(const T &low, const T &high) {
            (*this)[0] = constrain_value<T>((*this)[0], low, high);
            (*this)[1] = constrain_value<T>((*this)[1], low, high);
        }
        
        /*
         * @brief if the matrix contains angles (euler, radian), they can be constrained to fit between 180° and 360°
         */
        void constrain(const int flag) {
            (*this)[0] = constrain_angle<T>((*this)[0], flag);
            (*this)[1] = constrain_angle<T>((*this)[1], flag);
        }
        
        /*
         * @brief constrains the values of the matrix to fit within a given range: low and high
         */
        VectorE2<T> constrained(const T &low, const T &high) const {
            VectorE2<T> tmp;
            tmp[0] = constrain_value<T>((*this)[0], low, high);
            tmp[1] = constrain_value<T>((*this)[1], low, high);
            return tmp;
        }
        /*
         * @brief if the matrix contains angles (euler, radian), they can be constrained to fit between 180° and 360°
         */
        VectorE2<T>  constrained(const int flag) const {
            VectorE2<T> tmp;
            tmp[0] = constrain_angle<T>((*this)[0], flag);
            tmp[1] = constrain_angle<T>((*this)[1], flag);
            return tmp;
        }
        
        // some functions
        float length() const {
            float sum = powf(x, 2) + powf(y, 2);
            return sqrt(sum);
        }
        bool is_nan(void) {
            return isnan((*this)[0]) || isnan((*this)[1]);
        }
        bool is_inf(void) {
            return isinf((*this)[0]) || isinf((*this)[1]);
        }
        float angle(const VectorE2<T> &v) const {
            VectorE2<T> v1 = this->normalized();
            VectorE2<T> v2 = v.normalized();
            return acosf(v1.dot(v2) );
        }
    };

    template <class T>
    class VectorE3 : public IntfDef::Vector3<T> {
    public:
        const T &x() const { return (*this)[0]; }
        const T &y() const { return (*this)[1]; }
        const T &z() const { return (*this)[2]; }

        VectorE3()                                               : IntfDef::Vector3<T>(VectorE3::Zero(3)) { }
        template <class OtherDerived>
        VectorE3(const Eigen::MatrixBase<OtherDerived>& other)   : IntfDef::Vector3<T>(other) { }
        template <class OtherDerived>
        VectorE3(const Vector3<OtherDerived> &v)                 : IntfDef::Vector3<T>(v.x, v.y, v.z) { }
        VectorE3(const T &sx, const T &sy, const T &sz)          : IntfDef::Vector3<T>(sx, sy, sz) { }


        template <class OtherDerived>
        operator Vector3<OtherDerived>() const {
            return Vector3<OtherDerived>((*this)[0], (*this)[1], (*this)[2]);
        }
        VectorE3<T> &operator=(const VectorE3<T> &other) {
            (*this)[0] = other[0];
            (*this)[1] = other[1];
            (*this)[2] = other[2];
            return *this;
        }
        VectorE3<T> &operator=(const Vector3<T> &other) {
            (*this)[0] = other.x;
            (*this)[1] = other.y;
            (*this)[2] = other.z;
            return *this;
        }

        /*
         * @brief If <T> is a floating point type, 
         * this functions rounds to the closest integer and returns it as <T>.
         */
        template <class IntType, class FloatType>
        static VectorE3<IntType> get_nearest_integral(const VectorE3<FloatType> &other) {
            // just a warning that the rounding is useless
            if(!std::is_integral<IntType>::value || !std::is_floating_point<FloatType>::value) {
                printf("VectorE3::get_nearest_integral(): template parameters do not make sense\n");
                //std::cerr << "VectorE3::get_nearest_integral(): template parameters do not make sense" << std::endl;
            }
            
            IntType vx = safeFloatToInt<IntType>(round(other[0]) );
            IntType vy = safeFloatToInt<IntType>(round(other[1]) );
            IntType vz = safeFloatToInt<IntType>(round(other[2]) );
            return VectorE3<IntType>(vx, vy, vz);
        }
        /*
         * @brief If <T> is a floating point type, 
         * this functions rounds always down and returns it as <T>.
         */
        template <class IntType, class FloatType>
        static VectorE3<IntType> get_integral(const VectorE3<FloatType> &other) {
            // just a warning that the rounding is useless
            if(!std::is_integral<IntType>::value || !std::is_floating_point<FloatType>::value) {
                printf("VectorE3::get_integral(): template parameters do not make sense\n");
                //std::cerr << "VectorE3::get_integral(): template parameters do not make sense" << std::endl;
            }
            
            IntType vx = safeFloatToInt<IntType>(other[0] );
            IntType vy = safeFloatToInt<IntType>(other[1] );
            IntType vz = safeFloatToInt<IntType>(other[2] );
            return VectorE3<IntType>(vx, vy, vz);
        }
        
        /*
         * @brief constrains the values of the matrix to fit within a given range: low and high
         */
        void constrain(const T &low, const T &high) {
            (*this)[0] = constrain_value<T>((*this)[0], low, high);
            (*this)[1] = constrain_value<T>((*this)[1], low, high);
            (*this)[2] = constrain_value<T>((*this)[2], low, high);
        }
        /*
         * @brief if the matrix contains angles (euler, radian), they can be constrained to fit between 180° and 360°
         */
	//template <ANGLE_TYPE ANG = EULER, ANGLE_RANGE RNG = DEG_180>
        void constrain(const int flag) {
            (*this)[0] = constrain_angle<T>((*this)[0], flag);
            (*this)[1] = constrain_angle<T>((*this)[1], flag);
	    (*this)[2] = constrain_angle<T>((*this)[2], flag);
        }
        
        /*
         * @brief constrains the values of the matrix to fit within a given range: low and high
         */
        VectorE3<T> constrained(const T &low, const T &high) const {
            VectorE3<T> tmp;
            tmp[0] = constrain_value<T>((*this)[0], low, high);
            tmp[1] = constrain_value<T>((*this)[1], low, high);
            tmp[2] = constrain_value<T>((*this)[2], low, high);
            return tmp;
        }
        /*
         * @brief if the matrix contains angles (euler, radian), they can be constrained to fit between 180° and 360°
         */
        VectorE3<T>  constrained(const int flag) const {
            VectorE3<T> tmp;
            tmp[0] = constrain_angle<T>((*this)[0], flag);
            tmp[1] = constrain_angle<T>((*this)[1], flag);
            tmp[2] = constrain_angle<T>((*this)[2], flag);
            return tmp;
        }
        
        float length() const {
            float sum = powf((*this)[0], 2) + powf((*this)[1], 2) + powf((*this)[2], 2);
            return sqrt(sum);
        }
        bool is_nan(void) {
            return isnanf((*this)[0]) || isnanf((*this)[1]) || isnanf((*this)[2]);
        }
        bool is_inf(void) {
            return isinf((*this)[0]) || isinf((*this)[2]) || isinf((*this)[2]);
        }
        float angle(const VectorE3<T> &v) const {
            VectorE3<T> v1 = this->normalized();
            VectorE3<T> v2 = v.normalized();
            return acosf(v1.dot(v2) );
        }
    };

    template <class T>
    class MatrixE3 : public IntfDef::Matrix3<T> {        
    public:
        MatrixE3()                                             : IntfDef::Matrix3<T>(MatrixE3::Zero(3,3)) { }
        template <class OtherDerived>
        MatrixE3(const Eigen::MatrixBase<OtherDerived>& other) : IntfDef::Matrix3<T>(other) { }
        template <class OtherDerived>
        MatrixE3(const Matrix3<OtherDerived>& m)               : IntfDef::Matrix3<T>() {
            (*this) <<  m.a.x, m.a.y, m.a.z,
                        m.b.x, m.b.y, m.b.z,
                        m.c.x, m.c.y, m.c.z;
        }
        MatrixE3(T ax, T ay, T az, T bx, T by, T bz, T cx, T cy, T cz) : IntfDef::Matrix3<T>() {          
            (*this) <<  ax, ay, az,
                        bx, by, bz,
                        cx, cy, cz;
        }

        template <class OtherDerived>
        operator Matrix3<OtherDerived>() const {
            OtherDerived ax = (*this)(0,0);
            OtherDerived ay = (*this)(0,1);
            OtherDerived az = (*this)(0,2);

            OtherDerived bx = (*this)(1,0);
            OtherDerived by = (*this)(1,1);
            OtherDerived bz = (*this)(1,2);

            OtherDerived cx = (*this)(2,0);
            OtherDerived cy = (*this)(2,1);
            OtherDerived cz = (*this)(2,2);
            
            return Matrix3<OtherDerived>(ax, ay, az, 
                                         bx, by, bz, 
                                         cx, cy, cz);
        }        
        MatrixE3<T> &operator=(const Matrix3<T> &other) {
            (*this)(0,0) = other.a.x;
            (*this)(0,1) = other.a.y;
            (*this)(0,2) = other.a.z;
            
            (*this)(1,0) = other.b.x;
            (*this)(1,1) = other.b.y;
            (*this)(1,2) = other.b.z;
            
            (*this)(2,0) = other.c.x;
            (*this)(2,1) = other.c.y;
            (*this)(2,2) = other.c.z;

            return *this;
        }
        
        /*
         * @brief If <T> is a floating point type, 
         * this functions rounds to the closest integer and returns it as <T>.
         */
        template <class IntType, class FloatType>
        static MatrixE3<IntType> get_nearest_integral(const MatrixE3<FloatType> &other) {
            // just a warning that the rounding is useless
            if(!std::is_integral<IntType>::value || !std::is_floating_point<FloatType>::value) {
                printf("MatrixE3::get_nearest_integral(): template parameters do not make sense\n");
                //std::cerr << "MatrixE3::get_nearest_integral(): template parameters do not make sense" << std::endl;
            }
            
            MatrixE3<IntType> tmp = MatrixE3<IntType>::Zero(3,3);
            for(int y = 0; y < other.rows(); y++) {
                for(int x = 0; x < other.cols(); x++) {
                    tmp(y,x) = safeFloatToInt<IntType>(round(other(y,x)) );
                }
            }
            return tmp;
        }
        /*
         * @brief If <T> is a floating point type, 
         * this functions rounds always down and returns it as <T>.
         */
        template <class IntType, class FloatType>
        static MatrixE3<IntType> get_integral(const MatrixE3<FloatType> &other) {
            // just a warning that the rounding is useless
            if(!std::is_integral<IntType>::value || !std::is_floating_point<FloatType>::value) {
                printf("MatrixE3::get_integral(): template parameters do not make sense\n");
                //std::cerr << "MatrixE3::get_integral(): template parameters do not make sense" << std::endl;
            }
            
            MatrixE3<IntType> tmp = MatrixE3<IntType>::Zero(3,3);
            for(int y = 0; y < other.rows(); y++) {
                for(int x = 0; x < other.cols(); x++) {
                    tmp(y,x) = safeFloatToInt<IntType>(other(y,x) );
                }
            }
            return tmp;
        }
        
        /*
         * @brief constrains the values of the matrix to fit within a given range: low and high
         */
        void constrain(const T &low, const T &high) {
            for(int y = 0; y < this->rows(); y++) {
                for(int x = 0; x < this->cols(); x++) {
                    (*this)(y,x) = constrain_value<T>((*this)(y,x), low, high);
                }
            }
        }
        /*
         * @brief if the matrix contains angles (euler, radian), they can be constrained to fit between 180° and 360°
         */
        void constrain(const int flag) {
            for(int y = 0; y < this->rows(); y++) {
                for(int x = 0; x < this->cols(); x++) {
                    (*this)(y,x) = constrain_angle<T>((*this)(y,x), flag);
                }
            }
        }
        
        /*
         * @brief constrains the values of the matrix to fit within a given range: low and high
         */
        MatrixE3<T> constrained(const T &low, const T &high) const {
            MatrixE3<T> tmp;
            for(int y = 0; y < this->rows(); y++) {
                for(int x = 0; x < this->cols(); x++) {
                    tmp(y,x) = constrain_value<T>((*this)(y,x), low, high);
                }
            }
        } 
        /*
         * @brief if the matrix contains angles (euler, radian), they can be constrained to fit between 180° and 360°
         */
        MatrixE3<T> constrained(const int flag) const {
            MatrixE3<T> tmp;
            for(int y = 0; y < this->rows(); y++) {
                for(int x = 0; x < this->cols(); x++) {
                    tmp(y,x) = constrain_angle<T>((*this)(y,x), flag);
                }
            }
        }
        
        bool is_nan(void) {
            for(int y = 0; y < this->rows(); y++) {
                for(int x = 0; x < this->cols(); x++) {
                    if(isnanf((*this)(y,x) ) ) {
                        return true;
                    }
                }
            }
            return false;
        }
        bool is_inf(void) {
            for(int y = 0; y < this->rows(); y++) {
                for(int x = 0; x < this->cols(); x++) {
                    if(isinf((*this)(y,x) ) ) {
                        return true;
                    }
                }
            }
            return false;
        }
    };
};

/**
 * @brief Convinient types
 */
namespace AP_Eigen {
    typedef IntfImp::VectorE2<float>    Vector2f;
    typedef IntfImp::VectorE2<double>   Vector2d;
    typedef IntfImp::VectorE2<int>      Vector2i;
    typedef IntfImp::VectorE2<long>     Vector2l;

    typedef IntfImp::VectorE3<float>    Vector3f;
    typedef IntfImp::VectorE3<double>   Vector3d;
    typedef IntfImp::VectorE3<int>      Vector3i;
    typedef IntfImp::VectorE3<long>     Vector3l;

    typedef IntfImp::MatrixE3<float>    Matrix3f;
    typedef IntfImp::MatrixE3<double>   Matrix3d;
    typedef IntfImp::MatrixE3<int>      Matrix3i;
    typedef IntfImp::MatrixE3<long>     Matrix3l;
}

#endif //AP_INTF_H