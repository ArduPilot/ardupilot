/*
 * ArduPilot compatibility typedefs using the Eigen library
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */

#ifndef AP_INTF_H
#define AP_INTF_H

#include <AP_Eigen/AP_Algebra.h>
#include <AP_Eigen/AP_Eigen.h>
#include <AP_Math/AP_Math.h>


namespace AP_Eigen {
    template <class T> using CompatBase_Vector2 = Eigen::Matrix<T, 2, 1>;
    template <class T> using CompatBase_Vector3 = Eigen::Matrix<T, 3, 1>;
    template <class T> using CompatBase_Matrix3 = Eigen::Matrix<T, 3, 3>;
  
    template <class T>
    class Compat_Vector2 : public CompatBase_Vector2<T> {
    public:
        const T &x() const;
        const T &y() const;
        
        Compat_Vector2();
        Compat_Vector2(const T &sx, const T &sy);
        template <class OtherDerived> Compat_Vector2(const Eigen::MatrixBase<OtherDerived>& other);
        template <class OtherDerived> Compat_Vector2(const Vector2<OtherDerived> &v);

        template <class OtherDerived> operator Vector2<OtherDerived>() const;
        Compat_Vector2<T> &operator=(const Vector2<T> &other);
    };

    template <class T>
    class Compat_Vector3 : public CompatBase_Vector3<T> {
    public:
        const T &x() const;
        const T &y() const;
        const T &z() const;

        Compat_Vector3();
        Compat_Vector3(const T &sx, const T &sy, const T &sz);
        template <class OtherDerived> Compat_Vector3(const Eigen::MatrixBase<OtherDerived>& other);
        template <class OtherDerived> Compat_Vector3(const Vector3<OtherDerived> &v);
        
        template <class OtherDerived> operator Vector3<OtherDerived>() const;
        Compat_Vector3<T> &operator=(const Vector3<T> &other);
    };

    template <class T>
    class Compat_Matrix3 : public CompatBase_Matrix3<T> {        
    public:
        Compat_Matrix3();
        Compat_Matrix3(T ax, T ay, T az, T bx, T by, T bz, T cx, T cy, T cz);
        template <class OtherDerived> Compat_Matrix3(const Eigen::MatrixBase<OtherDerived>& other);
        template <class OtherDerived> Compat_Matrix3(const Matrix3<OtherDerived>& m);

        template <class OtherDerived> operator Matrix3<OtherDerived>() const;
        Compat_Matrix3<T> &operator=(const Matrix3<T> &other);
    };
};

/*
 * Implementation of the compatibility classes
 * I don't want to implement type initializers, so I include like this:
 */
#include "AP_Compat.imp"    

/**
 * @brief Convinient types
 */
namespace AP_Eigen {
    typedef Compat_Vector2<float>    Compat_Vector2f;
    typedef Compat_Vector2<double>   Compat_Vector2d;
    typedef Compat_Vector2<int>      Compat_Vector2i;
    typedef Compat_Vector2<long>     Compat_Vector2l;

    typedef Compat_Vector3<float>    Compat_Vector3f;
    typedef Compat_Vector3<double>   Compat_Vector3d;
    typedef Compat_Vector3<int>      Compat_Vector3i;
    typedef Compat_Vector3<long>     Compat_Vector3l;

    typedef Compat_Matrix3<float>    Compat_Matrix3f;
    typedef Compat_Matrix3<double>   Compat_Matrix3d;
    typedef Compat_Matrix3<int>      Compat_Matrix3i;
    typedef Compat_Matrix3<long>     Compat_Matrix3l;
}

#endif //AP_INTF_H