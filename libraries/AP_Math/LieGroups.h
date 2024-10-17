#pragma once

#include "AP_Math.h"
#include "ftype.h"
#include "vector3.h"
#include "vector2.h"
#include "matrix3.h"

class GL2
{
public:
    GL2(): _a11(1.), _a12(0.), _a21(0.), _a22(1.) {}
    GL2(ftype a11, ftype a12, ftype a21, ftype a22) : _a11(a11), _a12(a12), _a21(a21), _a22(a22) {}

    friend GL2 operator*(const ftype& k, const GL2& A)
    {
        return GL2(k*A._a11,k*A._a12,k*A._a21,k*A._a22);
    }
    GL2 operator*(const GL2& rhs) const;
    GL2 operator+(const GL2& rhs) const;
    GL2 operator-(const GL2& rhs) const;
    Vector2F operator*(const Vector2F& rhs) const;
    GL2 inverse() const;
    GL2 transposed() const;
    ftype det() const;
    ftype trace() const;

    static GL2 identity()
    {
        return GL2(1.,0.,0.,1.);
    };
    static GL2 exponential(const GL2& S);

    const ftype& a11() const
    {
        return _a11;
    }
    const ftype& a12() const
    {
        return _a12;
    }
    const ftype& a21() const
    {
        return _a21;
    }
    const ftype& a22() const
    {
        return _a22;
    }


private:
    ftype _a11;
    ftype _a12;
    ftype _a21;
    ftype _a22;
};



class Gal3
{
public:
    // Default constructor
    Gal3()
        : _rot(), _pos(), _vel(), _tau(0.0f)
    {

    }

    Gal3(const Matrix3F& rot, const Vector3F& pos, const Vector3F& vel, const ftype& tau)
        : _rot(rot), _pos(pos), _vel(vel), _tau(tau)
    {

    }

    // Matrix multiplication
    Gal3 operator *(const Gal3& rhs) const;
    Gal3 inverse() const;

    static Gal3 identity();

    //Matrix Exponential
    static Gal3 exponential(const Vector3F &drot, const Vector3F &dpos, const Vector3F &dvel, ftype dtau); //needs to be static
    //Inverse of the ZHat Matrix used in CINS
    Matrix3F& rot()
    {
        return _rot;
    }
    const Matrix3F& rot() const
    {
        return _rot;
    }
    const Vector3F& pos() const
    {
        return _pos;
    }
    const Vector3F& vel() const
    {
        return _vel;
    }
    ftype tau() const
    {
        return _tau;
    }

    void init(Matrix3F &rot)
    {
        _rot = rot;
    }

private:
    Matrix3F _rot;
    Vector3F _pos;
    Vector3F _vel;
    ftype _tau;
};


class SIM23
{
public:
    SIM23() = default;
    SIM23(const Matrix3F& R, const Vector3F& W1, const Vector3F& W2, const GL2& A) : _R(R), _W1(W1), _W2(W2), _A(A) {}
    SIM23(const Gal3& X) : _R(X.rot()), _W1(X.vel()), _W2(X.pos()), _A(1.,X.tau(), 0., 1.) {}

    SIM23 operator*(const SIM23& rhs) const;
    SIM23 inverse() const;

    static SIM23 identity();

    const Matrix3F& R() const
    {
        return _R;
    }
    const Vector3F & W1() const
    {
        return _W1;
    }
    const Vector3F & W2() const
    {
        return _W2;
    }
    const GL2& A() const
    {
        return _A;
    }
    Matrix3F& R()
    {
        return _R;
    }
    Vector3F & W1()
    {
        return _W1;
    }
    Vector3F & W2()
    {
        return _W2;
    }
    GL2& A()
    {
        return _A;
    }


private:
    Matrix3F _R;
    Vector3F _W1;
    Vector3F _W2;
    GL2 _A;
};

typedef Gal3 Gal3F;
