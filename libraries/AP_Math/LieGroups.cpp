#include "AP_Math.h"
#include "LieGroups.h"

SIM23 SIM23::operator*(const SIM23& rhs) const
{
    return SIM23(
               _R * rhs._R,
               _W1 * rhs._A.a11() + _W2 * rhs._A.a21() + _R * rhs._W1,
               _W1 * rhs._A.a12() + _W2 * rhs._A.a22() + _R * rhs._W2,
               _A * rhs._A
           );
}

SIM23 SIM23::inverse() const
{
    const GL2 AInv = _A.inverse();
    const Matrix3F RT = _R.transposed();
    const Vector3F RTW1 = RT * _W1;
    const Vector3F RTW2 = RT * _W2;
    return SIM23(
               RT,
               -RTW1 * AInv.a11() - RTW2 * AInv.a21(),
               -RTW1 * AInv.a12() - RTW2 * AInv.a22(),
               AInv
           );
}

SIM23 SIM23::identity()
{
    Matrix3F R;
    R.identity();
    return SIM23(R, Vector3F(),Vector3F(), GL2::identity());
}

GL2 GL2::operator*(const GL2& rhs) const
{
    ftype c11 = _a11*rhs._a11 + _a12*rhs._a21;
    ftype c12 = _a11*rhs._a12 + _a12*rhs._a22;
    ftype c21 = _a21*rhs._a11 + _a22*rhs._a21;
    ftype c22 = _a21*rhs._a12 + _a22*rhs._a22;
    return GL2(c11,c12,c21,c22);
}
GL2 GL2::operator+(const GL2& rhs) const
{
    return GL2(
               _a11 + rhs._a11,
               _a12 + rhs._a12,
               _a21 + rhs._a21,
               _a22 + rhs._a22
           );
}
GL2 GL2::operator-(const GL2& rhs) const
{
    return GL2(
               _a11 - rhs._a11,
               _a12 - rhs._a12,
               _a21 - rhs._a21,
               _a22 - rhs._a22
           );
}

Vector2F GL2::operator*(const Vector2F& rhs) const
{
    ftype v1 = _a11*rhs.x + _a12*rhs.y;
    ftype v2 = _a21*rhs.x + _a22*rhs.y;
    return Vector2F(v1,v2);
}

GL2 GL2::inverse() const
{
    // Must have nonzero determinant!
    const ftype Dinv = 1.0/det();
    ftype c11 = Dinv * _a22;
    ftype c12 = - Dinv * _a12;
    ftype c21 = - Dinv * _a21;
    ftype c22 = Dinv * _a11;
    return GL2(c11,c12,c21,c22);
}

GL2 GL2::transposed() const
{
    return GL2(_a11,_a21,_a12,_a22);
}

ftype GL2::det() const
{
    return _a11 * _a22 - _a12 * _a21;
}

ftype GL2::trace() const
{
    return _a11 + _a22;
}

GL2 GL2::exponential(const GL2& S)
{
    const ftype& a = S._a11;
    const ftype& b = S._a12;
    const ftype& c = S._a21;
    const ftype& d = S._a22;


    ftype m11,m12,m21,m22;

    const ftype s = 0.5 * (a+d); // 0.5 trace(A)

    // Deal with real and complex eigenvalue cases
    const ftype eigendiff = (a-d)*(a-d) + 4*b*c;
    // Need to compute sinh(q) / q and cosh(q), where q = 0.5*sqrt(eigendiff)
    ftype shcq, chq;
    if (eigendiff >= 0.) {
        ftype q = 0.5*sqrtF(eigendiff);
        shcq = (q > 1e-6)? sinhF(q)/q : 1.;
        chq = coshF(q);
    } else {
        // q is now imaginary
        ftype q_im = 0.5*sqrtF(-eigendiff);
        shcq = (q_im > 1e-6) ? sinF(q_im) / q_im : 1.;
        chq = cosF(-q_im);
    }

    const ftype e = expF(s);

    // Result should be: exp(A) = exp(s) * ( (cosh(q) - s sinh(q)/q) I_2 + sinh(q)/q A )
    m11 = e * ((chq - s* shcq) + shcq * a);
    m12 = e * (shcq * b);
    m21 = e * (shcq * c);
    m22 = e * ((chq - s* shcq) + shcq * d);

    return GL2(m11,m12,m21,m22);
}


Gal3 Gal3::operator *(const Gal3& rhs) const
{
    Matrix3F rot_result = _rot * rhs._rot;
    rot_result.normalize();
    Vector3F x_result = _pos + _rot * rhs._pos + _vel*rhs._tau;
    Vector3F w_result = _vel + _rot * rhs._vel;
    ftype alpha_result = _tau + rhs._tau;
    return (Gal3(rot_result, x_result, w_result, alpha_result));
}
// Calculates the matrix exponential of Gal3, based on the working by Mr. Shawn (Yixiao) Ge
Gal3 Gal3::exponential(const Vector3F &S, const Vector3F &x,const Vector3F &w, ftype alpha)
{
    const ftype theta = S.length();
    Matrix3F input_matrix = Matrix3F::skew_symmetric(S); //example of using statics
    ftype A, B, C, D, E;
    if (theta > 0.0000001f) {
        A = sinF(theta) / theta;
        B = (1.0f - cosF(theta)) / (theta * theta);
        C = (1.0f - A) / (theta * theta);
        D = (theta - sinF(theta)) / (theta * theta * theta);
        E = (1.0f - 0.5f*theta*theta - cosF(theta)) / (theta * theta * theta*theta);
    } else {
        A = 1.0f;
        B = 0.5f;
        C = 1.0f / 6.0f;
        D = 1.0f / 24.0f;
        E = 1.0f / 120.0f;
    }
    //create Identity matrix
    Matrix3F identity_3;  // Declare an object of Matrix3
    identity_3.a.x = 1; identity_3.a.y = 0; identity_3.a.z = 0;  // Set the values to create an identity matrix
    identity_3.b.x = 0; identity_3.b.y = 1; identity_3.b.z = 0;
    identity_3.c.x = 0; identity_3.c.y = 0; identity_3.c.z = 1;
    //Create intermeadiatries for results
    Matrix3F result_rot = identity_3 + input_matrix*A + input_matrix*input_matrix*B;
    Matrix3F V = identity_3 + input_matrix*B + input_matrix*input_matrix*C;
    Matrix3F V_2 = identity_3*0.5f + input_matrix*D + input_matrix*input_matrix*E;
    //Assign results
    return (Gal3(result_rot, V*x + V_2*w*alpha, V*w, alpha));
}

Gal3 Gal3::inverse() const
{
    const Matrix3F RT = _rot.transposed();
    return Gal3(RT,
                -RT * _pos +  RT * _vel*_tau,
                -RT * _vel,
                -_tau);
}

Gal3 Gal3::identity()
{
    Matrix3F R;
    R.identity();
    return Gal3(R, Vector3F(),Vector3F(), 0.);
}