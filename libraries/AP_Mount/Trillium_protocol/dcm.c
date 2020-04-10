#include "dcm.h"
#include "Constants.h"
#include <math.h>

#include <string.h>

/*!
 * Get a specific element of a DCM
 * \param M is the DCM pointer
 * \param row is the row index
 * \param col is the column index
 * \return the value at <row, col>
 */
#define get(M, row, col) (M->data[(row)*3 + (col)])

/*!
 * Set a specific element of a DCM
 * \param M is the DCM pointer
 * \param row is the row index
 * \param col is the column index
 * \value is the new value to place at <row, col>
 */
#define set(M, row, col, value) (get(M, row, col) = (value))


/*!
 * Get a specific element of a dcm
 * \param M is the dcm pointer
 * \param row is the row index
 * \param col is the column index
 * \return the value at <row, col>
 */
float dcmGet(const DCM_t* M, uint32_t row, uint32_t col)
{
    return get(M, row, col);
}


/*! Get a specific element of a dcm, when all you have is the raw data
 * \param data is the dcm data pointer
 * \param row is the row index
 * \param col is the column index
 * \return the value at <row, col>
 */
float dcmGetFromRawData(const float* data, uint32_t row, uint32_t col)
{
	return data[row*3 + col];
}


/*!
 * Copy a direction cosine matrix. This function is faster than matrixCopyf()
 * since it uses a-priori knowledge about the size of the matrix.
 * \param A is the source DCM
 * \param B receives the copy of A. Only the DCM elements are copied since the
 *        size variables are assumed to be already correct.
 */
void dcmCopy(const DCM_t* A, DCM_t* B)
{
	memcpy(B->data, A->data, sizeof(float[9]));

}// dcmCopy


/*!
 * Set a direction cosine matrix to identity. This function is faster than
 * matrixIdentityf() since it uses a-priori knowledge about the size of
 * the matrix.
 * \param A is set to identity
 */
void dcmSetIdentity(DCM_t* A)
{
	set(A, 0, 0, 1);
	set(A, 0, 1, 0);
	set(A, 0, 2, 0);

	set(A, 1, 0, 0);
	set(A, 1, 1, 1);
	set(A, 1, 2, 0);

	set(A, 2, 0, 0);
	set(A, 2, 1, 0);
	set(A, 2, 2, 1);

}// dcmSetIdentity


/*!
 * Add identity to a direction cosine matrix. This function is faster than
 * matrixAddIdentityf() since it uses a-priori knowledge about the size of
 * the matrix.
 * \param A has identity added to it
 */
void dcmAddIdentity(DCM_t* A)
{
	get(A, 0, 0) += 1;
	get(A, 1, 1) += 1;
	get(A, 2, 2) += 1;

}// dcmAddIdentity


/*!
 * Add two DCMs together, placing the result back into the first DCM. This is
 * faster than called matrixAddEqualsf() since it uses a-priori knowledge about
 * the size of the matrix.
 * \param A is a DCM to add, which also receives the result
 * \param B is a DCM to add
 */
void dcmAddEquals(DCM_t* A, const DCM_t* B)
{
	A->data[0] += B->data[0];
	A->data[1] += B->data[1];
	A->data[2] += B->data[2];
	A->data[3] += B->data[3];
	A->data[4] += B->data[4];
	A->data[5] += B->data[5];
	A->data[6] += B->data[6];
	A->data[7] += B->data[7];
	A->data[8] += B->data[8];
}


/*!
 * Scale all the terms of a direction cosine matrix. This function is faster
 * than matrixScalef() since it uses a-priori knowledge about the size of
 * the matrix.
 * \param A has all its terms scaled
 * \param scaler is the value to multiply by all the terms of A
 */
void dcmScale(DCM_t* A, float scaler)
{
	A->data[0] *= scaler;
	A->data[1] *= scaler;
	A->data[2] *= scaler;
	A->data[3] *= scaler;
	A->data[4] *= scaler;
	A->data[5] *= scaler;
	A->data[6] *= scaler;
	A->data[7] *= scaler;
	A->data[8] *= scaler;

}// dcmScale


/*!
 * Set a specific element of a dcm
 * \param M is the dcm pointer
 * \param row is the row index
 * \param col is the column index
 * \param value is the new value to place at <row, col>
 */
void dcmSet(DCM_t* M, uint32_t row, uint32_t col, float value)
{
    set(M, row, col, value);
}


/*! Allocate a DCM, initializing its memory. The memory will be allocated
 * in a single step so all of the DCM's memory can be released by calling
 * free(DCM). The DCM will be set to identity.
 * \return a pointer to the newly allocated DCM, or
 *         NULL if the allocation failed.
 */
DCM_t* dcmAllocate(void)
{
    DCM_t* dcm = matrixAllocatef(3, 3);
    if(dcm != NULL)
    {
        // Make identity
        set(dcm, 0, 0, 1.0f);
        set(dcm, 1, 1, 1.0f);
        set(dcm, 2, 2, 1.0f);
    }

    return dcm;

}// dcmAllocate


/*!
 * Fill out the direction cosine matrix from an Euler roll angle
 * This DCM represents the local to global transform.
 * \param dcm is filled out according to the angle
 * \param roll is the Euler roll angle in radians
 */
void setDCMBasedOnRoll(DCM_t* dcm, float roll)
{
    float sinRoll = sinf(roll);
    float cosRoll = cosf(roll);

    dcmSetIdentity(dcm);
    set(dcm, 1, 1,  cosRoll);
    set(dcm, 2, 2,  cosRoll);
    set(dcm, 1, 2, -sinRoll);
    set(dcm, 2, 1,  sinRoll);

}// setDCMBasedOnRoll


/*!
 * Fill out the direction cosine matrix from an Euler pitch angle
 * This DCM represents the local to global transform.
 * \param dcm is filled out according to the angle
 * \param pitch is the Euler pitch angle in radians
 */
void setDCMBasedOnPitch(DCM_t* dcm, float pitch)
{
    float sinPitch = sinf(pitch);
    float cosPitch = cosf(pitch);

    dcmSetIdentity(dcm);
    set(dcm, 0, 0,  cosPitch);
    set(dcm, 2, 2,  cosPitch);
    set(dcm, 0, 2,  sinPitch);
    set(dcm, 2, 0, -sinPitch);

}// setDCMBasedOnPitch


/*!
 * Fill out the direction cosine matrix from an Euler yaw angle
 * This DCM represents the local to global transform.
 * \param dcm is filled out according to the angle
 * \param yaw is the Euler yaw angle in radians
 */
void setDCMBasedOnYaw(DCM_t* dcm, float yaw)
{
    float sinYaw = sinf(yaw);
    float cosYaw = cosf(yaw);

    dcmSetIdentity(dcm);
    set(dcm, 0, 0,  cosYaw);
    set(dcm, 1, 1,  cosYaw);
    set(dcm, 0, 1, -sinYaw);
    set(dcm, 1, 0,  sinYaw);

}// setDCMBasedOnYaw


/*!
 * Fill out the direction cosine matrix from an Euler, yaw, then pitch, then roll rotation.
 * This DCM represents the local to global transform.
 * \param dcm is filled out according to the angle.
 * \param yaw is the Euler yaw angle in radians
 * \param pitch is the Euler pitch angle in radians
 * \param roll is the Euler roll angle in radians
 */
void setDCMBasedOnEuler(DCM_t* dcm, float yaw, float pitch, float roll)
{
    float cosRoll = cosf(roll);
    float sinRoll = sinf(roll);
    float cosPitch = cosf(pitch);
    float sinPitch = sinf(pitch);
    float cosYaw = cosf(yaw);
    float sinYaw = sinf(yaw);

    set(dcm, 0, 0, cosPitch*cosYaw);
    set(dcm, 0, 1, sinRoll*sinPitch*cosYaw - cosRoll*sinYaw);
    set(dcm, 0, 2, cosRoll*sinPitch*cosYaw + sinRoll*sinYaw);
    set(dcm, 1, 0, cosPitch*sinYaw);
    set(dcm, 1, 1, sinRoll*sinPitch*sinYaw + cosRoll*cosYaw);
    set(dcm, 1, 2, cosRoll*sinPitch*sinYaw - sinRoll*cosYaw);
    set(dcm, 2, 0, -1.0f*sinPitch);
    set(dcm, 2, 1, sinRoll*cosPitch);
    set(dcm, 2, 2, cosRoll*cosPitch);

}// setDCMBasedOnEuler


/*!
 * Fill out the direction cosine matrix from a Pan then a Tilt rotation
 * \param dcm is filled out according to the angle.
 * \param pan is the pan angle in radians
 * \param tilt is the tilt angle in radians
 */
void setDCMBasedOnPanTilt(DCM_t* dcm, float pan, float tilt)
{
	// Pan and Tilt are equivalent to the an Euler yaw and pitch
    float cosPitch = cosf(tilt);
    float sinPitch = sinf(tilt);
    float cosYaw = cosf(pan);
    float sinYaw = sinf(pan);

    set(dcm, 0, 0, cosPitch*cosYaw);
    set(dcm, 0, 1, -1.0f*sinYaw);
    set(dcm, 0, 2, sinPitch*cosYaw);
    set(dcm, 1, 0, cosPitch*sinYaw);
    set(dcm, 1, 1, cosYaw);
    set(dcm, 1, 2, sinPitch*sinYaw);
    set(dcm, 2, 0, -1.0f*sinPitch);
    set(dcm, 2, 1, 0);
    set(dcm, 2, 2, cosPitch);

}// setDCMBasedOnPanTilt


/*!
 * Compute the Euler yaw angle of a dcm.
 * \param dcm is the body to reference rotation matrix.
 * \return the Euler yaw angle in radians, from -PI to PI.
 */
float dcmYaw(const DCM_t* dcm)
{
    return atan2f(get(dcm, 1, 0), get(dcm, 0, 0));

}// dcmYaw


/*!
 * Compute the Euler pitch rotation of a dcm.
 * \param dcm is the body to reference rotation matrix.
 * \return the Euler pitch angle in radians, from -PI/2 to PI/2.
 */
float dcmPitch(const DCM_t* dcm)
{
    return asinf(dcmSinPitch(dcm));

}// dcmPitch


/*!
 * Compute the cosine of the Euler pitch angle of a DCM
 * \param dcm is the body to reference rotation matrix.
 * \return the cosine of the Euler pitch angle.
 */
float dcmCosPitch(const DCM_t* dcm)
{
    float sinp = dcmSinPitch(dcm);

    // pitch goes from -90 to 90, so cosine of pitch must be positive.
    return sqrtf(1.0f - sinp*sinp);

}// dcmCosPitch


/*!
 * Compute the sin of the Euler pitch angle of a DCM
 * \param dcm is the body to reference rotation matrix.
 * \return the sin of the Euler pitch angle.
 */
float dcmSinPitch(const DCM_t* dcm)
{
    return SATURATE(-1.0f*get(dcm, 2, 0), 1.0f);
}


/*!
 * Compute the Euler roll rotation.
 * \param dcm is the body to reference rotation matrix.
 * \return the Euler roll angle in radians, from -PI to PI.
 */
float dcmRoll(const DCM_t* dcm)
{
    return atan2f(get(dcm, 2, 1), get(dcm, 2, 2));

}// roll


/*!
 * Compute the cosine of the Euler roll angle of a DCM
 * \param dcm is the body to reference rotation matrix.
 * \return the cosine of the Euler roll angle.
 */
float dcmCosRoll(const DCM_t* dcm)
{
    return cosf(dcmRoll(dcm));
}


/*!
 * Compute the sin of the Euler roll angle of a DCM
 * \param dcm is the body to reference rotation matrix.
 * \return the sin of the Euler roll angle.
 */
float dcmSinRoll(const DCM_t* dcm)
{
    return sinf(dcmRoll(dcm));
}


/*!
 * Use a DCM to rotate a vector. The input and output vector can be the same vector
 * \param rotation is the DCM rotation matrix
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void dcmApplyRotation(const DCM_t* rotation, const float input[], float output[])
{
    rawdcmApplyRotation(rotation->data, input, output);

}// dcmApplyRotation


/*!
 * Use a DCM to rotate a vector, in the reverse direction. The input and output
 * vector can be the same vector. This is equivalent to multiplying the input
 * by the transpose of the rotation matrix.
 * \param rotation is the DCM rotation matrix
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void dcmApplyReverseRotation(const DCM_t* rotation, const float input[], float output[])
{
    rawdcmApplyReverseRotation(rotation->data, input, output);

}// dcmApplyReverseRotation


/*!
 * Use a raw dcm to rotate a vector. The input and output vector can be the same vector
 * \param rotation is the DCM rotation matrix
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void rawdcmApplyRotation(const float dcmdata[], const float input[], float output[])
{
    // Doing it this way makes it possible for input and output to be the same vector
    float zero = dcmdata[0]*input[0] + dcmdata[1]*input[1] + dcmdata[2]*input[2];
    float one  = dcmdata[3]*input[0] + dcmdata[4]*input[1] + dcmdata[5]*input[2];
    float two  = dcmdata[6]*input[0] + dcmdata[7]*input[1] + dcmdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// rawdcmApplyRotation


/*!
 * Use a raw dcm to rotate a vector, in the reverse direction. The input and output
 * vector can be the same vector. This is equivalent to multiplying the input
 * by the transpose of the rotation matrix.
 * \param rotation is the DCM rotation matrix
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void rawdcmApplyReverseRotation(const float dcmdata[], const float input[], float output[])
{
    // Doing it this way makes it possible for input and output to be the same vector
    float zero = dcmdata[0]*input[0] + dcmdata[3]*input[1] + dcmdata[6]*input[2];
    float one  = dcmdata[1]*input[0] + dcmdata[4]*input[1] + dcmdata[7]*input[2];
    float two  = dcmdata[2]*input[0] + dcmdata[5]*input[1] + dcmdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// rawdcmApplyReverseRotation


/*!
 * Multiply two DCMs together. This is numerically equivalent to matrixMultiplyf(),
 * but faster because no dimension checking is done, no looping is done, and no
 * index computation is done.
 * \param A is the left side DCM.
 * \param B is the right side DCM.
 * \param C receives the result.
 */
void dcmMultiply(const DCM_t* A, const DCM_t* B, DCM_t* C)
{
    rawdcmMultiply(A->data, B->data, C->data);

}// dcmMultiply


/*!
 * Multiply two raw DCMs together. This is numerically equivalent to matrixMultiplyf(),
 * but faster because no dimension checking is done, no looping is done, and no
 * index computation is done.
 * \param Adata is the left side raw DCM.
 * \param Bdata is the right side raw DCM.
 * \param Cdata receives the result.
 */
void rawdcmMultiply(const float Adata[9], const float Bdata[9], float Cdata[9])
{
    // First row of C and A
    Cdata[0] = Adata[0]*Bdata[0] + Adata[1]*Bdata[3] + Adata[2]*Bdata[6];
    Cdata[1] = Adata[0]*Bdata[1] + Adata[1]*Bdata[4] + Adata[2]*Bdata[7];
    Cdata[2] = Adata[0]*Bdata[2] + Adata[1]*Bdata[5] + Adata[2]*Bdata[8];

    // Second row of C and A
    Cdata[3] = Adata[3]*Bdata[0] + Adata[4]*Bdata[3] + Adata[5]*Bdata[6];
    Cdata[4] = Adata[3]*Bdata[1] + Adata[4]*Bdata[4] + Adata[5]*Bdata[7];
    Cdata[5] = Adata[3]*Bdata[2] + Adata[4]*Bdata[5] + Adata[5]*Bdata[8];

    // Third row of C and A
    Cdata[6] = Adata[6]*Bdata[0] + Adata[7]*Bdata[3] + Adata[8]*Bdata[6];
    Cdata[7] = Adata[6]*Bdata[1] + Adata[7]*Bdata[4] + Adata[8]*Bdata[7];
    Cdata[8] = Adata[6]*Bdata[2] + Adata[7]*Bdata[5] + Adata[8]*Bdata[8];

}// rawdcmMultiply



/*!
 * Multiply the transpose of a DCM against another DCM. This is numerically
 * equivalent to matrixMultiplyTransAf(), but faster because no dimension
 * checking is done, no looping is done, and no index computation is done.
 * \param A is the left side DCM, whose transpose is used.
 * \param B is the right side DCM.
 * \param C receives the result.
 */
void dcmMultiplyTransA(const DCM_t* A, const DCM_t* B, DCM_t* C)
{
    const float* Adata = A->data;
    const float* Bdata = B->data;
    float* Cdata = C->data;

    // First row of C and A
    Cdata[0] = Adata[0]*Bdata[0] + Adata[3]*Bdata[3] + Adata[6]*Bdata[6];
    Cdata[1] = Adata[0]*Bdata[1] + Adata[3]*Bdata[4] + Adata[6]*Bdata[7];
    Cdata[2] = Adata[0]*Bdata[2] + Adata[3]*Bdata[5] + Adata[6]*Bdata[8];

    // Second row of C and A
    Cdata[3] = Adata[1]*Bdata[0] + Adata[4]*Bdata[3] + Adata[7]*Bdata[6];
    Cdata[4] = Adata[1]*Bdata[1] + Adata[4]*Bdata[4] + Adata[7]*Bdata[7];
    Cdata[5] = Adata[1]*Bdata[2] + Adata[4]*Bdata[5] + Adata[7]*Bdata[8];

    // Third row of C and A
    Cdata[6] = Adata[2]*Bdata[0] + Adata[5]*Bdata[3] + Adata[8]*Bdata[6];
    Cdata[7] = Adata[2]*Bdata[1] + Adata[5]*Bdata[4] + Adata[8]*Bdata[7];
    Cdata[8] = Adata[2]*Bdata[2] + Adata[5]*Bdata[5] + Adata[8]*Bdata[8];

}// dcmMultiplyTransA


/*!
 * Multiply a DCM against the transpose of another DCM. This is numerically
 * equivalent to matrixMultiplyTransBf(), but faster because no dimension
 * checking is done, no looping is done, and no index computation is done.
 * \param A is the left side DCM.
 * \param B is the right side DCM, whose transpose is used.
 * \param C receives the result.
 */
void dcmMultiplyTransB(const DCM_t* A, const DCM_t* B, DCM_t* C)
{
    const float* Adata = A->data;
    const float* Bdata = B->data;
    float* Cdata = C->data;

    // First row of C and A
    Cdata[0] = Adata[0]*Bdata[0] + Adata[1]*Bdata[1] + Adata[2]*Bdata[2];
    Cdata[1] = Adata[0]*Bdata[3] + Adata[1]*Bdata[4] + Adata[2]*Bdata[5];
    Cdata[2] = Adata[0]*Bdata[6] + Adata[1]*Bdata[7] + Adata[2]*Bdata[8];

    // Second row of C and A
    Cdata[3] = Adata[3]*Bdata[0] + Adata[4]*Bdata[1] + Adata[5]*Bdata[2];
    Cdata[4] = Adata[3]*Bdata[3] + Adata[4]*Bdata[4] + Adata[5]*Bdata[5];
    Cdata[5] = Adata[3]*Bdata[6] + Adata[4]*Bdata[7] + Adata[5]*Bdata[8];

    // Third row of C and A
    Cdata[6] = Adata[6]*Bdata[0] + Adata[7]*Bdata[1] + Adata[8]*Bdata[2];
    Cdata[7] = Adata[6]*Bdata[3] + Adata[7]*Bdata[4] + Adata[8]*Bdata[5];
    Cdata[8] = Adata[6]*Bdata[6] + Adata[7]*Bdata[7] + Adata[8]*Bdata[8];

}// dcmMultiplyTransB


/*!
 * Transpose a DCM in place
 * \param A is the DCM to transpose
 */
void dcmTransposeInPlace(DCM_t* A)
{
	float* Adata = A->data;
	float temp;

	// (0, 1) swaps with (1, 0)
	temp = Adata[1];
	Adata[1] = Adata[3];
	Adata[3] = temp;

	// (0, 2) swaps with (2, 0)
	temp = Adata[2];
	Adata[2] = Adata[6];
	Adata[6] = temp;

	// (1, 2) swaps with (2, 1)
	temp = Adata[5];
	Adata[5] = Adata[7];
	Adata[7] = temp;

}// dcmTransposeInPlace


/*!
 *  Add two DCMs together and multiply each element by 0.5. This might be
 *  faster than using separate Add() and Scale() functions since the compiler
 *  can utilize a multiply-accumulate instruction. This is faster than
 *  matrixAveragef() since the matrix size is known a-priori. Note that
 *  average in place can be done if C points to the same matrix as A or B.
 * \param A is a matrix to average
 * \param B is a matrix to average
 * \param C receives the result
 */
void dcmAverage(const DCM_t* A, const DCM_t* B, DCM_t* C)
{
	set(C, 0, 0, 0.5f*(get(A, 0, 0) + get(B, 0, 0)));
	set(C, 0, 1, 0.5f*(get(A, 0, 1) + get(B, 0, 1)));
	set(C, 0, 2, 0.5f*(get(A, 0, 2) + get(B, 0, 2)));

	set(C, 1, 0, 0.5f*(get(A, 1, 0) + get(B, 1, 0)));
	set(C, 1, 1, 0.5f*(get(A, 1, 1) + get(B, 1, 1)));
	set(C, 1, 2, 0.5f*(get(A, 1, 2) + get(B, 1, 2)));

	set(C, 2, 0, 0.5f*(get(A, 2, 0) + get(B, 2, 0)));
	set(C, 2, 1, 0.5f*(get(A, 2, 1) + get(B, 2, 1)));
	set(C, 2, 2, 0.5f*(get(A, 2, 2) + get(B, 2, 2)));

}// dcmAverage


/*!
 * Convert a 3 element column vector to a skew symmetric DCM
 * \param dcm receives the 3x3 skew symmetric matrix
 * \param vector is the X, Y, Z column vector
 */
void vectorSkewSymmetric(DCM_t* dcm, const float vector[])
{
    skewSymmetric(dcm, vector[0], vector[1], vector[2]);

}// vectorSkewSymmetric


/*!
 * Convert 3 elements to a skew symmetric DCM
 * \param dcm receives the 3x3 skew symmetric vector
 * \param x is the first element of the column vector
 * \param y is the second element of the column vector
 * \param z is the third element of the column vector
 */
void skewSymmetric(DCM_t* dcm, float x, float y, float z)
{
    set(dcm, 0, 0, 0.0f);set(dcm, 0, 1, -z);   set(dcm, 0, 2,  y);
    set(dcm, 1, 0,  z);  set(dcm, 1, 1, 0.0f); set(dcm, 1, 2, -x);
    set(dcm, 2, 0, -y);  set(dcm, 2, 1,  x);   set(dcm, 2, 2, 0.0f);

}// skewSymmetric


/*!
 * Convert a 3 element column vector to a skew symmetric DCM with 1s on the diagonal
 * \param dcm receives the DCM
 * \param vector is the X, Y, Z column vector
 */
void vectorAttitudeIncrement(DCM_t* dcm, const float vector[])
{
	attitudeIncrement(dcm, vector[0], vector[1], vector[2]);

}// vectorAttitudeIncrement


/*!
 * Convert a 3 elements to a skew symmetric DCM with 1s on the diagonal. This
 * is typically used to propagate a DCM with a new set of delta angles.
 * \param dcm receives the DCM
 * \param x is the first element of the column vector
 * \param y is the second element of the column vector
 * \param z is the third element of the column vector
 */
void attitudeIncrement(DCM_t* dcm, float x, float y, float z)
{
	set(dcm, 0, 0, 1.0f);set(dcm, 0, 1, -z);  set(dcm, 0, 2,  y);
	set(dcm, 1, 0,  z);  set(dcm, 1, 1, 1.0f);set(dcm, 1, 2, -x);
	set(dcm, 2, 0, -y);  set(dcm, 2, 1,  x);  set(dcm, 2, 2, 1.0f);

}// attitudeIncrement


/*!
 * Convert a 3 elements to a skew symmetric DCM with 1s on the diagonal. This
 * is typically used to propagate a DCM with a new set of delta angles. This
 * function uses the sin and cosine of the z (yaw) term
 * \param dcm receives the DCM
 * \param x is the small rotation in radians about the x axis
 * \param y is the small rotation in radians about the y axis
 * \param z is the rotation in radians about the z axis
 */
void attitudeIncrementBigYaw(DCM_t* dcm, float x, float y, float z)
{
    float sinz = sinf(z);
    float cosz = cosf(z);

    set(dcm, 0, 0, cosz); set(dcm, 0, 1, -sinz); set(dcm, 0, 2,  y);
    set(dcm, 1, 0, sinz); set(dcm, 1, 1,  cosz); set(dcm, 1, 2, -x);
    set(dcm, 2, 0, -y);   set(dcm, 2, 1,  x);    set(dcm, 2, 2, 1.0f);

}// attitudeIncrementBigYaw


/*!
 * Verify correct operation of key matrix and DCM operations.
 * \return TRUE if the test passed.
 */
BOOL testLinearAlgebra(void)
{
    uint32_t row, col;
    float error = 0;
    float vector[3] = {1, 2, 3};
    stackAllocateDCM(A);
    stackAllocateDCM(B);
    stackAllocateMatrixf(C, 3, 3);

    stackAllocateMatrixf(D, 10, 1);
    stackAllocateMatrixf(E, 10, 10);
    stackAllocateMatrixf(F, 1, 1);

    for(row = 0; row < D.numRows; row++)
        matrixSetf(&D, row, 0, 1.0f);

    // This multiply should produce a 10x10 with all 1s
    if(!matrixMultiplyTransBf(&D, &D, &E))
        return FALSE;

    // Test against all 1s
    for(row = 0; row < E.numRows; row++)
        for(col = 0; col < E.numCols; col++)
            error += fabsf(1.0f - matrixGetf(&E, row, col));

    // This multiply should produce a 1x1 with 10
    if(!matrixMultiplyTransAf(&D, &D, &F))
        return FALSE;

    error += fabsf(10.0f - matrixGetf(&F, 0, 0));

    // A random rotation
    setDCMBasedOnEuler(&A, .1f, .2f, .3f);

    if(matrixInversef(&A, &B))
    {
        dcmMultiply(&A, &B, &C);

        // C should be identity
        error += testForIdentityf(&C);
    }
    else
        return FALSE;

    // Since A is a rotation its transpose is its inverse, hence this should yield identity
    if(!matrixMultiplyTransBf(&A, &A, &C))
        return FALSE;

    error += testForIdentityf(&C);

    // And the other direction as well
    if(!matrixMultiplyTransAf(&A, &A, &C))
        return FALSE;

    error += testForIdentityf(&C);

    // Rotate a vector forwards and backwards
    dcmApplyRotation(&A, vector, vector);
    dcmApplyReverseRotation(&A, vector, vector);
    error += fabsf(1.0f - vector[0]);
    error += fabsf(2.0f - vector[1]);
    error += fabsf(3.0f - vector[2]);

    // DCM that describes pan 90 degrees, tilt 0 degrees
    setDCMBasedOnPanTilt(&A, deg2radf(90), deg2radf(0));

    // Start with unit vector in camera frame
    vector[0] = 1;
    vector[1] = 0;
    vector[2] = 0;

    // Rotate the same vector to be in crown frame. The vector
    // should be pointing to the right in the crown frame
    dcmApplyRotation(&A, vector, vector);
    error += fabsf(0 - vector[0]);
    error += fabsf(1 - vector[1]);
    error += fabsf(0 - vector[2]);

    // DCM that describes pan 0 degrees, tilt -45 degrees
    setDCMBasedOnPanTilt(&A, deg2radf(0), deg2radf(-45));

    // Start with unit vector in camera frame
    vector[0] = 1;
    vector[1] = 0;
    vector[2] = 0;

    // Rotate the same vector to be in crown frame. The vector
    // should be pointing forward and down in the crown frame
    dcmApplyRotation(&A, vector, vector);
    error += fabsf(0.70710678118654752440084436210485f - vector[0]);
    error += fabsf(0 - vector[1]);
    error += fabsf(0.70710678118654752440084436210485f - vector[2]);

    if(error < 0.0001f)
        return TRUE;

    return FALSE;

}// testLinearAlgebra
