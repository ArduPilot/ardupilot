/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, taken from unknown source.
 */

#ifndef MatrixMath_h
#define MatrixMath_h

//#include "Arduino.h"
#include "math.h"

class MatrixMath
{
public:
    MatrixMath();
    static void MatrixCopy(float* A, int n, int m, float* B);
    static void MatrixMult(float* A, float* B, int m, int p, int n, float* C);
    static void MatrixMultTranspose(float* A, float* B, int m, int p, int n, float* C);
    static void MatrixAdd(float* A, float* B, int m, int n, float* C);
    static void MatrixSubtract(float* A, float* B, int m, int n, float* C);
    static void MatrixTranspose(float* A, int m, int n, float* C);
    static void MatrixMultScalar(float* A, float s, int m, int n, float* C);
    static int MatrixInvert(float* A, int n);
    static void MatrixForceSymmetry(float* A, int n);
};

#endif