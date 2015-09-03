#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/*
 *    generic matrix inverse code
 *
 *    @param     x,     input nxn matrix
 *    @param     n,     dimension of square matrix
 *    @returns   determinant of square matrix
 *    Known Issues/ Possible Enhancements:
 *                -more efficient method should be available, following is code generated from matlab
 */
float detnxn(const float C[],const uint8_t n)
{
  float f;
  float *A = new float[n*n];
  if( A == NULL) {
      return 0;
  }
  int8_t *ipiv = new int8_t[n];
  if(ipiv == NULL) {
      delete[] A;
      return 0;
  }
  int32_t i0;
  int32_t j;
  int32_t c;
  int32_t iy;
  int32_t ix;
  float smax;
  int32_t jy;
  float s;
  int32_t b_j;
  int32_t ijA;
  bool isodd;

  memcpy(&A[0], &C[0], n*n * sizeof(float));
  for (i0 = 0; i0 < n; i0++) {
    ipiv[i0] = (int8_t)(1 + i0);
  }

  for (j = 0; j < n-1; j++) {
    c = j * (n+1);
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 2; jy <= n - 1 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy - 1;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (int8_t)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < n; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += n;
          iy += n;
        }
      }

      i0 = (c - j) + n;
      for (iy = c + 1; iy + 1 <= i0; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + n;
    for (b_j = 1; b_j <= n - 1 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i0 = (iy - j) + (2*n);
        for (ijA = n + 1 + iy; ijA + 1 <= i0; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += n;
      iy += n;
    }
  }

  f = A[0];
  isodd = false;
  for (jy = 0; jy < (n-1); jy++) {
    f *= A[(jy + n * (1 + jy)) + 1];
    if (ipiv[jy] > 1 + jy) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    f = -f;
  }
  delete[] A;
  delete[] ipiv;
  return f;
}
/*
 *    generic matrix inverse code
 *
 *    @param     x,     input nxn matrix
 *    @param     y,     Output inverted nxn matrix
 *    @param     n,     dimension of square matrix
 *    @returns          false = matrix is Singular, true = matrix inversion successful
 *    Known Issues/ Possible Enhancements:
 *                -more efficient method should be available, following is code generated from matlab
 */

bool inversenxn(const float x[], float y[], const uint8_t n)
{
    if (is_zero(detnxn(x,n))) {
        return false;
    }

    float *A = new float[n*n];
    if( A == NULL ){
        return false;
    }
    int32_t i0;
    int32_t *ipiv = new int32_t[n];
    if(ipiv == NULL) {
        delete[] A;
        return false;
    }
    int32_t j;
    int32_t c;
    int32_t pipk;
    int32_t ix;
    float smax;
    int32_t k;
    float s;
    int32_t jy;
    int32_t ijA;
    int32_t *p = new int32_t[n];
    if(p == NULL) {
        delete[] A;
        delete[] ipiv;
        return false;
    }
    
    for (i0 = 0; i0 < n*n; i0++) {
        A[i0] = x[i0];
        y[i0] = 0.0f;
    }

    for (i0 = 0; i0 < n; i0++) {
        ipiv[i0] = (int32_t)(1 + i0);
    }

    for (j = 0; j < (n-1); j++) {
        c = j * (n+1);
        pipk = 0;
        ix = c;
        smax = fabsf(A[c]);
        for (k = 2; k <= (n-1) - j; k++) {
            ix++;
            s = fabsf(A[ix]);
            if (s > smax) {
                pipk = k - 1;
                smax = s;
            }
        }

        if (A[c + pipk] != 0.0f) {
            if (pipk != 0) {
                ipiv[j] = (int32_t)((j + pipk) + 1);
                ix = j;
                pipk += j;
                for (k = 0; k < n; k++) {
                    smax = A[ix];
                    A[ix] = A[pipk];
                    A[pipk] = smax;
                    ix += n;
                    pipk += n;
                }
            }

            i0 = (c - j) + n;
            for (jy = c + 1; jy + 1 <= i0; jy++) {
                A[jy] /= A[c];
            }
        }

        pipk = c;
        jy = c + n;
        for (k = 1; k <= (n-1) - j; k++) {
            smax = A[jy];
            if (A[jy] != 0.0f) {
                ix = c + 1;
                i0 = (pipk - j) + (2*n);
                for (ijA = (n+1) + pipk; ijA + 1 <= i0; ijA++) {
                    A[ijA] += A[ix] * -smax;
                    ix++;
                }
            }

            jy += n;
            pipk += n;
        }
    }

    for (i0 = 0; i0 < n; i0++) {
        p[i0] = (int32_t)(1 + i0);
    }

    for (k = 0; k < (n-1); k++) {
        if (ipiv[k] > 1 + k) {
            pipk = p[ipiv[k] - 1];
            p[ipiv[k] - 1] = p[k];
            p[k] = (int32_t)pipk;
        }
    }

    for (k = 0; k < n; k++) {
        y[k + n * (p[k] - 1)] = 1.0;
        for (j = k; j + 1 < (n+1); j++) {
            if (y[j + n * (p[k] - 1)] != 0.0f) {
                for (jy = j + 1; jy + 1 < (n+1); jy++) {
                    y[jy + n * (p[k] - 1)] -= y[j + n * (p[k] - 1)] * A[jy + n * j];
                }
            }
        }
    }

    for (j = 0; j < n; j++) {
        c = n * j;
        for (k = (n-1); k > -1; k += -1) {
            pipk = n * k;
            if (y[k + c] != 0.0f) {
                y[k + c] /= A[k + pipk];
                for (jy = 0; jy + 1 <= k; jy++) {
                    y[jy + c] -= y[k + c] * A[jy + pipk];
                }
            }
        }
    }
    delete[] A;
    delete[] ipiv;
    delete[] p;
    return true;
}

/*
 *    matrix inverse code only for 3x3 square matrix
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */

bool inverse3x3(float m[], float invOut[])
{
    float inv[9];
    // computes the inverse of a matrix m
    float  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (is_zero(det)){
        return false;
    }

    float invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[5] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(uint8_t i = 0; i < 9; i++){
        invOut[i] = inv[i];
    }

    return true;
}

/*
 *    matrix inverse code only for 4x4 square matrix copied from
 *    gluInvertMatrix implementation in
 *    opengl for 4x4 matrices.
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */

bool inverse4x4(float m[],float invOut[])
{
    float inv[16], det;
    uint8_t i;

    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (is_zero(det)){
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

/*
 *    generic matrix inverse code
 *
 *    @param     x,     input nxn matrix
 *    @param     y,     Output inverted nxn matrix
 *    @param     n,     dimension of square matrix
 *    @returns          false = matrix is Singular, true = matrix inversion successful
 */
bool inverse(float x[], float y[], uint16_t dim)
{
    switch(dim){
        case 3: return inverse3x3(x,y);
        case 4: return inverse4x4(x,y);
        default: return inversenxn(x,y,dim);
    }
}
