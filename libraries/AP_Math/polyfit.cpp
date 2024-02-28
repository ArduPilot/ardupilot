/*
  polynomial fitting class, originally written by Siddharth Bharat Purohit
  re-worked for ArduPilot by Andrew Tridgell
*/

#include "polyfit.h"
#include "AP_Math.h"
#include "vector3.h"

template <uint8_t order, typename xtype, typename vtype>
void PolyFit<order,xtype,vtype>::update(xtype x, vtype y)
{
    xtype temp = 1;

    for (int8_t i = 2*(order-1); i >= 0; i--) {
        int8_t k = (i<order)?0:i - order + 1;
        for (int8_t j = i - k; j >= k; j--) {
            mat[j][i-j] += temp;
        }
        temp *= x;
    }

    temp = 1;
    for (int8_t i = order-1; i >= 0; i--) {
        vec[i] += y * temp;
        temp *= x;
    }
}

template <uint8_t order, typename xtype, typename vtype>
bool PolyFit<order,xtype,vtype>::get_polynomial(vtype res[order]) const
{
    // we dynamically allocate the inverse matrix to keep stack usage low
    xtype *inv_mat = new xtype[order*order];
    if (inv_mat == nullptr) {
        return false;
    }
    if (!mat_inverse(&mat[0][0], inv_mat, order)) {
        delete[] inv_mat;
        return false;
    }
    // the summation must be done with double precision to get
    // good accuracy
    Vector3d resd[order] {};
    for (uint8_t i = 0; i < order; i++) {
        for (uint8_t j = 0; j < order; j++) {
            resd[i].x += vec[j].x * inv_mat[i*order+j];
            resd[i].y += vec[j].y * inv_mat[i*order+j];
            resd[i].z += vec[j].z * inv_mat[i*order+j];
        }
    }
    for (uint8_t j = 0; j < order; j++) {
        res[j].x = resd[j].x;
        res[j].y = resd[j].y;
        res[j].z = resd[j].z;
    }
    delete[] inv_mat;
    return true;
}

// instantiate for order 4 double with Vector3f
template class PolyFit<4, double, Vector3f>;
