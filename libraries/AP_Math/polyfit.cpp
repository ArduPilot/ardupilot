/*
  polynomial fitting class, originally written by Siddharth Bharat Purohit
  re-worked for ArduPilot by Andrew Tridgell
*/

#include "polyfit.h"
#include "AP_Math.h"

template <uint8_t order>
void PolyFit<order>::update(float x, float y)
{
    float temp = 1;

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

template <uint8_t order>
bool PolyFit<order>::get_polynomial(float res[order]) const
{
    float inv_mat[order][order];
    if (!inverse(&mat[0][0], &inv_mat[0][0], order)) {
        return false;
    }
    for (uint8_t i = 0; i < order; i++) {
        res[i] = 0.0;
        for (uint8_t j = 0; j < order; j++) {
            res[i] += inv_mat[i][j] * vec[j];
        }
    }
    return true;
}

// instantiate for order 4
template class PolyFit<4>;
