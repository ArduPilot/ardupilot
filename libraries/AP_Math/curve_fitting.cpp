#include "curve_fitting.h"

#include <algorithm>
#include <vector>

// The coef is in ascending order,
// i.e., f(x) = coef[0] + coef[1] * x + coef[2] * x^2 ...
float evaluate_polynomial(const std::deque<float>& coef, const float p)
 {
    float r = 0.0;
    const int N = coef.size() - 1;
    for (int i = N; i >= 0; --i) {
        r = r * p + coef[i];
    }
    return r;
}

// y = a * x + b
std::deque<float> least_square(const std::deque<float>& points, const float dt, float* ptr_error_square)
{
    // translate vector from deque
    const std::size_t M = points.size();
    std::vector<float> x(M);
    std::vector<float> y(M);
    for(std::size_t k = 0; k< M;k++){
        x[k] = k * dt;
        y[k] = points[k];
    }

    float t1 = 0,t2 = 0,t3 =0,t4 = 0;
    for(std::size_t i=0; i<x.size(); ++i){
        t1 += x[i]*x[i];
        t2 += x[i];
        t3 += x[i]*y[i];
        t4 += y[i];
    }
    std::deque<float> coefs(2);
    coefs[0] = (t1*t4 - t2*t3) / (t1*M- t2*t2); // b
    coefs[1] = (t3*M - t2*t4) / (t1*M - t2*t2); // a

    if (ptr_error_square != nullptr) {
        *ptr_error_square = 0.0;
        int i = 0;
        for (const float& point : points) {
            float error = evaluate_polynomial(coefs, i*dt) - point;
            *ptr_error_square += error * error;
            i++;
        }
    }
    return coefs;
}


