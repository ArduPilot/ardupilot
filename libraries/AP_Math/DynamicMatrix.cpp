#include "DynamicMatrix.h"
#include "AP_Math.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

// constuctor with size
template <typename T>
DynamicMatrix<T>::DynamicMatrix(void *&_heap, bool &_error_flag, uint8_t _rows, uint8_t _columns) : heap(_heap), error_flag(_error_flag) {
    init(_rows, _columns);
}

// destructor, free memory
template <typename T>
DynamicMatrix<T>::~DynamicMatrix() {
    if ((heap != nullptr) && (v != nullptr)) {
        hal.util->heap_realloc(heap, v, 0);
    }
}

// set size and grab memory, note that size is always set, this stops matrix sizeing panics when a matrix is used
template <typename T>
void DynamicMatrix<T>::init(uint8_t _rows, uint8_t _columns) {
    if ((_rows > INT8_MAX) || (_columns > INT8_MAX)) {
        AP_HAL::panic("Matrix init fail, too big");
        error_flag = true;
        return;
    }
    rows = _rows;
    columns = _columns;
    if (heap == nullptr) {
        error_flag = true;
        return;
    }
    if (v != nullptr) {
        // allow runtime re-size
        hal.util->heap_realloc(heap, v, 0);
    }
    size_t size = sizeof(T)*rows*columns;
    v = (T *)hal.util->heap_realloc(heap, nullptr, size);
    if (v == nullptr) {
        error_flag = true;
        return;
    }
    memset(v, 0, size);
}

// uniform operators
#define UNIFORM_OP(sym) \
template <typename T> \
DynamicMatrix<T> DynamicMatrix<T>::operator sym(const T num) const { \
    DynamicMatrix m_ret{heap,error_flag,rows,columns}; \
    if ((v == nullptr) || (m_ret.v == nullptr)) { \
        error_flag = true; \
        return m_ret; \
    } \
    for (uint16_t i = 0; i < rows*columns; i++) { \
        *(m_ret.v + i) = *(v + i) sym num; \
    } \
    return m_ret;\
}

UNIFORM_OP(*)
UNIFORM_OP(/)
UNIFORM_OP(+)
UNIFORM_OP(-)

// uniform operators in place
#define UNIFORM_OP_IN_PLACE(sym) \
template <typename T> \
void DynamicMatrix<T>::operator sym (const T num) { \
    if (v == nullptr) { \
        error_flag = true; \
        return; \
    } \
    for (uint16_t i = 0; i < rows*columns; i++) { \
        *(v + i) sym num; \
    } \
}

UNIFORM_OP_IN_PLACE(=) // could memcpy this
UNIFORM_OP_IN_PLACE(*=)
UNIFORM_OP_IN_PLACE(/=)
UNIFORM_OP_IN_PLACE(+=)
UNIFORM_OP_IN_PLACE(-=)

// element wise operators
#define ELEMENT_OP(sym) \
template <typename T> \
DynamicMatrix<T> DynamicMatrix<T>::operator sym(const DynamicMatrix<T>& m) const { \
    DynamicMatrix m_ret{heap,error_flag,rows,columns}; \
    if ((rows != m.rows) || (columns != m.columns)) { \
        if (CONFIG_HAL_BOARD == HAL_BOARD_SITL) { \
            AP_HAL::panic("element wise matrices must be the same size"); \
        } \
        error_flag = true; \
        return m_ret; \
    } \
    if ((v == nullptr) || (m.v == nullptr) || (m_ret.v == nullptr)) { \
        error_flag = true; \
        return m_ret; \
    } \
    for (uint16_t i = 0; i < rows*columns; i++) { \
        (*(m_ret.v + i)) = (*(v + i)) sym (*(m.v + i)); \
    } \
    return m_ret;\
}

ELEMENT_OP(+)
ELEMENT_OP(-)

// element wise operators in place
#define ELEMENT_OP_IN_PLACE(sym) \
template <typename T> \
void DynamicMatrix<T>::operator sym(const DynamicMatrix<T>& m) { \
    if ((rows != m.rows) || (columns != m.columns)) { \
        if (CONFIG_HAL_BOARD == HAL_BOARD_SITL) { \
            AP_HAL::panic("in place element wise matrices must be the same size"); \
        } \
        error_flag = true; \
        return; \
    } \
    if ((v == nullptr) || (m.v == nullptr)) { \
        error_flag = true; \
        return; \
    } \
    for (uint16_t i = 0; i < rows*columns; i++) { \
        *(v + i) sym *(m.v + i); \
    } \
}

ELEMENT_OP_IN_PLACE(=)
ELEMENT_OP_IN_PLACE(+=)
ELEMENT_OP_IN_PLACE(-=)

// matrix multiplication
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::operator *(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,b.columns};
    if (columns != b.rows) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("matrix mult size mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < b.columns; j++) {
            T sum = 0.0;
            for (uint8_t k = 0; k < columns; k++) {
                sum += get_c(i,k) * b.get_c(k,j);
            }
            x.get(i,j) = sum;
        }
    }
    return x;
}

// per element multiplication for matrices of the same size
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::per_element_mult(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,columns};
    if ((rows != b.rows) || (columns != b.columns)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("per element mult matrix mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    for (uint16_t i = 0; i < rows*columns; i++) {
        (*(x.v + i)) = (*(v + i)) * (*(b.v + i));
    }
    return x;
}

// per element division for matrices of the same size
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::per_element_div(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,columns};
    if ((rows != b.rows) || (columns != b.columns)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("per element divide matrix mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    for (uint16_t i = 0; i < rows*columns; i++) {
        (*(x.v + i)) = (*(v + i)) / (*(b.v + i));
    }
    return x;
}

// per element multiplication for matrices and vectors
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::per_element_mult_mv(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,columns};
    if (((rows != b.rows) && (columns != b.columns)) || ((b.rows != 1) && (b.columns != 1))) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("per element mult matrix vector mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    const bool row_op = rows == b.rows;
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < columns; j++) {
            x.get(i,j) = get_c(i,j) * (*(b.v + (row_op ? i : j)));
        }
    }
    return x;
}


// per element division for matrices and vectors
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::per_element_div_mv(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,columns};
    if (((rows != b.rows) && (columns != b.columns)) || ((b.rows != 1) && (b.columns != 1))) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("per element divide matrix vector mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    const bool row_op = rows == b.rows;
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < columns; j++) {
            x.get(i,j) = get_c(i,j) / *(b.v + (row_op ? i : j));
        }
    }
    return x;
}

// perform Cholesky decomposition in place
// Note that this method does not zero the upper triangle
// No need with forward_sub implementation
template <typename T>
void DynamicMatrix<T>::cholesky() {
    if (rows != columns) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("in place cholesky requires square matrix");
#endif
        error_flag = true;
        return;
    }
    if (v == nullptr) {
        error_flag = true;
        return;
    }
    for (uint8_t i = 0; i < rows; i++) {
        get(i,i) = sqrtf(get(i,i)); // using sqrtf always goes via float, may need double?
        for (uint8_t j = i+1; j < rows; j++) {
            get(j,i) /= get(i,i);
        }
        for (uint8_t k = i+1; k < rows; k++) {
            for (uint8_t j = k; j < rows; j++) {
                get(j,k) -= get(j,i)*get(k,i);
            }
        }
    }
}

// forward substitution
// equivelent to: x = a \ b if a is lower triangular
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::forward_sub(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,1};
    if ((rows != columns) || (b.columns != 1) || (b.rows != rows)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("forward sub matrix mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    for (uint8_t i = 0; i < rows; i++) {
        x.get(i,0) = b.get_c(i,0);
        for (uint8_t j = 0; j < i; j++) {
            x.get(i,0) -= get_c(i,j) * x.get(j,0);
        }
        x.get(i,0) /= get_c(i,i);
    }
    return x;
}

// backwards substitution
// equivelent to: x = a \ b if a is upper triangular
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::backward_sub(const DynamicMatrix<T>& b) const {
    DynamicMatrix x{heap,error_flag,rows,1};
    if ((rows != columns) || (b.columns != 1) || (b.rows != rows)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("forward sub matrix mis-match");
#endif
        error_flag = true;
        return x;
    }
    if ((v == nullptr) || (b.v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    for (int8_t i = rows-1; i >= 0; i--) {
        x.get(i,0) = b.get_c(i,0);
        for (int8_t j = rows-1; j > i; j--) {
            x.get(i,0) -= get_c(i,j)*x.get(j,0);
        }
        x.get(i,0) /= get_c(i,i);
    }
    return x;
}

// return transpose
template <typename T>
DynamicMatrix<T> DynamicMatrix<T>::transposed() const {
    DynamicMatrix x{heap,error_flag,columns,rows};
    if ((v == nullptr) || (x.v == nullptr)) {
        error_flag = true;
        return x;
    }
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < columns; j++) {
            x.get(j,i) = get_c(i,j);
        }
    }
    return x;
}

// dot product of two vectors
template <typename T>
T DynamicMatrix<T>::dot(const DynamicMatrix<T>& b) const {
    T ret = 0;
    if (((rows > 1) && (columns > 1)) || ((b.rows > 1) && (b.columns > 1)) || ((rows*columns) != (b.rows * b.columns))) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("dot product vector miss-match");
#endif
        error_flag = true;
        return ret;
    }
    if ((v == nullptr) || (b.v == nullptr)) {
        error_flag = true;
        return ret;
    }
    for (uint16_t i = 0; i < rows*columns; i++) {
        ret += (*(v + i)) * (*(b.v + i));
    }
    return ret;
}

// vector length squared
template <typename T>
T DynamicMatrix<T>::length_squared() const {
    T ret = 0;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if ((rows > 1) && (columns > 1)) {
        AP_HAL::panic("length squared valid for vector only");
        error_flag = true;
        return ret;
    }
#endif
    if (v == nullptr) {
        error_flag = true;
        return ret;
    }
    for (uint16_t i = 0; i < rows*columns; i++) {
        ret += (*(v + i)) * (*(v + i));
    }
    return ret;
}

// get (row index, column index)
template <typename T>
T DynamicMatrix<T>::operator () (uint8_t i, uint8_t j)
{
    if ((i >= rows) || (j >= columns)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("index outside matrix range");
#endif
        error_flag = true;
        return 0.0;
    }
    if (v == nullptr) {
        error_flag = true;
        return 0.0;
    }
    return get(i,j);
}

// set (row index, column index, value)
template <typename T>
void DynamicMatrix<T>::operator () (uint8_t i, uint8_t j, T val)
{
    if ((i >= rows) || (j >= columns)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("index outside matrix range");
#endif
        error_flag = true;
        return;
    }
    if (v == nullptr) {
        error_flag = true;
        return;
    }
    get(i,j) = val;
}

// indexing helper, does not check for nullptr! internal only
template <typename T>
T& DynamicMatrix<T>::get(uint8_t i, uint8_t j)
{
    return *(v + i*columns + j);
}

// const version of above
template <typename T>
const T& DynamicMatrix<T>::get_c(uint8_t i, uint8_t j) const
{
    return *(v + i*columns + j);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// print matrix to console for debugging
template <typename T>
void DynamicMatrix<T>::print(const char* name) const
{
    hal.console->printf("Matrix: %s [%u,%u]\n", name, rows, columns);
    if (v == nullptr) {
        hal.console->printf("\tnullptr\n");
        return;
    }
    for (uint8_t i = 0; i < rows; i++) {
        hal.console->printf("\t");
        for (uint8_t j = 0; j < columns; j++) {
            hal.console->printf("%0.4f", get_c(i,j));
            if (j < (columns - 1)) {
                hal.console->printf(", ");
            }
        }
        hal.console->printf("\n");
    }
}
#endif

template class DynamicMatrix<double>;

// interior point method quadratic programming solver
// Inspired by: https://github.com/jarredbarber/eigen-QP
// solves min( 0.5*x'Hx + f'x )
// with constraints A'x >= b
template <typename T>
DynamicMatrix<T> interior_point_solve(const DynamicMatrix<T> &H, const DynamicMatrix<T> &f, const DynamicMatrix<T> &A, const DynamicMatrix<T> &b, void *&heap, bool &error_flag)
{
    const T eta = 0.95;

    // problem size
    const uint8_t nH = H.get_rows();
    const uint8_t nA = A.get_columns();

    // init matrix size
    DynamicMatrix<T> x{heap,error_flag,nH,1};
    DynamicMatrix<T> z{heap,error_flag,nA,1};
    DynamicMatrix<T> s{heap,error_flag,nA,1};
    DynamicMatrix<T> rL{heap,error_flag,nH,1};
    DynamicMatrix<T> rs{heap,error_flag,nA,1};
    DynamicMatrix<T> rsz{heap,error_flag,nA,1};
    DynamicMatrix<T> H_bar{heap,error_flag,nH,nH};
    DynamicMatrix<T> r_bar{heap,error_flag,nH,1};
    DynamicMatrix<T> f_bar{heap,error_flag,nH,1};
    DynamicMatrix<T> dx{heap,error_flag,nH,1};
    DynamicMatrix<T> dz{heap,error_flag,nA,1};
    DynamicMatrix<T> ds{heap,error_flag,nA,1};

    // pre-compute A transpose
    DynamicMatrix<T> At{heap,error_flag,nA,nH};
    At = A.transposed();

    // setup starting points
    x = 0.0;
    z = 1.0;
    s = 1.0;

    // compute residuals
    rL = H*x + f - A*z;
    rs = s - At*x + b;
    rsz = s.per_element_mult(z);
    T mu = z.dot(s)/nA;

    const T tol_sq = (1e-6) * (1e-6);
    // limit to 15 iterations
    for (uint8_t k = 0; k < 15; k++) {

        // Pre-decompose to speed up solve
        H_bar = H + A*At.per_element_mult_mv(z.per_element_div(s));
        H_bar.cholesky();

        T alpha;
        for (uint8_t i = 0; i < 2; i++) {
            // centering on fist iteration, correction on seccond

            // Solve system
            r_bar = A*((rsz - z.per_element_mult(rs)).per_element_div(s));
            f_bar = (rL + r_bar) * -1.0;
            dx = H_bar.transposed().backward_sub(H_bar.forward_sub(f_bar));
            ds = At*dx - rs;
            dz = (rsz + z.per_element_mult(ds)).per_element_div(s) * -1.0;

            // Compute alpha
            alpha = 1.0;
            for (uint8_t j = 0; j < nA; j++) {
                if (dz(j,0) < 0) {
                    alpha = MIN(alpha, -z(j,0)/dz(j,0));
                }
                if (ds(j,0) < 0) {
                    alpha = MIN(alpha, -s(j,0)/ds(j,0));
                }
            }

            if (i == 1) {
                break;
            }

            // affine duality gap
            T mu_a = ((z+dz*alpha).dot(s+ds*alpha))/nA;

            // centering parameter
            T sigma = mu_a/mu;
            sigma = sigma * sigma * sigma;
            rsz += ds.per_element_mult(dz) - sigma*mu;

        }

        // Update x, z, s
        x += dx*alpha*eta;
        z += dz*alpha*eta;
        s += ds*alpha*eta;

        // Update rhs and mu
        rL = H*x + f - A*z;
        rs = s - At*x + b;
        rsz = s.per_element_mult(z);
        mu = z.dot(s)/nA;
        
        if (((mu*mu) < tol_sq) || (rL.length_squared() < tol_sq) || (rs.length_squared() < tol_sq)) {
            break;
        }
    }
    return x;
}
template DynamicMatrixd interior_point_solve<double>(const DynamicMatrixd &H, const DynamicMatrixd &f, const DynamicMatrixd &A, const DynamicMatrixd &b, void *&heap, bool &error_flag);
