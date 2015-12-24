/*
 * This is an extension file for the Eigen library
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */


bool is_nan(void) {
    for (int y = 0; y < this->rows(); y++) {
        for (int x = 0; x < this->cols(); x++) {
            if (isnanf((*this)(y,x) ) ) {
                return true;
            }
        }
    }
    return false;
}

bool is_inf(void) {
    for (int y = 0; y < this->rows(); y++) {
        for (int x = 0; x < this->cols(); x++) {
            if (isinf((*this)(y,x) ) ) {
                return true;
            }
        }
    }
    return false;
}
