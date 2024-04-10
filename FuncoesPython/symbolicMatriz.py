from sympy import symbols, Matrix

L1, k = symbols('L1 k')
M_specific = Matrix([[k, k], [k*L1, -k*L1]])
M_inv_new = M_specific.inv()
print(M_inv_new)