import numpy as np
from scipy.linalg import expm
from pylie import SO3

def generate_Gal3():
    X = np.eye(5)
    X[0:3,0:3] = SO3.exp(10*np.random.randn(3,1)).as_matrix()
    X[0:3,3:5] = 10*np.random.randn(3,2)
    X[4,3] = np.random.randn()
    return X

def format_Gal3(X):
    R = X[0:3,0:3]
    x = X[0:3,3]
    v = X[0:3,4]
    a = X[4,3]
    Rs = "Matrix3F({{ {}, {}, {} }},{{ {}, {}, {} }}, {{ {}, {}, {} }})".format(*[R.ravel()[i] for i in range(9)])
    xs = "Vector3F({}, {}, {})".format(x[0], x[1], x[2])
    vs = "Vector3F({}, {}, {})".format(v[0], v[1], v[2])

    return "Gal3({}, {}, {}, {})".format(Rs, xs, vs, a)

def generate_test_params():
    s = """    {{
    .X1 = {},
    .X2 = {},
    .X1X2 = {},
    .X1Inv = {},
    }},"""
    X1 = generate_Gal3()
    X2 = generate_Gal3()
    s = s.format(format_Gal3(X1),format_Gal3(X2),format_Gal3(X1@X2),format_Gal3(np.linalg.inv(X1)))
    return s


for _ in range(5):
    print(generate_test_params())
