import numpy as np
from scipy.linalg import expm
from pylie import SO3

def generate_SIM23():
    X = np.eye(5)
    X[0:3,0:3] = SO3.exp(10*np.random.randn(3,1)).as_matrix()
    X[0:3,3:5] = 10*np.random.randn(3,2)
    X[3:5,3:5] = expm(0.2*np.random.randn(2,2))
    return X

def format_SIM23(X):
    R = X[0:3,0:3]
    W1 = X[0:3,3]
    W2 = X[0:3,4]
    A = X[3:5,3:5]
    Rs = "Matrix3F({{ {}, {}, {} }},{{ {}, {}, {} }}, {{ {}, {}, {} }})".format(*[R.ravel()[i] for i in range(9)])
    W1s = "Vector3F({}, {}, {})".format(W1[0], W1[1], W1[2])
    W2s = "Vector3F({}, {}, {})".format(W2[0], W2[1], W2[2])
    As = "GL2({},{},{},{})".format(*[A.ravel()[i] for i in range(4)])

    return "SIM23({}, {}, {}, {})".format(Rs, W1s, W2s, As)

def generate_test_params():
    s = """    {{
    .X1 = {},
    .X2 = {},
    .X1X2 = {},
    .X1Inv = {},
    }},"""
    X1 = generate_SIM23()
    X2 = generate_SIM23()
    s = s.format(format_SIM23(X1), format_SIM23(X2), format_SIM23(X1@X2), format_SIM23(np.linalg.inv(X1)))
    return s


for _ in range(5):
    print(generate_test_params())
