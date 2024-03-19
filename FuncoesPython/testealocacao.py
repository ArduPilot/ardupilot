import math

aloc = 400.0

t = 0.0
s = 1.0
hip = math.sqrt(t*t + s*s)

Ta = aloc * t / hip
Sa = aloc * s / hip

T = math.fabs(Ta / (math.fabs(Ta) + math.fabs(Sa)))
S = math.fabs(Sa / (math.fabs(Ta) + math.fabs(Sa)))

ft = t * T * aloc
fs = s * S * aloc

alocDir = ft + fs
alocEsq = ft - fs

print(Ta,T, "ft =", ft )
print(Sa,S, "fs =", fs )
print("dir = ", alocDir, "esq = ", alocEsq)