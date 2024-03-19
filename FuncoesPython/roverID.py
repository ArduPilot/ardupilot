import numpy as np
from pandas import Series,DataFrame
import pandas as pd
import sympy as sym
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import seaborn as sns

#('throttle,steering,pmwl,pwmr,x,y')
data = pd.read_csv(f'aData.csv')
print(data.columns)


data['time_corrected'] = data['time'].copy()
time_diff_zero = data['time'].diff() == 0
data.loc[time_diff_zero, 'time_corrected'] = data['time'][time_diff_zero] + 100

# Convertendo o tempo de milissegundos para segundos para calcular as velocidades em m/s e rad/s
data['time_seconds'] = data['time_corrected'] / 1000.0  # Convertendo ms para s

# Recalculando as velocidades com os dados corrigidos e convertidos
data['velocity_x'] = data['x'].diff() / (data['time_seconds'].diff())
data['velocity_y'] = data['y'].diff() / (data['time_seconds'].diff())
data['absolute_velocity'] = (data['velocity_x']**2 + data['velocity_y']**2)**0.5
data['angular_velocity'] = data['bearing'].diff() / (data['time_seconds'].diff())

# Preenchendo NaNs com zeros
data.fillna(0, inplace=True)

# Preenchendo NaNs com zeros
data.fillna(0, inplace=True)

# Exibindo as primeiras linhas do DataFrame corrigido
print(data.head())

data.to_csv(f'a0Data.csv')
