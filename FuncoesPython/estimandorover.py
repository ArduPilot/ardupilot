from scipy.optimize import minimize
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import optunity
from scipy.integrate import cumtrapz


# Definindo as funções conforme o código original
def moving_average(data, window_size):
    """
    Calcula a média móvel de uma série de dados.

    :param data: Lista ou array de dados numéricos.
    :param window_size: Tamanho da janela para a média móvel.
    :return: Array contendo a média móvel.
    """
    return pd.Series(data).rolling(window=window_size).mean().to_numpy()

def robot_dynamics(params, inputs):
    r, L, k = params
    Pwmr, Pwml = inputs.T

    predicted_v = r * (k*Pwmr + k*Pwml)/2
    predicted_thetadot = r * (k*Pwmr - k*Pwml)/L

    return np.vstack((predicted_v, predicted_thetadot)).T



def objective_function(params):
    predicted_outputs = robot_dynamics(params, inputs)
    error = np.sum((outputs - predicted_outputs)**2)
    return error




# Preparando os dados de entrada e saída
xdata = pd.read_csv(f'data_478593200.csv')
xdata.fillna(0, inplace=True)

data = pd.DataFrame()

xdata['time_seconds']= xdata['time']

for campo in xdata.columns:
    data[campo] = moving_average(xdata[campo],10)

#
# data.to_csv("saida1_rover")
# ('time,TL,TR,x,y,bearing,yaw,vel,gyroratings\n')
# Supondo que 'pwml' e 'pwmr' são os sinais de controle para os motores
inputs = data[['TL', 'TR']].values

# Supondo que 'x', 'y', 'bearing' são as saídas medidas
outputs = data[['vel', 'gyroratings']].values

def nobjective_function(r, L, k):
    params = [r, L, k]
    predicted_outputs = robot_dynamics(params, inputs)
    error = np.sum((outputs - predicted_outputs)**2)
    return error

optimal_pars, details, _ = optunity.minimize(nobjective_function, num_evals=10000, r=[.10, .5], L = [0.3,1.5], k=[0.1,1000])

print(optimal_pars)
nparam = [optimal_pars['r'],optimal_pars['L'],optimal_pars['k']]
predicted_outputs = robot_dynamics(nparam, inputs)



# #
# #
# #
# # Resultados da otimização
# estimated_params = result.x
# print(estimated_params)
#
# # Calculando as saídas estimadas usando os parâmetros otimizados
# predicted_outputs = robot_dynamics(estimated_params, inputs, outputs)



#
# Plotando os resultados: sistema real vs sistema estimado
plt.figure(figsize=(15, 5))
print(max(outputs[:, 0]))
# Plotando x
plt.subplot(1, 3, 1)
plt.plot(data['time_seconds'], outputs[:, 0]/max(outputs[:, 0]), label='Real x')
plt.plot(data['time_seconds'], predicted_outputs[:, 0]/max(predicted_outputs[:, 0]), label='Estimado x', linestyle='--')
plt.xlabel('Tempo (s)')
plt.ylabel('Posição em x')
plt.title('Comparação em x')
plt.legend()


# Plotando theta
plt.subplot(1, 3, 3)
plt.plot(data['time_seconds'], outputs[:, 1], label='Real theta')
plt.plot(data['time_seconds'], predicted_outputs[:, 1], label='Estimado theta', linestyle='--')
plt.xlabel('Tempo (s)')
plt.ylabel('Ângulo theta')
plt.title('Comparação em theta')
plt.legend()

plt.tight_layout()
plt.show()