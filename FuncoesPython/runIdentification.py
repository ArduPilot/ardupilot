import numpy as np
from pandas import Series,DataFrame
import pandas as pd
import sympy as sym
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import seaborn as sns


def retornaAB(dados):
    n = len(dados)
    vel = []
    sigma = []
    b = []
    for i in range(n):
        vel.append(dados.loc[dados.index[i], 'throttle'])
        sigma.append(0)
        b.append(dados.loc[dados.index[i], 'pwml'] + dados.loc[dados.index[i], 'pwmr'] - 3000)

        vel.append(0)
        sigma.append(dados.loc[dados.index[i], 'steering'])
        b.append(-dados.loc[dados.index[i], 'pwml'] + dados.loc[dados.index[i], 'pwmr'])

    A = np.column_stack((vel, sigma))
    B = np.array(b)

    estimacao = np.dot(np.linalg.pinv(A), B)

    ia = estimacao[0]/2
    ib = estimacao[1]/2

    Mi = [[ia, -ib], [ia, ib]]

    return Mi


nclusters = 1
kmeans = KMeans(n_clusters=nclusters)

#('throttle,steering,pmwl,pwmr,x,y')
dadosMissao = pd.read_csv(f'aData.csv')
print(dadosMissao.columns)

dadospCluster = dadosMissao[dadosMissao.columns[[0,1]]]

kmeans.fit(dadospCluster)
y_kmeans = kmeans.predict(dadospCluster)
dadosMissao['cluster'] = y_kmeans

print(kmeans.cluster_centers_)

#
# plt.figure(figsize=(10, 6))
# sns.scatterplot(data=dadosMissao, x='x', y='y', hue='cluster', palette='rainbow')
# plt.title('Gráfico de Dispersão x, y com Clusters de Throttle e Steering')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.legend(title='Cluster')
# plt.grid(True)
# plt.show()


# Definindo o layout do dashboard
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

# Gráfico de dispersão para 'x' e 'y'
sns.scatterplot(data=dadosMissao, x='x', y='y', ax=axs[0, 0])
axs[0, 0].set_title('Gráfico de Dispersão de x vs y')
axs[0, 0].set_xlabel('x')
axs[0, 0].set_ylabel('y')

# Histograma para 'throttle'
sns.histplot(data=dadosMissao, x='throttle', kde=True, ax=axs[0, 1])
axs[0, 1].set_title('Histograma de Throttle')
axs[0, 1].set_xlabel('Throttle')
axs[0, 1].set_ylabel('Frequência')

# Histograma para 'steering'
sns.histplot(data=dadosMissao, x='steering', kde=True, ax=axs[1, 0])
axs[1, 0].set_title('Histograma de Steering')
axs[1, 0].set_xlabel('Steering')
axs[1, 0].set_ylabel('Frequência')

# Gráfico de linha para 'pmwl' e 'pwmr'
axs[1, 1].plot(dadosMissao['pwml'], label='PMWL')
axs[1, 1].plot(dadosMissao['pwmr'], label='PWMR')
axs[1, 1].set_title('Gráfico de Linha de PMWL e PWMR')
axs[1, 1].set_xlabel('Índice')
axs[1, 1].set_ylabel('Valor')
axs[1, 1].legend()

# Ajustando o layout
#plt.tight_layout()
#plt.show()


##### identificacao
#('throttle,steering,pmwl,pwmr,x,y')
for i in range(nclusters):
    dadosC = dadosMissao[dadosMissao.cluster == i]
    Mc = retornaAB(dadosC)
    print(Mc)



#print(dadosMissao)
# A = dadosMissao[dadosMissao.columns[[0,1]]].to_numpy()
# B = dadosMissao[dadosMissao.columns[[2]]].to_numpy()
# Psdeu = np.dot(np.linalg.pinv(A),B)
#
#
# Mab = 1/Psdeu
#
# A = Mab[0][0]
# B = Mab[1][0]
#
# Mi = [[A, A], [-B, B]]
# iM = np.linalg.inv(Mi)
# print(iM)
#
#
# a, b = sym.symbols('a b')
# matrix = sym.Matrix([[a, a], [-b , b]])
# iMn = matrix.inv()
# print(iMn)