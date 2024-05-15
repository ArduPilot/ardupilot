import math
def point_relative_to_vector(ax, ay, bx, by, px, py):
    """
    Determina a posição relativa do ponto P(px, py) em relação ao vetor AB(ax, ay) -> (bx, by).

    Retorna:
    -1 : Ponto P está à direita do vetor AB
     1 : Ponto P está à esquerda do vetor AB
     0 : Ponto P está na linha do vetor AB
    """
    # Componentes do vetor AB
    v_x = bx - ax
    v_y = by - ay

    # Produto vetorial em 2D
    cross_product = (v_x) * (py - ay) - (v_y) * (px - ax)

    if cross_product > 0:
        return 1  # P está à esquerda
    elif cross_product < 0:
        return -1  # P está à direita
    else:
        return 0  # P está na linha

   
def vector_magnitude(x, y):
    """Calcula a magnitude de um vetor."""
    return math.sqrt(x**2 + y**2)

def dot_product(u_x, u_y, v_x, v_y):
    """Calcula o produto escalar de dois vetores."""
    return u_x * v_x + u_y * v_y

def calculate_angle(u_x, u_y, v_x, v_y):
    """Calcula o ângulo entre dois vetores."""
    dot = dot_product(u_x, u_y, v_x, v_y)
    mag_u = vector_magnitude(u_x, u_y)
    mag_v = vector_magnitude(v_x, v_y)
    cos_theta = dot / (mag_u * mag_v)
    # Assegura que o valor de cos_theta esteja no intervalo válido para acos
    cos_theta = max(min(cos_theta, 1), -1)
    angle = math.acos(cos_theta)  # Resultado em radianos
    return math.degrees(angle)  # Converte o resultado para graus

# Vetores (1, 2) e (2, 2)
v_x, v_y = 1,2
u_x, u_y = 2, 2
angle = calculate_angle(u_x, u_y, v_x, v_y)
dot = dot_product(u_x, u_y, v_x, v_y)
print(f"Ângulo e dot entre os vetores (em graus): {angle},{dot}")

# Exemplo de uso
ax, ay = 0, 0
bx, by = 2, 2
px, py = 2, 1

result = point_relative_to_vector(ax, ay, bx, by, px, py)
print(f"resultado : {result}")
