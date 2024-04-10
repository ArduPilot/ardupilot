import pygame
import sys

pygame.init()

# Configurações da janela
largura, altura = 400, 300
janela = pygame.display.set_mode((largura, altura))
pygame.display.set_caption("Leitura de Controle Remoto")

# Configurações de fonte
fonte = pygame.font.Font(None, 36)

# Inicialização do controle
num_controles = pygame.joystick.get_count()
if num_controles == 0:
    print("Nenhum controle encontrado.")
    pygame.quit()
    sys.exit()

controle = pygame.joystick.Joystick(0)
controle.init()

print("Controle conectado:", controle.get_name())

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Limpar a tela
        janela.fill((255, 255, 255))

        # Ler os valores dos botões
        for i in range(controle.get_numbuttons()):
            button_state = controle.get_button(i)
            text = fonte.render(f"Botão {i+1}: {button_state}", True, (0, 0, 0))
            janela.blit(text, (20, 20 + i * 30))

        # Ler os valores dos eixos
        for i in range(controle.get_numaxes()):
            axis_value = controle.get_axis(i)
            text = fonte.render(f"Eixo {i+1}: {axis_value:.2f}", True, (0, 0, 0))
            janela.blit(text, (200, 20 + i * 30))

        pygame.display.flip()

except KeyboardInterrupt:
    pygame.quit()
