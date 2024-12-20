Aqui está um relatório completo, com base nas comparações detalhadas das diferenças entre os códigos da versão 4.4.0 e 4.7.0 do ArduPilot.

---

## **Relatório de Comparação entre as Versões 4.4.0 e 4.7.0 do ArduPilot: Análise das Diferenças no Código**

### **1. Introdução**
O ArduPilot, na versão 4.4.0 e na versão 4.7.0, apresenta mudanças significativas no código que impactam a modularidade, o controle de dirigíveis, e as operações matemáticas e físicas. Este relatório tem como objetivo comparar essas duas versões, destacando as principais diferenças nas declarações de funções, nas bibliotecas utilizadas e nas lógicas de controle de entradas e saídas.

### **2. Diferenças nas Bibliotecas Inclusas**

#### **Versão 4.4.0**
- **Bibliotecas principais**:
  - `AP_Baro/AP_Baro.h`: Para leitura de dados do barômetro.
  - `AP_HAL/AP_HAL.h`: Abstração de hardware de baixo nível.
  - `AP_Vehicle/AP_Vehicle.h`: Controle do veículo, incluindo a lógica de voo.

#### **Versão 4.7.0**
- **Bibliotecas principais**:
  - Inclui as mesmas bibliotecas da versão 4.4.0, com **adição das bibliotecas**:
    - `AP_Math/AP_Math.h`: Para operações matemáticas mais complexas, incluindo manipulação de matrizes e álgebra linear.
    - `AP_Notify/AP_Notify.h`: Para notificações e controle de displays ou alertas.

Essa adição de bibliotecas sugere que a versão 4.7.0 traz **cálculos matemáticos e feedback mais detalhados**, permitindo maior flexibilidade na manipulação de dados e controle do dirigível.

### **3. Declarações de Funções e Estrutura do Código**

#### **Funções na Versão 4.4.0**
As funções da versão 4.4.0 são mais simples e diretas, focando principalmente no **controle manual** e **cálculos básicos**.

Exemplo de funções na versão 4.4.0:
```cpp
void ZefControl::mult_mat(const float matrix[12][12], const double vector[12], double (&results)[12]);
void ZefControl::find_matrix(const double longit_speed);
void ZefControl::add_traction(double (&U_array)[12], double longit_speed);
void ZefControl::adjust_for_manual(double (&U_array)[12]);
void ZefControl::manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch);
```
Essas funções lidam com operações como multiplicação de matrizes, ajuste de entradas de controle e cálculo da tração. O código não lida com matrizes de controle dinâmicas ou coeficientes ajustáveis.

#### **Funções na Versão 4.7.0**
A versão 4.7.0 introduz uma **maior modularização** e uma estrutura de código mais robusta, com **funções mais complexas** para lidar com novos cálculos e sensores, como a manipulação de quaternions, o controle dos infladores e o ajuste de posição.

Exemplo de funções na versão 4.7.0:
```cpp
void ZefControl::mult_mat(const float matrix[8][12], const double vector[12], double (&results)[12]);
void ZefControl::add_traction(double (&U_array)[12], double longit_speed);
void ZefControl::adjust_for_manual(double (&U_array)[12]);
void ZefControl::manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch);
void ZefControl::set_RHO(double rho);
void ZefControl::getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]);
void ZefControl::rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector);
void ZefControl::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit);
```
Principais melhorias:
- **Controle de infladores** (`operateInflators`) com base na diferença de pressão do barômetro.
- **Manipulação de quaternions** (`rotate_inertial_to_body`) para rotacionar vetores do referencial inercial para o referencial da aeronave.
- **Cálculo do erro de posição** (`getPositionError`) para corrigir a posição do dirigível.
- **Ajuste dinâmico da densidade do ar** (`set_RHO`).

Essas mudanças aumentam a **precisão**, **modularidade** e **flexibilidade** no controle do dirigível.

### **4. Diferenças no Controle de Força e Tracionamento**

#### **Versão 4.4.0**
Na versão 4.4.0, o cálculo da força dos motores é direto e ajustado manualmente. A função `add_traction` adiciona um valor fixo de tração, baseado em parâmetros como **coeficiente de arrasto** e **volume**:
```cpp
void ZefControl::add_traction(double (&U_array)[12], double longit_speed) {
    double Cd = 0.03;
    double Volume_2_3 = 9.0;
    for (int i = 0; i < 8; i++) {
        if(i % 2 == 0) {
            double f_static = 0.5 * RHO_air_density * Cd * Volume_2_3 * (powf(longit_speed, 2) / 4);
            U_array[i] += f_static;
        }
    }
}
```

#### **Versão 4.7.0**
Na versão 4.7.0, a tração é calculada de forma mais complexa, incluindo a **força antagonista** e coeficientes ajustáveis. A função `add_traction` agora adiciona a força antagonista em determinados eixos e calcula a tração baseada na velocidade longitudinal:
```cpp
void ZefControl::add_traction(double (&U_array)[12], double longit_speed) {
    double Cd = 0.03;
    double Volume_2_3 = 9.0;
    for (int i = 0; i < 8; i++) {
        if(i % 2 == 0) {
            double f_static = 0.5 * RHO_air_density * Cd * Volume_2_3 * (powf(longit_speed, 2) / 4);
            U_array[i] += f_static;
        }
    }
    U_array[0] += antagonist_force;
    U_array[2] += antagonist_force * -1;
    U_array[4] += antagonist_force;
    U_array[6] += antagonist_force * -1;
}
```
Isso permite um **controle mais preciso** da força e tração nos motores.

### **5. Controle Manual**

#### **Versão 4.4.0**
A versão 4.4.0 lida diretamente com as entradas dos controles manuais, fazendo ajustes simples em cada canal de controle (aileron, elevator, rudder, throttle):
```cpp
void ZefControl::manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch) {
    J1_cmd_roll = aileron / 1.0;
    J2_cmd_pitch = elevator / 1.0;
    J3_cmd_throttle = throttle / 1.0;
    J4_cmd_yaw = rudder / 1.0;
}
```

#### **Versão 4.7.0**
A versão 4.7.0 introduz **normalização** e **zonas mortas** para garantir que os comandos de controle sejam mais precisos e responsivos. O ajuste das entradas agora leva em conta zonas mortas para cada eixo de controle:
```cpp
double control_aileron = rc().channel(CH_1)->get_control_in();
control_aileron /= normalizador_inputs_apy;
control_aileron = zefiroControl.dead_zone(control_aileron, dead_zone);
zefiroControl.manual_inputs_update(control_aileron, control_pitch, control_yaw, control_throttle, control_right_switch, control_left_switch);
```
Isso melhora a **precisão** e **suaviza as transições** nos comandos de controle.

### **6. Controle de Estabilizadores e Infladores**

#### **Versão 4.4.0**
O controle de estabilizadores e infladores é mais simples e direto. Não há lógica avançada para a manipulação de infladores baseada em pressão:
```cpp
void ZefControl::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit);
```

#### **Versão 4.7.0**
Na versão 4.7.0, o controle dos infladores é **dinâmico** e depende da **diferença de pressão** medida pelo barômetro. Além disso, a função `operateInflators` é mais detalhada, permitindo a ativação ou desativação dos infladores com base em limites de pressão:
```cpp
void ZefControl::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit) {
    double pressure_diff = (barometer->get_pressure(1) - barometer->get_pressure(0)) * 0.01;
    if (pressure_diff < min_p && inflator_state == 0) {
        inflator_state = 1;
        hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin, 1);
    } else {
        if (inflator_state == 1 && pressure_diff > max_p) {
            inflator_state = 0;
        }
    }
}
```
Isso permite um **controle mais eficiente** da pressão interna dos infladores, garantindo melhor estabilidade.

### **7. Conclusão**

A **versão 4.7.0** do ArduPilot oferece uma série de **melhorias significativas** em relação à versão 4.4.0, com maior **modularidade**, **precisão** e **flexibilidade**. A introdução de funções como o controle de infladores, a manipulação de quaternions, o cálculo de erros de posição, e o uso de matrizes dinâmicas melhora consideravelmente a capacidade de controle do dirigível. Além disso, o código agora lida de forma mais sofisticada com as entradas manuais e o cálculo das forças, utilizando coeficientes ajustáveis e técnicas de normalização para garantir a precisão e suavidade do controle.
