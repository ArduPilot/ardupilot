#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>

//#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
/*#include <AP_HAL_ChibiOS/hwdef/common/halconf.h>
#include <../modules/ChibiOS/os/hal/templates/mcuconf.h>
#include <../modules/ChibiOS/os/hal/templates/hal_lld.h>
#include <../modules/ChibiOS/os/hal/templates/hal_i2c_lld.h>
#include <AP_HAL_ChibiOS/I2CDevice.h>*/

#define HAL_USE_I2C     TRUE
#define HAL_USE_I2S     FALSE
/*extern "C" {
    #include <stm32f47_mcuconf.h>
    #include <stm32_registry.h>
    #include <osal.h>
    #include <stm32_dma.h>
    #include <hal_lld.h>
    #include <hal_i2c.h>
}*/

#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/Display.h>
//#include <AP_Notify/Display_>

#include "Plane.h"
#include "zef_control.h"

using namespace AP;

//static Display_tca_i2c tca9548a;
//AP_Notify ap_notify;

//função para multiplicação matricial (pode usar biblioteca também)
void ZefControl::mult_mat(const float matrix[12][12], const double vector[12], double (&results)[12]) {
    int rows = 12;
    int cols = 12;
    for (int i = 0; i < rows; i++) {
        results[i] = 0.0;
        for (int j = 0; j < cols; j++) {
            results[i] += ((double)matrix[i][j]) * vector[j];
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "linha %d >>> valor: %f --- vector: %f --- result: %f", i, matrix[i][j], vector[i], ((double)matrix[i][j]) * vector[j]);
        }
        //if( i < 7 ) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "U%d: %.2e", i, results[i]);
        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%d >>> X: %f --- Y: %f --- matriz: %d", i, results[i], vector[i], ref_index);
    }
    /*
    # função para multiplicação matricial (pode usar biblioteca também)
    funcao mult_mat(T(i)(j), Y(j)) retornando X(i):
        para i de 0 a tamanho (T,1):
            X(i) = 0
            para j de 0 a tamanho (V):
                X(i) = X(i) + T(i)(j)*Y(j)
            fim
        fim
    fim
    */
}

void ZefControl::find_matrix(const double longit_speed) {
    int max_options = sizeof(speed_refs);
    ref_index = max_options - 1;
    for (int i = max_options-1; i >= 0; i--) {
        if (longit_speed < speed_refs[i]) {
            ref_index = i;
        }
    }
    //ajusta o indice para ficar no range de k_matrix_all
    int len = sizeof(k_matrix_all) / sizeof(k_matrix_all[0]);
    if(ref_index > len-1) ref_index = len-1;

    /*
    # encontra a matriz K a ser utilizada
    int max_options = 3;
    ref_index = max_options - 1;
    for (int i = max_options-1; i >=0; i--) {
        if (longit_speed < speed_refs[i]) {
            ref_index = i;
        }

    */
}

void ZefControl::add_traction(double (&U_array)[12], double longit_speed) {
    double Cd = 0.03;
    double Volume_2_3 = 9.0;
    for (int i = 0; i < 8; i++) {
        if(i % 2 == 0) {
            //double old_U = U_array[i];
            double f_static = 0.5*RHO_air_density*Cd*Volume_2_3*(powf(longit_speed,2)/4);
            U_array[i] += f_static;
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%d >>> rho_density: %f --- longit_speed: %f --- Volume_2_3: %f", i, RHO_air_density, longit_speed, Volume_2_3);
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%d >>> old_U: %f --- new_U: %f --- f: %f", i, old_U, U_array[i], f_static);
        }
    }
    
    U_array[0] += antagonist_force;
    U_array[2] += antagonist_force*-1;
    U_array[4] += antagonist_force;
    U_array[6] += antagonist_force*-1;

    /*# adicionar a tração para manter a posição (@carlos, favor confirmar se entra separado realmente)
    --> impares pois são a componente X... Y são pares
    U(1)=U(1)+1/2*rho*U^2/4
    U(3)=U(3)+1/2*rho*U^2/4
    U(5)=U(5)+1/2*rho*U^2/4
    U(7)=U(7)+1/2*rho*U^2/4*/
}

void ZefControl::adjust_for_manual(double (&U_array)[12]) {
/*    double ajuste_cal_roll_mot = 0.15;
    double ajuste_cal_pitch_mot = 0.3;
    double ajuste_cal_yaw_mot = 0.3;
    double ajuste_cal_cima_baixo = 0.3;
    double ajuste_cal_roll_estab = 0.3;
    double ajuste_cal_dir_esq = 0.3;
    double center_value = 0.0;*/

    double ajuste_cal_roll_mot = 0.3;
    double ajuste_cal_pitch_mot = 0.6;
    double ajuste_cal_yaw_mot = 0.6;
    double ajuste_cal_cima_baixo = 0.6;
    double ajuste_cal_roll_estab = 0.3;
    double ajuste_cal_dir_esq = 0.3;
    double center_value = 0.0;

    /*double ajuste_cal_roll_mot = plane.g2.cal_roll_mot;
    double ajuste_cal_pitch_mot = plane.g2.cal_pitch_mot;
    double ajuste_cal_yaw_mot = plane.g2.cal_yaw_mot;
    double ajuste_cal_cima_baixo = plane.g2.cal_cima_baixo;
    double ajuste_cal_roll_estab = plane.g2.cal_roll_estab;
    double ajuste_cal_dir_esq = plane.g2.cal_dir_esq;
    double center_value = 0.0;*/

	U_array[0] = J3_cmd_throttle + ajuste_cal_pitch_mot*J2_cmd_pitch;
	U_array[1] = -ajuste_cal_dir_esq*RS_cmd_up_down;

	U_array[2] = J3_cmd_throttle - ajuste_cal_yaw_mot*J4_cmd_yaw - ajuste_cal_pitch_mot*J2_cmd_pitch/2;
	U_array[3] = ajuste_cal_cima_baixo*LS_cmd_letf_right + ajuste_cal_roll_mot*J1_cmd_roll;

	U_array[4] = J3_cmd_throttle - ajuste_cal_pitch_mot*J2_cmd_pitch;
	U_array[5] = ajuste_cal_dir_esq*RS_cmd_up_down + ajuste_cal_roll_mot*J1_cmd_roll;

	U_array[6] = J3_cmd_throttle + ajuste_cal_yaw_mot*J4_cmd_yaw - ajuste_cal_pitch_mot*J2_cmd_pitch/2;
	U_array[7] = -ajuste_cal_cima_baixo*LS_cmd_letf_right + ajuste_cal_roll_mot*J1_cmd_roll;

	U_array[8] = center_value + J4_cmd_yaw + (ajuste_cal_roll_estab*J1_cmd_roll); //(leme superior)
	U_array[9] = center_value + J2_cmd_pitch + (ajuste_cal_roll_estab*J1_cmd_roll); //(profundor direito)

	U_array[10] = center_value + (-J4_cmd_yaw) + (ajuste_cal_roll_estab*J1_cmd_roll); //(leme inferior)
	U_array[11] = center_value + (-J2_cmd_pitch) + (ajuste_cal_roll_estab*J1_cmd_roll); //(profundor esquerdo)

    /*for(int i = 0; i < 8; i+=2) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%d: %.2f , %.2f", i, U_array[i], U_array[i+1]);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=");*/

    for(int i = 0; i < 8; i++) {
        U_array[i] *= F_max_mot;
    }

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "leme superior: %f", U_array[8]);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "profundor direito: %f", U_array[9]);

    /*
	# obter os comandos do radio controle
	J1 = obter o comando do stick do aileron
	J2 = obter o comando do stick do profundor
	J3 = obter o comando do stick do acelerador
	J4 = Obter o comando do stick do leme

	# Ganhos do controlador manual (pode entrar como variável global)
	define ab ajuste_cal_roll_mot, a2 ajuste_cal_pitch_mot, a4 ajuste_cal_yaw_mot, al ajuste_cal_cima_baixo, aa ajuste_cal_roll_estab, ar ajuste_cal_dir_esq

	U(1) = J3 + ajuste_cal_pitch_mot*J2
	U(2) = ajuste_cal_dir_esq*RS + ajuste_cal_roll_mot*J1
	U(3) = J3 - ajuste_cal_yaw_mot*J4
	U(4) = ajuste_cal_cima_baixo*LS + ajuste_cal_roll_mot*J1
	U(5) = J3 - ajuste_cal_pitch_mot*J2
	U(6) = ajuste_cal_dir_esq*RS + ajuste_cal_roll_mot*J1
	U(7) = J3 + ajuste_cal_yaw_mot*J4
	U(8) = ajuste_cal_cima_baixo*LS + ajuste_cal_roll_mot*J1
	U(9) = J4 + ajuste_cal_roll_estab*J1 (leme superior)
	U(10) = J2 + ajuste_cal_roll_estab*J1 (profundor direito)
	U(11) = -J4 + ajuste_cal_roll_estab*J1 (leme inferior)
	U(12) = -J2 + ajuste_cal_roll_estab*J1 (profundor esquerdo)*/
}

void ZefControl::manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch) {
    double multiplicador_comandos = 1.0; //4500;
    double multiplicador_motor = 1.0; //100;
    J1_cmd_roll = aileron/multiplicador_comandos;
    J2_cmd_pitch = elevator/multiplicador_comandos;
    J3_cmd_throttle = throttle/multiplicador_motor;
    J4_cmd_yaw = rudder/multiplicador_comandos;
    RS_cmd_up_down = right_switch;
    LS_cmd_letf_right = left_switch;

    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J1: %f", J1);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J2: %f", J2);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J3: %f", J3);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J4: %f", J4);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RS: %f", RS);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LS: %f", LS);*/
}

double ZefControl::set_angle_range(double last_angle, double v1, double v0) {
    double pi = 3.14159265358979311600;
    double Ang_max = 210/180*pi;
    double d_angulo_motor_ant = last_angle;
    double d_angulo_motor = atan2f(v1, v0);
    if (d_angulo_motor>(2*pi-Ang_max) && d_angulo_motor_ant < -(2*pi-Ang_max) ) {
        d_angulo_motor =  -(2*pi-d_angulo_motor);
    }
    if (d_angulo_motor<-(2*pi-Ang_max) && d_angulo_motor_ant > (2*pi-Ang_max) ) {
        d_angulo_motor =  (2*pi-d_angulo_motor);
    }
    return d_angulo_motor;
}

double ZefControl::get_engine_command(double motor_force) {
    double command = 0.0;
    if( motor_force >= F_min_mot ) {
        if (motor_force > F_max_mot) {
            command = comando_maximo;
        } else {
            command =  (coeficiente_quadratico_forca * powf(motor_force, 2)) + (coeficiente_linear_forca * motor_force) + coeficiente_nulo_forca;
        }
    }

    return command;
}

void ZefControl::set_power_and_angles(double (&U)[12]) {
	double ang_anterior = 0.0;
    double coef_antec_mov = 0.5;
    double f_min_servo = F_min_mot * coef_antec_mov;

    // Encontra a força dos motores absoluta e o angulo dos motores
    F1_forca_motor = sqrtf(powf(U[0], 2) + powf(U[1], 2));
    if( F1_forca_motor >= f_min_servo ) {
        ang_anterior = d1_angulo_motor;
        d1_angulo_motor = set_angle_range(ang_anterior, U[1], U[0]);
    }

    F2_forca_motor = sqrtf(powf(U[2], 2) + powf(U[3], 2));
    if( F2_forca_motor >= f_min_servo ) {
        ang_anterior = d2_angulo_motor;
        d2_angulo_motor = set_angle_range(ang_anterior, U[3], U[2]);
    }

    F3_forca_motor = sqrtf(powf(U[4], 2) + powf(U[5], 2));
    if( F3_forca_motor >= f_min_servo ) {
        ang_anterior = d3_angulo_motor;
        d3_angulo_motor = set_angle_range(ang_anterior, U[5], U[4]);
    }

    F4_forca_motor = sqrtf(powf(U[6], 2) + powf(U[7], 2));
    if( F4_forca_motor >= f_min_servo ) {
        ang_anterior = d4_angulo_motor;
        d4_angulo_motor = set_angle_range(ang_anterior, U[7], U[6]);
    }

    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[0], U[1], powf(U[0], 2), powf(U[1], 2), F1_forca_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[2], U[3], powf(U[2], 2), powf(U[3], 2), F2_forca_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[4], U[5], powf(U[4], 2), powf(U[5], 2), F3_forca_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[6], U[7], powf(U[6], 2), powf(U[7], 2), F4_forca_motor);*/

    dvu_ang_estab_vert_cima = U[8];
    dvd_ang_estab_vert_baixo = U[9];

    dhr_ang_estab_horiz_direito = U[10];
    dhl_ang_estab_horiz_esquerdo = U[11];

    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1: %f -- ang: %f", F1_forca_motor, d1_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2: %f -- ang: %f", F2_forca_motor, d2_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3: %f -- ang: %f", F3_forca_motor, d3_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4: %f -- ang: %f", F4_forca_motor, d4_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");*/
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang v cima: %f -- ang v baixo: %f", dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang h dir: %f -- ang h esq: %f", dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo);*/
}

void ZefControl::put_forces_in_range() {
    /*
    Encontra o comando necessário para gerar a força desejada
    com base em uma função quadrática de ajuste.
    Se a força for menor que a mínima, zera e se for maior que a máxima, limita pela máxima.
    */

    comando_M1 = get_engine_command(F1_forca_motor);
    comando_M2 = get_engine_command(F2_forca_motor);
    comando_M3 = get_engine_command(F3_forca_motor);
    comando_M4 = get_engine_command(F4_forca_motor);
}

void ZefControl::print_output_data() {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1: %f -- ang: %f", F1_forca_motor, d1_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2: %f -- ang: %f", F2_forca_motor, d2_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3: %f -- ang: %f", F3_forca_motor, d3_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4: %f -- ang: %f", F4_forca_motor, d4_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang v cima: %f -- ang v baixo: %f", dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang h dir: %f -- ang h esq: %f", dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo);
}

double sign(double x);
double min(double x, double y);

double sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

double min(double x, double y) {
    if(x < y) return x;
    if(y < x) return y;
    return x;
}

void ZefControl::update(
        double U_longit_speed, double V_lateral_speed, double W_vertical_speed,
        double P_v_roll, double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw,
        double x_position_n_s, double y_position_e_w, double z_position_height) {
    //ajuste de saída para controle manual aqui
    if(is_manual_mode) {
        double U[12];
        adjust_for_manual(U);
        set_power_and_angles(U); //now set again for use
        put_forces_in_range();
        //print_output_data();
    } else {
        double X[12] = {
            U_longit_speed,
            V_lateral_speed,
            W_vertical_speed,
            P_v_roll,
            Q_v_pitch,
            R_v_yaw,
            Roll,
            Pitch,
            Yaw,
            x_position_n_s,
            y_position_e_w,
            z_position_height,
        };
        //double X[12] = {1,1,1,1,1,1,1,1,1,1,1,1};

        /*if (semaforo == 0) {
            for(int i = 0; i < 12;i+=3) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "X%d: %.2e, %.2e, %.2e", i, X[i], X[i+1], X[i+2]);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");
        }*/

        /*double w_speed = 0.0;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "dir: %.2f", wind_direction);
        double direction = 30*(3.141592653589793 / 180.0);
        if(wind_direction < direction || wind_direction > -direction) {
            w_speed = wind_speed;
        }
        find_matrix(w_speed);*/
        //If (sign(U)*U^2/(U^2+V^2+W^2)>cos(30/180*pi)

        double w_speed = 0.0;
        double pi = 3.141592653589793;
        if (sign(U_longit_speed)*powf(U_longit_speed,2)/(powf(U_longit_speed,2)+powf(V_lateral_speed,2)+powf(W_vertical_speed,2))>cosf(30/180*pi)) {
            w_speed = U_longit_speed;
        }
        if(plane.g.matrix_index == -1) {
            find_matrix(w_speed);
        } else {
            int max_option = sizeof(speed_refs) - 1;
            int curr_index = plane.g.matrix_index;
            if(curr_index > max_option) curr_index = max_option;
            find_matrix(curr_index);
        }

        double U[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double vetor_erro[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double X_ref[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double alpha = 0.2;
        for( int i = 0; i < 12; i++) {
            X[i] = (last_X[i] * (1-alpha)) + (X[i] * alpha);
        }
        for( int i = 0; i < 9; i++) {
            vetor_erro[i] = X_ref[i] - X[i];
        }
        // Calcula o erro de posição, projetado na direção do dirigível
        // Para implementação posterior
        for ( int i = 9; i < 12; i++) {
            vetor_erro[i] = X_ref[i] - X[i];
        }

        /*if (1) {
            for(int i = 0; i < 12;i+=3) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "vetor_erro%d: %.2e, %.2e, %.2e", i, vetor_erro[i], vetor_erro[i+1], vetor_erro[i+2]);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");
        }*/

        mult_mat(k_matrix_all[ref_index], vetor_erro, U);
        memcpy(last_X, X, sizeof(last_X));

        /*if (1) {
            for(int i = 0; i < 12;i+=3) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "U%d: %.2e, %.2e, %.2e", i, U[i], U[i+1], U[i+2]);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");
        }*/
        ///*if (semaforo == 1){semaforo = 0;}else{semaforo = 1;}*/

        add_traction(U, U_longit_speed);
        //print_output_data();
        set_power_and_angles(U);
        put_forces_in_range(); //converted to -1 to 1
    }

}

void ZefControl::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit) {
    //TCA9548A(3);

    //ap_notify.update();
    //TCA9548A(2);


    double pressure_diff = (barometer->get_pressure(1) - barometer->get_pressure(0)) * 0.01;
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "press diff: %f", pressure_diff);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "min: %f", min_p);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "state: %d", inflator_state);*/

    if ( pressure_diff < min_p && inflator_state == 0 ) { //plane.g2.min_balloon_pressure_diff
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "inflando");
        inflator_state = 1;
        hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin, 1);
        hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin+1, 1);
    } else {
        if ( inflator_state == 1 && pressure_diff > max_p ) {
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "desinflando");
            inflator_state = 0;
        }
        if ( inflator_state == 1 ) {
            hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin, 1);
            hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin+1, 1);
        } else {
            hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin, 0);
            hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin+1, 0);
        }
    }
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "state: %d", inflator_state);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "press diff: %f", pressure_diff);

    /*double pressure_diff1 = (barometer->get_pressure(0) - barometer->get_pressure(1)) * 0.01;
    double pressure_diff2 = (barometer->get_pressure(0) - barometer->get_pressure(2)) * 0.01;
    double min_pressure = min(pressure_diff1, pressure_diff2);

    if(barometer->get_pressure(1) <= 0) deactivate_unit = 1;
    if(barometer->get_pressure(2) <= 0) deactivate_unit = 2;
    if( deactivate_unit == 1 ) {
        pressure_diff1 = 0;
        min_pressure = pressure_diff2; //inverte pq queremos ignorar o zero
    }
    if( deactivate_unit == 2 ) {
        pressure_diff2 = 0;
        min_pressure = pressure_diff1; //inverte pq queremos ignorar pressao zero
    }

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "press 0: %f >> press 1: %f >> press 2: %f", barometer->get_pressure(0), barometer->get_pressure(1), barometer->get_pressure(2));
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "press diff1: %f", pressure_diff1);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "press diff2: %f", pressure_diff2);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "min_pressure: %f", min_pressure);

    if ( min_pressure < min_p && inflator_state == 0 ) { //plane.g2.min_balloon_pressure_diff
        inflator_state = 1;
        //activates inflator 1
        if( pressure_diff1 < max_p && deactivate_unit != 1) {
            inflator1_state = 1;
            hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin, 1);
        }
        //activates inflator 2
        if( pressure_diff2 < max_p && deactivate_unit != 2) {
            inflator2_state = 1;
            hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin+1, 1);
        }
    } else {
        if ( inflator1_state == 1 && pressure_diff1 > max_p ) {
            inflator1_state = 0;
            hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin, 0);
        }
        if ( inflator2_state == 1 && pressure_diff2 > max_p ) {
            inflator2_state = 0;
            hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin+1, 0);
        }

        if( deactivate_unit == 1 ) inflator1_state = 0;
        if( deactivate_unit == 2 ) inflator2_state = 0;

        if ( inflator_state == 1 && inflator1_state == 0 && inflator2_state == 0 ) {
            inflator_state = 0;
        }
    }*/
}

void ZefControl::stopInflators(){
    inflator_state = 0;
    hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(gpio_pin, 0);
    hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
    hal.gpio->write(gpio_pin+1, 0);
}

// Função para criar um quaternion a partir de roll, pitch e yaw e rotacionar o vetor
Vector3f ZefControl::rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector) {
    // Criar um quaternion a partir dos ângulos de Euler (roll, pitch, yaw)
    Quaternion q;
    q.from_euler(roll, pitch, yaw);

    // Rotacionar o vetor do referencial inercial para o referencial da aeronave
    Vector3f body_vector = inertial_vector;
    q.earth_to_body(body_vector);

    return body_vector;
}

void ZefControl::getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]) {
    double earth_radius = 6.371e6;
    double convert_degrees_to_radians = 0.01745329252;
    double adjust_gps_reading = 1e-7;
    // pi/180
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "d_lat: %d - d_lon %d", (int)desired_position_lat, (int)desired_position_long);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "c_lat: %d - c_lon %d", (int)current_position_lat, (int)current_position_long);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");*/

    double error_x_earth_ref = convert_degrees_to_radians * ( desired_position_lat - current_position_lat ) * adjust_gps_reading * earth_radius;
    double error_y_earth_ref =  convert_degrees_to_radians * ( desired_position_long - current_position_long ) * adjust_gps_reading * earth_radius * cosf ( convert_degrees_to_radians * current_position_lat );
    double error_z_earth_ref = - ( desired_position_alt - current_position_alt )*0.01;
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "x: %f -- y: %f -- z: %f", error_x_earth_ref, error_y_earth_ref, error_z_earth_ref);

    // Negativo por causa do referencial North-East-Down (NED)
    double error_x_airship_ref = sinf( azimute ) * error_x_earth_ref + cosf( azimute ) *  error_y_earth_ref;
    double error_y_airship_ref = cosf( azimute ) * error_x_earth_ref - sinf( azimute ) *  error_y_earth_ref;
    double error_z_airship_ref = error_z_earth_ref;
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>> x: %f -- y: %f -- z: %f", error_x_airship_ref, error_y_airship_ref, error_z_airship_ref);

    ret_errors[0] = error_x_airship_ref;
    ret_errors[1] = error_y_airship_ref;
    ret_errors[2] = error_z_airship_ref;
}

void ZefControl::TCA9548A(uint16_t bus){
    if(tca_initialized == 0) {
        //i2cStart((&tca9548a)->config->i2cp, (&tca9548a)->config->i2ccfg);
        //i2cStart(I2CDriver, NULL);

        tca_initialized = 1;
    }

    //tca9548a.set_pixel(bus, 0);
    /*int ret;
    // calculate a timeout as twice the expected transfer time, and set as min of 4ms
    uint8_t send;
    uint32_t send_len;
    uint8_t *recv;
    uint32_t recv_len;
    uint32_t timeout_ms = 1+2*(((8*1000000UL/400000)*(send_len+recv_len))/1000);
    ret = i2cMasterTransmitTimeout(I2CD[bus].i2c, 0x70, send, send_len,
                                   recv, recv_len, chTimeMS2I(timeout_ms));*/

    /*uint8_t i2cAddress = 0x70;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    _dev = hal.i2c_mgr->get_device(bus, i2cAddress);
    if (_dev) {
        //
    }*/
}

ZefControl zefiroControl;