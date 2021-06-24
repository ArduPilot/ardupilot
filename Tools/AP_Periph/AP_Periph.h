#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include "SRV_Channel/SRV_Channel.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "../AP_Bootloader/app_comms.h"
#include "hwing_esc.h"
#include <AP_CANManager/AP_CANManager.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/CANIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#endif

#if defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_NCP5623_LED_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_NCP5623_BGR_LED_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_TOSHIBA_LED_WITHOUT_NOTIFY)
#define AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY
#endif

#ifdef HAL_PERIPH_ENABLE_NOTIFY
    #ifndef HAL_PERIPH_ENABLE_RC_OUT
        #error "HAL_PERIPH_ENABLE_NOTIFY requires HAL_PERIPH_ENABLE_RC_OUT"
    #endif
    #ifdef HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY
        #error "You cannot enable HAL_PERIPH_ENABLE_NOTIFY and HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY at the same time. Notify already includes it"
    #endif
    #ifdef AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY
        #error "You cannot enable HAL_PERIPH_ENABLE_NOTIFY and any HAL_PERIPH_ENABLE_<device>_LED_WITHOUT_NOTIFY at the same time. Notify already includes them all"
    #endif
    #ifdef HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY
        #error "You cannot use HAL_PERIPH_ENABLE_NOTIFY and HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY at the same time. Notify already includes it. Set param OUTx_FUNCTION=120"
    #endif
#endif

#if defined(HAL_PERIPH_ENABLE_BATTERY_MPPT_PACKETDIGITAL) && HAL_MAX_CAN_PROTOCOL_DRIVERS < 2
#error "Battery MPPT PacketDigital driver requires at least two CAN Ports"
#endif


#include "Parameters.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init();
void stm32_watchdog_pat();
#endif
/*
  app descriptor compatible with MissionPlanner
 */
extern const struct app_descriptor app_descriptor;

class AP_Periph_FW {
public:
    AP_Periph_FW();

    CLASS_NO_COPY(AP_Periph_FW);

    static AP_Periph_FW* get_singleton()
    {
        if (_singleton == nullptr) {
            AP_HAL::panic("AP_Periph_FW used before allocation.");
        }
        return _singleton;
    }

    void init();
    void update();

    Parameters g;

    void can_start();
    void can_update();
    void can_mag_update();
    void can_gps_update();
    void can_baro_update();
    void can_airspeed_update();
    void can_rangefinder_update();
    void can_battery_update();

    void load_parameters();
    void prepare_reboot();

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
    void check_for_serial_reboot_cmd(const int8_t serial_index);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static ChibiOS::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static HALSITL::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];
#endif

    AP_SerialManager serial_manager;

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY
    struct AP_Periph_Battery {
        void handle_battery_failsafe(const char* type_str, const int8_t action) { }
        AP_BattMonitor lib{0, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::AP_Periph_Battery::handle_battery_failsafe, void, const char*, const int8_t), nullptr};

        uint32_t last_read_ms;
        uint32_t last_can_send_ms;
    } battery;
#endif

#if HAL_NUM_CAN_IFACES >= 2
    // This allows you to change the protocol and it continues to use the one at boot.
    // Without this, changing away from UAVCAN causes loss of comms and you can't
    // change the rest of your params or veryofy it suceeded.
    AP_CANManager::Driver_Type can_protocol_cached[HAL_NUM_CAN_IFACES];
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    struct {
        AP_MSP msp;
        MSP::msp_port_t port;
        uint32_t last_gps_ms;
        uint32_t last_baro_ms;
        uint32_t last_mag_ms;
        uint32_t last_airspeed_ms;
    } msp;
    void msp_init(AP_HAL::UARTDriver *_uart);
    void msp_sensor_update(void);
    void send_msp_packet(uint16_t cmd, void *p, uint16_t size);
    void send_msp_GPS(void);
    void send_msp_compass(void);
    void send_msp_baro(void);
    void send_msp_airspeed(void);
#endif
    
#ifdef HAL_PERIPH_ENABLE_ADSB
    void adsb_init();
    void adsb_update();
    void can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg);
    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    void pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    void pwm_hardpoint_init();
    void pwm_hardpoint_update();
    struct {
        uint8_t last_state;
        uint32_t last_ts_us;
        uint32_t last_send_ms;
        uint16_t pwm_value;
        uint16_t highest_pwm;
    } pwm_hardpoint;
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    HWESC_Telem hwesc_telem;
    void hwesc_telem_update();
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
    SRV_Channels servo_channels;
    bool rcout_has_new_data_to_update;

    void rcout_init();
    void rcout_init_1Hz();
    void rcout_esc(int16_t *rc, uint8_t num_channels);
    void rcout_srv(const uint8_t actuator_id, const float command_value);
    void rcout_update();
    void rcout_handle_safety_state(uint8_t safety_state);
#endif


#if defined(HAL_PERIPH_ENABLE_NOTIFY) || defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY)
    void update_rainbow();
#endif
#ifdef HAL_PERIPH_ENABLE_NOTIFY
    // notification object for LEDs, buzzers etc
    AP_Notify notify;
#endif

#if HAL_LOGGING_ENABLED
    static const struct LogStructure log_structure[];
    AP_Logger logger;
#endif

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;

    static AP_Periph_FW *_singleton;

    // show stack as DEBUG msgs
    void show_stack_free();
};

namespace AP
{
    AP_Periph_FW& periph();
}

extern AP_Periph_FW periph;

extern "C" {
void can_printf(const char *fmt, ...);
}

