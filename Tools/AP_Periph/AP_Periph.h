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
#include <AP_Proximity/AP_Proximity.h>
#include <AP_EFI/AP_EFI.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include <AP_TemperatureSensor/AP_TemperatureSensor.h>
#include "../AP_Bootloader/app_comms.h"
#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include "hwing_esc.h"
#include <AP_CANManager/AP_CAN.h>
#include <AP_CANManager/AP_SLCANIface.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_HAL/CANIface.h>
#include <AP_Stats/AP_Stats.h>


#include <AP_NMEA_Output/AP_NMEA_Output.h>
#if HAL_NMEA_OUTPUT_ENABLED && !(HAL_GCS_ENABLED && defined(HAL_PERIPH_ENABLE_GPS))
    // Needs SerialManager + (AHRS or GPS)
    #error "AP_NMEA_Output requires Serial/GCS and either AHRS or GPS. Needs HAL_GCS_ENABLED and HAL_PERIPH_ENABLE_GPS"
#endif

#if HAL_GCS_ENABLED
#include "GCS_MAVLink.h"
#endif

#if defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_NCP5623_LED_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_NCP5623_BGR_LED_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_TOSHIBA_LED_WITHOUT_NOTIFY)
#define AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY
#endif

#ifdef HAL_PERIPH_ENABLE_NOTIFY
    #if !defined(HAL_PERIPH_ENABLE_RC_OUT) && !defined(HAL_PERIPH_NOTIFY_WITHOUT_RCOUT)
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

#include "Parameters.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init();
void stm32_watchdog_pat();
#endif
/*
  app descriptor for firmware checking
 */
extern const app_descriptor_t app_descriptor;

extern "C" {
void can_printf(const char *fmt, ...) FMT_PRINTF(1,2);
}

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
    void send_moving_baseline_msg();
    void send_relposheading_msg();
    void can_baro_update();
    void can_airspeed_update();
    void can_rangefinder_update();
    void can_battery_update();
    void can_proximity_update();

    void load_parameters();
    void prepare_reboot();
    bool canfdout() const { return (g.can_fdmode == 1); }

#ifdef HAL_PERIPH_ENABLE_EFI
    void can_efi_update();
#endif

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
    void check_for_serial_reboot_cmd(const int8_t serial_index);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static ChibiOS::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static HALSITL::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];
#endif

#if AP_CAN_SLCAN_ENABLED
    static SLCAN::CANIface slcan_interface;
#endif

    AP_SerialManager serial_manager;

#if AP_STATS_ENABLED
    AP_Stats node_stats;
#endif

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#if HAL_NUM_CAN_IFACES >= 2
    int8_t gps_mb_can_port = -1;
#endif
#endif

#if HAL_NMEA_OUTPUT_ENABLED
    AP_NMEA_Output nmea;
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
    // change the rest of your params or verify it succeeded.
    AP_CAN::Protocol can_protocol_cached[HAL_NUM_CAN_IFACES];
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
    uint32_t last_sample_ms;
#endif

#ifdef HAL_PERIPH_ENABLE_PRX
    AP_Proximity proximity;
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

#ifdef HAL_PERIPH_ENABLE_EFI
    AP_EFI efi;
    uint32_t efi_update_ms;
#endif

#if AP_KDECAN_ENABLED
    AP_KDECAN kdecan;
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
#if HAL_WITH_ESC_TELEM
    AP_ESC_Telem esc_telem;
    uint32_t last_esc_telem_update_ms;
    void esc_telem_update();
    uint32_t esc_telem_update_period_ms;
#endif

    SRV_Channels servo_channels;
    bool rcout_has_new_data_to_update;

    uint32_t last_esc_raw_command_ms;
    uint8_t  last_esc_num_channels;

    void rcout_init();
    void rcout_init_1Hz();
    void rcout_esc(int16_t *rc, uint8_t num_channels);
    void rcout_srv_unitless(const uint8_t actuator_id, const float command_value);
    void rcout_srv_PWM(const uint8_t actuator_id, const float command_value);
    void rcout_update();
    void rcout_handle_safety_state(uint8_t safety_state);
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
    AP_TemperatureSensor temperature_sensor;
#endif

#if defined(HAL_PERIPH_ENABLE_NOTIFY) || defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY)
    void update_rainbow();
#endif
#ifdef HAL_PERIPH_ENABLE_NOTIFY
    // notification object for LEDs, buzzers etc
    AP_Notify notify;
    uint64_t vehicle_state = 1; // default to initialisation
    float yaw_earth;
    uint32_t last_vehicle_state;

    // Handled under LUA script to control LEDs
    float get_yaw_earth() { return yaw_earth; }
    uint32_t get_vehicle_state() { return vehicle_state; }
#elif defined(AP_SCRIPTING_ENABLED)
    // create dummy methods for the case when the user doesn't want to use the notify object
    float get_yaw_earth() { return 0.0; }
    uint32_t get_vehicle_state() { return 0.0; }
#endif

#if AP_SCRIPTING_ENABLED
    AP_Scripting scripting;
#endif

#if HAL_LOGGING_ENABLED
    static const struct LogStructure log_structure[];
    AP_Logger logger;
#endif

#if HAL_GCS_ENABLED
    GCS_Periph _gcs;
#endif
    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_gps_yaw_ms;
    uint32_t last_relposheading_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;
    bool saw_gps_lock_once;

    static AP_Periph_FW *_singleton;

    enum {
        DEBUG_SHOW_STACK,
        DEBUG_AUTOREBOOT
    };

    // show stack as DEBUG msgs
    void show_stack_free();

    static bool no_iface_finished_dna;
    static constexpr auto can_printf = ::can_printf;
};

namespace AP
{
    AP_Periph_FW& periph();
}

extern AP_Periph_FW periph;


