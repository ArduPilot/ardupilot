#pragma once

#include <AP_HAL/AP_HAL.h>
#include <canard.h>
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
#include <AP_RPM/AP_RPM.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_ESC_Telem/AP_ESC_Telem_config.h>
#if HAL_WITH_ESC_TELEM
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#endif
#ifdef HAL_PERIPH_ENABLE_RTC
#include <AP_RTC/AP_RTC.h>
#endif
#include <AP_RCProtocol/AP_RCProtocol_config.h>
#include "rc_in.h"
#include "batt_balance.h"
#include "networking.h"
#include "serial_options.h"
#if AP_SIM_ENABLED
#include <SITL/SITL.h>
#endif
#include <AP_AHRS/AP_AHRS.h>

#ifdef HAL_PERIPH_ENABLE_RELAY
#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    #error "Relay and PWM_HARDPOINT both use hardpoint message"
#endif
#include <AP_Relay/AP_Relay.h>
#if !AP_RELAY_ENABLED
    #error "HAL_PERIPH_ENABLE_RELAY requires AP_RELAY_ENABLED"
#endif
#endif

#include <AP_NMEA_Output/AP_NMEA_Output.h>
#if HAL_NMEA_OUTPUT_ENABLED && !(HAL_GCS_ENABLED && defined(HAL_PERIPH_ENABLE_GPS))
    // Needs SerialManager + (AHRS or GPS)
    #error "AP_NMEA_Output requires Serial/GCS and either AHRS or GPS. Needs HAL_GCS_ENABLED and HAL_PERIPH_ENABLE_GPS"
#endif

#if HAL_GCS_ENABLED
#include "GCS_MAVLink.h"
#endif

#include "esc_apd_telem.h"

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

#ifndef AP_PERIPH_SAFETY_SWITCH_ENABLED
#define AP_PERIPH_SAFETY_SWITCH_ENABLED defined(HAL_PERIPH_ENABLE_RC_OUT)
#endif

#ifndef HAL_PERIPH_CAN_MIRROR
#define HAL_PERIPH_CAN_MIRROR 0
#endif

#if defined(HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT) && !defined(HAL_DEBUG_BUILD) && !defined(HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_NON_DEBUG)
/* this checking for reboot can lose bytes on GPS modules and other
 * serial devices. It is really only relevent on a debug build if you
 * really want it for non-debug build then define
 * HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_NON_DEBUG in hwdef.dat
 */
#undef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
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

struct CanardInstance;
struct CanardRxTransfer;

#define MAKE_TRANSFER_DESCRIPTOR(data_type_id, transfer_type, src_node_id, dst_node_id)             \
    (((uint32_t)(data_type_id)) | (((uint32_t)(transfer_type)) << 16U) |                            \
    (((uint32_t)(src_node_id)) << 18U) | (((uint32_t)(dst_node_id)) << 25U))

#ifndef HAL_CAN_POOL_SIZE
#if HAL_CANFD_SUPPORTED
    #define HAL_CAN_POOL_SIZE 16000
#elif GPS_MOVING_BASELINE
    #define HAL_CAN_POOL_SIZE 8000
#else
    #define HAL_CAN_POOL_SIZE 4000
#endif
#endif

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
    void can_battery_send_cells(uint8_t instance);
    void can_proximity_update();
    void can_buzzer_update(void);
    void can_safety_button_update(void);
    void can_safety_LED_update(void);

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

#ifdef HAL_PERIPH_ENABLE_RPM
    AP_RPM rpm_sensor;
    uint32_t rpm_last_update_ms;
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY
    void handle_battery_failsafe(const char* type_str, const int8_t action) { }
    AP_BattMonitor battery_lib{0, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::handle_battery_failsafe, void, const char*, const int8_t), nullptr};
    struct {
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
        uint32_t last_heartbeat_ms;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
    uint32_t last_sample_ms;
#endif

#ifdef HAL_PERIPH_ENABLE_PROXIMITY
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
    
#ifdef HAL_PERIPH_ENABLE_ESC_APD
    ESC_APD_Telem *apd_esc_telem[APD_ESC_INSTANCES];
    void apd_esc_telem_update();
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

#ifdef HAL_PERIPH_ENABLE_RCIN
    void rcin_init();
    void rcin_update();
    void can_send_RCInput(uint8_t quality, uint16_t *values, uint8_t nvalues, bool in_failsafe, bool quality_valid);
    bool rcin_initialised;
    uint32_t rcin_last_sent_RCInput_ms;
    const char *rcin_rc_protocol;  // protocol currently being decoded
    Parameters_RCIN g_rcin;
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY_BALANCE
    void batt_balance_update();
    BattBalance battery_balance;
#endif

#ifdef HAL_PERIPH_ENABLE_SERIAL_OPTIONS
    SerialOptions serial_options;
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

#ifdef HAL_PERIPH_ENABLE_NETWORKING
    Networking_Periph networking_periph;
#endif

#ifdef HAL_PERIPH_ENABLE_RTC
    AP_RTC rtc;
#endif

#if HAL_GCS_ENABLED
    GCS_Periph _gcs;
#endif

#ifdef HAL_PERIPH_ENABLE_RELAY
    AP_Relay relay;
#endif

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

#ifdef HAL_PERIPH_ENABLE_EFI
    uint32_t last_efi_update_ms;
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
    uint32_t last_mag_update_ms;
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
    uint32_t last_gps_update_ms;
    uint32_t last_gps_yaw_ms;
#endif
    uint32_t last_relposheading_ms;
#ifdef HAL_PERIPH_ENABLE_BARO
    uint32_t last_baro_update_ms;
#endif
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    uint32_t last_airspeed_update_ms;
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
    bool saw_gps_lock_once;
#endif

    static AP_Periph_FW *_singleton;

    enum class DebugOptions {
        SHOW_STACK = 0,
        AUTOREBOOT = 1,
        ENABLE_STATS = 2,
    };

    // check if an option is set
    bool debug_option_is_set(const DebugOptions option) const {
        return (uint8_t(g.debug.get()) & (1U<<uint8_t(option))) != 0;
    }
    
    // show stack as DEBUG msgs
    void show_stack_free();

    static bool no_iface_finished_dna;
    static constexpr auto can_printf = ::can_printf;

    bool canard_broadcast(uint64_t data_type_signature,
                          uint16_t data_type_id,
                          uint8_t priority,
                          const void* payload,
                          uint16_t payload_len,
                          uint8_t iface_mask=0);

    bool canard_respond(CanardInstance* canard_instance,
                        CanardRxTransfer* transfer,
                        uint64_t data_type_signature,
                        uint16_t data_type_id,
                        const uint8_t *payload,
                        uint16_t payload_len);
    
    void onTransferReceived(CanardInstance* canard_instance,
                            CanardRxTransfer* transfer);
    bool shouldAcceptTransfer(const CanardInstance* canard_instance,
                              uint64_t* out_data_type_signature,
                              uint16_t data_type_id,
                              CanardTransferType transfer_type,
                              uint8_t source_node_id);

    // reboot the peripheral, optionally holding in bootloader
    void reboot(bool hold_in_bootloader);

#if AP_UART_MONITOR_ENABLED
    void handle_tunnel_Targetted(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void send_serial_monitor_data();
    int8_t get_default_tunnel_serial_port(void) const;

    struct {
        ByteBuffer *buffer;
        uint32_t last_request_ms;
        AP_HAL::UARTDriver *uart;
        int8_t uart_num;
        uint8_t node_id;
        uint8_t protocol;
        uint32_t baudrate;
        bool locked;
    } uart_monitor;
#endif

    // handlers for incoming messages
    void handle_get_node_info(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_param_getset(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_param_executeopcode(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_begin_firmware_update(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_allocation_response(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_safety_state(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_arming_status(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_RTCMStream(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_MovingBaselineData(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_esc_rawcommand(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_act_command(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_beep_command(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_lightscommand(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_notify_state(CanardInstance* canard_instance, CanardRxTransfer* transfer);
    void handle_hardpoint_command(CanardInstance* canard_instance, CanardRxTransfer* transfer);

    void process1HzTasks(uint64_t timestamp_usec);
    void processTx(void);
    void processRx(void);
#if HAL_PERIPH_CAN_MIRROR
    void processMirror(void);
#endif // HAL_PERIPH_CAN_MIRROR
    void cleanup_stale_transactions(uint64_t timestamp_usec);
    void update_rx_protocol_stats(int16_t res);
    void node_status_send(void);
    bool can_do_dna();
    uint8_t *get_tid_ptr(uint32_t transfer_desc);
    uint16_t pool_peak_percent();
    void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue);

    struct dronecan_protocol_t {
        CanardInstance canard;
        uint32_t canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
        struct tid_map {
            uint32_t transfer_desc;
            uint8_t tid;
            tid_map *next;
        } *tid_map_head;
        /*
         * Variables used for dynamic node ID allocation.
         * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
         */
        uint32_t send_next_node_id_allocation_request_at_ms; ///< When the next node ID allocation request should be sent
        uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request
        uint8_t tx_fail_count;
        uint8_t dna_interface = 1;
    } dronecan;

#if AP_SIM_ENABLED
    SITL::SIM sitl;
#endif
#if AP_AHRS_ENABLED
    AP_AHRS ahrs;
#endif
};

#ifndef CAN_APP_NODE_NAME
#define CAN_APP_NODE_NAME "org.ardupilot." CHIBIOS_BOARD_NAME
#endif

namespace AP
{
    AP_Periph_FW& periph();
}

extern AP_Periph_FW periph;
