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
#include <AP_DroneCAN/AP_Canard_iface.h>
#include <dronecan_msgs.h>
#include <canard.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
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

class AP_Periph_DroneCAN;
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
    void rcout_esc(const int16_t *rc, uint8_t num_channels);
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

    // reboot the peripheral, optionally holding in bootloader
    void reboot(bool hold_in_bootloader);

#if AP_UART_MONITOR_ENABLED
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

    void process1HzTasks(uint64_t timestamp_usec);

    void update_rx_protocol_stats(int16_t res);
    void node_status_send(void);
    bool can_do_dna();
    void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue);

    AP_Periph_DroneCAN* dronecan;
    /*
    * Variables used for dynamic node ID allocation.
    * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    */
    uint32_t send_next_node_id_allocation_request_at_ms; ///< When the next node ID allocation request should be sent
    uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request

#if AP_SIM_ENABLED
    SITL::SIM sitl;
#endif
#if AP_AHRS_ENABLED
    AP_AHRS ahrs;
#endif
};

class AP_Periph_DroneCAN {
    uint32_t canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
    uint8_t tx_fail_count;
    uint8_t dna_interface = 1;
public:
    AP_Periph_DroneCAN();

    CanardInterface canard_iface{0};

#if AP_UART_MONITOR_ENABLED
    Canard::Publisher<uavcan_tunnel_Targetted> tunnel_pub{canard_iface};
#endif

    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> dynamic_node_id_pub{canard_iface};
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{canard_iface};
#if HAL_ENABLE_SENDING_STATS
    Canard::Publisher<dronecan_protocol_Stats> stats_pub{canard_iface};
    Canard::Publisher<dronecan_protocol_CanStats> can_stats_pub{canard_iface};
#endif
    Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength> mag_pub{canard_iface};

#ifdef HAL_PERIPH_ENABLE_GPS
    Canard::Publisher<uavcan_equipment_gnss_Fix2> gnss_fix2_pub{canard_iface};
    Canard::Publisher<uavcan_equipment_gnss_Auxiliary> gnss_aux_pub{canard_iface};
    Canard::Publisher<ardupilot_gnss_Status> gnss_status_pub{canard_iface};

    Canard::Publisher<ardupilot_gnss_MovingBaselineData> moving_baseline_pub{canard_iface};

    Canard::Publisher<ardupilot_gnss_RelPosHeading> relposheading_pub{canard_iface};
    Canard::Publisher<ardupilot_gnss_Heading> gnss_heading_pub{canard_iface};
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    Canard::Publisher<uavcan_equipment_air_data_StaticPressure> static_pressure_pub{canard_iface};
    Canard::Publisher<uavcan_equipment_air_data_StaticTemperature> static_temperature_pub{canard_iface};
#endif

    Canard::Publisher<uavcan_protocol_debug_LogMessage> log_pub{canard_iface};

#ifdef HAL_GPIO_PIN_SAFE_BUTTON
    Canard::Publisher<ardupilot_indication_Button> button_pub{canard_iface};
#endif

    Canard::Publisher<uavcan_equipment_esc_Status> esc_status_pub{canard_iface};
    Canard::Publisher<ardupilot_equipment_trafficmonitor_TrafficReport> traffic_report_pub{canard_iface};
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    Canard::Publisher<uavcan_equipment_air_data_RawAirData> raw_air_data_pub{canard_iface};
#endif
    Canard::Publisher<ardupilot_equipment_power_BatteryInfoAux> battery_info_aux_pub{canard_iface};
    Canard::Publisher<uavcan_equipment_power_BatteryInfo> battery_info_pub{canard_iface};

#ifdef HAL_PERIPH_ENABLE_EFI
    Canard::Publisher<uavcan_equipment_ice_reciprocating_Status> reciprocating_engine_status_pub{canard_iface};
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    Canard::Publisher<uavcan_equipment_hardpoint_Command> hardpoint_command_pub{canard_iface};
#endif
#ifdef HAL_PERIPH_ENABLE_PROXIMITY
    Canard::Publisher<ardupilot_equipment_proximity_sensor_Proximity> proximity_pub{canard_iface};
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    Canard::Publisher<uavcan_equipment_range_sensor_Measurement> range_sensor_measurement_pub{canard_iface};
#endif

#ifdef HAL_PERIPH_ENABLE_RCIN
    Canard::Publisher<dronecan_sensors_rc_RCInput> rc_input_pub{canard_iface};
#endif

    // handlers for incoming messages
    static void handle_get_node_info(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest &req);
    Canard::StaticCallback<uavcan_protocol_GetNodeInfoRequest> get_node_info_callback{&AP_Periph_DroneCAN::handle_get_node_info};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server{canard_iface, get_node_info_callback};

    static void handle_param_getset(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest &req);
    Canard::StaticCallback<uavcan_protocol_param_GetSetRequest> param_getset_callback{&AP_Periph_DroneCAN::handle_param_getset};
    Canard::Server<uavcan_protocol_param_GetSetRequest> param_getset_server{canard_iface, param_getset_callback};

    static void handle_param_executeopcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest &req);
    Canard::StaticCallback<uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_callback{&AP_Periph_DroneCAN::handle_param_executeopcode};
    Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_server{canard_iface, param_executeopcode_callback};

    static void handle_begin_firmware_update(const CanardRxTransfer& transfer, const uavcan_protocol_file_BeginFirmwareUpdateRequest &req);
    Canard::StaticCallback<uavcan_protocol_file_BeginFirmwareUpdateRequest> begin_firmware_update_callback{&AP_Periph_DroneCAN::handle_begin_firmware_update};
    Canard::Server<uavcan_protocol_file_BeginFirmwareUpdateRequest> begin_firmware_update_server{canard_iface, begin_firmware_update_callback};

    static void handle_allocation_response(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation &msg);
    Canard::StaticCallback<uavcan_protocol_dynamic_node_id_Allocation> allocation_response_callback{&AP_Periph_DroneCAN::handle_allocation_response};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_response_sub{allocation_response_callback, 0};

#if defined(HAL_GPIO_PIN_SAFE_LED) || defined(HAL_PERIPH_ENABLE_RC_OUT)
    static void handle_safety_state(const CanardRxTransfer& transfer, const ardupilot_indication_SafetyState &msg);
    Canard::StaticCallback<ardupilot_indication_SafetyState> safety_state_callback{&AP_Periph_DroneCAN::handle_safety_state};
    Canard::Subscriber<ardupilot_indication_SafetyState> safety_state_sub{safety_state_callback, 0};
#endif

    static void handle_arming_status(const CanardRxTransfer& transfer, const uavcan_equipment_safety_ArmingStatus &msg);
    Canard::StaticCallback<uavcan_equipment_safety_ArmingStatus> arming_status_callback{&AP_Periph_DroneCAN::handle_arming_status};
    Canard::Subscriber<uavcan_equipment_safety_ArmingStatus> arming_status_sub{arming_status_callback, 0};

#ifdef HAL_PERIPH_ENABLE_GPS
    static void handle_RTCMStream(const CanardRxTransfer& transfer, const uavcan_equipment_gnss_RTCMStream &req);
    Canard::StaticCallback<uavcan_equipment_gnss_RTCMStream> RTCMStream_callback{&AP_Periph_DroneCAN::handle_RTCMStream};
    Canard::Subscriber<uavcan_equipment_gnss_RTCMStream> RTCMStream_sub{RTCMStream_callback, 0};

#if GPS_MOVING_BASELINE
    static void handle_MovingBaselineData(const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData &msg);
    Canard::StaticCallback<ardupilot_gnss_MovingBaselineData> MovingBaselineData_callback{&AP_Periph_DroneCAN::handle_MovingBaselineData};
    Canard::Subscriber<ardupilot_gnss_MovingBaselineData> MovingBaselineData_sub{MovingBaselineData_callback, 0};
#endif
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
    static void handle_esc_rawcommand(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand &msg);
    Canard::StaticCallback<uavcan_equipment_esc_RawCommand> esc_rawcommand_callback{&AP_Periph_DroneCAN::handle_esc_rawcommand};
    Canard::Subscriber<uavcan_equipment_esc_RawCommand> esc_rawcommand_sub{esc_rawcommand_callback, 0};

    static void handle_act_command(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand &msg);
    Canard::StaticCallback<uavcan_equipment_actuator_ArrayCommand> act_command_callback{&AP_Periph_DroneCAN::handle_act_command};
    Canard::Subscriber<uavcan_equipment_actuator_ArrayCommand> act_command_sub{act_command_callback, 0};
#endif

#if defined(HAL_PERIPH_ENABLE_NOTIFY) || defined(HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY)
    static void handle_beep_command(const CanardRxTransfer& transfer, const uavcan_equipment_indication_BeepCommand &msg);
    Canard::StaticCallback<uavcan_equipment_indication_BeepCommand> beep_command_callback{&AP_Periph_DroneCAN::handle_beep_command};
    Canard::Subscriber<uavcan_equipment_indication_BeepCommand> beep_command_sub{beep_command_callback, 0};
#endif

#if defined(AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_NOTIFY)
    static void handle_lightscommand(const CanardRxTransfer& transfer, const uavcan_equipment_indication_LightsCommand &msg);
    Canard::StaticCallback<uavcan_equipment_indication_LightsCommand> lightscommand_callback{&AP_Periph_DroneCAN::handle_lightscommand};
    Canard::Subscriber<uavcan_equipment_indication_LightsCommand> lightscommand_sub{lightscommand_callback, 0};
#endif

#if defined(HAL_PERIPH_ENABLE_NOTIFY)
    static void handle_notify_state(const CanardRxTransfer& transfer, const ardupilot_indication_NotifyState &msg);
    Canard::StaticCallback<ardupilot_indication_NotifyState> notify_state_callback{&AP_Periph_DroneCAN::handle_notify_state};
    Canard::Subscriber<ardupilot_indication_NotifyState> notify_state_sub{notify_state_callback, 0};
#endif

#if AP_UART_MONITOR_ENABLED
    static void handle_tunnel_Targetted(const CanardRxTransfer& transfer, const uavcan_tunnel_Targetted &pkt);
    Canard::StaticCallback<uavcan_tunnel_Targetted> tunnel_targetted_callback{&AP_Periph_DroneCAN::handle_tunnel_Targetted};
    Canard::Subscriber<uavcan_tunnel_Targetted> tunnel_targetted_sub{tunnel_targetted_callback, 0};
#endif

    static void handle_restart_node(const CanardRxTransfer& transfer, const uavcan_protocol_RestartNodeRequest &req);
    Canard::StaticCallback<uavcan_protocol_RestartNodeRequest> restart_node_callback{&AP_Periph_DroneCAN::handle_restart_node};
    Canard::Server<uavcan_protocol_RestartNodeRequest> restart_node_server{canard_iface, restart_node_callback};

#ifdef HAL_PERIPH_ENABLE_RELAY
    static void handle_hardpoint_command(const CanardRxTransfer& transfer, const uavcan_equipment_hardpoint_Command &cmd);
    Canard::StaticCallback<uavcan_equipment_hardpoint_Command> hardpoint_command_callback{&AP_Periph_DroneCAN::handle_hardpoint_command};
    Canard::Subscriber<uavcan_equipment_hardpoint_Command> hardpoint_command_sub{hardpoint_command_callback, 0};
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
