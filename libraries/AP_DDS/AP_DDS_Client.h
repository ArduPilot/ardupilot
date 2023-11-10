#pragma once

#if AP_DDS_ENABLED

#include "uxr/client/client.h"
#include "ucdr/microcdr.h"
#include "builtin_interfaces/msg/Time.h"

#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "sensor_msgs/msg/Joy.h"
#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/TwistStamped.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"
#include "rosgraph_msgs/msg/Clock.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>
#include "AP_DDS_config.h"

#define DDS_MTU             512
#define DDS_STREAM_HISTORY  8
#define DDS_BUFFER_SIZE     DDS_MTU * DDS_STREAM_HISTORY

#if AP_DDS_UDP_ENABLED
#include <AP_HAL/utility/Socket.h>
#endif

extern const AP_HAL::HAL& hal;

class AP_DDS_Client
{

private:

    AP_Int8 enabled;

    // Serial Allocation
    uxrSession session; //Session
    bool is_using_serial; // true when using serial transport

    // input and output stream
    uint8_t *input_reliable_stream;
    uint8_t *output_reliable_stream;
    uxrStreamId reliable_in;
    uxrStreamId reliable_out;

    // Outgoing Sensor and AHRS data
    builtin_interfaces_msg_Time time_topic;
    geographic_msgs_msg_GeoPoseStamped geo_pose_topic;
    geometry_msgs_msg_PoseStamped local_pose_topic;
    geometry_msgs_msg_TwistStamped tx_local_velocity_topic;
    sensor_msgs_msg_BatteryState battery_state_topic;
    sensor_msgs_msg_NavSatFix nav_sat_fix_topic;
    rosgraph_msgs_msg_Clock clock_topic;
    // incoming joystick data
    static sensor_msgs_msg_Joy rx_joy_topic;
    // incoming REP147 velocity control
    static geometry_msgs_msg_TwistStamped rx_velocity_control_topic;
    // outgoing transforms
    tf2_msgs_msg_TFMessage tx_static_transforms_topic;
    tf2_msgs_msg_TFMessage tx_dynamic_transforms_topic;
    // incoming transforms
    static tf2_msgs_msg_TFMessage rx_dynamic_transforms_topic;

    HAL_Semaphore csem;

    // connection parametrics
    bool status_ok{false};
    bool connected{false};

    static void update_topic(builtin_interfaces_msg_Time& msg);
    bool update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance) WARN_IF_UNUSED;
    static void populate_static_transforms(tf2_msgs_msg_TFMessage& msg);
    static void update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance);
    static void update_topic(geometry_msgs_msg_PoseStamped& msg);
    static void update_topic(geometry_msgs_msg_TwistStamped& msg);
    static void update_topic(geographic_msgs_msg_GeoPoseStamped& msg);
    static void update_topic(rosgraph_msgs_msg_Clock& msg);

    // subscription callback function
    static void on_topic_trampoline(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length, void* args);
    void on_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length);

    // service replier callback function
    static void on_request_trampoline(uxrSession* session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length, void* args);
    void on_request(uxrSession* session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length);

    // delivery control parameters
    uxrDeliveryControl delivery_control {
        .max_samples = UXR_MAX_SAMPLES_UNLIMITED,
        .max_elapsed_time = 0,
        .max_bytes_per_second = 0,
        .min_pace_period = 0
    };

    // The last ms timestamp AP_DDS wrote a Time message
    uint64_t last_time_time_ms;
    // The last ms timestamp AP_DDS wrote a NavSatFix message
    uint64_t last_nav_sat_fix_time_ms;
    // The last ms timestamp AP_DDS wrote a BatteryState message
    uint64_t last_battery_state_time_ms;
    // The last ms timestamp AP_DDS wrote a Local Pose message
    uint64_t last_local_pose_time_ms;
    // The last ms timestamp AP_DDS wrote a Local Velocity message
    uint64_t last_local_velocity_time_ms;
    // The last ms timestamp AP_DDS wrote a GeoPose message
    uint64_t last_geo_pose_time_ms;
    // The last ms timestamp AP_DDS wrote a Clock message
    uint64_t last_clock_time_ms;

    // functions for serial transport
    bool ddsSerialInit();
    static bool serial_transport_open(uxrCustomTransport* args);
    static bool serial_transport_close(uxrCustomTransport* transport);
    static size_t serial_transport_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
    static size_t serial_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);
    struct {
        AP_HAL::UARTDriver *port;
        uxrCustomTransport transport;
    } serial;

#if AP_DDS_UDP_ENABLED
    // functions for udp transport
    bool ddsUdpInit();
    static bool udp_transport_open(uxrCustomTransport* args);
    static bool udp_transport_close(uxrCustomTransport* transport);
    static size_t udp_transport_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
    static size_t udp_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);

    struct {
        AP_Int32 port;
        // UDP endpoint
        const char* ip = "127.0.0.1";
        // UDP Allocation
        uxrCustomTransport transport;
        SocketAPM *socket;
    } udp;
#endif
    // pointer to transport's communication structure
    uxrCommunication *comm{nullptr};

    // client key we present
    static constexpr uint32_t key = 0xAAAABBBB;

public:
    ~AP_DDS_Client();

    bool start(void);
    void main_loop(void);

    //! @brief Initialize the client's transport
    //! @return True on successful initialization, false on failure
    bool init_transport() WARN_IF_UNUSED;

    //! @brief Initialize the client's uxr session and IO stream(s)
    //! @return True on successful initialization, false on failure
    bool init_session() WARN_IF_UNUSED;

    //! @brief Set up the client's participants, data read/writes,
    //         publishers, subscribers
    //! @return True on successful creation, false on failure
    bool create() WARN_IF_UNUSED;

    //! @brief Serialize the current time state and publish to the IO stream(s)
    void write_time_topic();
    //! @brief Serialize the current nav_sat_fix state and publish to the IO stream(s)
    void write_nav_sat_fix_topic();
    //! @brief Serialize the static transforms and publish to the IO stream(s)
    void write_static_transforms();
    //! @brief Serialize the current nav_sat_fix state and publish it to the IO stream(s)
    void write_battery_state_topic();
    //! @brief Serialize the current local_pose and publish to the IO stream(s)
    void write_local_pose_topic();
    //! @brief Serialize the current local velocity and publish to the IO stream(s)
    void write_tx_local_velocity_topic();
    //! @brief Serialize the current geo_pose and publish to the IO stream(s)
    void write_geo_pose_topic();
    //! @brief Serialize the current clock and publish to the IO stream(s)
    void write_clock_topic();
    //! @brief Update the internally stored DDS messages with latest data
    void update();

    //! @brief GCS message prefix
    static constexpr const char* msg_prefix = "DDS:";

    //! @brief Parameter storage
    static const struct AP_Param::GroupInfo var_info[];

    //! @brief Convenience grouping for a single "channel" of data
    struct Topic_table {
        const uint8_t topic_id;
        const uint8_t pub_id;
        const uint8_t sub_id;    // added sub_id fields to avoid confusion
        const uxrObjectId dw_id;
        const uxrObjectId dr_id; // added dr_id fields to avoid confusion
        const char* topic_profile_label;
        const char* dw_profile_label;
        const char* dr_profile_label;
    };
    static const struct Topic_table topics[];

    //! @brief Convenience grouping for a single "channel" of services
    struct Service_table {
        //! @brief Request ID for the service
        const uint8_t req_id;

        //! @brief Reply ID for the service
        const uint8_t rep_id;

        //! @brief Profile Label for the service requester
        const char* req_profile_label;

        //! @brief Profile Label for the service replier
        const char* rep_profile_label;
    };
    static const struct Service_table services[];
};

#endif // AP_DDS_ENABLED


