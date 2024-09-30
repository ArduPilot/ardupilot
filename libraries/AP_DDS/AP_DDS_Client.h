#pragma once

#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "uxr/client/client.h"
#include "ucdr/microcdr.h"

#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "builtin_interfaces/msg/Time.h"

#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#if AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Imu.h"
#endif // AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Joy.h"
#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/TwistStamped.h"
#include "geographic_msgs/msg/GeoPointStamped.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"
#include "rosgraph_msgs/msg/Clock.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>

#define DDS_MTU             512
#define DDS_STREAM_HISTORY  8
#define DDS_BUFFER_SIZE     DDS_MTU * DDS_STREAM_HISTORY

#if AP_DDS_UDP_ENABLED
#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking_address.h>
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
    geographic_msgs_msg_GeoPointStamped gps_global_origin_topic;
    geographic_msgs_msg_GeoPoseStamped geo_pose_topic;
    geometry_msgs_msg_PoseStamped local_pose_topic;
    geometry_msgs_msg_TwistStamped tx_local_velocity_topic;
    sensor_msgs_msg_BatteryState battery_state_topic;
    sensor_msgs_msg_NavSatFix nav_sat_fix_topic;
#if AP_DDS_IMU_PUB_ENABLED
    sensor_msgs_msg_Imu imu_topic;
#endif // AP_DDS_IMU_PUB_ENABLED
    rosgraph_msgs_msg_Clock clock_topic;
    // incoming joystick data
    static sensor_msgs_msg_Joy rx_joy_topic;
    // incoming REP147 velocity control
    static geometry_msgs_msg_TwistStamped rx_velocity_control_topic;
    // incoming REP147 goal interface global position
    static ardupilot_msgs_msg_GlobalPosition rx_global_position_control_topic;
    // outgoing transforms
    tf2_msgs_msg_TFMessage tx_static_transforms_topic;
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
#if AP_DDS_IMU_PUB_ENABLED
    static void update_topic(sensor_msgs_msg_Imu& msg);
#endif // AP_DDS_IMU_PUB_ENABLED
    static void update_topic(rosgraph_msgs_msg_Clock& msg);
    static void update_topic(geographic_msgs_msg_GeoPointStamped& msg);

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
#if AP_DDS_IMU_PUB_ENABLED
    // The last ms timestamp AP_DDS wrote an IMU message
    uint64_t last_imu_time_ms;
#endif // AP_DDS_IMU_PUB_ENABLED
    // The last ms timestamp AP_DDS wrote a Local Pose message
    uint64_t last_local_pose_time_ms;
    // The last ms timestamp AP_DDS wrote a Local Velocity message
    uint64_t last_local_velocity_time_ms;
    // The last ms timestamp AP_DDS wrote a GeoPose message
    uint64_t last_geo_pose_time_ms;
    // The last ms timestamp AP_DDS wrote a Clock message
    uint64_t last_clock_time_ms;
    // The last ms timestamp AP_DDS wrote a gps global origin message
    uint64_t last_gps_global_origin_time_ms;

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
        AP_Networking_IPV4 ip{AP_DDS_DEFAULT_UDP_IP_ADDR};
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
#if AP_DDS_IMU_PUB_ENABLED
    //! @brief Serialize the current IMU data and publish to the IO stream(s)
    void write_imu_topic();
#endif // AP_DDS_IMU_PUB_ENABLED
    //! @brief Serialize the current clock and publish to the IO stream(s)
    void write_clock_topic();
    //! @brief Serialize the current gps global origin and publish to the IO stream(s)
    void write_gps_global_origin_topic();
    //! @brief Update the internally stored DDS messages with latest data
    void update();

    //! @brief GCS message prefix
    static constexpr const char* msg_prefix = "DDS:";

    //! @brief Parameter storage
    static const struct AP_Param::GroupInfo var_info[];

    //! @brief ROS_DOMAIN_ID
    AP_Int32 domain_id;

    //! @brief Timeout in milliseconds when pinging the XRCE agent
    AP_Int32 ping_timeout_ms;

    //! @brief Maximum number of attempts to ping the XRCE agent before exiting
    AP_Int8 ping_max_retry;

    //! @brief Enum used to mark a topic as a data reader or writer
    enum class Topic_rw : uint8_t {
        DataReader = 0,
        DataWriter = 1,
    };

    //! @brief Convenience grouping for a single "channel" of data
    struct Topic_table {
        const uint8_t topic_id;
        const uint8_t pub_id;
        const uint8_t sub_id;    // added sub_id fields to avoid confusion
        const uxrObjectId dw_id;
        const uxrObjectId dr_id; // added dr_id fields to avoid confusion
        const Topic_rw topic_rw;
        const char* topic_name;
        const char* type_name;
        const uxrQoS_t qos;
    };
    static const struct Topic_table topics[];

    //! @brief Enum used to mark a service as a requester or replier
    enum class Service_rr : uint8_t {
        Requester = 0,
        Replier = 1,
    };

    //! @brief Convenience grouping for a single "channel" of services
    struct Service_table {
        //! @brief Request ID for the service
        const uint8_t req_id;

        //! @brief Reply ID for the service
        const uint8_t rep_id;

        //! @brief Service is requester or replier
        const Service_rr service_rr;

        //! @brief Service name as it appears in ROS
        const char* service_name;

        //! @brief Service requester message type
        const char* request_type;

        //! @brief Service replier message type
        const char* reply_type;

        //! @brief Service requester topic name
        const char* request_topic_name;

        //! @brief Service replier topic name
        const char* reply_topic_name;

        //! @brief QoS for the service
        const uxrQoS_t qos;
    };
    static const struct Service_table services[];
};

#endif // AP_DDS_ENABLED


