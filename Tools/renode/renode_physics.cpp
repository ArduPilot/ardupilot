/*
  Standalone lockstep server adapting ArduPilot's existing SITL physics
  models to the Renode physics protocol.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/Socket_native.h>
#include <AP_HAL_SITL/HAL_SITL_Class.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_JSON/AP_JSON.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <SITL/SIM_Aircraft.h>
#include <SITL/SITL.h>
#include <SITL/SITL_Input.h>

#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
extern const HAL_SITL& hal_sitl;

namespace {

constexpr uint8_t magic[] = {'A', 'P', 'R', 'P'};
constexpr uint16_t protocol_version = 1;
constexpr uint16_t actuator_count = 32;
constexpr uint16_t rpm_count = 32;
constexpr uint16_t rangefinder_count = 10;
constexpr uint32_t maximum_payload = 1024U * 1024U;
constexpr uint32_t io_timeout_ms = UINT32_MAX;
constexpr uint16_t default_port = 9002;
constexpr float unpaced_speedup = 1000.0f;
constexpr uint8_t actuator_protocol_pwm = 1;
constexpr uint8_t actuator_flag_valid = 1;
constexpr size_t envelope_size = 12;
constexpr size_t step_header_size = 16;
constexpr size_t actuator_size = 4;
constexpr size_t step_payload_size = step_header_size + actuator_count * actuator_size;
constexpr size_t state_payload_size = 16 + 6 * sizeof(double) + 63 * sizeof(float);

enum class MessageType : uint16_t {
    HELLO = 1,
    HELLO_REPLY = 2,
    CONFIGURE = 3,
    CONFIGURE_REPLY = 4,
    STEP = 5,
    STATE = 6,
    ERROR = 7,
};

struct Message {
    MessageType type;
    std::vector<uint8_t> payload;
};

struct Step {
    uint64_t timestamp_us;
    uint32_t sequence;
    uint16_t pwm[actuator_count];
    bool valid[actuator_count];
};

struct ModelConfiguration {
    bool configured;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    float heading;
};

uint16_t listen_port = default_port;
std::string selected_model = "quad";
SITL::SIM sitl;
AP_Baro barometer;

// expose the SIM_ parameters so model defaults can be adjusted by name
const AP_Param::Info physics_var_info[] = {
    { "SIM_", (const void *)&sitl, {group_info : SITL::SIM::var_info}, 0, 1, AP_PARAM_GROUP },
    AP_VAREND
};
AP_Param param_loader(physics_var_info);

class PhysicsGCS : public GCS {
public:
    uint32_t custom_mode() const override { return 0; }
    MAV_TYPE frame_type() const override { return MAV_TYPE_GENERIC; }
    GCS_MAVLINK *chan(const uint8_t) override { return nullptr; }
    const GCS_MAVLINK *chan(const uint8_t) const override { return nullptr; }
    void send_textv(MAV_SEVERITY, const char *, va_list,
                    mavlink_channel_mask_t) override {}

protected:
    GCS_MAVLINK *new_gcs_mavlink_backend(AP_HAL::UARTDriver &) override
    {
        return nullptr;
    }
};

PhysicsGCS physics_gcs;

uint16_t read_u16(const uint8_t *data)
{
    return uint16_t(data[0]) | uint16_t(data[1]) << 8;
}

uint32_t read_u32(const uint8_t *data)
{
    return uint32_t(data[0]) | uint32_t(data[1]) << 8 |
           uint32_t(data[2]) << 16 | uint32_t(data[3]) << 24;
}

uint64_t read_u64(const uint8_t *data)
{
    return read_u32(data) | uint64_t(read_u32(data + 4)) << 32;
}

void append_u16(std::vector<uint8_t> &data, uint16_t value)
{
    data.push_back(uint8_t(value));
    data.push_back(uint8_t(value >> 8));
}

void append_u32(std::vector<uint8_t> &data, uint32_t value)
{
    data.push_back(uint8_t(value));
    data.push_back(uint8_t(value >> 8));
    data.push_back(uint8_t(value >> 16));
    data.push_back(uint8_t(value >> 24));
}

void append_u64(std::vector<uint8_t> &data, uint64_t value)
{
    append_u32(data, uint32_t(value));
    append_u32(data, uint32_t(value >> 32));
}

void append_float(std::vector<uint8_t> &data, float value)
{
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value), "float is not 32 bits");
    memcpy(&bits, &value, sizeof(bits));
    append_u32(data, bits);
}

void append_double(std::vector<uint8_t> &data, double value)
{
    uint64_t bits;
    static_assert(sizeof(bits) == sizeof(value), "double is not 64 bits");
    memcpy(&bits, &value, sizeof(bits));
    append_u64(data, bits);
}

bool recv_exact(SocketAPM_native &socket, uint8_t *data, size_t length)
{
    size_t offset = 0;
    while (offset < length) {
        const ssize_t count = socket.recv(data + offset, length - offset, io_timeout_ms);
        if (count <= 0) {
            return false;
        }
        offset += size_t(count);
    }
    return true;
}

bool send_exact(SocketAPM_native &socket, const uint8_t *data, size_t length)
{
    size_t offset = 0;
    while (offset < length) {
        const ssize_t count = socket.send(data + offset, length - offset);
        if (count <= 0) {
            return false;
        }
        offset += size_t(count);
    }
    return true;
}

bool send_message(SocketAPM_native &socket, MessageType type,
                  const uint8_t *payload, size_t payload_length)
{
    if (payload_length > maximum_payload) {
        return false;
    }
    std::vector<uint8_t> envelope;
    envelope.reserve(envelope_size);
    envelope.insert(envelope.end(), magic, magic + sizeof(magic));
    append_u16(envelope, protocol_version);
    append_u16(envelope, uint16_t(type));
    append_u32(envelope, uint32_t(payload_length));
    return send_exact(socket, envelope.data(), envelope.size()) &&
           (payload_length == 0 || send_exact(socket, payload, payload_length));
}

bool send_message(SocketAPM_native &socket, MessageType type,
                  const std::string &payload)
{
    return send_message(socket, type,
                        reinterpret_cast<const uint8_t *>(payload.data()),
                        payload.size());
}

bool receive_message(SocketAPM_native &socket, Message &message,
                     std::string &error)
{
    uint8_t envelope[envelope_size];
    if (!recv_exact(socket, envelope, sizeof(envelope))) {
        error = "connection closed or timed out";
        return false;
    }
    if (memcmp(envelope, magic, sizeof(magic)) != 0) {
        error = "invalid protocol magic";
        return false;
    }
    const uint16_t version = read_u16(envelope + 4);
    if (version != protocol_version) {
        error = "unsupported protocol version";
        return false;
    }
    const uint16_t type = read_u16(envelope + 6);
    if (type < uint16_t(MessageType::HELLO) || type > uint16_t(MessageType::ERROR)) {
        error = "unknown message type";
        return false;
    }
    const uint32_t length = read_u32(envelope + 8);
    if (length > maximum_payload) {
        error = "payload exceeds limit";
        return false;
    }
    message.type = MessageType(type);
    message.payload.resize(length);
    if (length != 0 && !recv_exact(socket, message.payload.data(), length)) {
        error = "connection closed or timed out while reading payload";
        return false;
    }
    return true;
}

bool parse_json(const std::vector<uint8_t> &payload, AP_JSON::value &value,
                std::string &error)
{
    const std::string json(reinterpret_cast<const char *>(payload.data()), payload.size());
    error = AP_JSON::parse(value, json);
    if (!error.empty()) {
        error = "invalid JSON control payload";
        return false;
    }
    if (!value.is<AP_JSON::value::object>()) {
        error = "JSON control payload must be an object";
        return false;
    }
    return true;
}

bool json_string(const AP_JSON::value &object, const char *key,
                 std::string &result)
{
    const auto &value = object.get(key);
    if (!value.is<std::string>()) {
        return false;
    }
    result = value.get<std::string>();
    return true;
}

bool json_number(const AP_JSON::value &object, const char *key, double &result)
{
    const auto &value = object.get(key);
    if (!value.is<double>()) {
        return false;
    }
    result = value.get<double>();
    return isfinite(result);
}

bool parse_step(const std::vector<uint8_t> &payload, Step &step,
                std::string &error)
{
    if (payload.size() != step_payload_size) {
        error = "invalid STEP payload length";
        return false;
    }
    step.timestamp_us = read_u64(payload.data());
    step.sequence = read_u32(payload.data() + 8);
    if (read_u16(payload.data() + 12) != actuator_count ||
        read_u16(payload.data() + 14) != 0) {
        error = "invalid STEP actuator count or reserved field";
        return false;
    }
    for (uint16_t i = 0; i < actuator_count; i++) {
        const size_t offset = step_header_size + i * actuator_size;
        const uint8_t protocol = payload[offset + 2];
        const uint8_t flags = payload[offset + 3];
        if (protocol > 2) {
            error = "unknown actuator protocol";
            return false;
        }
        step.pwm[i] = read_u16(payload.data() + offset);
        step.valid[i] = protocol == actuator_protocol_pwm &&
                        (flags & actuator_flag_valid) != 0;
    }
    return true;
}

std::string json_quote(const std::string &value)
{
    static const char hex[] = "0123456789abcdef";
    std::string quoted;
    quoted.reserve(value.size() + 2);
    quoted.push_back('"');
    for (const uint8_t character : value) {
        switch (character) {
        case '"':
            quoted += "\\\"";
            break;
        case '\\':
            quoted += "\\\\";
            break;
        case '\b':
            quoted += "\\b";
            break;
        case '\f':
            quoted += "\\f";
            break;
        case '\n':
            quoted += "\\n";
            break;
        case '\r':
            quoted += "\\r";
            break;
        case '\t':
            quoted += "\\t";
            break;
        default:
            if (character < 0x20) {
                quoted += "\\u00";
                quoted.push_back(hex[character >> 4]);
                quoted.push_back(hex[character & 0x0f]);
            } else {
                quoted.push_back(char(character));
            }
            break;
        }
    }
    quoted.push_back('"');
    return quoted;
}

bool send_error(SocketAPM_native &socket, const std::string &error)
{
    return send_message(socket, MessageType::ERROR,
                        std::string("{\"error\":") + json_quote(error) + "}");
}

bool configure_model(const AP_JSON::value &request, SITL::Aircraft &model,
                     ModelConfiguration &active, std::string &error)
{
    std::string model_name;
    if (!json_string(request, "model", model_name) || model_name != selected_model) {
        error = "requested model does not match the running SITL model";
        return false;
    }
    double rate;
    if (!json_number(request, "rate_hz", rate) || rate < 1 || rate > 10000 ||
        !is_equal(rate, trunc(rate))) {
        error = "rate_hz must be an integer from 1 to 10000";
        return false;
    }
    if (rate > model.get_rate_hz()) {
        error = "rate_hz exceeds the physics model rate";
        return false;
    }
    const auto &location = request.get("location");
    double latitude;
    double longitude;
    double altitude;
    double heading;
    if (!location.is<AP_JSON::value::object>() ||
        !json_number(location, "latitude_deg", latitude) ||
        !json_number(location, "longitude_deg", longitude) ||
        !json_number(location, "altitude_m", altitude) ||
        !json_number(location, "heading_deg", heading) ||
        latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180 ||
        altitude < -10000 || altitude > LOCATION_ALT_MAX_M ||
        heading < -360 || heading > 360) {
        error = "invalid physics location";
        return false;
    }
    const ModelConfiguration requested{
        true,
        int32_t(round(latitude * 1.0e7)),
        int32_t(round(longitude * 1.0e7)),
        int32_t(round(altitude * 100.0)),
        float(heading),
    };
    if (active.configured) {
        if (requested.latitude != active.latitude ||
            requested.longitude != active.longitude ||
            requested.altitude != active.altitude ||
            !is_equal(requested.heading, active.heading)) {
            error = "restart the physics sidecar to change location";
            return false;
        }
    } else {
        const Location home{
            requested.latitude,
            requested.longitude,
            requested.altitude,
            Location::AltFrame::ABSOLUTE,
        };
        model.set_start_location(home, requested.heading);
        active = requested;
    }
    // Renode owns realtime pacing. The sidecar should return each lockstep
    // result as soon as the host can calculate it.
    model.set_speedup(unpaced_speedup);
    printf("Physics model %s running at %.1f Hz\n",
           selected_model.c_str(), double(model.get_rate_hz()));
    return true;
}

float optional_sensor_value(float value)
{
    return isfinite(value) ? value : 0.0f;
}

std::vector<uint8_t> state_payload(const Step &step, const SITL::Aircraft &model,
                                   const SITL::sitl_fdm &fdm)
{
    std::vector<uint8_t> payload;
    payload.reserve(state_payload_size);
    append_u64(payload, step.timestamp_us);
    append_u32(payload, step.sequence);
    append_u32(payload, 0);
    append_double(payload, fdm.latitude);
    append_double(payload, fdm.longitude);
    append_double(payload, fdm.altitude);

    const Vector3d position = model.get_position_relhome();
    append_double(payload, position.x);
    append_double(payload, position.y);
    append_double(payload, position.z);

    append_float(payload, fdm.quaternion.q1);
    append_float(payload, fdm.quaternion.q2);
    append_float(payload, fdm.quaternion.q3);
    append_float(payload, fdm.quaternion.q4);
    append_float(payload, radians(float(fdm.rollRate)));
    append_float(payload, radians(float(fdm.pitchRate)));
    append_float(payload, radians(float(fdm.yawRate)));
    append_float(payload, float(fdm.xAccel));
    append_float(payload, float(fdm.yAccel));
    append_float(payload, float(fdm.zAccel));
    append_float(payload, float(fdm.speedN));
    append_float(payload, float(fdm.speedE));
    append_float(payload, float(fdm.speedD));
    append_float(payload, float(fdm.airspeed));
    append_float(payload, fdm.bodyMagField.x);
    append_float(payload, fdm.bodyMagField.y);
    append_float(payload, fdm.bodyMagField.z);
    float pressure;
    float temperature;
    AP_Baro::get_pressure_temperature_for_alt_amsl(float(fdm.altitude), pressure, temperature);
    append_float(payload, pressure);
    append_float(payload, temperature);
    append_float(payload, float(fdm.battery_voltage));
    append_float(payload, float(fdm.battery_current));
    for (uint16_t i = 0; i < rpm_count; i++) {
        append_float(payload, optional_sensor_value(fdm.rpm[i]));
    }
    for (uint16_t i = 0; i < rangefinder_count; i++) {
        append_float(payload, optional_sensor_value(fdm.rangefinder_m[i]));
    }
    return payload;
}

bool serve_client(SocketAPM_native &socket, SITL::Aircraft &model,
                  ModelConfiguration &configuration_state)
{
    Message message;
    std::string error;
    if (!receive_message(socket, message, error) || message.type != MessageType::HELLO) {
        send_error(socket, error.empty() ? "expected HELLO" : error);
        return false;
    }
    AP_JSON::value hello;
    std::string role;
    if (!parse_json(message.payload, hello, error) ||
        !json_string(hello, "role", role) || role != "renode") {
        send_error(socket, error.empty() ? "HELLO role must be renode" : error);
        return false;
    }
    const std::string hello_reply = std::string("{\"models\":[") +
                                    json_quote(selected_model) +
                                    "],\"role\":\"physics\"}";
    if (!send_message(socket, MessageType::HELLO_REPLY, hello_reply) ||
        !receive_message(socket, message, error) ||
        message.type != MessageType::CONFIGURE) {
        send_error(socket, error.empty() ? "expected CONFIGURE" : error);
        return false;
    }
    AP_JSON::value configuration;
    if (!parse_json(message.payload, configuration, error) ||
        !configure_model(configuration, model, configuration_state, error)) {
        send_error(socket, error);
        return false;
    }
    const std::string configure_reply = std::string("{\"model\":") +
                                        json_quote(selected_model) +
                                        ",\"status\":\"configured\"}";
    if (!send_message(socket, MessageType::CONFIGURE_REPLY, configure_reply)) {
        return false;
    }

    struct sitl_input input {};
    struct sitl_input default_input {};
    const bool is_plane = selected_model.rfind("plane", 0) == 0 ||
                          selected_model.rfind("quadplane", 0) == 0;
    for (auto &servo : default_input.servos) {
        servo = is_plane ? 1500 : 1000;
    }
    if (is_plane) {
        default_input.servos[2] = 1000;
    }
    input = default_input;
    SITL::sitl_fdm fdm {};
    model.fill_fdm(fdm);
    const uint64_t model_timestamp_base = fdm.timestamp_us;
    uint32_t previous_sequence = 0;
    uint64_t previous_timestamp_us = 0;
    while (receive_message(socket, message, error)) {
        if (message.type != MessageType::STEP) {
            send_error(socket, "expected STEP");
            return false;
        }
        Step step;
        if (!parse_step(message.payload, step, error) ||
            step.sequence <= previous_sequence ||
            step.timestamp_us <= previous_timestamp_us) {
            send_error(socket, error.empty() ? "STEP sequence and timestamp must increase" : error);
            return false;
        }
        if (step.timestamp_us - previous_timestamp_us > 1000000U) {
            send_error(socket, "STEP interval exceeds one second");
            return false;
        }
        if (step.timestamp_us > UINT64_MAX - model_timestamp_base) {
            send_error(socket, "STEP timestamp exceeds model time range");
            return false;
        }
        previous_sequence = step.sequence;
        previous_timestamp_us = step.timestamp_us;
        for (uint16_t i = 0; i < actuator_count; i++) {
            input.servos[i] = step.valid[i] ? step.pwm[i] : default_input.servos[i];
        }
        const uint64_t model_target_us = model_timestamp_base + step.timestamp_us;
        while (fdm.timestamp_us < model_target_us) {
            model.update_model(input);
            model.fill_fdm(fdm);
            hal.scheduler->stop_clock(fdm.timestamp_us);
        }

        const auto payload = state_payload(step, model, fdm);
        if (payload.size() != state_payload_size ||
            !send_message(socket, MessageType::STATE, payload.data(), payload.size())) {
            return false;
        }
    }
    return false;
}

void run_server()
{
    auto *state = hal_sitl.get_sitl_state();
    SITL::Aircraft *model = state == nullptr ? nullptr : state->get_physics_model();
    if (model == nullptr) {
        AP_HAL::panic("Renode physics model was not created");
    }
    if (selected_model.rfind("quadplane", 0) == 0) {
        // the plane model handles drag; this generic build otherwise
        // inherits the multicopter frame drag defaults that ArduPlane
        // SITL builds disable
        AP_Param::set_by_name("SIM_FRM_MDRAG", 0);
        AP_Param::set_by_name("SIM_FRM_BBDRAG", 0);
    }

    SocketAPM_native listener(false);
    if (!listener.reuseaddress() || !listener.bind("127.0.0.1", listen_port) ||
        !listener.listen(1)) {
        AP_HAL::panic("failed to listen on physics port %u", unsigned(listen_port));
    }
    printf("PHYSICS_PORT %u\n", unsigned(listen_port));
    fflush(stdout);
    ModelConfiguration configuration {};
    while (true) {
        std::unique_ptr<SocketAPM_native> client(listener.accept(UINT32_MAX));
        if (client == nullptr) {
            continue;
        }
        serve_client(*client, *model, configuration);
    }
}

bool parse_port(const char *text, uint16_t &port)
{
    char *end = nullptr;
    errno = 0;
    const unsigned long value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value < 1 || value > 65535) {
        return false;
    }
    port = uint16_t(value);
    return true;
}

void setup()
{
    run_server();
}

void loop()
{
}

} // namespace

extern "C" int main(int argc, char * const argv[]);

extern "C" int main(int argc, char * const argv[])
{
    std::vector<char *> hal_argv;
    hal_argv.reserve(size_t(argc) + 5);
    hal_argv.push_back(argv[0]);
    bool have_model = false;
    bool have_serial0 = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--physics-port") == 0) {
            if (++i >= argc || !parse_port(argv[i], listen_port)) {
                fprintf(stderr, "--physics-port requires a port from 1 to 65535\n");
                return 2;
            }
            continue;
        }
        if (strcmp(argv[i], "--model") == 0 || strncmp(argv[i], "--model=", 8) == 0) {
            const char *value = nullptr;
            if (argv[i][7] == '=') {
                value = argv[i] + 8;
            } else if (i + 1 < argc && argv[i + 1][0] != '-') {
                value = argv[i + 1];
            }
            if (value == nullptr || *value == '\0') {
                fprintf(stderr, "--model requires a model name\n");
                return 2;
            }
            selected_model = value;
            have_model = true;
        }
        if (strcmp(argv[i], "--serial0") == 0 || strcmp(argv[i], "--uartA") == 0 ||
            strncmp(argv[i], "--serial0=", 10) == 0 || strncmp(argv[i], "--uartA=", 8) == 0) {
            have_serial0 = true;
        }
        hal_argv.push_back(argv[i]);
    }
    char model_option[] = "--model";
    char default_model[] = "quad";
    if (!have_model) {
        hal_argv.push_back(model_option);
        hal_argv.push_back(default_model);
    }
    char serial_option[] = "--serial0";
    char no_serial[] = "none";
    if (!have_serial0) {
        hal_argv.push_back(serial_option);
        hal_argv.push_back(no_serial);
    }
    sitl.init();
    hal_sitl.get_sitl_state()->enable_model_command_line();
    AP_HAL::HAL::FunCallbacks callbacks(setup, loop);
    hal.run(int(hal_argv.size()), hal_argv.data(), &callbacks);
    return 0;
}
