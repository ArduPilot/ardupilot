# AP_DDS Odometry Subscriber — Changelog & Technical Documentation

## Overview

This document describes all changes made to ArduPilot's `AP_DDS` library to add a
`nav_msgs/Odometry` subscriber on topic `/ap/odom`. This allows a ROS 2 SLAM system
(or any odometry source) to feed **both position AND velocity** into ArduPilot's EKF3
through a single message, replacing or supplementing the existing `/ap/tf`
(position-only) path.

### Why not just use `/ap/tf`?

The existing `/ap/tf` subscriber only carries `geometry_msgs/TransformStamped`, which
provides position + orientation but **no velocity**. For indoor navigation without GPS,
the EKF needs an external velocity source (`EK3_SRC1_VELXY = 6`). Without it,
velocity is derived from differentiating noisy position — producing poor control.

`nav_msgs/Odometry` bundles `PoseWithCovariance` + `TwistWithCovariance` in one
message with one timestamp, solving this cleanly.

---

## Files Changed (ArduPilot side)

### 1. New IDL Files (3 files)

These define the DDS wire format. Micro-XRCE-DDS-Gen processes them at build time
into C headers/sources.

#### `Idl/geometry_msgs/msg/PoseWithCovariance.idl` (NEW)

```
geometry_msgs::msg::Pose  pose
double                    covariance[36]   // 6×6 row-major
```

**Why:** Dependency of `Odometry.idl`. The existing IDLs only had `Pose.idl` (no
covariance variant).

#### `Idl/geometry_msgs/msg/TwistWithCovariance.idl` (NEW)

```
geometry_msgs::msg::Twist twist
double                    covariance[36]   // 6×6 row-major
```

**Why:** Dependency of `Odometry.idl`. The existing IDLs only had `Twist.idl` (no
covariance variant).

#### `Idl/nav_msgs/msg/Odometry.idl` (NEW)

```
std_msgs::msg::Header                     header
string                                    child_frame_id
geometry_msgs::msg::PoseWithCovariance    pose
geometry_msgs::msg::TwistWithCovariance   twist
```

**Why:** The main message definition. Includes both pose (for `handle_pose_estimate`)
and twist (for `handle_vision_speed_estimate`). The `waf` build system auto-detects
new `.idl` files and generates the corresponding C serialization/deserialization code.

---

### 2. `AP_DDS_config.h` (MODIFIED — 3 lines added)

```cpp
#ifndef AP_DDS_ODOM_SUB_ENABLED
#define AP_DDS_ODOM_SUB_ENABLED AP_DDS_VISUALODOM_ENABLED
#endif
```

**Where:** Inserted after `AP_DDS_DYNAMIC_TF_SUB_ENABLED`.

**Why:** Feature flag for the entire odometry subscriber. Defaults to enabled whenever
`AP_VisualOdom` is compiled in (`HAL_VISUALODOM_ENABLED`), which matches the existing
`/ap/tf` subscriber's gate. Can be overridden per-board via `hwdef.dat`.

---

### 3. `AP_DDS_Topic_Table.h` (MODIFIED — 3 sections)

#### a. Include guard (top of file)

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
#include "nav_msgs/msg/Odometry.h"
#endif // AP_DDS_ODOM_SUB_ENABLED
```

**Why:** Pull in the generated C header for the Odometry message type.

#### b. `TopicIndex` enum

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
    ODOM_SUB,
#endif // AP_DDS_ODOM_SUB_ENABLED
```

**Where:** After `DYNAMIC_TRANSFORMS_SUB`, before `VELOCITY_CONTROL_SUB`.

**Why:** Each DDS entity (topic, reader, writer) needs a unique ID. The enum
auto-assigns sequential values, so insertion order matters for compatibility with
existing readers.

#### c. Topic table entry

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::ODOM_SUB),
        .pub_id   = to_underlying(TopicIndex::ODOM_SUB),
        .sub_id   = to_underlying(TopicIndex::ODOM_SUB),
        .dw_id    = uxrObjectId{.id=to_underlying(TopicIndex::ODOM_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id    = uxrObjectId{.id=to_underlying(TopicIndex::ODOM_SUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataReader,           // <-- subscriber
        .topic_name = "rt/ap/odom",                 // <-- ROS topic: /ap/odom
        .type_name  = "nav_msgs::msg::dds_::Odometry_",
        .qos = {
            .durability  = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,  // <-- must match publisher
            .history     = UXR_HISTORY_KEEP_LAST,
            .depth       = 5,
        },
    },
#endif // AP_DDS_ODOM_SUB_ENABLED
```

**Why:**
- `Topic_rw::DataReader` — this is a subscriber (ArduPilot receives data).
- `"rt/ap/odom"` — the `rt/` prefix is the DDS↔ROS 2 naming convention; maps to
  ROS topic `/ap/odom`.
- `BEST_EFFORT` / `VOLATILE` — matching the QoS that ROS 2 publishers must use.
  RELIABLE publishers will NOT be received by a BEST_EFFORT subscriber (QoS
  incompatibility).
- `depth = 5` — small buffer, we want fresh data not queued stale data.

---

### 4. `AP_DDS_Client.h` (MODIFIED — 2 sections)

#### a. Include

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
#include "nav_msgs/msg/Odometry.h"
#endif // AP_DDS_ODOM_SUB_ENABLED
```

#### b. Receive buffer

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
    // incoming odometry for external position and velocity
    static nav_msgs_msg_Odometry rx_odom_topic;
#endif // AP_DDS_ODOM_SUB_ENABLED
```

**Why:** Static buffer into which incoming DDS messages are deserialized. Must be
`static` because the XRCE callback is a C function pointer, not a C++ member.

---

### 5. `AP_DDS_Client.cpp` (MODIFIED — 2 sections)

#### a. Static definition (near top)

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
nav_msgs_msg_Odometry AP_DDS_Client::rx_odom_topic {};
#endif // AP_DDS_ODOM_SUB_ENABLED
```

#### b. `on_topic` callback handler

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
    case topics[to_underlying(TopicIndex::ODOM_SUB)].dr_id.id: {
        const bool success = nav_msgs_msg_Odometry_deserialize_topic(ub, &rx_odom_topic);
        if (success == false) {
            break;
        }
#if AP_DDS_VISUALODOM_ENABLED
        AP_DDS_External_Odom::handle_external_odom(rx_odom_topic);
#endif // AP_DDS_VISUALODOM_ENABLED
        break;
    }
#endif // AP_DDS_ODOM_SUB_ENABLED
```

**Where:** In the `on_topic` switch-case, after the `DYNAMIC_TRANSFORMS_SUB` case.

**Why:** When the Micro-XRCE-DDS agent delivers data for our DataReader, this
callback fires. It deserializes the raw bytes into the `rx_odom_topic` struct, then
passes it to the handler.

---

### 6. `AP_DDS_External_Odom.h` (MODIFIED — 5 lines added)

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
#include "nav_msgs/msg/Odometry.h"
#endif // AP_DDS_ODOM_SUB_ENABLED

// ... inside class AP_DDS_External_Odom:
#if AP_DDS_ODOM_SUB_ENABLED
    // Handler for external position and velocity localization from nav_msgs/Odometry
    static void handle_external_odom(const nav_msgs_msg_Odometry& msg);
#endif // AP_DDS_ODOM_SUB_ENABLED
```

**Why:** Overloaded `handle_external_odom` — the existing one takes `TFMessage`
(position-only), the new one takes `Odometry` (position + velocity).

---

### 7. `AP_DDS_External_Odom.cpp` (MODIFIED — ~50 lines added)

This is the core logic. Added `#include <AP_HAL/AP_HAL.h>` and the new handler:

```cpp
#if AP_DDS_ODOM_SUB_ENABLED
void AP_DDS_External_Odom::handle_external_odom(const nav_msgs_msg_Odometry& msg)
{
    auto *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    // ── Timestamp (CRITICAL) ──────────────────────────────────────────
    const uint64_t remote_time_us {AP_DDS_Type_Conversions::time_u64_micros(msg.header.stamp)};
    const uint32_t time_ms {AP_HAL::millis()};

    // ── Position  (ENU → NED) ─────────────────────────────────────────
    const Vector3f ap_position {
        float(msg.pose.pose.position.x),
        float(-msg.pose.pose.position.y),
        float(-msg.pose.pose.position.z)
    };

    // ── Orientation (ENU → NED) ───────────────────────────────────────
    Quaternion ap_rotation;
    ap_rotation.q1 =  msg.pose.pose.orientation.w;
    ap_rotation.q2 =  msg.pose.pose.orientation.x;
    ap_rotation.q3 = -msg.pose.pose.orientation.y;
    ap_rotation.q4 = -msg.pose.pose.orientation.z;
    ap_rotation.normalize();

    // ── Velocity  (ENU → NED) ─────────────────────────────────────────
    const Vector3f ap_velocity {
        float(msg.twist.twist.linear.x),
        float(-msg.twist.twist.linear.y),
        float(-msg.twist.twist.linear.z)
    };

    // ── Covariance → error estimate ───────────────────────────────────
    float posErr {0.0};
    if (msg.pose.covariance[0] > 0) {
        posErr = sqrtf(float(msg.pose.covariance[0]));
    }
    const float angErr {0.0};
    const uint8_t reset_counter {0};

    // ── Feed EKF ──────────────────────────────────────────────────────
    visual_odom->handle_pose_estimate(remote_time_us, time_ms, ...);
    visual_odom->handle_vision_speed_estimate(remote_time_us, time_ms, ap_velocity, ...);
}
#endif // AP_DDS_ODOM_SUB_ENABLED
```

#### Key design decisions in this handler:

**a) Timestamp: `AP_HAL::millis()` instead of converting the ROS timestamp**

This is the most important change and fixes the "Need Position Estimate" bug.

The existing TF handler and our initial implementation both did:
```cpp
const uint32_t time_ms {static_cast<uint32_t>(remote_time_us * 1E-3)};
```

This is fundamentally broken because:

1. **Zero timestamps** — `ros2 topic pub` sends `sec: 0, nanosec: 0`. This produces
   `time_ms = 0`. The EKF's `writeExtNavData()` rate-limits with:
   ```cpp
   if ((timeStamp_ms - extNavMeasTime_ms) < extNavIntervalMin_ms) return;
   ```
   With `extNavMeasTime_ms` initialized to 0, and every incoming `timeStamp_ms = 0`,
   the delta is always `0 < 20ms`, so **every single message is silently dropped**.

2. **Epoch mismatch** — ROS 2 uses Unix epoch (e.g., `1737760000` seconds). ArduPilot
   uses boot-relative time (starts at 0). The EKF's freshness check:
   ```cpp
   bool extNavDataIsFresh = (imuSampleTime_ms - extNavMeasTime_ms < 500);
   ```
   compares against `imuSampleTime_ms` which is boot-relative. A wrapped Unix
   timestamp would make this comparison nonsensical.

3. **What MAVLink does** — The MAVLink path calls
   `correct_offboard_timestamp_usec_to_ms()` which converts the remote timestamp
   into local ArduPilot time. The DDS path had a TODO for this but never implemented
   it. Using `AP_HAL::millis()` directly is a simple correct solution.

The `remote_time_us` from the ROS header is still preserved and passed along for
logging purposes only.

**b) ENU → NED coordinate conversion**

ROS uses ENU (East-North-Up). ArduPilot uses NED (North-East-Down).
- Position: `x → x`, `y → -y`, `z → -z`
- Quaternion: `w → q1`, `x → q2`, `y → -q3`, `z → -q4`
- Velocity: same as position

This matches the existing TF handler's `convert_transform()` logic.

**c) Velocity: body-frame → world-frame rotation (CRITICAL)**

The `twist` field in `nav_msgs/Odometry` is in the body frame (`child_frame_id`,
i.e. `base_link`, FLU). But `handle_vision_speed_estimate` → EKF3's
`writeExtNavVelData` expects **NED world-frame** velocity (same format as GPS
velocity). So after converting FLU → FRD, we must rotate from body to world:

```cpp
ap_velocity = ap_rotation * ap_velocity;  // FRD body → NED world
```

This matches exactly what ArduPilot's MAVLink ODOMETRY handler does:
```cpp
// GCS_Common.cpp line 4077-4078
Vector3f vel{m.vx, m.vy, m.vz};
vel = q * vel;    // FRD body → NED world
```

Without this rotation, the EKF receives body-frame velocity but interprets it as
world-frame. When the vehicle turns, the velocity axes don't match the EKF's
prediction → innovation variance spikes → **EKF failsafe**.

**d) Two EKF calls from one message**

```cpp
visual_odom->handle_pose_estimate(...)         // → AP_AHRS → EKF3::writeExtNavData()
visual_odom->handle_vision_speed_estimate(...) // → AP_AHRS → EKF3::writeExtNavVelData()
```

Both receive the same `time_ms` from the same message, ensuring temporal consistency.

**d) Covariance passthrough**

If `pose.covariance[0] > 0` (the xx diagonal element), its square root is used as
`posErr`. This allows SLAM systems to communicate confidence. If zero/unset, `posErr`
defaults to 0.0, which AP_VisualOdom_MAV clamps to `VISO_POS_NOISE` (the
`_frontend.get_pos_noise()` minimum).

---

## Data Flow Diagram

```
ROS 2 Publisher (/ap/odom, BEST_EFFORT)
        │
        ▼
micro_ros_agent (UDP 2019)
        │
        ▼
Micro-XRCE-DDS Client (inside ArduPilot)
        │
        ▼
AP_DDS_Client::on_topic()
        │  deserializes nav_msgs_msg_Odometry
        ▼
AP_DDS_External_Odom::handle_external_odom(Odometry)
        │  ENU→NED conversion
        │  AP_HAL::millis() for timestamp
        ├──► AP_VisualOdom::handle_pose_estimate()
        │        └──► AP_VisualOdom_MAV::handle_pose_estimate()
        │                 └──► AP_AHRS::writeExtNavData()
        │                          └──► NavEKF3_core::writeExtNavData()
        │                                   → storedExtNav buffer
        │                                   → extNavDataToFuse = true
        │
        └──► AP_VisualOdom::handle_vision_speed_estimate()
                 └──► AP_VisualOdom_MAV::handle_vision_speed_estimate()
                          └──► AP_AHRS::writeExtNavVelData()
                                   └──► NavEKF3_core::writeExtNavVelData()
                                            → storedExtNavVel buffer
                                            → useExtNavVel = true
```

---

## EKF Parameters for Ground Rover (Indoor, No GPS)

```
VISO_TYPE       1      # MAV backend (receives from DDS via AP_VisualOdom)
EK3_SRC1_POSXY  6      # ExternalNav for XY position
EK3_SRC1_POSZ   1      # Barometer for altitude (reliable on ground)
EK3_SRC1_VELXY  6      # ExternalNav for XY velocity  ← THE NEW CAPABILITY
EK3_SRC1_VELZ   6      # ExternalNav for Z velocity (or 0=NONE for baro-only Z)
EK3_SRC1_YAW    1      # Compass for yaw (reliable on ground with good cal)
```

---

## QoS Requirements

The subscriber uses **BEST_EFFORT** / **VOLATILE**. Any ROS 2 publisher to `/ap/odom`
**must** use matching QoS:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
```

A `RELIABLE` publisher will **not** be matched by a `BEST_EFFORT` subscriber — data
will silently not flow.

---

## Build & Test

```bash
# Build (from ardupilot root)
./waf configure --board sitl --enable-DDS
./waf rover

# waf automatically:
#   1. Finds new .idl files under Idl/
#   2. Runs microxrceddsgen to produce C headers + (de)serialization code
#   3. Compiles everything together
```

---

## Files Summary

| File | Status | Lines Changed | Purpose |
|------|--------|---------------|---------|
| `Idl/geometry_msgs/msg/PoseWithCovariance.idl` | NEW | 23 | IDL for Pose + 6×6 covariance |
| `Idl/geometry_msgs/msg/TwistWithCovariance.idl` | NEW | 23 | IDL for Twist + 6×6 covariance |
| `Idl/nav_msgs/msg/Odometry.idl` | NEW | 33 | IDL for the Odometry message |
| `AP_DDS_config.h` | MODIFIED | +3 | `AP_DDS_ODOM_SUB_ENABLED` feature flag |
| `AP_DDS_Topic_Table.h` | MODIFIED | +22 | Include, enum entry, topic table entry |
| `AP_DDS_Client.h` | MODIFIED | +6 | Include + `rx_odom_topic` buffer |
| `AP_DDS_Client.cpp` | MODIFIED | +13 | Static init + `on_topic` handler case |
| `AP_DDS_External_Odom.h` | MODIFIED | +5 | New overloaded handler declaration |
| `AP_DDS_External_Odom.cpp` | MODIFIED | +52 | Handler impl + `AP_HAL.h` include |
