# Camera identity and MAVLink links

AP_Camera supports native MAVLink cameras and cameras controlled through other
interfaces, including mixtures of the two. The central rule is that managing a
native MAVLink camera must not advertise it as a second, flight-controller-owned
camera. Camera identity must stay consistent across discovery, control and status.

## Camera identities

| Camera | GCS-visible identity | Control endpoint |
| --- | --- | --- |
| Native MAVLink camera | Original camera system/component IDs | Camera system/component IDs |
| Non-MAVLink camera | Flight controller plus camera instance where supported | Flight controller, with the command's camera selector |

For native cameras, `AP_Camera_MAVLinkCamV2` preserves both system and component
IDs. This applies to forwarded heartbeats, camera information, video streams,
thermal data, tracking status and extended-parameter replies. Stream IDs and
storage IDs identify resources within a camera; they do not identify the camera.
Two cameras can therefore both advertise stream 1 without colliding.
`gimbal_device_id` describes the associated gimbal, not the camera's identity.

Even when a message has a `camera_device_id`, native camera messages retain the
original identity and payload. In particular, `CAMERA_TRACKING_IMAGE_STATUS` and
`PARAM_EXT_VALUE`/`PARAM_EXT_ACK` in this tree have no camera-instance field that
could distinguish their sources if they were all re-emitted as the FC component.

Non-MAVLink cameras keep the existing FC-owned interface. Protocols with
`camera_device_id` use that field to identify FC-attached cameras. The MAVLink
definitions currently pinned by this tree predate that field in several camera
messages: AP_Camera uses `CAMERA_INFORMATION.lens_id` as its zero-based instance
identifier, command-specific camera selectors, and `CAMERA_FEEDBACK.cam_idx`.
Not every legacy status message can distinguish multiple non-MAVLink cameras.
This limitation must not be worked around by merging native camera identities
into the FC identity as well.

## Configuration and discovery

Set `CAMn_TYPE=6` for each native camera managed by the MAVLink camera-v2 backend.
`CAMn_COMPID=0` (the default) associates camera slot 1 with `MAV_COMP_ID_CAMERA`
(100) and slot 2 with `MAV_COMP_ID_CAMERA2` (101). Set `CAMn_COMPID` to a non-zero
component ID (1–255) to override that slot's mapping, then reboot. The backend
learns the system ID and link from a camera heartbeat with the selected component
ID. This also applies to mixed backend types: a native camera in slot 2 defaults
to component 101 but can use another component ID via `CAM2_COMPID`.
The current frontend supports two slots.
Native camera slots must select distinct component IDs, including IDs selected
by the zero default. If IDs collide, only the first slot can discover that ID;
the later slot reports a warning. Out-of-range IDs also report a warning and
disable discovery until the configuration is corrected.
The legacy `CAMn_TYPE=5` MAVLink trigger backend is not this camera-v2 interface;
it does not discover or expose a native camera-v2 endpoint.

Only one camera/gimbal unit per physical MAVLink link is supported. Additional
units use separate links and distinct camera identities. Multiple video streams
from a single camera are supported. Use MAVLink 2 for the camera protocol.

The backend starts searching after ten seconds. Once associated, it relays the
camera's actual heartbeats on isolated links; it does not manufacture a permanent
heartbeat for a disconnected camera. A GCS can discover the native endpoint and
request its `CAMERA_INFORMATION` directly. The FC continues to advertise only
its own FC heartbeat, not a second camera heartbeat for the same device.

## Broadcast, unicast and private links

`MAVn_OPTIONS` applies to a MAVLink instance, not directly to `SERIALn`. Instances
are assigned in serial-port order among ports using MAVLink. Check the mapping
when configuring a camera port.

| Camera link | Addressed traffic | Broadcast traffic |
| --- | --- | --- |
| Normal/broadcast | Normal MAVLink routing | Normal MAVLink forwarding |
| Unicast, bit 4 (value 16) | Exact learned system/component routes | Isolated, except camera-backend relays to normal links |
| Legacy private, bit 1 (value 2) | Requests can enter on an exact route; replies cannot normally leave | Isolated, except camera-backend relays to normal links |

Unicast is recommended for full GCS access. It prevents other links' broadcasts
from reaching the device, and prevents device broadcasts from reaching other
links through the generic router. It still processes incoming messages locally
and learns routes. Addressed requests and replies, including MAVFTP and
`COMMAND_ACK`, can traverse the link normally.

Unicast links send the FC heartbeat but do not start the normal FC telemetry
streams. A camera or gimbal requests the inputs it needs with
`MAV_CMD_SET_MESSAGE_INTERVAL` or `MAV_CMD_REQUEST_MESSAGE`.
Event-driven FC broadcasts, such as home/origin changes, are also suppressed;
explicit requests for those messages still work.

Private mode is not equivalent to unicast: it blocks addressed replies such as
MAVFTP responses and command acknowledgements. Relaying camera broadcasts does
not remove this restriction. Setting both bits retains the private restrictions.

## Camera-owned broadcast relay

Camera protocol messages often have no destination fields, even when they answer
an addressed request. The generic router does not have camera-specific exceptions.
`AP_Camera_MAVLinkCamV2::handle_message()` selects the supported broadcasts and
`resend_message()` provides the shared relay path:

- Accept messages from the associated camera system/component IDs and input link.
- Relay only when that link is private or unicast. Normal links already forward
  the original packet and must not get an additional backend copy.
- Send to active normal MAVLink links, excluding private, unicast and high-latency
  destinations. Do not reflect messages onto the camera's link.
- Preserve the original packet, including source identity, payload and signature.
  Relaying is best-effort when the destination's transmit buffer is full.

The relay covers discovery (`HEARTBEAT`, `CAMERA_INFORMATION`), settings, storage,
capture status and image notifications, field of view, video stream information
and status, thermal range, image/geographic tracking status, and extended-parameter
values and acknowledgements. Adding a message requires dispatching it to AP_Camera
and adding it to the backend's message handling, not changing routing policy.
Unrecognised device broadcasts remain isolated; this is not an unrestricted
forwarding exception for everything sent by a camera.

## Requests, capabilities and cached replies

A GCS sends native camera commands to the discovered camera system/component IDs.
This includes zoom/focus, point/rectangle/stop tracking, message requests and
message interval changes. The camera supplies the command result and capability
flags. An addressed command is not translated into an FC camera command by the
router. ArduPilot's existing mission/RC camera controls can independently use the
configured backend.

For thermal and tracking telemetry, the GCS requests the message or stream at the
camera endpoint. A targeted request does not give the resulting broadcast a
destination field; the backend relay carries that response across a unicast link.

The GCS uses `cam_definition_uri` to locate the XML file. A `comp=NN` selector in
an FTP URI identifies the **file server**, not the parameter or tracking endpoint.
`PARAM_EXT_REQUEST_LIST`, `PARAM_EXT_REQUEST_READ` and `PARAM_EXT_SET` address the
camera's discovered identity. Their broadcast replies retain that identity, so
concurrent transactions with different cameras remain distinguishable.

The FC also supports requests for cached camera information, capture status and
video stream information. These replies use the original camera system/component
IDs, not the FC IDs, and retain camera timestamps and native gimbal associations.
Cached camera information falls back to the configured `CAMn_MNT_INST` association if
the camera reports no gimbal and belongs to the FC's system. A non-zero native
association is never replaced, and raw forwarded camera packets are unchanged.
This fallback is available through FC cache requests, not direct camera requests.
Cached packets are finalized using the outgoing channel's sequence and signing state.
They do not change the global MAVLink identity. The cache bounds video streams
and resumes multi-message replies when transmit space becomes available.

Cached replies and original camera packets use different sequence counters under
the same source identity. A GCS's sequence-based packet-loss estimate can therefore
show apparent loss even when delivery is complete. Direct camera requests avoid
adding FC-generated cached packets to that sequence space.

Capture status expires after three seconds without an update. Cached capture
status also includes interval capture scheduled by ArduPilot. Until remote status
is available, the native backend does not invent FC-owned capture status. It also
does not emit the generic FC-owned settings or field-of-view messages for native
cameras: those are obtained from the camera itself. Explicit FC requests are a
compatibility path; direct camera requests provide the camera's full capabilities
without the limitations of the FC cache.
An accepted FC message request schedules a send; it does not guarantee that a
backend can supply that message. In particular, request native settings and
field-of-view messages directly from the camera rather than through the FC.

## Tests

The Copter autotests exercise two-camera relay identity, discovery and addressed
control, exclusion of unrelated devices and isolated destinations, broadcast-link
duplicate prevention, mixed native/non-MAVLink cameras, cached reply identity and
backpressure, and real MT11 MAVFTP directory listing and XML download over unicast.
The video fixtures are independent of the routing design.
