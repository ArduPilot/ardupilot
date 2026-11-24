---
version: alpha
colors:
  primary: "#38bdf8"
  background: "#0a0f1e"
  panel: "#1e2d45"
  border: "rgba(100, 140, 200, 0.20)"
  text: "#e8edf5"
  muted: "#9baec8"
  telemetry: "#38bdf8"
  healthy: "#22c55e"
  caution: "#fbbf24"
  danger: "#f87171"
typography:
  display:
    fontFamily: "Inter, sans-serif"
  data:
    fontFamily: "JetBrains Mono, ui-monospace, monospace"
rounded:
  panel: "12px"
  instrument: "8px"
  status: "99px"
omitted:
  - section: components
    reason: "The first port is static HTML using the reference dashboard CSS; shared components will be considered as more functional panels arrive."
---

## Purpose and source

The ArduSub dashboard at `/` is the first telemetry-focused port of the operator UI in `ATAL_controller/web_dashboard`. It uses that reference's `dashboard.css` and three SVG assets, along with its topbar, navigation labels, dark/light colors, and three-column instrument arrangement. `integration/static/ros_dashboard.css` is the preserved reference stylesheet; `integration/static/operator_dashboard.css` owns only the ArduSub adaptation. The earlier small instrument view remains at `/diagnostics`.

`/classic` is the canonical **source-layout port**. It renders the byte-for-byte ROS 2 templates and CSS from `integration/reference_dashboard/`, removes their ROS controller JavaScript, and adds `classic_adapter.css` and `classic_adapter.js`. This route keeps the familiar cockpit and tab names while replacing only fields whose ArduSub meaning is verified. The current `/` page remains an interim functional surface while the rest of the original interactions are mapped.

## Visual rules

Use the reference dark navy surfaces and cyan instrument accent. Light mode uses the reference `html.light` token override. The header and tab strip stay together at the top; the navigation scrolls horizontally when necessary. On narrower screens the columns stack, while attitude and depth remain legible. Numbers use a tabular monospace face. The attitude horizon and water column are the visual focus, matching the reference cockpit vocabulary. Motion is limited to live instrument updates and is removed for reduced-motion users.

## Behavior rules

Show link, arming, IMU, pressure, position, and output freshness in text as well as color. Missing and stale readings use an em dash. The old ROS command areas appear only as unavailable navigation labels until real ArduSub behavior is implemented; they do not look clickable. Dashboard, Plots, Raw Data, Mission Builder, theme toggle, and diagnostic link work now. Mission Builder uses the same card and form language for local drafts with validation and JSON import/export, without implying upload readiness. The plot retains up to 60 seconds of received data and leaves gaps for missing samples. Raw Data displays the normalized API snapshot, including freshness flags, without implying every retained value is current.

The depth gauge's 10 m range is a display-only default in `operator_dashboard.html` (`data-display-range-m`); edit it when the operator's useful scale is known. It is not a pressure sensor calibration or depth limit. Motor channel names remain SERVO 1–8 until physical wiring is verified. No browser action can command or arm the vehicle yet.

On `/classic`, the orientation, output, mode, pressure, EKF, mission-status, Raw Data, and rolling Plots surfaces use live read-only snapshots. The original Gate tab shows optional proposal-only companion values. Other original panels are retained in source but hidden behind explicit unported notices; their controls are disabled. A persistent banner distinguishes synthetic software demo, live telemetry, and lost stream. The original ROS scripts never run on this route.

The Mission Status card follows the same read-only instrument style. It reads ArduSub `MISSION_CURRENT` only; unknown or stale state and counts appear unavailable. It never starts a mission or establishes navigation validity.
The System Control card shows the reported ArduSub flight mode beside link and arming state; the mode clears when heartbeat is stale.
It also shows the latest EKF status report as a qualified observation. Uninitialized, constant-position, predicted-position and absolute-position flags use explicit text; no flag is presented as mission permission or a substitute for ArduSub's internal `position_ok()`.
The Gate Guidance card shows fresh ROS link health and proposal values only when the optional ROS subscriber is enabled. Guidance is advisory telemetry and never an ArduSub command. Missing, expired, or invalid proposals have no numeric output.
The Subsea Camera card accepts an editable HTTP image or MJPEG URL, retaining the address in browser storage without automatically reconnecting. Its empty, connecting, live, and error messages use the same card language. Stream transport and camera hardware configuration remain separate from the telemetry API.
The Power card shows fresh ArduSub SYS_STATUS voltage, current, and remaining percentage when those values are reported. Missing values and expired messages remain unavailable rather than displaying a guessed charge state.
