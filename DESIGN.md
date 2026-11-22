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

The ArduSub dashboard at `/` is the first read-only port of the operator UI in `ATAL_controller/web_dashboard`. It uses that reference's `dashboard.css` and three SVG assets, along with its topbar, navigation labels, dark/light colors, and three-column instrument arrangement. `integration/static/ros_dashboard.css` is the preserved reference stylesheet; `integration/static/operator_dashboard.css` owns only the ArduSub adaptation. The earlier small instrument view remains at `/diagnostics`.

## Visual rules

Use the reference dark navy surfaces and cyan instrument accent. Light mode uses the reference `html.light` token override. The header and tab strip stay together at the top; the navigation scrolls horizontally when necessary. On narrower screens the columns stack, while attitude and depth remain legible. Numbers use a tabular monospace face. The attitude horizon and water column are the visual focus, matching the reference cockpit vocabulary. Motion is limited to live instrument updates and is removed for reduced-motion users.

## Behavior rules

Show link, arming, IMU, pressure, position, and output freshness in text as well as color. Missing and stale readings use an em dash. The old ROS command areas appear only as unavailable navigation labels until real ArduSub behavior is implemented; they do not look clickable. Dashboard, Plots, Raw Data, theme toggle, and diagnostic link work now. The plot retains up to 60 seconds of received data and leaves gaps for missing samples. Raw Data displays the normalized API snapshot, including freshness flags, without implying every retained value is current.

The depth gauge's 10 m range is a display-only default in `operator_dashboard.html` (`data-display-range-m`); edit it when the operator's useful scale is known. It is not a pressure sensor calibration or depth limit. Motor channel names remain SERVO 1–8 until physical wiring is verified. No browser action can command or arm the vehicle yet.

The Mission Status card follows the same read-only instrument style. It reads ArduSub `MISSION_CURRENT` only; unknown or stale state and counts appear unavailable. It never starts a mission or establishes navigation validity.
The System Control card shows the reported ArduSub flight mode beside link and arming state; the mode clears when heartbeat is stale.
