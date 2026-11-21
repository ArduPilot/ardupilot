---
version: alpha
colors:
  primary: "#51c9db"
  background: "#091420"
  panel: "#112334"
  border: "#294358"
  text: "#e7f3f5"
  muted: "#9db3bf"
  telemetry: "#51c9db"
  healthy: "#6ed6a4"
  caution: "#ffc878"
  danger: "#ff8b83"
typography:
  display:
    fontFamily: "system-ui, sans-serif"
  data:
    fontFamily: "ui-monospace, monospace"
rounded:
  panel: "14px"
  instrument: "10px"
  status: "99px"
omitted:
  - section: spacing
    reason: "One small telemetry screen; spacing is set locally in dashboard_view.html."
  - section: components
    reason: "No reusable application component library exists yet."
---

## Overview

The ATAL ArduSub dashboard is an instrument panel for a tethered AUV operator. This first page is a temporary read-only telemetry view while the existing ROS 2 dashboard is migrated. The final dashboard should closely retain that dashboard's layout, assets, and interaction model, with ArduSub-specific data and control connections. This temporary page gives immediate evidence of link, arming, sensor freshness, attitude, water readings, and motor telemetry. Its visual cue is a restrained attitude horizon, backed by explicit numbers and status words.

## Colors

Deep blue surfaces echo the original ROS dashboard. Cyan identifies telemetry; green means a live or safe state; amber means missing or stale data; red identifies armed state or connection failure. Color always accompanies text. The CSS custom properties in `integration/dashboard_view.html` are the runtime source for these values and match the frontmatter above.

## Typography

System sans is used for headings and explanation. Monospace digits keep live values stable while they update. The page uses English, matching the existing ROS dashboard.

## Layout

The horizon and water/motion readings share the first row on wide screens. Motor channels form a full-width row and wrap to four, then two, columns on narrower screens. Status remains visible in text at all sizes.

## Elevation & Depth

One thin border separates cards from the dark canvas. Shadows are intentionally absent so the horizon remains the focal instrument.

## Shapes

Cards use a 14px radius; status badges are pills. The horizon uses a 10px radius.

## Components

The one current page is implemented in `integration/dashboard_view.html`. A future dashboard port should extract shared tokens and components when a second screen exists.

## Do's and Don'ts

Show stale and unavailable data as unavailable. Do not leave a last-known value looking live. Do not add control affordances until the corresponding ArduSub command, safety, and acknowledgement paths are implemented and tested.
