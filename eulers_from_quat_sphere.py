#!/usr/bin/env python3
"""Interactive sphere visualisation of backend attitude-representation divergence.

Each attitude (roll, pitch) is mapped to the direction "up" points in the
body frame (yaw-independent, matching the measured yaw-independence of the
divergence when AHRS_TRIM_Z is zero).  Surface colour shows how far the
euler angles a backend supplies on master diverge from the euler angles
derived from the quaternion it supplies, with AHRS_TRIM_X=3deg,
AHRS_TRIM_Y=-2deg.  SITL-measured poses from the EulersFromQuatPosed
autotest are overlaid as markers.

Writes eulers_from_quat_sphere.html (self-contained, plotly.js from CDN).
"""
import json
import math
import os

import numpy as np

MEASURED_JSON_CANDIDATES = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "eulers_from_quat_posed_master.json"),
    os.path.expanduser("~/rc/buildlogs/eulers_from_quat_posed.json"),
]

TRIM = np.radians([3.0, -2.0, 0.0])


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def from_euler(r, p, y):
    # ArduPilot Matrix3::from_euler convention
    return Rz(y) @ Ry(p) @ Rx(r)


def to_euler(R):
    return np.array([
        math.atan2(R[2, 1], R[2, 2]),
        -math.asin(max(-1.0, min(1.0, R[2, 0]))),
        math.atan2(R[1, 0], R[0, 0]),
    ])


def axis_angle(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


R_TRIM = from_euler(*TRIM)          # autopilot-body -> vehicle-body euler matrix
R_VB2AB = R_TRIM.T                  # the matrix applied in getRotationBodyToNED
R_AA = axis_angle(-TRIM)            # what quat.rotate(-trim) applies


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rotation_angle_between(R1, R2):
    Re = R1.T @ R2
    skew = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return math.atan2(0.5 * np.linalg.norm(skew), (np.trace(Re) - 1) * 0.5)


def deltas(roll_deg, pitch_deg, yaw_deg):
    """Return (ekf_delta, dcm_delta, ekf_rot_angle) in degrees for a pose.

    The first two replicate what QVAL measured on master: backend-supplied
    eulers vs eulers of the backend-supplied quaternion (which used
    rotate(-trim)).  The third is the rotation angle of the attitude the
    EKF's euler output implies vs the quaternion's attitude - the
    geometrically meaningful error, free of euler-singularity blowup.
    """
    R_a = from_euler(math.radians(roll_deg), math.radians(pitch_deg),
                     math.radians(yaw_deg))
    R_quat = R_a @ R_AA
    e_quat = to_euler(R_quat)
    # EKF on master: euler component subtraction of trim
    e_sub = to_euler(R_a) - TRIM
    ekf = max(abs(wrap(d)) for d in (e_sub - e_quat))
    # DCM on master: eulers from _body_dcm_matrix (matrix trim convention)
    e_mat = to_euler(R_a @ R_VB2AB)
    dcm = max(abs(wrap(d)) for d in (e_mat - e_quat))
    rot = rotation_angle_between(from_euler(*e_sub), R_quat)
    return math.degrees(ekf), math.degrees(dcm), math.degrees(rot)


# SITL-measured poses (EulersFromQuatPosed, trimmed phase, stage A / master):
# (roll, pitch, yaw) -> (DCM, EKF2, EKF3) max euler delta in degrees
MEASURED = [
    (-150, -60, 45, 0.090, 3.715, 3.735),
    (-60, -60, 45, 0.053, 3.318, 3.377),
    (0, -60, 45, 0.098, 0.120, 0.088),
    (60, -60, 45, 0.058, 3.273, 3.274),
    (150, -60, 45, 0.094, 3.697, 3.684),
    (-150, 0, 45, 0.046, 3.749, 3.754),
    (-60, 0, 45, 0.046, 1.748, 1.753),
    (0, 0, 45, 0.052, 0.042, 0.052),
    (60, 0, 45, 0.044, 1.709, 1.704),
    (150, 0, 45, 0.044, 3.709, 3.702),
    (-150, 60, 45, 0.089, 3.780, 3.770),
    (-60, 60, 45, 0.048, 3.655, 3.593),
    (0, 60, 45, 0.112, 0.035, 0.085),
    (60, 60, 45, 0.051, 3.541, 3.492),
    (150, 60, 45, 0.085, 3.729, 3.719),
    (30, 30, 0, 0.054, 1.148, 1.121),
    (30, 30, 90, 0.054, 1.147, 1.123),
    (30, 30, 180, 0.054, 1.148, 1.124),
    (30, 30, 270, 0.054, 1.147, 1.122),
]


def load_measured():
    """Return (trimmed-pose list, full-window list) from the dense autotest
    JSON artifact if present, else the hardcoded 19-pose measurements.

    trimmed-pose list entries: (roll, pitch, dcm, ekf2, ekf3) euler deltas.
    full-window list entries: the raw JSON dicts (both phases)."""
    for path in MEASURED_JSON_CANDIDATES:
        if not os.path.exists(path):
            continue
        with open(path) as f:
            data = json.load(f)
        out = []
        for w in data["poses"]:
            if w["phase"] != "trimmed":
                continue
            b = w["backends"]
            if not all(k in b for k in ("DCM", "EKF2", "EKF3")):
                continue
            out.append((w["roll"], w["pitch"],
                        b["DCM"][0], b["EKF2"][0], b["EKF3"][0]))
        print("loaded %u dense measured poses from %s" % (len(out), path))
        return out, data["poses"]
    return ([(r, p, dcm, e2, e3) for (r, p, y, dcm, e2, e3) in MEASURED], None)


def validate_model(measured):
    # near the euler singularity the divergence gradient is steep, so a
    # fraction of a degree of EKF estimation error (markers are placed at
    # the commanded pose) shows up amplified: use a relative tolerance
    # there and an absolute one elsewhere
    worst = 0.0
    for (r, p, dcm_m, ekf2_m, ekf3_m) in measured:
        ekf, dcm, _ = deltas(r, p, 45.0)
        for (got, want) in ((ekf, ekf3_m), (dcm, dcm_m)):
            err = abs(got - want)
            score = min(err, err / want if want > 0.5 else err)
            worst = max(worst, score)
    print("model-vs-SITL worst disagreement over %u poses: %.3f "
          "(deg, or relative when >0.5 deg)" % (len(measured), worst))
    assert worst < 0.15, "model does not reproduce SITL measurements"


def sphere_point(roll_deg, pitch_deg):
    """Map attitude to display point: where 'up' points in the body frame,
    z negated so that level attitude is at the top of the sphere."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    return (math.sin(p), -math.cos(p) * math.sin(r), math.cos(p) * math.cos(r))


# 9-point Viridis approximation, matching the sphere's colorscale
VIRIDIS = [(68, 1, 84), (72, 40, 120), (62, 74, 137), (49, 104, 142),
           (38, 130, 142), (31, 158, 137), (53, 183, 121), (109, 205, 89),
           (253, 231, 37)]


def viridis_cell(value, cmax, fmt="%.3f"):
    """an html <td> coloured like the sphere for this value"""
    f = max(0.0, min(1.0, value / cmax)) * (len(VIRIDIS) - 1)
    i = min(int(f), len(VIRIDIS) - 2)
    t = f - i
    (r, g, b) = [int(VIRIDIS[i][c] + t * (VIRIDIS[i+1][c] - VIRIDIS[i][c]))
                 for c in range(3)]
    luminance = 0.299 * r + 0.587 * g + 0.114 * b
    fg = "#000" if luminance > 140 else "#fff"
    return ('<td style="background:#%02x%02x%02x;color:%s">' + fmt + '</td>') % (
        r, g, b, fg, value)


EKF_CMAX = 4.0
DCM_CMAX = 0.15
MATRIX_CMAX = 0.15


def summary_table_html(windows):
    if windows is None:
        return "<p>(no JSON artifact found; summary table unavailable)</p>"
    agg = {}  # (phase, backend) -> [max_euler, max_matrix]
    for w in windows:
        for (backend, (eul, mat)) in w["backends"].items():
            key = (w["phase"], backend)
            prev = agg.get(key, [0.0, 0.0])
            agg[key] = [max(prev[0], eul), max(prev[1], mat)]
    rows = []
    for (phase, backend) in sorted(agg):
        (eul, mat) = agg[(phase, backend)]
        eul_cmax = DCM_CMAX if backend == "DCM" else EKF_CMAX
        rows.append("<tr><td>%s</td><td>%s</td>%s%s</tr>"
                    % (phase, backend,
                       viridis_cell(eul, eul_cmax, fmt="%.4f"),
                       viridis_cell(mat, MATRIX_CMAX, fmt="%.4f")))
    return """<h2>Summary: max over poses (degrees) of each backend's
supplied attitude vs that derived from its own quaternion</h2>
<table class="t" id="summary">
<thead><tr><th>phase</th><th>backend</th><th>max euler delta</th><th>max matrix delta</th></tr></thead>
<tbody>%s</tbody></table>""" % "\n".join(rows)


def pose_table_html(measured):
    rows = []
    for (r, p, dcm_m, ekf2_m, ekf3_m) in sorted(measured, key=lambda x: (x[1], x[0])):
        model_ekf, model_dcm, _ = deltas(r, p, 45.0)
        rows.append("<tr><td>%.1f</td><td>%.1f</td>%s%s%s%s%s</tr>"
                    % (r, p,
                       viridis_cell(dcm_m, DCM_CMAX),
                       viridis_cell(ekf2_m, EKF_CMAX),
                       viridis_cell(ekf3_m, EKF_CMAX),
                       viridis_cell(model_ekf, EKF_CMAX),
                       viridis_cell(model_dcm, DCM_CMAX)))
    return """<h2>Per-pose euler divergence: backend-supplied eulers vs eulers
of the same backend's quaternion, trimmed phase (degrees; click headers to sort).<br>
Cell colours use the sphere's Viridis scale: EKF columns saturate at 4 deg,
DCM and matrix columns at 0.15 deg.</h2>
<table class="t sortable" id="poses">
<thead><tr><th>roll</th><th>pitch</th><th>DCM</th><th>EKF2</th><th>EKF3</th>
<th>model EKF</th><th>model DCM</th></tr></thead>
<tbody>%s</tbody></table>""" % "\n".join(rows)


def main():
    measured, windows = load_measured()
    validate_model(measured)

    pitches = np.arange(-88, 88.01, 4)
    rolls = np.arange(-180, 180.01, 4)

    X, Y, Z = [], [], []
    EKF, DCM, ROT, HOVER = [], [], [], []
    for p in pitches:
        xs, ys, zs, es, ds, rs, hs = [], [], [], [], [], [], []
        for r in rolls:
            x, y, z = sphere_point(r, p)
            ekf, dcm, rot = deltas(r, p, 45.0)
            xs.append(round(x, 4))
            ys.append(round(y, 4))
            zs.append(round(z, 4))
            es.append(round(ekf, 3))
            ds.append(round(dcm, 3))
            rs.append(round(rot, 3))
            hs.append("roll %.0f pitch %.0f" % (r, p))
        X.append(xs)
        Y.append(ys)
        Z.append(zs)
        EKF.append(es)
        DCM.append(ds)
        ROT.append(rs)
        HOVER.append(hs)

    def surface(values, name, cmax, visible):
        # plotly does not substitute %{surfacecolor} in surface
        # hovertemplates, so bake the values into per-point text:
        text = [["%s<br>%s<br>%.3f deg" % (name, h, v)
                 for (h, v) in zip(hrow, vrow)]
                for (hrow, vrow) in zip(HOVER, values)]
        return {
            "type": "surface",
            "name": name,
            "x": X, "y": Y, "z": Z,
            "surfacecolor": values,
            "hovertext": text,
            "hoverinfo": "text",
            "colorscale": "Viridis",
            "cmin": 0, "cmax": cmax,
            "colorbar": {"title": {"text": "deg"}},
            "visible": visible,
            "showlegend": False,
            "lighting": {"ambient": 0.9, "diffuse": 0.3, "specular": 0.05},
        }

    # measured markers, slightly outside the sphere; dedupe poses
    # sharing a sphere point (yaw-varied poses)
    seen = set()
    mx, my, mz, mtext, m_ekf, m_dcm = [], [], [], [], [], []
    for (r, p, dcm_m, ekf2_m, ekf3_m) in measured:
        if (r, p) in seen:
            continue
        seen.add((r, p))
        x, yy, z = sphere_point(r, p)
        s = 1.01
        mx.append(round(x * s, 4))
        my.append(round(yy * s, 4))
        mz.append(round(z * s, 4))
        m_ekf.append(ekf3_m)
        m_dcm.append(dcm_m)
        (model_ekf, model_dcm, _) = deltas(r, p, 45.0)
        mtext.append("SITL pose roll %.0f pitch %.0f"
                     "<br>measured: backend-supplied eulers vs eulers of its"
                     "<br>own quaternion, max component over sample window:"
                     "<br>DCM %.3f deg, EKF2 %.3f deg, EKF3 %.3f deg"
                     "<br>model: DCM %.3f deg, EKF %.3f deg" %
                     (r, p, dcm_m, ekf2_m, ekf3_m, model_dcm, model_ekf))

    def marker_trace(values, cmax, visible):
        return {
            "type": "scatter3d",
            "mode": "markers",
            "name": "SITL-measured pose (EulersFromQuatPosed autotest;<br>"
                    "hover for per-backend measured divergence)",
            "showlegend": True,
            "visible": visible,
            "x": mx, "y": my, "z": mz,
            "text": mtext,
            "hovertemplate": "%{text}<extra></extra>",
            "marker": {"size": 6, "color": values, "colorscale": "Viridis",
                       "cmin": 0, "cmax": cmax, "showscale": False,
                       "line": {"color": "red", "width": 1}},
        }

    data = [
        surface(EKF, "EKF2/EKF3 euler divergence (master)", 4.0, True),
        surface(DCM, "DCM euler divergence (master)", 0.15, False),
        surface(ROT, "EKF2/EKF3 rotation-angle error (master)", 6.0, False),
        marker_trace(m_ekf, 4.0, True),
        marker_trace(m_dcm, 0.15, False),
    ]

    layout = {
        "title": {"text":
                  "Backend euler angles vs quaternion-derived euler angles, "
                  "AHRS_TRIM 3/-2 deg (master)<br>"
                  "<sub>sphere point = direction 'up' points in the body frame "
                  "(top = level, bottom = inverted, poles on X = nose "
                  "vertical); yaw-independent.  Matrix-vs-quaternion "
                  "divergence is attitude-independent at 0.052 deg.  "
                  "Near pitch +/-90 the euler parameterisation degenerates "
                  "and euler deltas grow without bound (the rotation-angle "
                  "layer shows the underlying geometric error).  "
                  "After the branch fixes: 0.000 deg everywhere.</sub>"},
        "scene": {
            "xaxis": {"visible": False},
            "yaxis": {"visible": False},
            "zaxis": {"visible": False},
            "aspectmode": "data",
        },
        "showlegend": True,
        "legend": {"x": 0.99, "y": 0.02, "xanchor": "right",
                   "bgcolor": "rgba(255,255,255,0.7)"},
        "annotations": [{
            "text": "sphere colour:",
            "x": 0.0, "y": 1.04, "xref": "paper", "yref": "paper",
            "xanchor": "left", "showarrow": False,
        }],
        "updatemenus": [{
            "type": "buttons",
            "x": 0.0, "y": 1.0,
            "buttons": [
                {"label": "EKF2/EKF3 (euler subtraction)",
                 "method": "update",
                 "args": [{"visible": [True, False, False, True, False]}]},
                {"label": "DCM (convention split)",
                 "method": "update",
                 "args": [{"visible": [False, True, False, False, True]}]},
                {"label": "EKF2/EKF3 (rotation angle)",
                 "method": "update",
                 "args": [{"visible": [False, False, True, True, False]}]},
            ],
        }],
        "margin": {"l": 0, "r": 0, "b": 0, "t": 90},
    }

    html = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>ArduPilot backend attitude divergence</title>
<script src="https://cdn.plot.ly/plotly-2.32.0.min.js"></script>
<style>
body { font-family: monospace; }
table.t { border-collapse: collapse; margin: 1em 0; }
table.t th, table.t td { border: 1px solid #999; padding: 2px 8px; text-align: right; }
table.t th { background: #eee; cursor: pointer; }
</style>
</head>
<body>
<div id="plot" style="width:100vw;height:90vh;"></div>
%s
%s
<script>
Plotly.newPlot('plot', %s, %s, {responsive: true});
// minimal click-to-sort for tables marked sortable
document.querySelectorAll('table.sortable th').forEach(function (th, idx) {
    th.addEventListener('click', function () {
        var tbody = th.closest('table').querySelector('tbody');
        var rows = Array.from(tbody.querySelectorAll('tr'));
        var dir = th.dataset.dir === 'asc' ? -1 : 1;
        th.dataset.dir = dir === 1 ? 'asc' : 'desc';
        rows.sort(function (a, b) {
            return dir * (parseFloat(a.cells[idx].textContent)
                          - parseFloat(b.cells[idx].textContent));
        });
        rows.forEach(function (r) { tbody.appendChild(r); });
    });
});
</script>
</body>
</html>
""" % (summary_table_html(windows), pose_table_html(measured),
       json.dumps(data), json.dumps(layout))

    out = "eulers_from_quat_sphere.html"
    with open(out, "w") as f:
        f.write(html)
    print("wrote %s" % out)


if __name__ == '__main__':
    main()
