#!/usr/bin/env python3
"""
Plot wind estimation accuracy from PlaneWindEstimate test logs.
Generates a multi-panel figure: one column per wind speed, showing
SIMW (actual), XKF2 core-0, and DCM wind estimates over time.
"""

import math

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from pymavlink import DFReader  # noqa: E402


FLIGHT_LOGS = {
    0: 'logs/00000003.BIN',
    1: 'logs/00000005.BIN',
    2: 'logs/00000007.BIN',
    3: 'logs/00000009.BIN',
    4: 'logs/00000011.BIN',
    5: 'logs/00000013.BIN',
    6: 'logs/00000015.BIN',
    7: 'logs/00000017.BIN',
    8: 'logs/00000019.BIN',
    9: 'logs/00000021.BIN',
    10: 'logs/00000023.BIN',
    11: 'logs/00000025.BIN',
    12: 'logs/00000027.BIN',
    13: 'logs/00000029.BIN',
    14: 'logs/00000031.BIN',
    15: 'logs/00000033.BIN',
    16: 'logs/00000035.BIN',
    17: 'logs/00000037.BIN',
    18: 'logs/00000039.BIN',
    19: 'logs/00000041.BIN',
    20: 'logs/00000043.BIN',
    21: 'logs/00000045.BIN',
    22: 'logs/00000047.BIN',
    23: 'logs/00000049.BIN',
    24: 'logs/00000051.BIN',
    25: 'logs/00000053.BIN',
    26: 'logs/00000055.BIN',
    27: 'logs/00000057.BIN',
    28: 'logs/00000059.BIN',
    29: 'logs/00000061.BIN',
    30: 'logs/00000063.BIN',
    31: 'logs/00000065.BIN',
    32: 'logs/00000067.BIN',
    33: 'logs/00000069.BIN',
    34: 'logs/00000071.BIN',
    35: 'logs/00000073.BIN',
    36: 'logs/00000075.BIN',
    37: 'logs/00000077.BIN',
    38: 'logs/00000079.BIN',
    39: 'logs/00000081.BIN',
    40: 'logs/00000083.BIN',
}

COLORS = {
    'SIMW': '#222222',
    'XKF2': '#1f77b4',
    'DCM':  '#d62728',
}


def read_log(path):
    """Return dict of time-series arrays for SIMW, XKF2 core-0, DCM."""
    data = {k: {'t': [], 'vwn': [], 'vwe': [], 'spd': []}
            for k in ('SIMW', 'XKF2', 'DCM')}

    r = DFReader.DFReader_binary(path, zero_time_base=True)
    while True:
        m = r.recv_match(type=['SIMW', 'XKF2', 'DCM'])
        if m is None:
            break
        t_s = m.TimeUS * 1e-6
        mt = m.get_type()
        if mt == 'SIMW':
            d = data['SIMW']
            d['t'].append(t_s)
            d['vwn'].append(m.VWN)
            d['vwe'].append(m.VWE)
            d['spd'].append(math.sqrt(m.VWN**2 + m.VWE**2))
        elif mt == 'XKF2' and m.C == 0:
            d = data['XKF2']
            d['t'].append(t_s)
            d['vwn'].append(m.VWN)
            d['vwe'].append(m.VWE)
            d['spd'].append(math.sqrt(m.VWN**2 + m.VWE**2))
        elif mt == 'DCM':
            d = data['DCM']
            d['t'].append(t_s)
            d['vwn'].append(m.VWN)
            d['vwe'].append(m.VWE)
            d['spd'].append(math.sqrt(m.VWN**2 + m.VWE**2))

    for k in data:
        for field in data[k]:
            data[k][field] = np.array(data[k][field])
    return data


def make_timeseries_figure():
    wind_speeds = sorted(FLIGHT_LOGS.keys())
    n_spd = len(wind_speeds)
    n_rows = 2  # row 0 = VWN, row 1 = wind speed magnitude

    fig, axes = plt.subplots(n_rows, n_spd,
                             figsize=(3.2 * n_spd, 4.5 * n_rows))
    fig.suptitle('Wind estimation time series: SIMW (actual) vs EKF3 (XKF2) vs DCM\n'
                 'SIM_WIND_DIR=180° (southerly wind → VWN positive)',
                 fontsize=13, y=1.01)

    for col, spd in enumerate(wind_speeds):
        data = read_log(FLIGHT_LOGS[spd])

        # --- Row 0: wind speed magnitude ---
        ax0 = axes[0][col]
        for src in ('SIMW', 'XKF2', 'DCM'):
            d = data[src]
            if len(d['t']) == 0:
                continue
            t0 = d['t'][0]
            ax0.plot(d['t'] - t0, d['spd'],
                     color=COLORS[src],
                     lw=1.8 if src == 'SIMW' else 1.1,
                     ls='-' if src == 'SIMW' else ('--' if src == 'XKF2' else ':'),
                     label=src, alpha=0.9)
        ax0.axhline(spd, color='grey', lw=0.8, ls='-.', alpha=0.5,
                    label='target')
        ax0.set_ylim(bottom=0)
        ax0.set_title(f'{spd} m/s', fontsize=10, fontweight='bold')
        ax0.grid(True, alpha=0.3)
        ax0.tick_params(labelsize=7)
        if col == 0:
            ax0.set_ylabel('Wind speed (m/s)', fontsize=9)
        if col == n_spd - 1:
            ax0.legend(fontsize=7, loc='lower right')

        # --- Row 1: error (estimate - SIMW) interpolated onto SIMW time grid ---
        ax1 = axes[1][col]
        sim = data['SIMW']
        if len(sim['t']) > 1:
            t0 = sim['t'][0]
            t_sim = sim['t'] - t0
            for src in ('XKF2', 'DCM'):
                d = data[src]
                if len(d['t']) < 2:
                    continue
                t_est_abs = d['t'] - t0
                spd_interp = np.interp(t_sim, t_est_abs, d['spd'],
                                       left=float('nan'), right=float('nan'))
                error = spd_interp - sim['spd']
                ax1.plot(t_sim, error,
                         color=COLORS[src],
                         lw=1.1,
                         ls='--' if src == 'XKF2' else ':',
                         label=src, alpha=0.9)
        ax1.axhline(0, color='k', lw=0.7, ls='-', alpha=0.4)
        ax1.set_xlabel('Time (s)', fontsize=8)
        ax1.grid(True, alpha=0.3)
        ax1.tick_params(labelsize=7)
        if col == 0:
            ax1.set_ylabel('Speed error (m/s)\nestimate − SIMW', fontsize=9)
        if col == n_spd - 1:
            ax1.legend(fontsize=7, loc='lower right')

    plt.tight_layout()
    out = 'wind_estimate_timeseries.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print('Saved', out)
    plt.close(fig)


def make_scatter_figure():
    """Scatter plot: SIMW vs estimator, one panel per estimator."""
    wind_speeds = sorted(FLIGHT_LOGS.keys())
    n_tail = 10

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    fig.suptitle('Wind speed estimate vs actual (SIMW)\ntail-mean of final 10 samples per run',
                 fontsize=12)

    # collect all data first so both panels share the same colour scale
    all_data = {}
    for spd in wind_speeds:
        data = read_log(FLIGHT_LOGS[spd])

        def tm(arr):
            return float(np.mean(arr[-n_tail:])) if len(arr) >= 1 else float('nan')

        all_data[spd] = {
            'sim': tm(data['SIMW']['spd']),
            'XKF2': tm(data['XKF2']['spd']),
            'DCM':  tm(data['DCM']['spd']),
        }

    sim_spds = np.array([all_data[s]['sim'] for s in wind_speeds])
    norm = plt.Normalize(vmin=min(wind_speeds), vmax=max(wind_speeds))

    for src, ax in [('XKF2', axes[0]), ('DCM', axes[1])]:
        est_spds = np.array([all_data[s][src] for s in wind_speeds])

        lim = max(float(np.nanmax(sim_spds)), float(np.nanmax(est_spds))) * 1.08
        ax.plot([0, lim], [0, lim], 'k--', lw=0.9, alpha=0.45, label='1:1', zorder=1)

        ax.scatter(sim_spds, est_spds, c=wind_speeds, cmap='plasma',
                   norm=norm, s=55, zorder=5, edgecolors='none')

        # label every 5 m/s
        for i, spd in enumerate(wind_speeds):
            if spd % 5 == 0:
                ax.annotate(f'{spd}', (sim_spds[i], est_spds[i]),
                            textcoords='offset points', xytext=(4, 3),
                            fontsize=7, color='#333333')

        mask = np.isfinite(sim_spds) & np.isfinite(est_spds)
        if mask.sum() > 1:
            coeffs = np.polyfit(sim_spds[mask], est_spds[mask], 1)
            xfit = np.linspace(0, lim, 200)
            color = COLORS[src]
            ax.plot(xfit, np.polyval(coeffs, xfit), color=color, lw=1.8,
                    label=f'fit: y = {coeffs[0]:.3f}x + {coeffs[1]:.2f}')

        ax.set_xlabel('SIMW wind speed (m/s)', fontsize=10)
        ax.set_ylabel(f'{src} wind speed estimate (m/s)', fontsize=10)
        ax.set_title(src, fontsize=12, fontweight='bold', color=COLORS[src])
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_aspect('equal')

    cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap='plasma'),
                      ax=axes, shrink=0.8, pad=0.02)
    cb.set_label('SIM_WIND_SPD target (m/s)', fontsize=9)

    plt.tight_layout()
    out = 'wind_estimate_scatter.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print('Saved', out)
    plt.close(fig)


def make_error_figure():
    """Error (estimate - SIMW) vs SIMW wind speed, both estimators overlaid."""
    wind_speeds = sorted(FLIGHT_LOGS.keys())
    n_tail = 10

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle('Wind speed estimation error (estimate − SIMW) vs actual applied wind speed',
                 fontsize=12)

    for src, color in [('XKF2', COLORS['XKF2']), ('DCM', COLORS['DCM'])]:
        sim_vals, errors = [], []
        for spd in wind_speeds:
            data = read_log(FLIGHT_LOGS[spd])
            sim_v = float(np.mean(data['SIMW']['spd'][-n_tail:])) if len(data['SIMW']['spd']) else float('nan')
            est_v = float(np.mean(data[src]['spd'][-n_tail:])) if len(data[src]['spd']) else float('nan')
            sim_vals.append(sim_v)
            errors.append(est_v - sim_v)

        sim_vals = np.array(sim_vals)
        errors = np.array(errors)
        ax.plot(sim_vals, errors, 'o-', color=color, lw=1.5,
                label=src, markersize=4, alpha=0.85)

    ax.axhline(0, color='k', lw=0.9, ls='--', alpha=0.4)
    ax.fill_between(ax.get_xlim(), -1, 1, color='green', alpha=0.06,
                    label='±1 m/s band')
    ax.set_xlabel('SIMW wind speed (m/s)', fontsize=11)
    ax.set_ylabel('Speed error (m/s)\nestimate − SIMW', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    plt.tight_layout()
    out = 'wind_estimate_error.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print('Saved', out)
    plt.close(fig)


if __name__ == '__main__':
    make_scatter_figure()
    make_error_figure()
    print('Done.')
