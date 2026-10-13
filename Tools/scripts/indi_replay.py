#!/usr/bin/env python3
'''
Offline replay of INDI control-effectiveness identification against a
dataflash log: validation stage 1 of libraries/AP_INDI/SPEC.md.

Reads RATE messages (rate controller targets, gyro rates and normalised
roll/pitch/yaw outputs at the control loop rate), differentiates the
rates with the same 7-tap Savitzky-Golay filter the flight code uses,
passes the outputs through the same actuator model (delay, first-order
lag, then the two-pole low-pass the gyro went through, read from
INS_GYRO_FILTER, so command and response are filtered alike) and fits the effectiveness
of each axis's own command on its angular acceleration with weighted
recursive least squares on increments, as AP_INDI_Ident does but in
double precision. Only single strokes of at least half the minimum
stroke are accepted, as in flight. The fit is also repeated per segment
of the accepted strokes so its stability can be judged: the stage 1
pass criterion is a fitted G stable to within 10 % across the flight.
The offline fit cannot tell a deliberate probe from the loop countering
a disturbance, so it carries the closed-loop low bias SPEC.md section 5
describes where the flight code does not.

If the log also carries INDG messages (the in-flight estimate) they
are printed for comparison.

Usage:
    indi_replay.py [--segments N] [--tau S] [--delay S] [--gyro-filter HZ] [--prior G] LOG.BIN

AP_FLAKE8_CLEAN
'''

import argparse
import math

from pymavlink import DFReader

# Savitzky-Golay first-derivative coefficients, 7 taps, centre estimate;
# weights -3..3 over 28, newest sample last
SG_WEIGHTS = [-3, -2, -1, 0, 1, 2, 3]
SG_NORM = 28.0
SG_DELAY = 3
# a stroke is accepted from half the minimum stroke, as the flight code
# does, because the actuator lag spreads a pulse of du_min over more
# than one window
STROKE_GATE_FRAC = 0.5


class Biquad:
    '''two-pole Butterworth low-pass, the same coefficients and form as
    libraries/Filter/LowPassFilter2p.cpp so the command path matches
    the gyro filter sample for sample'''

    def __init__(self, sample_hz, cutoff_hz):
        # the flight code keeps the cutoff under 0.4 of the sample rate
        cutoff_hz = min(cutoff_hz, sample_hz * 0.4)
        self.enabled = cutoff_hz > 0
        if not self.enabled:
            return
        ohm = math.tan(math.pi * cutoff_hz / sample_hz)
        c = 1.0 + 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm
        self.b0 = ohm * ohm / c
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (ohm * ohm - 1.0) / c
        self.a2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm) / c
        self.d1 = None
        self.d2 = None

    def apply(self, x):
        if not self.enabled:
            return x
        if self.d1 is None:
            # start settled on the first sample, as reset(value) does
            self.d1 = x / (1.0 + self.a1 + self.a2)
            self.d2 = self.d1
        d0 = x - self.d1 * self.a1 - self.d2 * self.a2
        y = d0 * self.b0 + self.d1 * self.b1 + self.d2 * self.b2
        self.d2 = self.d1
        self.d1 = d0
        return y


class AxisRLS:
    '''weighted recursive least squares on increments for one axis, scalar
    regressor (the axis's own command), state kept as the ratio to the
    prior as the flight code does'''

    def __init__(self, prior, prior_rel_sigma=0.7, lam=0.995, du_min=0.05):
        self.prior = prior
        self.theta = 1.0
        self.P = prior_rel_sigma ** 2
        self.P0 = self.P
        self.r = 0.1 ** 2
        self.lam = lam
        self.du_min = du_min
        self.accepted = 0

    def update(self, du, dwd):
        if abs(du) < STROKE_GATE_FRAC * self.du_min:
            return False
        phi = du
        y = dwd / self.prior
        y_hat = self.theta * phi
        innov = y - y_hat
        denom = self.lam * self.r + phi * self.P * phi
        if denom <= 0:
            return False
        k = self.P * phi / denom
        self.theta += k * innov
        e_post = y - self.theta * phi
        r_sample = max(innov * e_post, 0.0)
        w = 1.0 / min(self.accepted + 1, 50)
        self.r = min((1 - w) * self.r + w * r_sample, 4 * self.r)
        self.r = max(self.r, 0.005 ** 2)
        self.P = (self.P - k * phi * self.P) / self.lam
        self.P = min(max(self.P, 0.02 ** 2), self.P0)
        self.accepted += 1
        return True

    def g(self):
        return self.theta * self.prior

    def rel_sigma(self):
        return math.sqrt(self.P) / abs(self.theta) if self.theta != 0 else float('inf')


def load_rate(path):
    '''return lists of (t_s, [rates rad/s], [outputs]) from RATE messages,
    the INDG estimates and the logged INS_GYRO_FILTER (None if absent)'''
    reader = DFReader.DFReader_binary(path, zero_time_base=True)
    samples = []
    indg = []
    gyro_filter_hz = None
    while True:
        m = reader.recv_match(type=['RATE', 'INDG', 'PARM'])
        if m is None:
            break
        mtype = m.get_type()
        if mtype == 'RATE':
            rates = [math.radians(m.R), math.radians(m.P), math.radians(m.Y)]
            outs = [m.ROut, m.POut, m.YOut]
            samples.append((m.TimeUS * 1.0e-6, rates, outs))
        elif mtype == 'INDG':
            indg.append((m.TimeUS * 1.0e-6, m.I, [m.G0, m.G1, m.G2][m.I], m.RS, m.St))
        elif m.Name == 'INS_GYRO_FILTER':
            gyro_filter_hz = m.Value
    return samples, indg, gyro_filter_hz


def stroke_is_monotonic(uf_hist, n, i, window_n):
    '''every step of the command over the window moves the same way, so
    the increment is one stroke and not a reversal (as in flight)'''
    direction = 0.0
    for k in range(window_n):
        step = uf_hist[n - k][i] - uf_hist[n - k - 1][i]
        if step == 0.0:
            continue
        if direction == 0.0:
            direction = step
        elif direction * step < 0:
            return False
    return True


def replay(samples, prior, tau_s, delay_s, window_s, lam, du_min, segments, gyro_filter_hz):
    if len(samples) < 50:
        raise SystemExit("not enough RATE samples (%u); log needs ATTITUDE_FAST logging" % len(samples))
    dt = (samples[-1][0] - samples[0][0]) / (len(samples) - 1)
    if dt > 0.02:
        print("WARNING: RATE is logged at only %.0f Hz; the identification needs the control loop rate. "
              "Log with LOG_BITMASK bit 0 (ATTITUDE_FAST) set and LOG_FILE_RATEMAX 0." % (1.0 / dt))
    delay_n = int(round(delay_s / dt))
    window_n = max(2, int(round(window_s / dt)))
    print("loop period %.4f s, %u samples, lag %.3f s, delay %u samples, window %u samples" %
          (dt, len(samples), tau_s, delay_n, window_n))

    whole = [AxisRLS(prior[i], lam=lam, du_min=du_min) for i in range(3)]
    # the strokes each axis accepts, kept so the flight can be cut into
    # segments of equal stroke count (a log is mostly ground and cruise
    # with nothing to identify from)
    strokes = [[] for _ in range(3)]

    u_f = [0.0, 0.0, 0.0]
    lpf = [Biquad(1.0 / dt, gyro_filter_hz or 0.0) for _ in range(3)]
    uf_hist = []
    wd_hist = []
    for n, (t, rates, outs) in enumerate(samples):
        # actuator model: pure delay, first-order lag, then the gyro's
        # own low-pass so command and response are filtered alike
        u_delayed = samples[max(0, n - delay_n)][2]
        if tau_s <= dt:
            u_f = list(u_delayed)
        else:
            u_f = [u_f[i] + (u_delayed[i] - u_f[i]) * dt / tau_s for i in range(3)]
        uf_hist.append([lpf[i].apply(u_f[i]) for i in range(3)])
        # angular acceleration by Savitzky-Golay over the last 7 rates
        if n >= 6:
            wd = []
            for i in range(3):
                acc = 0.0
                for k, w in enumerate(SG_WEIGHTS):
                    acc += w * samples[n - 6 + k][1][i]
                wd.append(acc / (SG_NORM * dt))
            wd_hist.append(wd)
        else:
            wd_hist.append(None)
        if n < 6 + window_n + SG_DELAY:
            continue
        if wd_hist[n] is None or wd_hist[n - window_n] is None:
            continue
        for i in range(3):
            # command aligned with the acceleration estimate (SG_DELAY back)
            du = uf_hist[n - SG_DELAY][i] - uf_hist[n - SG_DELAY - window_n][i]
            dwd = wd_hist[n][i] - wd_hist[n - window_n][i]
            if not stroke_is_monotonic(uf_hist, n - SG_DELAY, i, window_n):
                continue
            if whole[i].update(du, dwd):
                strokes[i].append((du, dwd))
    # stability check: refit each equal share of the accepted strokes
    seg_fits = [[] for _ in range(3)]
    for i in range(3):
        seg_len = len(strokes[i]) // segments
        if seg_len < 20:
            continue
        for k in range(segments):
            rls = AxisRLS(prior[i], lam=lam, du_min=du_min)
            for (du, dwd) in strokes[i][k * seg_len:(k + 1) * seg_len]:
                rls.update(du, dwd)
            seg_fits[i].append(rls.g())
    return whole, seg_fits


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log')
    parser.add_argument('--segments', type=int, default=10, help='flight segments for the stability check')
    parser.add_argument('--tau', type=float, default=0.010, help='actuator lag time constant, s')
    parser.add_argument('--delay', type=float, default=0.010, help='actuator plus sensor delay, s')
    parser.add_argument('--window', type=float, default=0.010, help='increment window, s')
    parser.add_argument('--lam', type=float, default=0.9995, help='forgetting factor')
    parser.add_argument('--du-min', type=float, default=0.05, help='minimum stroke, normalised')
    parser.add_argument('--gyro-filter', type=float, default=None,
                        help='gyro low-pass cutoff to match on the command, Hz (default: INS_GYRO_FILTER from the log)')
    parser.add_argument('--prior', type=float, nargs=3, default=[400.0, 400.0, 40.0],
                        help='effectiveness prior per axis, rad/s^2 per unit command')
    args = parser.parse_args()

    samples, indg, log_gyro_filter = load_rate(args.log)
    gyro_filter = args.gyro_filter if args.gyro_filter is not None else log_gyro_filter
    if gyro_filter is None:
        print("WARNING: no INS_GYRO_FILTER in the log and none given; command path left unfiltered")
    whole, seg_fits = replay(samples, args.prior, args.tau, args.delay, args.window, args.lam, args.du_min,
                             args.segments, gyro_filter)

    names = ['roll', 'pitch', 'yaw']
    ok = True
    for i in range(3):
        fits = [f for f in seg_fits[i] if f is not None]
        line = "%-5s whole-flight G=%8.1f rad/s^2/unit  rel_sigma=%.3f  accepted=%u" % (
            names[i], whole[i].g(), whole[i].rel_sigma(), whole[i].accepted)
        if len(fits) >= 2:
            mean = sum(fits) / len(fits)
            spread = (max(fits) - min(fits)) / abs(mean) if mean != 0 else float('inf')
            line += "  segments: %s  spread=%.1f%%" % (" ".join("%.0f" % f for f in fits), spread * 100)
            if spread > 0.10:
                ok = False
        else:
            line += "  (too few excited segments for a stability check)"
        print(line)
    if indg:
        last = {}
        for (t, i, g, rs, st) in indg:
            last[i] = (g, rs, st)
        for i in sorted(last):
            g, rs, st = last[i]
            print("in-flight INDG %-5s final G=%8.1f rel_sigma=%.3f state=%u" % (names[i], g, rs, st))
    print("stage 1 criterion (segment spread within 10%%): %s" % ("PASS" if ok else "FAIL"))


if __name__ == '__main__':
    main()
