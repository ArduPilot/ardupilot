#!/usr/bin/env python3
'''Animate one or more Renode flight tlogs into a single GIF.

Written to answer a specific question: the CubeOrangeZephyr copter lands tens
of metres from home while the ChibiOS build on the same platform lands inside a
metre, and the endpoint numbers alone do not say *when* the two tracks diverge.
Watching them fly side by side does.

Both tracks are drawn on ONE set of axes at equal aspect, because the whole
point is the comparison - separate panels auto-scale independently and hide the
thing you are looking for. A flight that stays in the mission box and a flight
that wanders six times outside it look identical on their own axes.

    ./Tools/scripts/plot_flight_gif.py build/renode-test/*/flight.tlog -o flight.gif

Each frame is 10 seconds of flight by default. Waypoints come from
Tools/renode/tests/test_mission.py rather than being repeated here: a tlog records only what
the vehicle sent, so the mission the harness uploaded is not in it.
'''

import argparse
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'modules' / 'mavlink'))
sys.path.insert(0, str(ROOT / 'Tools' / 'renode' / 'tests'))

from pymavlink import mavutil  # noqa: E402

import matplotlib                                    # noqa: E402
matplotlib.use('Agg')                                # no display in CI
import matplotlib.pyplot as plt                      # noqa: E402
from PIL import Image                                # noqa: E402

EARTH_RADIUS_M = 6378137.0

# One colour per track. Deliberately not a colormap lookup - two or three
# flights is the realistic case and named colours read better in a legend.
TRACK_COLOURS = ('#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#8c564b')


def ne_offset(origin_lat, origin_lon, lat, lon):
    '''Metres north and east of an origin, flat-earth.

    Distances here are tens of metres, so the small-angle approximation costs
    nothing and avoids a projection dependency.
    '''
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east = (math.radians(lon - origin_lon) * EARTH_RADIUS_M *
            math.cos(math.radians((origin_lat + lat) * 0.5)))
    return north, east


def read_track(path):
    '''(times, lats, lons, alts) from a tlog, dropping pre-fix samples.

    A vehicle publishes GLOBAL_POSITION_INT before it has a position, with
    lat/lon at 0. Those are 5000 km from Canberra and would set the axis limits
    for the whole animation on their own.
    '''
    conn = mavutil.mavlink_connection(str(path))
    times, lats, lons, alts = [], [], [], []
    while True:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg is None:
            break
        if msg.lat == 0 and msg.lon == 0:
            continue
        stamp = getattr(msg, '_timestamp', None)
        if stamp is None:
            continue
        times.append(stamp)
        lats.append(msg.lat * 1e-7)
        lons.append(msg.lon * 1e-7)
        alts.append(msg.relative_alt * 0.001)
    if not times:
        raise SystemExit('%s: no positioned GLOBAL_POSITION_INT messages' % path)
    start = times[0]
    return [t - start for t in times], lats, lons, alts


def mission_waypoints():
    '''Mission offsets in metres NE of home, from the harness definition.

    Imported rather than duplicated so this cannot drift from the mission the
    flights actually flew. Falls back to an empty list if the import fails -
    the animation is still useful without the waypoint markers.
    '''
    try:
        import test_mission
        return list(test_mission.WAYPOINT_OFFSETS)
    except Exception as error:                       # noqa: BLE001
        print('note: mission waypoints unavailable (%s)' % error)
        return []


def sample_at(times, values, when):
    '''Last value at or before `when`, or None before the track starts.

    Position is a step function between samples here rather than interpolated:
    the samples are ~2.6 s apart and inventing intermediate positions would
    draw a smoother path than the vehicle actually reported.
    '''
    if not times or when < times[0]:
        return None
    index = 0
    for i, t in enumerate(times):
        if t > when:
            break
        index = i
    return values[index]


def build_tracks(paths, labels):
    tracks = []
    for i, path in enumerate(paths):
        times, lats, lons, alts = read_track(path)
        label = labels[i] if i < len(labels) else Path(path).parent.name
        tracks.append({
            'label': label,
            'times': times,
            'lats': lats,
            'lons': lons,
            'alts': alts,
            'colour': TRACK_COLOURS[i % len(TRACK_COLOURS)],
        })
    return tracks


def render(tracks, waypoints, out_path, seconds_per_frame, width, height, dpi,
           frame_ms):
    # Every track is drawn against the FIRST track's launch point. Each flight
    # has its own home a few metres away, and re-centring per track would hide
    # exactly the offsets being compared.
    origin_lat, origin_lon = tracks[0]['lats'][0], tracks[0]['lons'][0]
    for track in tracks:
        track['ne'] = [ne_offset(origin_lat, origin_lon, la, lo)
                       for la, lo in zip(track['lats'], track['lons'])]
        track['north'] = [p[0] for p in track['ne']]
        track['east'] = [p[1] for p in track['ne']]

    everything_n = [n for t in tracks for n in t['north']] + [w[0] for w in waypoints] + [0.0]
    everything_e = [e for t in tracks for e in t['east']] + [w[1] for w in waypoints] + [0.0]
    margin = max(5.0, 0.08 * max(max(everything_e) - min(everything_e),
                                 max(everything_n) - min(everything_n)))
    limits = (min(everything_e) - margin, max(everything_e) + margin,
              min(everything_n) - margin, max(everything_n) + margin)

    duration = max(t['times'][-1] for t in tracks)
    frame_count = int(duration // seconds_per_frame) + 1
    print('%d tracks, %.0f s, %d frames at %.0f s each'
          % (len(tracks), duration, frame_count, seconds_per_frame))

    frames = []
    figsize = (width / dpi, height / dpi)
    for frame in range(frame_count):
        now = frame * seconds_per_frame
        fig = plt.figure(figsize=figsize, dpi=dpi)
        axes = fig.add_subplot(111)
        axes.set_aspect('equal', adjustable='box')
        axes.set_xlim(limits[0], limits[1])
        axes.set_ylim(limits[2], limits[3])
        axes.grid(True, alpha=0.3, linewidth=0.5)
        axes.set_xlabel('east (m)')
        axes.set_ylabel('north (m)')

        # launch, then the mission it was asked to fly
        axes.plot(0, 0, marker='*', markersize=18, color='black',
                  linestyle='none', label='launch', zorder=5)
        if waypoints:
            axes.plot([w[1] for w in waypoints], [w[0] for w in waypoints],
                      marker='s', markersize=8, markerfacecolor='none',
                      markeredgecolor='black', linestyle='none',
                      label='waypoints', zorder=4)
            for i, (north, east) in enumerate(waypoints, start=1):
                axes.annotate(str(i), (east, north), textcoords='offset points',
                              xytext=(6, 4), fontsize=8)

        for track in tracks:
            times = track['times']
            flown = [i for i, t in enumerate(times) if t <= now]
            if flown:
                upto = flown[-1] + 1
                axes.plot(track['east'][:upto], track['north'][:upto],
                          color=track['colour'], linewidth=1.4, alpha=0.85,
                          label=track['label'], zorder=3)
                axes.plot(track['east'][upto - 1], track['north'][upto - 1],
                          marker='o', markersize=9, color=track['colour'],
                          linestyle='none', zorder=6)
            else:
                # keep the label in the legend before this track has started
                axes.plot([], [], color=track['colour'], linewidth=1.4,
                          label=track['label'])

        # distance from launch is the number the test judges, so show it live
        readout = []
        for track in tracks:
            north = sample_at(track['times'], track['north'], now)
            east = sample_at(track['times'], track['east'], now)
            alt = sample_at(track['times'], track['alts'], now)
            if north is None:
                readout.append('%s: -' % track['label'])
            else:
                readout.append('%s: %5.1f m from launch, %4.1f m up'
                               % (track['label'], math.hypot(north, east), alt))
        axes.set_title('t = %3d s\n%s' % (now, '\n'.join(readout)),
                       fontsize=9, loc='left')
        axes.legend(loc='upper right', fontsize=8, framealpha=0.9)
        fig.tight_layout()

        fig.canvas.draw()
        frames.append(Image.frombytes(
            'RGBA', fig.canvas.get_width_height(),
            bytes(fig.canvas.buffer_rgba())).convert('P', palette=Image.ADAPTIVE))
        plt.close(fig)

    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=frame_ms, loop=0, optimize=True)
    print('wrote %s (%d frames, %.1f KB)'
          % (out_path, len(frames), os.path.getsize(out_path) / 1024.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('tlogs', nargs='+', help='flight.tlog files to animate')
    parser.add_argument('-o', '--output', default='flight.gif')
    parser.add_argument('--label', action='append', default=[],
                        help='name for each track, in the order given')
    parser.add_argument('--seconds-per-frame', type=float, default=10.0,
                        help='flight seconds each frame advances (default 10)')
    parser.add_argument('--frame-ms', type=int, default=400,
                        help='display time per frame in the GIF (default 400)')
    parser.add_argument('--width', type=int, default=800)
    parser.add_argument('--height', type=int, default=600)
    parser.add_argument('--dpi', type=int, default=100)
    args = parser.parse_args()

    tracks = build_tracks(args.tlogs, args.label)
    render(tracks, mission_waypoints(), args.output, args.seconds_per_frame,
           args.width, args.height, args.dpi, args.frame_ms)
    return 0


if __name__ == '__main__':
    sys.exit(main())
