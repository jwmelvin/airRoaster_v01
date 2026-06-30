#!/usr/bin/env python3
"""
make_inlet_background.py

Generate an Artisan background profile (.alog) whose INLET temperature setpoint
curve lives in an extra-device channel, for use as the SV source in Artisan's
PID "Follow Background" mode (inlet-temperature control, IKAWA-style).

The profile is built from four (time, temperature) anchors:

    start         -> (0,       T_start)
    dry end       -> (t_dry,   T_dry)
    first crack   -> (t_fc,    T_fc)
    drop          -> (t_drop,  T_drop)

A monotonic cubic Hermite (PCHIP-style) interpolation connects the anchors:
it passes through every anchor, introduces no overshoot/wiggle between them,
and is C1 (smooth slope) so the PID isn't fed slope discontinuities.

The curve is written into extra device #1 channel A (background variable B3).
Point Artisan's PID "Follow Background" at B3 and set the PID input to your
live Inlet probe.

No third-party dependencies (standard library only).

Example:
    python3 make_inlet_background.py \\
        --T_start 350 --T_drop 550 --t_drop 330 --ror_start 60 \\
        --title "350-550, 5:30 +60" --out bkgnd.alog
"""

import argparse
from datetime import datetime, timezone, timedelta


# --------------------------------------------------------------------------
# Monotonic cubic Hermite (PCHIP-style) interpolation, hand-rolled.
# --------------------------------------------------------------------------
def _pchip_slopes(xs, ys):
    """Compute Fritsch-Carlson monotonic slopes at each knot."""
    n = len(xs)
    if n == 2:
        s = (ys[1] - ys[0]) / (xs[1] - xs[0])
        return [s, s]

    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    delta = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n

    for i in range(1, n - 1):
        if delta[i - 1] == 0.0 or delta[i] == 0.0 or (delta[i - 1] > 0) != (delta[i] > 0):
            m[i] = 0.0
        else:
            w1 = 2 * h[i] + h[i - 1]
            w2 = h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i])

    def end_slope(h0, h1, d0, d1):
        s = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if (s > 0) != (d0 > 0):
            s = 0.0
        elif (d0 > 0) != (d1 > 0) and abs(s) > abs(3 * d0):
            s = 3 * d0
        return s

    m[0] = end_slope(h[0], h[1], delta[0], delta[1])
    m[n - 1] = end_slope(h[n - 2], h[n - 3] if n > 3 else h[n - 2],
                         delta[n - 2], delta[n - 3] if n > 3 else delta[n - 2])
    return m


def _hermite(x, xs, ys, ms):
    """Evaluate the piecewise cubic Hermite at scalar x."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    h = xs[hi] - xs[lo]
    t = (x - xs[lo]) / h
    t2 = t * t
    t3 = t2 * t
    h00 = 2 * t3 - 3 * t2 + 1
    h10 = t3 - 2 * t2 + t
    h01 = -2 * t3 + 3 * t2
    h11 = t3 - t2
    return (h00 * ys[lo] + h10 * h * ms[lo]
            + h01 * ys[hi] + h11 * h * ms[hi])


# --------------------------------------------------------------------------
# Profile construction
# --------------------------------------------------------------------------
def build_curve(anchors, interval):
    """anchors: list of (t_seconds, temp). Returns (timex, values) sampled
    every `interval` seconds from 0 to the last anchor time inclusive."""
    xs = [a[0] for a in anchors]
    ys = [a[1] for a in anchors]
    ms = _pchip_slopes(xs, ys)

    n_steps = int(round(xs[-1] / interval))
    timex = [round(i * interval, 3) for i in range(n_steps + 1)]
    values = [round(_hermite(t, xs, ys, ms), 2) for t in timex]
    return timex, values


def build_curve_ror(args):
    """Linear-declining-RoR inlet curve.

    RoR(t) = ror_start + (ror_end - ror_start) * (t / t_drop)  [deg/min]
    T(t)   = T_start + integral_0^t RoR(tau) dtau  (closed-form quadratic)
    """
    interval = args.interval
    t_drop = args.t_drop
    rs = args.ror_start
    re = args.ror_end

    n_steps = int(round(t_drop / interval))
    timex = []
    values = []
    fc_time = None
    a = rs / 60.0
    b = (re - rs) / (60.0 * t_drop) / 2.0
    for i in range(n_steps + 1):
        t = round(i * interval, 3)
        T = args.T_start + a * t + b * t * t
        timex.append(t)
        values.append(round(T, 2))
        if fc_time is None and args.T_fc is not None and T >= args.T_fc:
            fc_time = t
    return timex, values, fc_time


def solve_ror_end(T_start, T_drop, t_drop, ror_start):
    """Solve ror_end so a linearly-declining RoR lands exactly on T_drop."""
    avg = (T_drop - T_start) / (t_drop / 60.0)
    ror_end = 2.0 * avg - ror_start
    return ror_end, avg


def build_curve_ror_endpoint(args):
    """Endpoint-constrained straight-line-RoR inlet curve."""
    ror_end, avg = solve_ror_end(args.T_start, args.T_drop, args.t_drop, args.ror_start)

    interval = args.interval
    t_drop = args.t_drop
    rs = args.ror_start
    re = ror_end
    n_steps = int(round(t_drop / interval))
    timex = []
    values = []
    fc_time = None
    a = rs / 60.0
    b = (re - rs) / (60.0 * t_drop) / 2.0
    for i in range(n_steps + 1):
        t = round(i * interval, 3)
        T = args.T_start + a * t + b * t * t
        timex.append(t)
        values.append(round(T, 2))
        if fc_time is None and args.T_fc is not None and T >= args.T_fc:
            fc_time = t
    values[-1] = round(args.T_drop, 2)
    return timex, values, fc_time, ror_end, avg


def default_title(args):
    """Generate a title like '350 F, +60/min to 550 F @ 5:30' from curve params.

    Falls back gracefully when ror_start or T_drop is unavailable (e.g. anchor
    or plain ror mode)."""
    unit = args.unit.upper()
    t_drop = int(round(args.t_drop))
    mins, secs = divmod(t_drop, 60)
    clock = f"{mins}:{secs:02d}"

    T_s = f"{args.T_start:.0f} {unit}" if args.T_start is not None else "?"
    T_e = f"{args.T_drop:.0f} {unit}" if args.T_drop is not None else "?"

    if args.ror_start is not None:
        ror = f"{args.ror_start:+.0f}/min"
        return f"{T_s}, {ror} to {T_e} @ {clock}"
    return f"{T_s} to {T_e} @ {clock}"


# --------------------------------------------------------------------------
# Canonical skeleton — derived from a verified working airRoaster .alog.
# All roast-specific data is cleared; curve data and metadata are filled in
# by build_alog().
# --------------------------------------------------------------------------
def _skeleton(timex, inlet_values, no_mirror, mode, title, timeindex):
    npts = len(timex)
    ph = [-1.0] * npts
    temp_curve = ph if no_mirror else list(inlet_values)

    now = datetime.now()
    epoch = int(now.timestamp())
    utc_offset_sec = int(now.astimezone().utcoffset().total_seconds())

    return {
        'recording_version': '4.0.2',
        'recording_revision': '1f678a833',
        'recording_build': '0',
        'version': '4.0.2',
        'revision': '1f678a833',
        'build': '0',
        'artisan_os': 'macOS',
        'artisan_os_version': '15.7.7',
        'artisan_os_arch': 'x86_64',
        'mode': mode,
        'viewerMode': False,
        'timeindex': timeindex,
        'flavors': [5.0] * 9,
        'flavors_total_correction': 0.0,
        'flavorlabels': ['Acidity', 'Aftertaste', 'Clean Cup', 'Head',
                         'Fragrance', 'Sweetness', 'Aroma', 'Balance', 'Body'],
        'flavorstartangle': 90,
        'flavoraspect': 1.0,
        'title': title,
        'locale': 'en',
        'plus_store': '',
        'plus_store_label': '',
        'plus_coffee': '',
        'plus_coffee_label': '',
        'beans': '',
        'weight': [0.0, 0.0, 'g'],
        'defects_weight': 0.0,
        'volume': [0.0, 0.0, 'l'],
        'density': [0.0, 'g', 1.0, 'l'],
        'density_roasted': [0.0, 'g', 1.0, 'l'],
        'roastertype': 'airRoaster v01',
        'roastersize': 1.0,
        'roasterheating': 3,
        'machinesetup': '',
        'operator': '',
        'organization': '',
        'drumspeed': '',
        'heavyFC': False,
        'lowFC': False,
        'lightCut': False,
        'darkCut': False,
        'drops': False,
        'oily': False,
        'uneven': False,
        'tipping': False,
        'scorching': False,
        'divots': False,
        'whole_color': 0.0,
        'ground_color': 0.0,
        'color_system': '',
        'volumeCalcWeightIn': '',
        'volumeCalcWeightOut': '',
        'roastdate': now.strftime('%a %b %d %Y'),
        'roastisodate': now.strftime('%Y-%m-%d'),
        'roasttime': now.strftime('%H:%M:%S'),
        'roastepoch': epoch,
        'roasttzoffset': utc_offset_sec,
        'roastbatchnr': 0,
        'roastbatchprefix': '',
        'roastbatchpos': 0,
        'roastUUID': '',
        'beansize_min': '0',
        'beansize_max': '0',
        'specialevents': [],
        'specialeventstype': [],
        'specialeventsvalue': [],
        'specialeventsStrings': [],
        'default_etypes': [True, True, True, False, True],
        'default_etypes_set': [0, 0, 0, 0, 0],
        'etypes': ['Air', 'Drum', 'Damper', 'Heater', '--'],
        'roastingnotes': '',
        'cuppingnotes': '',
        'timex': list(timex),
        'temp1': temp_curve,
        'temp2': list(ph),
        'phases': [100, 199, 255, 399],
        'zmax': 64,
        'zmin': 0,
        'ymax': 600,
        'ymin': 0,
        'xmin': 0.0,
        'xmax': timex[-1] * 1.15,
        'ambientTemp': 0.0,
        'ambient_humidity': 0.0,
        'ambient_pressure': 0.0,
        'moisture_greens': 0.0,
        'greens_temp': 0.0,
        'moisture_roasted': 0.0,
        # Two extra devices matching the airRoaster Artisan setup:
        #   device 58 = Phidget TMP1200 1xRTD (Inlet probe)  -> B3 (extratemp1[0])
        #   device 22 = +PID SV/DUTY %                        -> B5 (extratemp1[1])
        'extradevices': [58, 22],
        'extraname1': ['Inlet', 'SV'],
        'extraname2': ['extra', 'Duty %'],
        'extratimex': [list(timex), list(timex)],
        'extratemp1': [list(inlet_values), list(ph)],
        'extratemp2': [list(ph), list(ph)],
        'extramathexpression1': ['', ''],
        'extramathexpression2': ['', ''],
        'extradevicecolor1': ['black', '#00f900ff'],
        'extradevicecolor2': ['black', '#73fdffff'],
        'extraLCDvisibility1': [True, True, False, False, False, False, False, True, True, False],
        'extraLCDvisibility2': [False, True, False, False, False, False, False, True, True, False],
        'extraCurveVisibility1': [True] * 10,
        'extraCurveVisibility2': [False, True, True, True, True, True, True, True, True, True],
        'extraDelta1': [False] * 10,
        'extraDelta2': [False] * 10,
        'extraFill1': [0] * 10,
        'extraFill2': [0] * 10,
        'extramarkersizes1': [6.0, 6.0],
        'extramarkersizes2': [6.0, 6.0],
        'extramarkers1': ['None', 'None'],
        'extramarkers2': ['None', 'None'],
        'extralinewidths1': [1.0, 1.0],
        'extralinewidths2': [1.0, 1.0],
        'extralinestyles1': ['-', '-'],
        'extralinestyles2': ['-', '-'],
        'extradrawstyles1': ['default', 'default'],
        'extradrawstyles2': ['default', 'default'],
        'externalprogram': 'test.py',
        'externaloutprogram': 'out.py',
        'extraNoneTempHint1': [True, False, True],
        'extraNoneTempHint2': [True, False, True],
        'alarmsetlabel': '',
        'alarmflag': [],
        'alarmguard': [],
        'alarmnegguard': [],
        'alarmtime': [],
        'alarmoffset': [],
        'alarmcond': [],
        'alarmsource': [],
        'alarmtemperature': [],
        'alarmaction': [],
        'alarmbeep': [],
        'alarmstrings': [],
        'backgroundpath': '',
        'samplinginterval': 1.0,
        'svLabel': '',
        'svValues': [0] * 8,
        'svRamps': [0] * 8,
        'svSoaks': [0] * 8,
        'svActions': [-1] * 8,
        'svBeeps': [False] * 8,
        'svDescriptions': [''] * 8,
        'pidKp': 0.1,
        'pidKi': 0.03,
        'pidPsetpointWeight': 1.0,
        'pidDsetpointWeight': 1.0,
        'pidSource': 5,
        'svLookahead': 0,
        'ramp_lookahead': 0,
        'pidKp1': 15.0,
        'pidKi1': 0.01,
        'pidKd1': 20.0,
        'pidKp2': 15.0,
        'pidKi2': 0.01,
        'pidKd2': 20.0,
        'pidSchedule0': 32,
        'pidSchedule1': 32,
        'pidSchedule2': 32,
        'gain_scheduling': False,
        'gain_scheduling_on_SV': True,
        'gain_scheduling_quadratic': False,
        'devices': ['Phidget TMP1200 1xRTD A', 'Phidget TMP1101 4xTC 01', '+PID SV/DUTY %'],
        'elevation': 0,
        'computed': {},
        'anno_positions': [],
        'flag_positions': [],
        'loadlabels': ['', '', '', ''],
        'loadratings': [0.0, 0.0, 0.0, 0.0],
        'ratingunits': [0, 0, 0, 0],
        'sourcetypes': [0, 0, 0, 0],
        'load_etypes': [0, 0, 0, 0],
        'presssure_percents': [False, False, False, False],
        'loadevent_zeropcts': [0, 0, 0, 0],
        'loadevent_hundpcts': [100, 100, 100, 100],
        'meterlabels': ['', ''],
        'meterunits': [3, 3],
        'meterfuels': [2, 2],
        'metersources': [0, 0],
        'meterreads': [[0.0] * 9, [0.0] * 9],
        'co2kg_per_btu': [6.288e-05, 5.291e-05, 0.0002964],
        'biogas_co2_reduction': 0.7562,
        'preheatDuration': 0,
        'preheatenergies': [0.0, 0.0, 0.0, 0.0],
        'betweenbatchDuration': 0,
        'betweenbatchenergies': [0.0, 0.0, 0.0, 0.0],
        'coolingDuration': 0,
        'coolingenergies': [0.0, 0.0, 0.0, 0.0],
        'betweenbatch_after_preheat': True,
        'electricEnergyMix': 0,
        'gasMix': 0,
        'bbp_begin': 'Start',
        'bbp_time_added_from_prev': 0.0,
        'bbp_endroast_epoch_msec': 0,
        'bbp_endevents': [],
        'bbp_dropevents': [],
        'bbp_dropbt': 0.0,
        'bbp_dropet': 0.0,
        'bbp_drop_to_end': 0.0,
        'plus_sync_record_hash': '',
    }


def build_alog(timex, inlet_values, args, anchors):
    """Assemble the Artisan profile dict from the canonical skeleton."""
    npts = len(timex)

    def idx_at(t):
        if t is None:
            return 0
        return min(range(npts), key=lambda i: abs(timex[i] - t))

    timeindex = [0,
                 idx_at(anchors[1][0]),
                 idx_at(anchors[2][0]),
                 0, 0, 0,
                 idx_at(anchors[3][0]),
                 0]

    mode = 'C' if args.unit.upper() == 'C' else 'F'
    title = args.title if args.title else default_title(args)

    profile = _skeleton(timex, inlet_values, args.no_mirror, mode, title, timeindex)

    # B3 = extra device #1 channel A (extratemp1[0]) — fixed for this setup.
    bvar = 3
    return profile, bvar


def main():
    epilog = """\
modes
-----
  anchor        Connect four (t,T) points with a smooth monotonic curve.
                Hits dry / FC / drop exactly.
                needs: --T_start --t_dry --T_dry --t_fc --T_fc --t_drop --T_drop

  ror           Linearly-declining rate-of-rise. You set the RoR shape; the end
                temperature falls out of it (not pinned).
                needs: --T_start --ror_start --ror_end --t_drop  [--T_fc]

  ror_endpoint  Straight-line declining RoR pinned to T_start and T_drop.
                ror_end is SOLVED so the curve lands on T_drop; ror_start sets
                the front-load bias.
                needs: --T_start --T_drop --t_drop --ror_start  [--T_fc]

examples
--------
  # ror_endpoint (most common)
  make_inlet_background.py --mode ror_endpoint \\
      --T_start 350 --T_drop 550 --t_drop 330 --ror_start 60 \\
      --title "350-550, 5:30 +60" --out bkgnd.alog

  # anchor: hit specific points
  make_inlet_background.py \\
      --T_start 350 --t_dry 180 --T_dry 400 \\
      --t_fc 270 --T_fc 490 --t_drop 330 --T_drop 550 \\
      --title "350-550, 5:30 anchor" --out bkgnd_anchor.alog

artisan usage
-------------
  Load the .alog as a background profile, set the software PID input to your
  live Inlet probe, and set PID mode = Follow Background referencing B3.
  Times are seconds; temps are in --unit (F default to match airRoaster).
"""
    p = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Generate an Artisan .alog background with an inlet SV curve "
                    "in extra device #1 channel A (background variable B3).",
        epilog=epilog)
    p.add_argument("--mode", choices=["anchor", "ror", "ror_endpoint"], default="ror_endpoint",
                   help="curve generation mode (default: ror_endpoint)")

    # Anchor-mode inputs
    p.add_argument("--T_start", type=float, help="starting temp")
    p.add_argument("--t_dry", type=float, help="[anchor] time to dry end (s)")
    p.add_argument("--T_dry", type=float, help="[anchor] dry-end temp")
    p.add_argument("--t_fc", type=float, help="[anchor] time to first crack (s)")
    p.add_argument("--T_fc", type=float,
                   help="[anchor] FC temp; [ror] FC threshold for prediction")
    p.add_argument("--t_drop", type=float, help="time to drop (s)")
    p.add_argument("--T_drop", type=float, help="drop temp (anchor/ror_endpoint)")

    # RoR-mode inputs
    p.add_argument("--ror_start", type=float, help="initial rate-of-rise (deg/min)")
    p.add_argument("--ror_end", type=float, help="[ror] final rate-of-rise (deg/min)")

    # Output options
    p.add_argument("--interval", type=float, default=1.0, help="sample interval (s)")
    p.add_argument("--unit", choices=["C", "F", "c", "f"], default="F",
                   help="temperature unit (default: F)")
    p.add_argument("--title", default=None,
                   help="profile title (default: auto-generated from curve params)")
    p.add_argument("--no-mirror", action="store_true",
                   help="blank the BT slot; only B3 carries the inlet SV curve")
    p.add_argument("--out", default="inlet_background.alog")

    args = p.parse_args()

    def require(names):
        missing = [n for n in names if getattr(args, n) is None]
        if missing:
            raise SystemExit("--mode {} requires: {}".format(
                args.mode, ", ".join("--" + m for m in missing)))

    if args.mode == "anchor":
        require(["T_start", "t_dry", "T_dry", "t_fc", "T_fc", "t_drop", "T_drop"])
        anchors = [
            (0.0, args.T_start),
            (args.t_dry, args.T_dry),
            (args.t_fc, args.T_fc),
            (args.t_drop, args.T_drop),
        ]
        ts = [a[0] for a in anchors]
        if not all(ts[i] < ts[i + 1] for i in range(len(ts) - 1)):
            raise SystemExit("Anchor times must be strictly increasing: "
                             "0 < t_dry < t_fc < t_drop. Got {}".format(ts))
        timex, inlet_values = build_curve(anchors, args.interval)
        fc_time = args.t_fc

    elif args.mode == "ror":
        require(["T_start", "t_drop", "ror_start", "ror_end"])
        timex, inlet_values, fc_time = build_curve_ror(args)
        anchors = [
            (0.0, args.T_start),
            (None, None),
            (fc_time, args.T_fc),
            (args.t_drop, inlet_values[-1]),
        ]

    else:  # ror_endpoint
        require(["T_start", "T_drop", "t_drop", "ror_start"])
        timex, inlet_values, fc_time, solved_ror_end, avg_ror = \
            build_curve_ror_endpoint(args)
        anchors = [
            (0.0, args.T_start),
            (None, None),
            (fc_time, args.T_fc),
            (args.t_drop, args.T_drop),
        ]

    profile, bvar = build_alog(timex, inlet_values, args, anchors)

    # Artisan reads .alog via ast.literal_eval (Python repr, not JSON).
    with open(args.out, "w") as f:
        f.write(repr(profile))

    title_used = profile['title']
    print("Wrote {} ({} samples, {:.0f}s, every {}s).".format(
        args.out, len(timex), timex[-1], args.interval))
    print("Title: {!r}".format(title_used))
    print("Mode: {}.  Unit: {}.".format(args.mode, args.unit.upper()))
    print("Inlet SV curve -> extra device #1 channel A -> background variable B{}.".format(bvar))
    print("In Artisan: load as background, set PID input = live Inlet probe, "
          "PID mode = Follow Background referencing B{}.".format(bvar))

    if args.mode == "ror":
        eff_end = inlet_values[-1]
        print("RoR {:.1f} -> {:.1f} deg/min over {:.0f}s; "
              "start {:.1f} -> end {:.1f}.".format(
                  args.ror_start, args.ror_end, args.t_drop,
                  args.T_start, eff_end))
        if args.T_fc is not None:
            if fc_time is not None:
                print("Predicted FC (T>={:.1f}) at {:.0f}s.".format(args.T_fc, fc_time))
            else:
                print("FC threshold {:.1f} never reached.".format(args.T_fc))

    elif args.mode == "ror_endpoint":
        bias = args.ror_start - avg_ror
        print("Endpoint-pinned: {:.1f} -> {:.1f} over {:.0f}s "
              "(avg RoR {:.1f} deg/min).".format(
                  args.T_start, args.T_drop, args.t_drop, avg_ror))
        print("Solved RoR: start {:.1f} -> end {:.1f} deg/min "
              "(front-load bias {:+.1f}).".format(
                  args.ror_start, solved_ror_end, bias))
        if solved_ror_end < 0:
            print("WARNING: solved ror_end is NEGATIVE ({:.1f}) -> inlet SV cools "
                  "late in roast. Lower --ror_start (ceiling: {:.1f}).".format(
                      solved_ror_end, 2 * avg_ror))
        if args.T_fc is not None:
            if fc_time is not None:
                print("Predicted FC (T>={:.1f}) at {:.0f}s.".format(args.T_fc, fc_time))
            else:
                print("FC threshold {:.1f} never reached before drop.".format(args.T_fc))

    else:
        print("Peak inlet {:.1f}; FC anchor {:.1f} at {:.0f}s; "
              "drop {:.1f} at {:.0f}s.".format(
                  max(inlet_values), args.T_fc, args.t_fc,
                  args.T_drop, args.t_drop))


if __name__ == "__main__":
    main()
