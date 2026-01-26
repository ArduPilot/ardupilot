import math

GRAVITY = 9.80665

def deg_to_rad(val):
    return val * (math.pi / 180)

def compute_braking_profile(starting_vel, max_accel, max_jerk):
    threshold = (max_accel * max_accel) / max_jerk  # v where plateau begins
    t1 = 0.0
    t2 = 0.0
    t_total = 0.0
    triangular = False

    if starting_vel <= threshold + 1e-9:
        # Triangular jerk-limited profile (no constant -max_accel phase)
        triangular = True
        t1 = math.sqrt(starting_vel / max_jerk)  # from starting_vel = j * t1^2
        t2 = 0.0
        t_total = 2 * t1
    else:
        # S-curve with constant -max_accel plateau in the middle
        triangular = False
        t1 = max_accel / max_jerk
        v_ramps = threshold         # Δv from both ramps
        v_const = starting_vel - v_ramps
        t2 = v_const / max_accel
        t_total = 2 * t1 + t2

    steps = 600
    dt = t_total / steps

    t_arr = []
    x_arr = []

    t = 0.0
    v = starting_vel
    x = 0.0

    # Range is inclusive of steps, so use steps + 1
    for i in range(steps + 1):
        # Compute acceleration at current time
        a = 0.0
        
        if triangular:
            if t <= t1:
                a = -max_jerk * t
            else:
                # second half: a(t) goes back to 0 linearly
                a = -max_jerk * (2 * t1 - t)
        else:
            if t <= t1:
                a = -max_jerk * t
            elif t <= t1 + t2:
                a = -max_accel
            else:
                a = -max_accel + max_jerk * (t - (t1 + t2))
                if a > 0: 
                    a = 0.0  # numerical tidy-up

        # Store current state
        t_arr.append(t)
        x_arr.append(x)

        # Integrate forward (Euler integration)
        v = v + a * dt
        if v < 0:
            v = 0.0
        x = x + v * dt
        t += dt

    total_time = t_arr[-1]
    total_distance = x_arr[-1]

    return total_time, total_distance

wpnav_speed_ms = 27
wpnav_accel_mss = 1.35
psc_jerk_xy_msss = 5
rate_r_max_deg = 45
rate_p_max_deg = 30
accel_r_max_cdegss = 110_000
accel_p_max_cdegss = 110_000
rate_ff_enabled = True
rtl_accel_mss = 2.5

if (rtl_accel_mss >= 0):        # Check for valid rtl_accel
    wpnav_accel_mss = rtl_accel_mss
    if (wpnav_accel_mss > 10):  # Cap if at 10 if using it
        wpnav_accel_mss = 10

# Get max jerk rate (AC_PosControl::set_max_speed_accel_xy)

accel_max_xy_cmss = wpnav_accel_mss * 100

# ensure horizontal jerk is less than what the vehicle is capable of
jerk_max_cmsss = min(deg_to_rad(rate_r_max_deg), deg_to_rad(rate_p_max_deg)) * GRAVITY * 100
snap_max_cmssss = min(deg_to_rad(accel_r_max_cdegss * 0.01), deg_to_rad(accel_p_max_cdegss * 0.01)) * GRAVITY * 100

# get specified jerk limit
jerk_max_xy_cmsss = psc_jerk_xy_msss * 100

# limit maximum jerk based on maximum angular rate
if (jerk_max_cmsss > 0 and rate_ff_enabled):
    jerk_max_xy_cmsss = min(jerk_max_xy_cmsss, jerk_max_cmsss)
    
# limit maximum possible average jerk based on angular acceleration
if (snap_max_cmssss > 0 and rate_ff_enabled):
    jerk_max_xy_cmsss = min(0.5 * math.sqrt(accel_max_xy_cmss * snap_max_cmssss), jerk_max_xy_cmsss)

total_time, total_distance = compute_braking_profile(
    starting_vel=wpnav_speed_ms,
    max_accel=wpnav_accel_mss,
    max_jerk=jerk_max_xy_cmsss / 100
)

print(f'Max jerk rate: {jerk_max_xy_cmsss / 100:.2f} m/s/s/s')
print(f'Braking time: {total_time:.2f} s')
print(f'Braking distance: {total_distance:.2f} m')