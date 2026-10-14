# Launch-Phase Control Effectiveness Identification

Design specification for `AP_INDI`. Status: draft, 2026-10-13; revised the same day from the SITL validation runs (sections 4, 6, 7 and the validation status).

## Purpose and scope

The controller learns each axis's control effectiveness from its first actuator strokes after launch, at the angular-acceleration level, and needs no per-aircraft gains. Target: a usable effectiveness estimate on every axis within 300 ms of launch, without inducing any attitude oscillation.

In scope:

- Rate loop: Incremental Nonlinear Dynamic Inversion (INDI), with the control effectiveness matrix G as its only vehicle-dependent quantity.
- Online identification of G and actuator lag by recursive least squares on gyro-derived angular acceleration.
- Launch-phase gain schedule: over-drive while G is uncertain, back off as the RLS covariance collapses.
- Attitude loop with fixed, vehicle-independent gains on top of the inverted rate loop.
- Fixed-wing, multirotor and VTOL; single precision throughout.

Out of scope here: the attitude and position observer (specified separately), structural resonance identification, and position or airspeed control. The observer is assumed to deliver attitude and bias-free rates before launch.

## Prior art and attribution

This design is an attempt to improve on earlier, unmerged INDI work for ArduPilot, by adding online identification of the control effectiveness, more modelling of the actuators and the sensor path, a fixed-wing backend and a test suite. It is not an independent invention: incremental nonlinear dynamic inversion, and INDI with online estimation of the effectiveness, are published methods, and ArduPilot already has an INDI implementation proposed and flown by Emre Saldiran.

Earlier ArduPilot work this builds on:

- Emre Saldiran (esaldiran), "INDI: Incremental Nonlinear Dynamic Inversion", ArduPilot/ardupilot pull request 20511, opened 2022, not merged. INDI for the Copter attitude, rate and position loops on a quad X, with measured motor speed (ESC telemetry) as the actuator state and the effectiveness from a physical model (inertia, arm length, thrust and torque coefficients). Flown by the author on a QAV250 in 2021, with logs shared in the pull request. Its SITL motor-speed support was split out as pull request 20543. It was developed for the author's master's thesis (https://drive.google.com/file/d/1P3TI2yp_r4fwnkqGnM_E844j_b1wMIpn/view), with a shorter version published as a Springer book chapter (https://link.springer.com/chapter/10.1007/978-3-031-39767-7_8).
- Emre Saldiran, "Custom Controller Backend Example: INDI and ADRC", ArduPilot/ardupilot pull request 21699, opened 2022, rebased by Michelle Rossouw in 2025, not merged. The same INDI as an `AC_CustomControl` backend, type 3 with `CC3_` parameters. The Copter backend here takes the same type and prefix because it is meant as a successor to that backend, not a second one beside it. Its optional motor-speed feedback (`CC3_ACT_SRC` 1, `CC3_THR2RTRSPD`) is ported from that pull request's `get_motor_speed()` and `calculate_torque_thrust_est()`; the changes made in porting are listed in `AC_CustomControl_INDI.cpp`.
- The `AC_CustomControl` framework both backends plug into is also Emre Saldiran's work (pull requests 21123 and 21216).

Literature:

- The INDI paper at arXiv:1701.07254v2, https://arxiv.org/html/1701.07254v2.
- Papers cited by pull request 20511, as given there: Tal, Ezra, and Sertac Karaman, "Accurate tracking of aggressive quadmotor trajectories using incremental nonlinear dynamic inversion and differential flatness", 2018 IEEE Conference on Decision and Control (CDC), IEEE, 2018; and Smeur, Ewoud J. J., Guido C. H. E. de Croon, and Qiping Chu, "Cascaded incremental nonlinear dynamic inversion for MAV disturbance rejection", Control Engineering Practice 73 (2018): 79–90.

What this design adds to the earlier ArduPilot work: online identification of G (weighted recursive least squares with class priors, excitation pulses, a sign test and convergence states), automatic per-axis fallback to the stock controllers, a modelled actuator delay and lag with filtering matched to the gyro and a runtime alignment check, a Savitzky–Golay derivative, a fixed-wing backend, SITL effectiveness test aids, unit tests and SITL autotests (the autotests requested in the review of pull request 21699), and an offline log replay tool. What the earlier work has that this does not: INDI on the position and linear-acceleration loops, a physical parameterisation, and flight results on real hardware.

## Definitions and signals

All quantities are in the body frame, SI units, f32.

| Symbol | Meaning | Units | Source |
| --- | --- | --- | --- |
| ω | Angular rate, 3-vector | rad/s | Gyro, bias removed by the observer |
| ω̇ | Angular acceleration, 3-vector | rad/s² | FIR differentiator on ω (section 7) |
| u | Actuator command vector, N actuators | normalised, −1…1 (surfaces) or 0…1 (motors) | Controller output, after mixer |
| u_f | u passed through the actuator lag model | same as u | Section 7 |
| Δu, Δω̇ | Change in u_f and ω̇ over one identification window | as above | Section 4 |
| G | Control effectiveness matrix, 3×N: ω̇ produced per unit u | rad/s² per unit | Identified (section 4) |
| P_G | RLS covariance of each row of G, N×N per axis | — | Section 4 |
| q̄ | Dynamic pressure, ½ρV² | Pa | Airspeed or GPS-derived; fixed-wing only |
| τ_act | Actuator time constant, per actuator | s | Identified (section 7) |
| T_s | Control loop period | s | 1/1000 s target, 1/400 s minimum |

Timing: ω̇ and u_f are time-aligned by delaying u through the lag model, so a sample pair (Δu_f, Δω̇) at time k refers to the same physical instant. The identification window is one actuator stroke: a run of samples during which u_f on the axis moves monotonically by at least Δu_min (section 4).

## Architecture

The identification sits in parallel with the rate loop and consumes the same two signals the loop already uses: the lag-compensated command u_f and the angular acceleration ω̇. There is no separate test mode.

Control path: observer (attitude, bias-free ω) → attitude loop (fixed gains, produces ω̇ desired) → INDI rate loop (Δu = G⁻¹(ω̇_des − ω̇)) → mixer and actuators (u, then the lag model gives u_f).

The rate loop forms ω̇_des = K (ω_des − ω). With the actuator lag compensated the closed rate loop is second order, ω̈ + (1/τ_act − L) ω̇ + (K/τ_act) ω = (K/τ_act) ω_des, with L the airframe's own rate damping (zero on a multirotor, −3 to −12 1/s on the SITL plane). K is capped at 2 / τ_act, which keeps the damping above 0.35 with no aerodynamic damping and above 0.5 with a plane's pitch damping; 14 rad/s on the SITL plane's 0.14 s servo. Because the increment is taken on the filtered actuator state, a constant innovation moves the actuator at (ω̇_des − ω̇) / (G τ_act): a slow servo makes a low K sluggish for large demands (at K = 5 the SITL plane took two seconds to reverse a 60 degree bank), and the limit cycle seen at 15 rad/s was the 50 Hz loop's 60 ms differentiator delay, not the gain. On a fixed-wing the body pitch and yaw rate targets carry the coordinated-turn offsets the stock controllers use, g tan(φ) sin(φ) cos(θ) / V on pitch and g sin(φ) / V on yaw; without them the yaw loop drives the rudder to its stop against every turn.

The inversion never divides by less than 0.8 of the prior on an axis. The loop is stable only while the estimate is at least half the truth (section 8, residual risks), and an estimate biased low by disturbed windows would raise the loop gain past that; a weaker airframe is flown a little sluggishly instead, which is the safe side, and the identified value is still logged and judged.

Measurement path: airframe and gyro (1 kHz ω samples) → notch and FIR differentiator (ω̇, aligned to u_f) → RLS on G (G, P_G) → back into the INDI inversion and the gain schedule.

Rates:

| Block | Rate | Notes |
| --- | --- | --- |
| Gyro sampling, notch, FIR differentiator | IMU rate, 1–8 kHz | ω̇ is the quality-critical signal |
| INDI rate loop | 1 kHz target, 400 Hz minimum | Same rate as actuator output |
| RLS update | Every control step while a stroke is active | Skipped otherwise, see section 4 |
| Attitude loop | 400 Hz | Unchanged from current ArduPilot structure |
| Observer | IMU rate | Specified separately |

## Identification of G

G is identified per axis by recursive least squares on increments, so inertia, damping and the trim moment never enter the regression. Each control step during an active stroke gives one equation per axis.

### Model

For axis i (roll, pitch, yaw) and the N actuators:

    Δω̇_i(k) = g_iᵀ Δu_f(k) + e_i(k)

where g_i is row i of G (length N), Δ denotes the change over the last n_w samples (n_w = 10 at 1 kHz), and e_i is disturbance plus differentiator noise. Using increments cancels every term that is constant or slowly varying over 10 ms: gravity moments, aerodynamic damping, trim, and gyro bias.

For fixed-wing surfaces, the regressor is scaled by dynamic pressure so that the identified quantity is airspeed-independent:

    Δω̇_i = q̄ g̃_iᵀ Δu_f,    G = q̄ G̃

Multirotor motor effectiveness scales with the square of the throttle setting in the simplest model; the regressor uses Δ(u²) for motors and Δu for surfaces.

### Recursion

Weighted RLS with forgetting factor λ and measurement noise variance r, per axis, state (g_i, P_i):

    k   = P_i Δu_f / (λ r + Δu_fᵀ P_i Δu_f)
    g_i ← g_i + k (Δω̇_i − g_iᵀ Δu_f)
    P_i ← (P_i − k Δu_fᵀ P_i) / λ

The noise variance r is estimated online per axis from the product of the prior and posterior residuals, which is unbiased for r regardless of the parameter uncertainty. Its growth is limited to 4× per sample so a wrong prior is absorbed by the parameters rather than written off as noise. Without r the covariance is only known up to the noise scale and both the gain and the reported σ are wrong. The initial value is deliberately pessimistic, (0.1 g_ref)²: with it one disturbed window (the ground-release torque at a multirotor liftoff, say) cannot swing the estimate across its band, and the estimator corrects it downward within the first few consistent samples.

Internally the state is kept as ratios to the prior so every coefficient is of order one in f32 (section 9). N is 2–4 for most airframes (ailerons, elevator, rudder, or 4 motors), so P_i is at most 8×8 and the update is a few hundred flops.

| Parameter | Launch phase | Cruise | Rationale |
| --- | --- | --- | --- |
| λ | 0.995 | 0.9995 | 200-sample memory at launch for fast convergence; 2 000 samples in cruise for noise rejection |
| P_i initial | diag(prior variance), section 5 | carried over | Encodes how much the prior is trusted |
| r initial | (0.1 g_ref)² | carried over | Pessimistic; corrected within the first 50 accepted samples |
| n_w | 10 samples | 10 samples | Shorter than any actuator time constant, longer than one differentiator tap |

### Sample acceptance

A sample pair is used only when all of the following hold. Otherwise the RLS step is skipped (no forgetting applied either).

1. Stroke amplitude: ‖Δu_f‖ ≥ Δu_min = 0.05 normalised (about 2° of surface, or 5 % of motor range). Below this the signal-to-noise ratio is too low to be worth the update.
2. Excitation fraction: the predicted Δω̇ from the command, g_iᵀΔu_f, explains at least 50 % of ‖Δω̇‖ once the fit is trustworthy. Before that, a probe window is accepted only if its response is between 0.2× and 5× of what the prior predicts, in either sign: a response outside the band the prior allows is a disturbance, not evidence about the airframe (an airframe truly outside its band is then never identified and flies on the fallback, as section 8 intends).
3. Not saturated: no actuator in the stroke is at its limit during the window.
4. No reversal within the window: u_f on the axis moves monotonically.
5. Gyro healthy: notch converged (section 7), no clipping.

### Convergence targets

| Milestone | Definition | Target time after launch |
| --- | --- | --- |
| Sign confirmed | Primary ratio of G has the prior's sign with ≥ 3σ margin | 30 ms per axis |
| Usable | Relative standard deviation σ/g ≤ 0.25 on the primary actuator | 100 ms per axis |
| Converged | Relative standard deviation ≤ 0.10 | 300 ms, all axes |
| Tracking | Cruise λ, continuous updates | Rest of flight |

Sign reversal handling: if the sign test fails on an axis, the controller flips that axis's command sign and restarts that axis's RLS from the mirrored prior. This replaces the current manual reversal checks and must be logged as an event.

The 3σ test alone is too slow to catch a reversed actuator: a wrong prior inflates the noise estimate by design, which keeps σ large for exactly as long as the controller is driving the axis the wrong way. A second, faster test runs alongside it: six consecutive probe samples whose raw response opposes the prior's sign by more than a fixed floor (0.02 g_ref) and more than a quarter of the predicted response fail the axis at once. On a reversed axis this fires within about 50 ms of the first stroke, while the rate error is still a few rad/s. The test reads a window only while the other actuators' strokes sum to at most half of the primary's own, so the verdict never rests on the coupling estimates: an earlier form subtracted the other actuators' predicted share first, and both false reversals seen in SITL (a quad's yaw, a plane's pitch, each on an axis already usable or converged) were read through a coupling estimate of several times the axis's own effectiveness. With couplings bounded to half of the reference (Cross-coupling below) the other strokes then account for at most a quarter of the predicted response, which is the test's own margin.

Only deliberate excitation (section 6, step 4) may serve as a probe for that fast test. A loop countering a disturbance moves the actuator against the measured acceleration, so its own strokes oppose the response by construction; on a multirotor liftoff this produced false reversals on one launch in six until the probe rule was added. A window counts as a probe only if the pulse can account for its stroke, at most twice the pulse's own largest increment: a larger stroke is the loop's own manoeuvre with the pulse buried in it, and its response is to whatever the loop was answering (the SITL plane's takeoff rotation, with the loop pulling back against a rising nose, read as a reversed elevator). The Plane backend also holds identification and pulses for the first second after the vehicle leaves the ground, which is the takeoff rotation on the SITL plane. The same closed-loop correlation biases the recursion low, so until an axis is converged only windows that contain a pulse are fed to it; from then on the explained-fraction and gust gates below carry the protection. Until converged an axis is also fed only windows in which its own actuator made a stroke above the gate: the recursion accepts on the norm over all actuators, and a window carried by another axis's stroke tells this axis nothing but a response of nothing, which on the SITL quad started the launch clock on the ground and walked estimates to the band edge before the vehicle could rotate.

The band verdict comes from an unclamped shadow of the primary ratio, updated with the same gain as the clamped estimate but against its own prediction (fed the clamped estimate's innovation it would keep every clamp correction as a permanent offset): the axis fails once the shadow is 35 % beyond an edge (the estimate noise at the edge is about 20 %) and beyond it by three of its own standard deviations, and not before ten accepted samples, since until then a single disturbed window can have put it there. The clamped estimate itself is not a usable verdict either way: an airframe just inside an edge (0.3× of the prior against a 0.25× edge) has an estimate that rests on the edge half the time and must not be failed for it, while one well outside drives the clamped value to the edge and then stops moving it, because the noise estimate absorbs the model error and the updates shrink.

### Cross-coupling

Off-diagonal terms of G (yaw from ailerons on a plane, roll/pitch coupling from asymmetric motors) are identified in the same recursion since Δu_f carries every actuator. They converge more slowly because they are small; the gain schedule uses only the primary actuator of each axis.

A term learned from a zero prior is bounded. The regressors move together in a coordinated manoeuvre (the stock yaw damper moves the rudder with the yaw rate, which follows the ailerons and the elevator through a turn) and least squares splits the response among them arbitrarily: in SITL a plane's pitch row credited the rudder with 88 rad/s² per unit against an elevator of 6, and a quad's yaw row credited roll with 48 against a yaw of 6, and both rows then failed their sign test through those couplings. The controller bounds a cross-axis command at half of the axis's own prior effectiveness, since no airframe is cross-coupled by more than that; the identifier's own default is the reference itself, because with motors as actuators every motor yaws as much as the reference one.

## Priors and bounds per vehicle class

The prior sets the first stroke's amplitude and the range the RLS may not leave. It is chosen by vehicle class, not by airframe, so the only configuration a user supplies is the class and the actuator layout already needed by the mixer.

The values below are the ranges the design must handle. They are working figures from typical airframes and must be replaced by measured values from the log survey in section 10 before release.

| Class | Diagonal g_ii prior, rad/s² per unit | Allowed band around prior | Prior relative σ (sets P_i initial) |
| --- | --- | --- | --- |
| Fixed-wing, SITL plane model (2 kg foam, 1.88 m span) | 18 roll, 9 pitch, 7 yaw at 13 m/s from the model coefficients, scaled by q̄ | 0.2× to 5× | 1.0 |
| Fixed-wing, 2–20 kg | 60 at q̄ = 200 Pa, scaled by q̄ | 0.2× to 5× | 1.0 |
| Multirotor, < 1 kg | 400 | 0.25× to 4× | 0.7 |
| Multirotor, 1–15 kg | 120 | 0.25× to 4× | 0.7 |
| VTOL hover | multirotor row | as multirotor | 0.7 |
| VTOL forward flight | fixed-wing row | as fixed-wing | 1.0 |

Rules:

- Sign of the diagonal is fixed by the actuator layout. The RLS may change its magnitude only; a fitted sign opposite to the prior triggers the reversal handling in section 4, not a silent sign flip.
- Off-diagonal prior is zero with relative σ of 0.3 of the diagonal prior, so coupling can be learned but starts as no coupling.
- Clamping: a G estimate that leaves its band is clamped to the band edge and the event is logged. The axis is marked failed (section 8) when the unclamped shadow estimate is 35 % beyond an edge after ten accepted samples and while the recursion is still in its launch phase (section 4); once every axis has converged the recursion switches to the cruise forgetting factor and gives no more band verdicts: the clamp is logged and the inversion floor carries the loop, since a shadow that drifts to an edge on the biased natural windows of cruise is a stale fit and not a different airframe, and a degraded actuator is the divergence trigger's case.
- Persistence: the last converged G of a flight is stored and becomes the prior for the next flight on the same airframe, with relative σ of 0.3. First-flight priors only apply once per airframe.
- Fixed-wing q̄ scaling uses airspeed when a sensor is present, otherwise ground speed from GPS with a 1.5× floor on the prior σ to absorb wind error.

### Prior inputs

User inputs are optional and layered: each one entered tightens the prior, none is required, and the RLS converges to the same G regardless. They only change how many strokes it takes.

| Inputs entered | How the prior is formed | Prior relative σ |
| --- | --- | --- |
| None | Class row above | 0.7 (multirotor), 1.0 (fixed-wing) |
| Prop diameter (multirotor) | Selects the class row: 5″ → under 1 kg; 7–10″ → 1–15 kg; 12″ and above → 1–15 kg, upper band. Frame size tracks prop size closely enough for this. | 0.7 |
| Prop diameter + take-off mass + motor-to-motor diagonal (multirotor) | Physics prior: thrust per motor from prop diameter (T ∝ ρ n² D⁴ with a class-typical max rpm), torque = thrust × diagonal/2, inertia ≈ 0.5 × mass × (diagonal/2)² for a typical mass distribution. g_ii = torque / inertia per unit command. | 0.4 |
| Wingspan + take-off mass (fixed-wing) | Selects the class row and scales the q̄ reference: light, high-aspect-ratio airframes sit at the top of the band. | 0.8 |
| Stored G from a previous flight (any class) | Overrides all of the above | 0.3 |

What is deliberately not an input:

- Inertia. It is not measurable from a hand-held tilt or roll of the airframe: the IMU sees angular acceleration but not the torque the hand applies, so the ratio is unobservable. Pendulum methods work but take minutes and a rig, which is the per-aircraft procedure this design removes. INDI never needs inertia separately from G.
- Motor KV, battery cell count, ESC type. They refine the thrust estimate but add questions the user may answer wrongly; the RLS recovers the same information in one stroke.

A hand-held tilt before arming remains useful as an observer check (tilt lock, gyro bias, IMU orientation and axis signs) and may be offered as an optional pre-arm step, but it contributes nothing to the prior on G.

## Launch sequence and gain scheduling

The loop over-drives while G is uncertain and relaxes continuously as P_G shrinks. There is no discrete tune-complete event; the schedule is a function of the current covariance.

### Sequence

1. Pre-arm, stationary: observer at high gain; gyro bias and tilt locked. Actuator lag τ_act measured by one surface step per surface (section 7). Multirotors skip the step test.
2. Armed, pre-launch: G = prior, P_i = prior covariance. Fixed-wing: no identification possible on the ground; controller runs on prior G with the launch over-drive factor. While the vehicle is known to be held by the ground (a wheeled fixed-wing on its runway roll, below 2 m above home) the incremental law has no actuator state to build on and would integrate the command onto a stop, so the command is the inversion of the demand alone, a proportional law like the stock controllers' on the runway; the SITL plane otherwise left the ground with full up elevator and rotated at 80 deg/s. The Plane backend treats the vehicle as held only until it first reaches 2 m above home: the landing flare passes back through the same height, and treating the vehicle as held there dropped the incremental law in the last second before touchdown (SITL: the elevator eased off while the nose fell 20° below the flare attitude). Pulses and the divergence watch do follow the height, since in the flare a stall pins the elevator against a nose that keeps falling, which the watch cannot tell from an inversion asking for more than the actuators have (SITL: fallback on all axes 1.5 s before touchdown). Multirotor: once the landing detector reports the vehicle may be airborne, excitation pulses (step 4) run straight away; a pulse the ground absorbs costs nothing, and a stroke that produces no response is not fed to the RLS (a vehicle that cannot rotate is constrained, not weak).
3. Launch: the first stroke that produces a measurable response. That instant, not the landing detector or a thrust threshold, starts the launch clock and the identification: on a multirotor the detector clears at takeoff intent and the thrust estimate is uncertain by more than the time the first strokes take, so both put the launch while the vehicle is still on the ground. Fixed-wing may use the existing accelerometer-threshold launch logic, which has the same meaning.
4. Excitation: if the attitude loop's own demand on an axis has not produced a stroke of ≥ Δu_min within the last 100 ms, inject a single square pulse of Δu_min on that axis, 30 ms wide, alternating sign on successive pulses so the net attitude change is zero. Axes that still need excitation are pulsed with no gap between pulses: the 3×N regression attributes each stroke to its axis, and a gap only lengthens the time to the identification targets (a 40 ms stagger put each axis's pulses 210 ms apart and the usable milestone past 700 ms on a multirotor). Pulsing all axes together with orthogonal sign codes (rows of a 4×4 Walsh matrix) was tried to triple the information rate: it fails. Within the first slots the three regressors move together, the recursion cannot attribute the response, and every axis is driven to its band edge within 35 ms on every launch. Concurrent excitation would need the recursion to withhold judgement until a full code block has been seen, which the sequential recursion does not do. The pulse is injected as the angular acceleration a Δu_min stroke of the prior would give, added to the rate loop's demand: the loop then produces and holds the stroke for the pulse width. Added to the actuator command instead, the loop reads the response as an error and cancels it within one actuator delay, leaving almost nothing to identify from.
5. Converging: each accepted sample updates G and P_i; the loop gain follows the schedule below.
6. Tracking: once all axes are converged, switch λ to the cruise value and keep updating from normal flight commands. Pulses are never injected again unless an axis is marked stale (section 8).

### Gain schedule

The INDI inversion uses the current G estimate once an axis's sign is confirmed, and the prior before that, so the first two or three samples cannot swing the inversion gain while they are still being judged. Coupling terms enter the inversion once the row is usable and the term itself stands clear of zero by the sign-test margin of three standard deviations; an uncertain coupling is a fit to whatever the other loops were doing and is left at zero. The over-drive acts on the attitude-loop bandwidth, not on the inversion, so a wrong G cannot be amplified through the schedule.

Let s_i = σ(g_ii)/g_ii be the relative standard deviation of the primary ratio on axis i, from P_i. The attitude-loop bandwidth on that axis is

    ω_att,i = ω_nom · clip(s_i / s_conv, 1, k_od)

with ω_nom = 6 rad/s (the vehicle-independent nominal, same for all classes), s_conv = 0.10 and k_od = 2.5. So an axis whose effectiveness is uncertain (s_i large) gets up to 2.5× the nominal demand, which produces larger strokes and faster identification; as s_i falls to 0.10 the bandwidth settles at nominal.

Two guards keep the over-drive from doing harm:

- The rate loop's own demand is limited to ω̇_max = 0.5 × (prior g_ii) × u_range, so even a prior that is 2× too optimistic cannot saturate an actuator. u_range is the command range the backend allows, which on a fixed-wing scales with dynamic pressure; a limit set at the reference airspeed would let the loop use an eighth of its authority at twice that speed.
- The identifier's gate and the pulse width follow the actuator: the gate is half of what a Δu_min stroke leaves in one window after the lag (all of it for a lag shorter than the window, 1 − exp(−window/τ_act) otherwise), and a pulse is held for at least two time constants so the actuator reaches 86 % of the stroke. A 30 ms pulse into a 0.14 s servo would otherwise leave a fifth of its height and no window above a fixed gate.
- The over-drive applies only while the identification is accepting samples. If acceptance stops for more than 200 ms (sustained saturation, gyro unhealthy), the factor decays to 1 with a 100 ms time constant.

### Expected timeline, fixed-wing hand launch

| Time after launch | State |
| --- | --- |
| 0 ms | Prior G, bandwidth 2.5× nominal, pulses armed |
| 0–100 ms | First strokes from the launch transient; sign confirmed on roll and pitch |
| 100–200 ms | Yaw pulse if the rudder has not moved; roll and pitch usable |
| 200–300 ms | All axes usable, bandwidth within 1.3× nominal |
| 300–600 ms | All axes converged; λ to cruise value |

## Actuator lag and filtering

INDI and the RLS both depend on ω̇ and u_f referring to the same instant. A mismatch of a few milliseconds biases G low and makes the inversion oscillate, so the lag is measured, not assumed.

### Actuator model

Each actuator is modelled as a first-order lag with time constant τ_act plus a pure delay d_act (servo bus and ESC latency):

    u_f(k) = u_f(k−1) + (T_s / τ_act) (u(k − d_act/T_s) − u_f(k−1))

| Actuator type | τ_act typical | d_act typical | Measured how |
| --- | --- | --- | --- |
| Analog servo | 40–80 ms | 5–20 ms | Ground step test, see below |
| Digital servo | 15–40 ms | 2–5 ms | Ground step test |
| Brushless motor, ESC | 5–20 ms | 1–3 ms | In-flight RLS with lag as an extra parameter |

### Measured actuator state

A backend may offer a measured actuator state in place of the model, in the same command units. The Copter backend derives one from motor speed (`CC3_ACT_SRC` 1), using the RPM feedback of pull request 21699 on a quad X: each motor's speed squared, as a fraction of `CC3_THR2RTRSPD` squared, is its thrust fraction, and the mixer's ±0.5 roll, pitch and yaw factors turn the four fractions into the three commands. The measurement then goes through the same gyro-matched filter and alignment as the model would.

The model keeps running alongside it. A measurement is flown on only after it has agreed with the model for half a second while carrying a signal, and is ignored for the rest of the flight once their mean disagreement, averaged over half a second, exceeds 90 % of their mean amplitude. That check catches gross faults only: in the unit tests a reversed measurement disagreed by 140 to 200 %, one stuck at zero by 100 %, but a correct measurement of an actuator four times slower than modelled by 64 %, more than one scaled 2× (50 %). Flown at once, a reversed measurement turned the feedback positive and diverged before the averaged check could reject it, hence the qualification. A scale error in the measurement is not detectable this way and is a configuration matter; because the identification runs on the measured increments, the identified effectiveness absorbs a constant scale, which then acts as a loop gain error rather than an inconsistency.

The measurement is not better in every case. In the controller unit tests, with the model fixed at 10 ms, the rate tracked better measured than modelled when the actuator was faster (5 ms: 0.33 against 0.50 of the step) or much slower (40 ms: 1.1 against 3.5), equally when the model was right, and worse when the actuator was two to three times slower than modelled (0.53 against 0.15 at 20 ms): a model that runs ahead of the actuator makes the law take smaller increments, which adds damping. The measurement needs RPM at the loop rate, as bidirectional DShot gives it; serial ESC telemetry at a few hertz is too slow.

### Ground step test, fixed-wing

At pre-arm, with the airframe held or on the ground, each surface is stepped by 20 % of its range and back. The gyro sees the reaction torque of the surface moving (small but measurable on light airframes) and, on larger airframes, nothing. Where the gyro response is below noise, τ_act falls back to the class default and the in-flight estimate refines it. The test takes under 0.5 s per surface and runs only once per boot.

### In-flight refinement

The RLS of section 4 is augmented with one extra regressor per axis: Δu_f computed with τ_act scaled by 1.3. The ratio of the two fitted coefficients gives the correction to τ_act. This is a cheap substitute for a full nonlinear lag fit and is only applied when both coefficients are converged.

### Gyro path

1. Notch filtering at the motor harmonic, using the existing ArduPilot harmonic notch (throttle or RPM driven). The notch delay is known from its coefficients and is added to d_act so u_f is delayed by the same amount.
2. Angular acceleration by a 7-tap Savitzky–Golay first-derivative FIR at the IMU rate. Group delay is 3 samples, also added to the alignment delay. A first difference is not acceptable: its noise gain is too high for the RLS at launch. The command is not smoothed the same way, and that is a known bias: the derivative's window flattens the response to a stroke shorter than the window while the command increment keeps its full size. Against a simulated plant with a known G the fit reads 20 % low at 400 Hz; passing the command through the matching smoother (the derivative kernel applied to the trapezoidal integral of the command, a 7-tap FIR (1.5, 4, 5.5, 6, 5.5, 4, 1.5)/28 with the same group delay) makes it exact (0.1 %). It is not in the flight code because, with the gates of sections 4 and 5 as tuned, it halved the signal each 10 ms window carries from a 30 ms pulse and raised the multirotor band fallbacks from 3 % to 8 % of launches; lengthening the window to the smoother's span pinned every launch on the band edge. At a 50 Hz fixed-wing loop the derivative window is 140 ms and the bias is a factor of about three, so the fixed-wing path needs either this smoother with re-tuned gates or an increment formulation that does not go through the differentiator (double differences of the rate against the integral of the matched command).
3. No additional low-pass on ω̇. The RLS forgetting factor provides the averaging; a low-pass here would add delay the inversion then has to fight.
4. The gyro that reaches the controller already carries the vehicle's gyro low-pass (INS_GYRO_FILTER, 20 Hz by default). A 10 ms increment of a signal filtered that hard is a fraction of the true increment, which biases G low by a factor of two or more. The command is therefore passed through the same low-pass before alignment and identification (matched filtering, standard INDI practice), so Δu_f and Δω̇ are attenuated and delayed alike and the ratio is unbiased.

### Alignment check

The cross-correlation of Δu_f and Δω̇ at lags of −4 to +4 samples around the current alignment is accumulated over the strokes seen and evaluated at 10 Hz once 40 strokes have accumulated. A peak off zero that exceeds the zero-lag value by 50 % moves the alignment one sample toward it, in either direction (the model may be slower than the vehicle as well as faster), within −3 to +6 samples. One sample at a time, because a shift changes the pairing the next correlation is built on and a larger step chases itself (an early version walked to +12 samples on a tenth of the launches). This is the only runtime adjustment to the delay and is logged. An alignment that is late biases G low: the modelled command increment arrives after the measured response, so each window pairs a large Δu_f with a small Δω̇.

## Safety, fallbacks and failure modes

The fallback is designed first: at every moment the controller must be able to drop to a fixed conservative configuration that flies the vehicle, and the adaptive path must never be the only thing holding it up.

### Fallback controller

A fixed-gain rate PID per class, tuned conservatively (about half the bandwidth a hand tune would reach), runs in parallel and shadows the INDI output. Switching to it is bumpless: its integrator is initialised from the current INDI increment. It is engaged on any of the triggers below and stays engaged until the operator or a mode change re-enables identification.

### Triggers and responses

| Condition | Detection | Response |
| --- | --- | --- |
| Wrong-sign fit on an axis | Section 4 sign test fails after the first 3 accepted strokes | Flip axis sign once, restart RLS from mirrored prior; a second failure engages fallback on that axis |
| G leaves its band | Two clamps within 1 s on one axis | Mark axis failed, fallback on that axis, stop injecting pulses |
| Identification stalls | No accepted sample for 2 s while airborne and manoeuvring | Mark axis stale; one pulse injected at nominal bandwidth; stale twice in 10 s engages fallback only while the axis is not yet converged. A converged estimate that gets no new samples is frozen, not wrong: in cruise the gust gate rejects most windows because its noise sigma was set at launch, so a stall there is the normal state (on the SITL quad it handed all three axes back 46 s into an otherwise clean flight). An actuator that degrades after convergence is caught by the divergence row |
| Divergent inversion | Rate-loop innovation ω̇_des − ω̇ grows for 100 ms (about two time constants of a slow servo; five 50 Hz steps, forty at 400 Hz) with saturated Δu | Fallback on all axes immediately. As five loop steps it was 12.5 ms at 400 Hz and fired on a landing flare |
| Gyro unhealthy | Clipping, notch not converged, IMU rate drop | Freeze RLS, hold last G, no pulses; fallback if longer than 500 ms |
| Gust during identification | Innovation energy above 3× the running RLS residual | Reject the window; no update |
| Observer not ready | Attitude or bias not locked at launch | Rate loop runs (needs only gyro); attitude loop at half nominal bandwidth until the observer reports ready |

### Rules that hold regardless of state

- Pulses are never injected while the vehicle is on the ground and known to be so (motors spooled down, or throttle pinned at its minimum after touchdown). A multirotor may pulse from the moment the landing detector reports it airborne; a minimum height for pulses is a parameter (default 0), because a vertical takeoff produces no strokes of its own and waiting for 3 m puts the first identification a second or more after liftoff. Fixed-wing: none within the first 50 ms of launch (the launch transient provides the excitation).
- Pulse amplitude is fixed at Δu_min of actuator travel; it never scales up to force convergence. Where the command is scaled by dynamic pressure (fixed-wing) the stroke and the gates scale with it, so the pulse stays Δu_min of surface travel: on the SITL plane at 24 m/s the unscaled pulse was 1.5 % of travel and its response was lost under the takeoff rotation.
- The over-drive factor can only raise the attitude-loop bandwidth, never the rate-loop authority limit.
- A failed axis is reported to the GCS once, with the fitted value and the band, so a human can decide whether to land.
- All RLS state, triggers and clamps are logged at the control rate for the first 2 s after launch and at 10 Hz afterwards. Post-flight review of a failed identification must be possible from the log alone.

### Known residual risks

- A launch into strong turbulence can delay convergence past 300 ms. The system remains safe on prior G; it is only slower.
- An airframe whose true effectiveness is outside the class band cannot be identified; it flies on the fallback. The band must be set from the log survey in section 10, not from these working figures.
- An airframe more than about twice as effective as the prior cannot be flown on the prior by the incremental inversion at all (stage 3 result, validation plan): the band's upper edge is a stability limit of the method, not just an identification limit, and must stay at or below 2× unless the inversion is made conservative while unconverged.
- The unclamped shadow of the primary ratio, which carries the band verdict, follows the clamped estimate through a recursion that multiplies their difference by 1 − k_p φ_p each sample, and with correlated regressors that factor can exceed one in magnitude. Off-tree, with four motors as the actuators, all with non-zero priors, and their strokes held in one pattern for 20 s, the shadow went to infinity in f32 and in f64; in the f32 run the primary then came to rest on the band clamp and the other ratios drifted until the combined effect was more than twenty times the truth after 200 s. Neither happened with the controller's configuration (three axis commands, zero-prior couplings) in the runs above. The band verdict is safety logic; this is left for review rather than changed here.
- INDI depends on the actuator-lag match. A servo that degrades in flight (stall, slow from load) shows up as a growing alignment error (section 7) and triggers the stall or divergence row above, but there is a window of up to 200 ms before that is detected.

## Numerics, f32

Everything runs in f32 with no heap allocation; the largest matrix is the 8×8 RLS covariance for a VTOL in transition. These are the conditions that keep it stable.

- P_i is stored in factorised form, P = U D Uᵀ with U unit upper triangular and D diagonal, never as P directly, and updated by Bierman's method (G. J. Bierman, *Factorization Methods for Discrete Sequential Estimation*, 1977, chapter V) with measurement variance λr followed by D ← D/λ: the weighted forgetting recursion of section 4, rearranged. Each element of D is only ever scaled by a ratio of positive sums, so P stays positive definite whatever the rounding. The earlier version of `AP_INDI_Ident` kept P directly, symmetrised it and clamped its diagonal element by element; the clamp left the off-diagonal terms alone, and when the roll and pitch commands moved together P went indefinite, in f64 as much as in f32, after which the innovation variance went negative and every later sample was refused. In six off-tree runs of that version (the controller's configuration, roll and pitch strokes correlated 0.7 to 1 with no independent part, 200 s of cruise, f32 and f64) it stopped learning in every one, with the combined effect of a stroke off by up to 52 %; the factorised form kept learning in all six and held the combined effect within 0.2 %. `KeepsLearningUnderCoordinatedStrokes` is that case: the earlier version accepts 55 % of its samples and ends 11 % high. With some independent excitation (1 % or 10 % of a stroke on each command) the two versions agree to within 0.5 %, and the factorised form matches the plain recursion run in f64 (`CovarianceMatchesDoublePrecisionReference`).
- Regressors are scaled to order 1 before the update: Δu_f is already normalised; Δω̇ is divided by g_ref, the largest |prior| on the axis, and each coefficient is the ratio of effectiveness to its prior (or to g_ref where the prior is zero). G is recovered by multiplying back.
- Forgetting is applied only on accepted samples, so P_i cannot blow up during a long period with no excitation (the usual RLS wind-up failure).
- The diagonal of P_i is held between (0.02)² relative and the prior variance without leaving the factorised form or losing positive definiteness. A short element is raised by adding its shortfall to D, a positive semidefinite addition to P, worked from the last element back so each is set once. Where an element is over the cap, P becomes S P S with S diagonal (s_j = √(cap_j / P_jj) there, one elsewhere), which keeps the correlations and is D_j ← s_j² D_j, U_ij ← U_ij s_i / s_j in factorised form.
- The inversion G⁻¹ is a weighted pseudo-inverse with a 1e−3 Tikhonov term relative to the largest singular value, solved by a fixed 3×N least-squares, no iterative method.
- The FIR differentiator and lag model are fixed-point-safe (bounded coefficients, no recursion on ω̇), so the control path has no accumulating error.
- Control-rate cost budget: RLS plus inversion for N = 4 is under 1 500 flops per step, about 3 µs on an STM32H7, well inside a 1 kHz loop.

## Validation plan

Each stage must pass before the next starts. Acceptance criteria are the convergence targets in section 4 and no fallback engagement in nominal conditions.

| Stage | What is run | Pass criterion |
| --- | --- | --- |
| 0. Unit tests | `tests/test_indi_ident.cpp`: convergence from a wrong prior, coupling from a zero prior and its bound, reversed actuator, rejection gates, band clamp, long noisy run in f32, the factorised covariance against an f64 reference, coordinated strokes | All pass (current state) |
| 1. Offline replay | RLS and lag model fed from existing dataflash logs (gyro at full rate, servo outputs) across at least 30 airframes per class | Fitted G stable across the flight to within 10 %; class bands and priors set from the spread |
| 2. SITL, matched airframe | Plane and Copter SITL, launch and takeoff repeated 100 times with randomised wind | Converged within 300 ms in 95 % of runs; no fallback |
| 3. SITL, mismatched airframe | Same, with the SITL physics model's effectiveness scaled by 0.3× and 3× relative to the prior, and one axis reversed | Sign recovered within 50 ms; converged within 500 ms; no divergence |
| 4. SITL, failure injection | Servo slowed 3× in flight, gyro dropouts, gusts during the first 200 ms | Every row of the section 8 trigger table fires as specified; fallback flies the vehicle |
| 5. Hardware in the loop | STM32H7 target against SITL physics over the real IMU and servo paths | Timing budget met; f32 results match the f64 reference to 1 % |
| 6. Flight, small airframes | 5-inch quad and a sub-1 kg foam plane, hand launched, with the fallback armed and a pilot ready | Logs show convergence timeline matching section 6; no fallback in calm air |
| 7. Flight, larger airframes | Only after 6, one class at a time | Same criteria |

Stage 1 is the one that determines whether the priors in section 5 are usable; it needs no new flying and should start first.

### Status, 2026-10-13

- Stage 0: passes. `test_indi_ident.cpp` (9 tests) and `test_indi_controller.cpp` (5 tests, closed loop around a simulated rigid body, including the measured actuator state and its rejection).
- Stage 1: the replay tool (`Tools/scripts/indi_replay.py`) runs on logs that carry RATE at the loop rate, with the actuator model, the matched gyro low-pass and the stroke gates of the flight code. On a SITL quad log of three INDI takeoffs (257 roll and 231 pitch strokes accepted) the whole-flight fit is 245 roll, 234 pitch, 9.8 yaw rad/s² per unit against 257, 269 and 12 identified in flight; the offline fit has no probe information and carries the closed-loop low bias of section 5. Split into three equal shares of strokes the fits spread 28 % (roll), 18 % (pitch) and 90 % (yaw, 89 strokes), so the 10 % stability criterion is not met on this log; the 30-airframes-per-class survey needs real dataflash logs, which this environment does not hold. Not passed.
- Stage 2, Copter (`INDIMatched`, 100 takeoffs of the default SITL quad with random wind, class prior 400/400/25; last run with the coupling bound and the probe rules above): no fallback and no sign reversal on 100 launches (the two runs before the coupling bound had 3 to 4 % fallbacks, the last of them three yaw rows failed by the sign test through a roll coupling of 48). The identification is consistent, G = 247 (239 to 261) roll, 258 (244 to 284) pitch, 13 (12 to 14) yaw rad/s² per unit (median, 5th to 95th percentile), and every launch converged on all axes within 2 s. The time target is missed: all axes usable within 300 ms on 2 % of launches, converged within 1 s on 53 %, median 0.93 s, 95th percentile 1.83 s; the probe rules trade this time for the false reversals. The time is set by the information a Δu_min pulse carries at 400 Hz with three axes pulsed in turn; the 300 ms figure in this document assumed a launch transient that excites every axis, which a vertical takeoff does not provide. Either the target for multirotors becomes 1 s, or the pulse amplitude is raised above Δu_min for the first strokes.
- Flight, Copter (`INDIFlies`, in the default suite): the same GUIDED profile flown first on the stock rate controllers and then with all three axes on INDI; takeoff to 10 m, a 20 m box with yaw changes, land. INDI: no axis handed back, attitude error rms 0.20° roll and 0.26° pitch, maximum 1.8° and 1.5°, against 0.57°, 0.44°, 2.6° and 3.8° on the stock controllers. Passes on every run since the stall and band rules above.
- Flight, Copter on motor speed (`INDIFliesRPM`, in the default suite): the same box on SITL's quad-X model (the RPM feedback handles quad X only), with ESC-telemetry RPM (`SIM_VIB_MOT_MAX` 250 Hz at full command) and `CC3_ACT_SRC` 1. The measured actuator state was used on 3846 of 3888 records (the rest on the ground before takeoff and during the half-second qualification), none rejected, no axis handed back; attitude error rms 0.20° roll and 0.31° pitch, maximum 2.8° and 4.2°, against 0.55°, 0.43°, 2.7° and 3.8° on the stock controller. On the modelled state the plus-frame box gave 0.19°, 0.27°, 1.8° and 1.6°: the SITL motors respond within a loop period, so the model is already right there, and the measurement adds only the interpolation `AP_ESC_Telem` applies between RPM samples. On the plus frame the backend reports that the feedback needs quad X and stays on the model. Repeated with the factorised covariance (Numerics, f32): five runs on one day, two on the earlier covariance and three on the factorised one, gave INDI rms 0.19 to 0.28° roll and 0.29 to 0.34° pitch and maxima 2.2 to 5.3° roll and 3.4 to 5.2° pitch, against 0.55°, 0.42 to 0.43°, 2.7° and 3.7 to 3.8° stock. The two forms differ by no more than the run-to-run spread, and on motor speed the INDI maximum was above the stock controller's in four of the five runs on each axis.
- Flight, Plane (`INDIPlaneFlies`, in the default suite): a TAKEOFF-mode departure to 50 m, 60 s of LOITER, fly home and land, stock first and then with aileron and elevator on INDI (the rudder stays with the stock yaw damper, see the Plane backend) at 400 Hz, the actuator model set to the SITL servo, rate gain 15 rad/s capped at 2/τ, attitude bandwidth 3 rad/s, rate demand limited to 150°/s (above the stock controllers' 90°/s: the navigation demand swings through 100° at the climb-to-loiter transition, and at 90°/s the attitude error peaked at the swing itself, 54° locally and 92° on a CI runner). INDI: no axis handed back, attitude error rms 5.63° roll and 3.97° pitch, maximum 53.5° and 17.3° (the roll maximum is the navigation demand's own step into the loiter), against 6.96°, 4.52°, 55.2° and 16.8° stock on the same run. Two landing findings came out of this test: with the held-vehicle law applied below 2 m on the way down, the elevator eased off in the flare and the nose fell to 22° below the flare attitude (a 32° excursion against 17° stock, at 8 m/s with the airframe stalling onto the runway); with the incremental law kept, the divergence watch then read the stall, elevator on its stop against a falling nose, as divergence and handed every axis back 1.5 s before touchdown. The held rule now ends at the first launch and the watch follows the height (Launch sequence), and the flare matches the stock controllers'. Passes since the coupling bound: the run before it handed back roll 2.4 s after launch and pitch 68 s into the loiter, both on sign reversals read through couplings of 5 and 10 times the axis's own effectiveness. The identification on the plane is still not trusted: the diagonal estimates ended the flight at 10 roll and 5.8 pitch rad/s² per unit against 17.5 and 8.9 from the model coefficients, and wandered between 4 and 23 (roll) and 2 and 19 (pitch) through the manoeuvres, so the inversion flew on its floor of 0.8 of the prior for most of the flight. The stall trigger fires every few seconds in cruise with no consequence, as the stall row above intends. The fork CI run of the same code handed pitch back on the band 124 s into the flight, with the shadow estimate at the 0.25× edge after 100 s of converged cruise on the biased natural windows; the band verdict is now given only in the launch phase (Priors and bounds per vehicle class), and the recursion switches to the cruise forgetting factor once every axis has converged, which this document specified and the code had not wired.
- Stage 2, Plane (`INDIPlaneMatched`, 100 TAKEOFF-mode departures of the SITL plane with random wind, prior 18/9/7 from the model coefficients, actuator model set to the SITL servo's 0.14 s lag and one 50 Hz loop of delay, identification gated on 2 m above home; run before the coupling bound and the probe rules of this status, not re-run since): not passed. Fallback engaged on 65 launches (roll band 33, pitch sign test 12, divergence 17, yaw band 17); all axes usable on 97 launches, median 0.7 s, converged on 79, median 1.4 s. The identified values are not trusted: roll settled at 6 to 8 rad/s² per unit against 17.5 from the model coefficients on every run, pitch scattered from −20 to 34 with 26 false sign reversals, yaw 5 (2 to 13) against 7. Three things are known to be wrong, in order of weight: the 50 Hz loop puts the differentiator window at 140 ms against a 30 ms pulse (the unmatched smoothing under Actuator lag and filtering, a factor of about three on the SITL plane); the takeoff rotation is a pitch manoeuvre of several rad/s² driven by the loop itself, so the pitch probes are read in the middle of large natural strokes; and the pitch loop oscillates at about 3.5 Hz with the 0.14 s servo (rate gain 15 rad/s against an actuator at 7 rad/s), which the sign test then reads as reversals. Earlier runs also showed that the stroke and pulse must scale with the dynamic-pressure command scale (now done, see Safety, fallbacks and failure modes) and that identification on the runway reads the gear, not the airframe (hence the 2 m gate). The Plane backend flies the vehicle through the takeoff in every run, with the stock controllers taking over the axes it gives up.
- Stage 3, Plane (`INDIPlaneMismatched`): written, not run until stage 2 passes on the plane.
- Stage 3, Copter (`INDIMismatched`, prior set to the stage 2 result, physics scaled by `SIM_CE_SCALE`):
  - 0.3× (truth 77 rad/s² per unit): identified on all five launches, G = 64 to 87; converged in 0.5 to 1.0 s (target 0.5 s missed, as in stage 2); one or two launches in five fall back on the band, run to run. Fallback is not the specified outcome for an airframe inside the band; it comes from the shadow estimate crossing the edge margin in noise while the estimate sits near 0.25× to 0.3×.
  - 3× (truth 771): not identified. Roll converged on 313 to 533, pitch never became usable, two launches raised false sign reversals, three fell back. The cause is a limit of the method, not of the code: flying on a prior three times too small gives the incremental inversion a loop gain of 3, and the inner INDI loop is only stable for gains below about 2 (the estimate must be at least half the truth). The loop oscillates until the identification catches up, and the oscillation is what the identification sees. The band's upper edge of 4× is therefore not flyable on the prior. Either the band becomes 0.25× to 2× (with class priors chosen toward the top of their range), or the inversion uses 2× the prior until the sign is confirmed, which trades a sluggish first 100 ms on a matched airframe for stability on an over-effective one.
  - Reversed roll: detected and mirrored on all five launches of the last run, 435 to 620 ms after launch (target 50 ms), with all axes converged in 0.6 to 1.6 s and no fallback. An earlier run tipped the vehicle on the ground before it left it. The 50 ms target is not reachable with the fast sign test as specified: six consecutive opposing probe windows on one axis take at least six pulse slots, and with three axes pulsed in turn that is 400 ms or more. A fixed-wing launch, with airspeed before any rotation is possible, is a different case.
- Stage 4, Copter (`INDIFailureInjection`): two of three cases pass on every run. Motors slowed to 60 ms in flight raised the divergence trigger and the fallback flew the vehicle; the primary gyro failed raised the gyro trigger, the vehicle failed over to another IMU inside 500 ms and no fallback was owed. The 40 rad/s² gust 50 to 200 ms after liftoff passed on one run (rejected in two windows, all axes converged within 400 ms) and failed on another: the gust arrived in the window that made pitch usable, before the gust gate (which needs a usable fit) could act, and pushed the estimate to the band edge. The gate has no protection for the first strokes; a single-launch case is also too few to call either way.

## Implementation plan

- Prototype in C++ in-tree. waf has no Rust support; a `no_std` Rust port of the identifier follows once the algorithm is settled on real logs.
- Copter integration through the existing `AC_CustomControl` backend interface (`AC_CustomControl_INDI`, type 3), which gives in-flight switching against the stock PID as the fallback. Done; parameters `CC3_*`. Axes the identification gives up on are handed back to the stock rate PID per axis, with its integrator seeded from the INDI command for a bumpless switch.
- Plane: `AP_CustomControl_INDI` (type 3, parameters `CP3_*`) on aileron, elevator and rudder with q̄ scaling. Done; see the fixed-wing status below for what it does and does not yet achieve.
- Angular acceleration: the Savitzky–Golay differentiator runs inside `AP_INDI` on the AHRS gyro at the control loop rate. Moving it to the IMU rate (the rate thread) is a later step.
- SITL: `SIM_CE_SCALE_X/Y/Z` (per-axis torque multiplier), `SIM_CE_REV_MSK` (reversed axes), `SIM_CE_MOT_TC` (extra motor lag), `SIM_CE_GUST_X/Y/Z` with `SIM_CE_GUST_T` and `SIM_CE_GUST_DUR` (angular acceleration disturbance after liftoff). Done.
- Log messages `INDG` (G and σ per axis) and `INDS` (state, triggers, pulses, limits), build option `AP_INDI_ENABLED` (`INDI` in build_options.py, SITL only by default).
- Autotests `INDIMatched`, `INDIMismatched` and `INDIFailureInjection` in `Tools/autotest/arducopter.py` run validation stages 2 to 4; `Tools/scripts/indi_replay.py` runs stage 1 on a dataflash log.

## Open questions

- Is the 30 ms square pulse acceptable for a hand-launched plane in the first 300 ms, or must the launch transient alone provide the excitation?
- Motor regressor: Δ(u²) assumes thrust proportional to the square of the command. Should the existing ArduPilot thrust-linearisation curve be used instead so the mixer and the RLS agree?
- Stored-prior persistence: per airframe by board ID, or per parameter set? A board moved to a new airframe must not inherit the old G.
- Yaw on a flying wing has no dedicated actuator; the identification will find near-zero effectiveness and the sign test cannot pass. The axis must be excluded by the mixer layout rather than failed at launch.
- Which observer specification pairs with this document: the nonlinear-observer design discussed earlier, or an interim on the existing estimator?
