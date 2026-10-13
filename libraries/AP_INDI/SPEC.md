# Launch-Phase Control Effectiveness Identification

Design specification for `AP_INDI`. Status: draft, 2026-10-13.

## Purpose and scope

The controller learns each axis's control effectiveness from its first actuator strokes after launch, at the angular-acceleration level, and needs no per-aircraft gains. Target: a usable effectiveness estimate on every axis within 300 ms of launch, without inducing any attitude oscillation.

In scope:

- Rate loop: Incremental Nonlinear Dynamic Inversion (INDI), with the control effectiveness matrix G as its only vehicle-dependent quantity.
- Online identification of G and actuator lag by recursive least squares on gyro-derived angular acceleration.
- Launch-phase gain schedule: over-drive while G is uncertain, back off as the RLS covariance collapses.
- Attitude loop with fixed, vehicle-independent gains on top of the inverted rate loop.
- Fixed-wing, multirotor and VTOL; single precision throughout.

Out of scope here: the attitude and position observer (specified separately), structural resonance identification, and position or airspeed control. The observer is assumed to deliver attitude and bias-free rates before launch.

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

The noise variance r is estimated online per axis from the product of the prior and posterior residuals, which is unbiased for r regardless of the parameter uncertainty. Its growth is limited to 4× per sample so a wrong prior is absorbed by the parameters rather than written off as noise. Without r the covariance is only known up to the noise scale and both the gain and the reported σ are wrong.

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
2. Excitation fraction: the predicted Δω̇ from the command, g_iᵀΔu_f, explains at least 50 % of ‖Δω̇‖ once the fit is trustworthy. Before that, accept all.
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

### Cross-coupling

Off-diagonal terms of G (yaw from ailerons on a plane, roll/pitch coupling from asymmetric motors) are identified in the same recursion since Δu_f carries every actuator. They converge more slowly because they are small; the gain schedule uses only the primary actuator of each axis.

## Priors and bounds per vehicle class

The prior sets the first stroke's amplitude and the range the RLS may not leave. It is chosen by vehicle class, not by airframe, so the only configuration a user supplies is the class and the actuator layout already needed by the mixer.

The values below are the ranges the design must handle. They are working figures from typical airframes and must be replaced by measured values from the log survey in section 10 before release.

| Class | Diagonal g_ii prior, rad/s² per unit | Allowed band around prior | Prior relative σ (sets P_i initial) |
| --- | --- | --- | --- |
| Fixed-wing, foam, < 2 kg | 150 at q̄ = 100 Pa, scaled by q̄ | 0.2× to 5× | 1.0 |
| Fixed-wing, 2–20 kg | 60 at q̄ = 200 Pa, scaled by q̄ | 0.2× to 5× | 1.0 |
| Multirotor, < 1 kg | 400 | 0.25× to 4× | 0.7 |
| Multirotor, 1–15 kg | 120 | 0.25× to 4× | 0.7 |
| VTOL hover | multirotor row | as multirotor | 0.7 |
| VTOL forward flight | fixed-wing row | as fixed-wing | 1.0 |

Rules:

- Sign of the diagonal is fixed by the actuator layout. The RLS may change its magnitude only; a fitted sign opposite to the prior triggers the reversal handling in section 4, not a silent sign flip.
- Off-diagonal prior is zero with relative σ of 0.3 of the diagonal prior, so coupling can be learned but starts as no coupling.
- Clamping: a G estimate that leaves its band is clamped to the band edge and the event is logged. Two clamps within one second on the same axis mark the identification as failed for that axis (section 8).
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
2. Armed, pre-launch: G = prior, P_i = prior covariance. Fixed-wing: no identification possible on the ground; controller runs on prior G with the launch over-drive factor. Multirotor: first strokes occur at spool-up; identification begins when thrust exceeds 30 % of hover estimate.
3. Launch detected (existing accelerometer-threshold logic): identification enabled on all axes. First commands are whatever the attitude loop demands plus the excitation in step 4.
4. Excitation, 0–300 ms: if the attitude loop's own demand on an axis has not produced a stroke of ≥ Δu_min within 100 ms, inject a single square pulse of Δu_min on that axis, 30 ms wide, alternating sign on successive pulses so the net attitude change is zero. Pulses on different axes are staggered by 40 ms so each stroke is attributable.
5. Converging: each accepted sample updates G and P_i; the loop gain follows the schedule below.
6. Tracking: once all axes are converged, switch λ to the cruise value and keep updating from normal flight commands. Pulses are never injected again unless an axis is marked stale (section 8).

### Gain schedule

The INDI inversion always uses the current G estimate. The over-drive acts on the attitude-loop bandwidth, not on the inversion, so a wrong G cannot be amplified through the schedule.

Let s_i = σ(g_ii)/g_ii be the relative standard deviation of the primary ratio on axis i, from P_i. The attitude-loop bandwidth on that axis is

    ω_att,i = ω_nom · clip(s_conv / s_i, 1, k_od)

with ω_nom = 6 rad/s (the vehicle-independent nominal, same for all classes), s_conv = 0.10 and k_od = 2.5. So an axis whose effectiveness is uncertain gets up to 2.5× the nominal demand, which produces larger strokes and faster identification; as s_i falls to 0.10 the bandwidth settles at nominal.

Two guards keep the over-drive from doing harm:

- The rate loop's own demand is limited to ω̇_max = 0.5 × (prior g_ii) × u_range, so even a prior that is 2× too optimistic cannot saturate an actuator.
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

### Ground step test, fixed-wing

At pre-arm, with the airframe held or on the ground, each surface is stepped by 20 % of its range and back. The gyro sees the reaction torque of the surface moving (small but measurable on light airframes) and, on larger airframes, nothing. Where the gyro response is below noise, τ_act falls back to the class default and the in-flight estimate refines it. The test takes under 0.5 s per surface and runs only once per boot.

### In-flight refinement

The RLS of section 4 is augmented with one extra regressor per axis: Δu_f computed with τ_act scaled by 1.3. The ratio of the two fitted coefficients gives the correction to τ_act. This is a cheap substitute for a full nonlinear lag fit and is only applied when both coefficients are converged.

### Gyro path

1. Notch filtering at the motor harmonic, using the existing ArduPilot harmonic notch (throttle or RPM driven). The notch delay is known from its coefficients and is added to d_act so u_f is delayed by the same amount.
2. Angular acceleration by a 7-tap Savitzky–Golay first-derivative FIR at the IMU rate. Group delay is 3 samples, also added to the alignment delay. A first difference is not acceptable: its noise gain is too high for the RLS at launch.
3. No additional low-pass on ω̇. The RLS forgetting factor provides the averaging; a low-pass here would add delay the inversion then has to fight.

### Alignment check

The cross-correlation of Δu_f and Δω̇ over the last 200 accepted samples is computed on each axis at 10 Hz. A peak more than 2 samples off zero shifts d_act by that amount. This is the only runtime adjustment to the delay and is logged.

## Safety, fallbacks and failure modes

The fallback is designed first: at every moment the controller must be able to drop to a fixed conservative configuration that flies the vehicle, and the adaptive path must never be the only thing holding it up.

### Fallback controller

A fixed-gain rate PID per class, tuned conservatively (about half the bandwidth a hand tune would reach), runs in parallel and shadows the INDI output. Switching to it is bumpless: its integrator is initialised from the current INDI increment. It is engaged on any of the triggers below and stays engaged until the operator or a mode change re-enables identification.

### Triggers and responses

| Condition | Detection | Response |
| --- | --- | --- |
| Wrong-sign fit on an axis | Section 4 sign test fails after the first 3 accepted strokes | Flip axis sign once, restart RLS from mirrored prior; a second failure engages fallback on that axis |
| G leaves its band | Two clamps within 1 s on one axis | Mark axis failed, fallback on that axis, stop injecting pulses |
| Identification stalls | No accepted sample for 2 s while airborne and manoeuvring | Mark axis stale; one pulse injected at nominal bandwidth; stale twice in 10 s engages fallback |
| Divergent inversion | Rate-loop innovation ω̇_des − ω̇ grows for 5 consecutive steps with saturated Δu | Fallback on all axes immediately |
| Gyro unhealthy | Clipping, notch not converged, IMU rate drop | Freeze RLS, hold last G, no pulses; fallback if longer than 500 ms |
| Gust during identification | Innovation energy above 3× the running RLS residual | Reject the window; no update |
| Observer not ready | Attitude or bias not locked at launch | Rate loop runs (needs only gyro); attitude loop at half nominal bandwidth until the observer reports ready |

### Rules that hold regardless of state

- Pulses are never injected below 3 m AGL on multirotors, or within the first 50 ms of fixed-wing launch (the launch transient provides the excitation).
- Pulse amplitude is fixed at Δu_min; it never scales up to force convergence.
- The over-drive factor can only raise the attitude-loop bandwidth, never the rate-loop authority limit.
- A failed axis is reported to the GCS once, with the fitted value and the band, so a human can decide whether to land.
- All RLS state, triggers and clamps are logged at the control rate for the first 2 s after launch and at 10 Hz afterwards. Post-flight review of a failed identification must be possible from the log alone.

### Known residual risks

- A launch into strong turbulence can delay convergence past 300 ms. The system remains safe on prior G; it is only slower.
- An airframe whose true effectiveness is outside the class band cannot be identified; it flies on the fallback. The band must be set from the log survey in section 10, not from these working figures.
- INDI depends on the actuator-lag match. A servo that degrades in flight (stall, slow from load) shows up as a growing alignment error (section 7) and triggers the stall or divergence row above, but there is a window of up to 200 ms before that is detected.

## Numerics, f32

Everything runs in f32 with no heap allocation; the largest matrix is the 8×8 RLS covariance for a VTOL in transition. These are the conditions that keep it stable.

- P_i is stored and updated in square-root form (Bierman UD or Cholesky factor), never as P directly. The plain RLS covariance update loses positive-definiteness in f32 within seconds at λ close to 1. The current `AP_INDI_Ident` keeps P directly with symmetrisation and diagonal bounds; the UD form is required before flight use.
- Regressors are scaled to order 1 before the update: Δu_f is already normalised; Δω̇ is divided by g_ref, the largest |prior| on the axis, and each coefficient is the ratio of effectiveness to its prior (or to g_ref where the prior is zero). G is recovered by multiplying back.
- Forgetting is applied only on accepted samples, so P_i cannot blow up during a long period with no excitation (the usual RLS wind-up failure).
- Diagonal of P_i is floored at (0.02)² relative and capped at the prior variance; a value outside either bound is clamped and logged.
- The inversion G⁻¹ is a weighted pseudo-inverse with a 1e−3 Tikhonov term relative to the largest singular value, solved by a fixed 3×N least-squares, no iterative method.
- The FIR differentiator and lag model are fixed-point-safe (bounded coefficients, no recursion on ω̇), so the control path has no accumulating error.
- Control-rate cost budget: RLS plus inversion for N = 4 is under 1 500 flops per step, about 3 µs on an STM32H7, well inside a 1 kHz loop.

## Validation plan

Each stage must pass before the next starts. Acceptance criteria are the convergence targets in section 4 and no fallback engagement in nominal conditions.

| Stage | What is run | Pass criterion |
| --- | --- | --- |
| 0. Unit tests | `tests/test_indi_ident.cpp`: convergence from a wrong prior, coupling from a zero prior, reversed actuator, rejection gates, band clamp, long noisy run in f32 | All pass (current state) |
| 1. Offline replay | RLS and lag model fed from existing dataflash logs (gyro at full rate, servo outputs) across at least 30 airframes per class | Fitted G stable across the flight to within 10 %; class bands and priors set from the spread |
| 2. SITL, matched airframe | Plane and Copter SITL, launch and takeoff repeated 100 times with randomised wind | Converged within 300 ms in 95 % of runs; no fallback |
| 3. SITL, mismatched airframe | Same, with the SITL physics model's effectiveness scaled by 0.3× and 3× relative to the prior, and one axis reversed | Sign recovered within 50 ms; converged within 500 ms; no divergence |
| 4. SITL, failure injection | Servo slowed 3× in flight, gyro dropouts, gusts during the first 200 ms | Every row of the section 8 trigger table fires as specified; fallback flies the vehicle |
| 5. Hardware in the loop | STM32H7 target against SITL physics over the real IMU and servo paths | Timing budget met; f32 results match the f64 reference to 1 % |
| 6. Flight, small airframes | 5-inch quad and a sub-1 kg foam plane, hand launched, with the fallback armed and a pilot ready | Logs show convergence timeline matching section 6; no fallback in calm air |
| 7. Flight, larger airframes | Only after 6, one class at a time | Same criteria |

Stage 1 is the one that determines whether the priors in section 5 are usable; it needs no new flying and should start first.

## Implementation plan

- Prototype in C++ in-tree. waf has no Rust support; a `no_std` Rust port of the identifier follows once the algorithm is settled on real logs.
- Copter integration through the existing `AC_CustomControl` backend interface (`AC_CustomControl_INDI`), which gives in-flight switching against the stock PID as the fallback. Plane has no equivalent hook yet; one is needed in `APM_Control`.
- Angular acceleration: either rebase the ω̇ plumbing from ArduPilot PR 20511 or add the Savitzky–Golay differentiator in the Copter fast rate thread. Coordinate with that PR's author first.
- SITL needs `SIM_` parameters to scale a frame's effectiveness and reverse an axis for validation stages 3 and 4.
- Parameter prefix `INDI_`, log messages `INDG` (G and σ per axis) and `INDS` (state, triggers, clamps), build option `AP_INDI_ENABLED`.

## Open questions

- Is the 30 ms square pulse acceptable for a hand-launched plane in the first 300 ms, or must the launch transient alone provide the excitation?
- Motor regressor: Δ(u²) assumes thrust proportional to the square of the command. Should the existing ArduPilot thrust-linearisation curve be used instead so the mixer and the RLS agree?
- Stored-prior persistence: per airframe by board ID, or per parameter set? A board moved to a new airframe must not inherit the old G.
- Yaw on a flying wing has no dedicated actuator; the identification will find near-zero effectiveness and the sign test cannot pass. The axis must be excluded by the mixer layout rather than failed at launch.
- Which observer specification pairs with this document: the nonlinear-observer design discussed earlier, or an interim on the existing estimator?
