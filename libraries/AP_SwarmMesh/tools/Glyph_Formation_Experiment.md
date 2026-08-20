# Decentralized Glyph Formation Experiment

This document describes the models, implementation, and measurement method for the AP_SwarmMesh `GSoC` glyph-formation experiment. The reference run used one leader and 56 spellers in ArduCopter SITL. Each speller generated its own setpoints onboard from mesh data received at 5 Hz; the Python harness did not assign runtime targets or command formation motion.

Note that the experiment is a demonstration and shouldn't be used for real aircraft yet. Its safety result is empirical and applies only to the tested SITL configuration.

## Reference result

| Measure | Result |
|---|---:|
| SITL instances airborne | 57/57 |
| Spellers placed | 56/56 |
| Unique cells occupied | 56/56 |
| Formation time | 944 s |
| Final cell error | 0.02 m median, 0.05 m worst |
| Minimum sampled 3D separation | 2.03 m |
| Samples below the 3 m CBF bound | 28 |
| Samples below the 1 m overlap threshold | 0 |
| Evaluated pair samples | 15,028,995 |

The 3 m value is a controller design bound, while 1 m is the experiment's physical overlap threshold. A sampled distance below 3 m is therefore reported as a CBF boundary violation, but the run only fails if it records an overlap below 1 m.

## System decomposition

The implementation separates orchestration from onboard coordination:

| Component | Responsibility |
|---|---|
| [`swarm_spell_test.py`](swarm_spell_test.py) | Starts SITL processes, writes parameters, arms and takes off the fleet, activates the task, logs tracks, and computes external metrics. |
| [`swarm_speller.lua`](../../AP_Scripting/applets/swarm_speller.lua) | Runs on every vehicle. It builds the glyph, allocates a local slot, schedules transit, filters velocity through the CBF, and commands `GUIDED` velocity. |
| AP_SwarmMesh | Exchanges position, velocity, and coordination state between peers. It does not interpret the glyph or produce assignments. |
| ArduCopter | Tracks the Lua-generated NED velocity setpoint through the normal SITL flight dynamics and inner control loops. |

The harness has access to every vehicle for experiment control and measurement. That does not make it the formation controller. After it changes the leader's `SCR_USER4`, all formation setpoints originate inside the Lua script running on each vehicle.

## Information model

### Leader state

The leader publishes a coordination basket containing:

- `role = leader`;
- the active word index in `task_id`;
- the geographic anchor in `target_lat`, `target_lng`, and `target_alt_mm`;
- milliseconds elapsed since the task changed, encoded in four user bytes.

The elapsed time defines a shared task epoch. A speller continually reconstructs `task_start_ms = local_time - leader_elapsed_ms`, so staggered SITL boot clocks and late reception do not place vehicles in different launch waves. The last valid task and anchor are cached across short coordination gaps.

### Speller state

An active speller publishes:

- `role`, `task_id`, deterministic `formation_slot`, and `priority`;
- its target location;
- intended NED velocity and horizontal acceleration;
- arrival, control phase, active constraint count, and horizontal ready state in four user bytes.

Inactive spellers and spellers that have completed their horizontal transit silence both their coordination and position streams. This bounds traffic by the number of active vehicles rather than the total fleet size.

## Glyph model

The onboard script contains a 5x7 bitmap font. For a word with $q$ characters, glyph width $W = 5$, height $H = 7$, and one column character gap, the total cell grid width is

$$
\mathrm{total\_width} = qW + (q - 1).
$$

Every occupied bitmap element becomes a target cell. Cells are enumerated in a fixed order: character, row, then column. For character index $k$, row $r$, and column $c$, the cell offset from the leader anchor is

$$
\begin{aligned}
\mathrm{east\_cells}  &= -\frac{\mathrm{total\_width} - 1}{2} + k(W + 1) + c, \\
\mathrm{north\_cells} &=  \frac{H - 1}{2} - r.
\end{aligned}
$$

Multiplication by `SCR_USER3` converts cell coordinates to metres. The target altitude is `SCR_USER2` above home. The harness parses this font directly from the Lua file, so its fleet sizing, target CSV, and external scoring use the same cell ordering as the onboard controller.

`GSoC` contains 56 occupied cells at 4 m spacing.

## Replicated slot allocation

The reference experiment uses `REPLICATED_ASSIGNMENT = true`. Every speller is given a persistent formation number through `SCR_USER5`. The harness sets it to the vehicle's stable speller index. Each vehicle independently applies

$$
\mathrm{slot}_i =
\begin{cases}
\mathrm{formation\_number}_i,
    & 1 \leq \mathrm{formation\_number}_i \leq \mathrm{number\_of\_cells}, \\
0,  & \text{otherwise}.
\end{cases}
$$

This is decentralized from the start as the leader never sends a per vehicle slot, and a speller does not need a complete peer table before engaging. The mapping is deterministic rather than negotiated.

The Lua script retains an experimental claim/negotiation path, but it is not used by this run. Earlier claim-based tests were sensitive to asymmetric stale views, missing peer claims, and identical choices by colocated vehicles. Stable formation numbers removed those convergence dependencies.

## Cohort, wave, and altitude model

A first-order safety filter should not be asked to resolve a dense, initially infeasible multi-agent swarm. The SITL experiment therefore combines the CBF with deterministic traffic scheduling.

For formation rank $r$, the zero based cohort and wave are

$$
\begin{aligned}
\operatorname{cohort}(r) &= \left\lfloor \frac{r - 1}{2} \right\rfloor, \\
\operatorname{wave}(r)   &= \left\lfloor \frac{\operatorname{cohort}(r)}{4} \right\rfloor.
\end{aligned}
$$

Consequently:

- each cohort contains at most two spellers;
- each wave contains four cohorts, or at most eight moving spellers;
- consecutive waves begin 100 s apart;
- all vehicles wait 50 s after the task epoch before any transit;
- cohort `c` uses transit altitude `H + 4c` metres, where `H` is the 20 m formation altitude in the reference run.

When a cohort becomes active, it enables its state streams and waits 3 s before departing. This gives its partner time to receive a position and intent sample. After reaching its target horizontally, it continues transmitting for 5 s, then silences the streams.

Descent is deliberately separated from horizontal transit. A speller may descend only after it is horizontally ready and the launch time of the final wave has passed:

$$
t_{\mathrm{descent}} \geq 50\,\mathrm{s}
    + \mathrm{number\_of\_waves} \cdot 100\,\mathrm{s}.
$$

For 56 spellers this produces 28 two agent cohorts and seven waves. The altitude layers provide separation between cohorts (the CBF only considers the other member of the same cohort). This distinction is important as fleet wide separation is produced by the composition of scheduling, altitude layering, and local CBF filtering, not by centralized CBF optimization.

## Guidance and vehicle model

The Lua control loop runs at 10 Hz. Horizontal nominal velocity is proportional position guidance:

$$
\mathbf{u}_{\mathrm{nom}}
    = \operatorname{clamp\_norm}\!\left(
        k_p(\mathbf{p}_{\mathrm{target}} - \mathbf{p}),\,
        v_{\max}
      \right)
$$

with $k_p = 0.35\,\mathrm{s}^{-1}$ and $v_{\max} = 1.5\,\mathrm{m/s}$. Vertical motion uses a separate proportional controller with gain $0.35\,\mathrm{s}^{-1}$ and a $1\,\mathrm{m/s}$ limit. Before descent, the vertical target is the cohort transit altitude (after it is the common 20 m formation altitude).

A vehicle is reported as arrived when it is within 1 m of its 3D target and its measured horizontal speed is at most 0.35 m/s.

The Lua script commands `vehicle:set_target_velocity_NED()` in Copter `GUIDED` mode. ArduCopter and the SITL vehicle model close the acceleration, attitude, and motor loops. The harness configures `WPNAV_SPEED = 2 m/s` and `WPNAV_ACCEL = 2 m/s^2` in addition to the scripts 1.5 m/s velocity limit.

## Delayed peer state model

Only the vehicle's own navigation state is available at the control loop frequency. Peer position state is transmitted at 5 Hz. For a cohort peer `j`, the controller caches:

- last position and its AP_SwarmMesh update timestamp;
- measured velocity when available;
- intended velocity and acceleration from coordination state as fallbacks.

A cached peer is discarded when it has not been refreshed for 600 ms. If its last position sample is $\tau$ seconds old, its position is propagated using a constant velocity model:

$$
\mathbf{p}_j(t_{\mathrm{now}})
    \approx \mathbf{p}_j(t_{\mathrm{sample}}) + \mathbf{v}_j\tau.
$$

The remaining uncertainty is represented by an enlarged guard distance:

$$
D(\tau) = d_{\mathrm{safe}} + v_{\max}T_s
    + \frac{1}{2}a_{\max}(T_s + \tau)^2,
$$

where $d_{\mathrm{safe}} = 3\,\mathrm{m}$, state period $T_s = 0.2\,\mathrm{s}$, $v_{\max} = 1.5\,\mathrm{m/s}$, and $a_{\max} = 2\,\mathrm{m/s^2}$. The extra $v_{\max}T_s$ covers one position stream interval (the acceleration term grows with sample age).

This guard assumes a 5 Hz position stream. `swarm_spell_test.py` warns when another `--sr-pos` value is selected because `T_s` is currently fixed in the Lua controller.

## Control barrier function

Let $\mathbf{p}_i$, $\mathbf{v}_i$ describe the local vehicle, $\mathbf{p}_j$, $\mathbf{v}_j$ the predicted peer, and

$$
\mathbf{r} = \mathbf{p}_i - \mathbf{p}_j.
$$

The robust distance barrier is

$$
h(\mathbf{r}) = \mathbf{r}^{\mathsf{T}}\mathbf{r} - D(\tau)^2.
$$

The controller applies the first order condition

$$
\dot{h} + \alpha h \geq 0,
$$

with $\alpha = 0.25$. The decision variable contains only the local horizontal velocity, $\mathbf{u} = [v_{i,N},\,v_{i,E}]^{\mathsf{T}}$. Vertical velocities are treated as known terms. For each peer the condition becomes the half plane

$$
2\mathbf{r}_{NE}^{\mathsf{T}}\mathbf{u}
    \geq 2\left(\mathbf{r}_{NE}^{\mathsf{T}}\mathbf{v}_{j,NE}
        + r_Dv_{j,D}\right)
        - 2r_Dv_{i,D} - \alpha h.
$$

Nearby constraints are created only when the predicted distance is within the 15 m neighbour radius plus the uncertainty guard.

At exact or near colocation, the gradient of the squared distance barrier is not useful. The controller substitutes a deterministic antisymmetric virtual bearing derived from the two sysids. Each member of the pair therefore receives the opposite separation direction.

## Velocity optimization

At each 10 Hz update, the onboard controller solves

$$
\begin{aligned}
\underset{\mathbf{u}}{\operatorname{minimize}}\quad
    & \left\lVert\mathbf{u} - \mathbf{u}_{\mathrm{nom}}\right\rVert^2 \\
\text{subject to}\quad
    & \mathbf{a}_k^{\mathsf{T}}\mathbf{u} \geq b_k,
      && \text{for every active CBF constraint } k, \\
    & \left\lVert\mathbf{u}\right\rVert \leq 1.5\,\mathrm{m/s}.
\end{aligned}
$$

This is a two dimensional convex projection problem. The Lua implementation enumerates every location at which the optimum can occur and selects the feasible candidate with the lowest objective value. Candidates include:

- the nominal velocity and its projection onto the speed disk;
- the projection onto each barrier boundary;
- intersections of pairs of barrier boundaries;
- intersections of each barrier boundary with the speed circle.

If no candidate is feasible, an emergency command sums outward vectors weighted by guard violation. A deterministic perpendicular direction breaks a reciprocal head on standstill.

The onboard QP does not contain an explicit acceleration disk. The script publishes a bounded intended acceleration computed from the command change, but the velocity command itself is not clipped by that value. Actual acceleration is supplied by ArduCopter's CL response. The CBF result in SITL is therefore an empirical robustness result, not a formal guarantee for the full vehicle dynamics.

## Python model

[`swarm_cbf_sim.py`](swarm_cbf_sim.py) is the lightweight mdel used before the full SITL integration. It has point mass agents with discrete kinematics, constant velocity peer prediction, robust CBF half planes, a speed disk, and an acceleration disk around the previous command. It also computes continuous pairwise distance over each integration interval.

The research model and SITL experiment share the CBF structure but are not identical:

| Element | Python research model default | SITL Lua controller |
|---|---:|---:|
| Local control rate | 20 Hz | 10 Hz |
| Peer state rate | 5 Hz | 5 Hz |
| Geometry | 2D point mass | 3D SITL vehicle, 2D CBF decision |
| Maximum horizontal speed | 3.0 m/s | 1.5 m/s |
| Acceleration constraint | Explicit 2 m/s² command disk | Closed by ArduCopter |
| Barrier `alpha` | 1.5 | 0.25 |
| Default safe distance | 1.75 m | 3.0 m |
| Assignment | Global Hungarian assignment inside the simulation | Replicated persistent formation number |
| Scheduling | Target row delay, 0.6 s | Two agent altitude cohorts, 100 s waves |

The Python model is useful for controller development, infeasibility detection, and parameter sweeps. Its convergence time and safety result should not be used as predictions of the ArduCopter SITL experiment.

Example:

```bash
python3 libraries/AP_SwarmMesh/tools/swarm_cbf_sim.py \
    --word GSoC --state-rate 5 --csv /tmp/swarm_cbf_track.csv
```

## SITL experiment procedure

### Initial geometry

The leader starts at the glyph anchor. Spellers use a phyllotaxis launch cloud with 4 m scatter spacing. The cloud is staged south of the glyph so takeoff locations do not masquerade as occupied target cells. Automatic standoff is

$$
\mathrm{glyph\_half\_height}
    + \mathrm{launch\_cloud\_radius}
    + 20\,\mathrm{m}.
$$

For the reference fleet this was approximately 62 m.

### Launch and task sequence

1. Launch the fleet in batches of 16, with 4 s between batches.
2. Connect, arm, and take off using up to 24 concurrent workers.
3. Wait 15 s for the mesh to converge.
4. Set the leader's `SCR_USER4` to the `GSoC` word index.
5. Let the replicated epoch schedule release each cohort.
6. Require full unique cell occupancy on two consecutive 3 s checks.
7. Hold the completed glyph for 30 s, stop logging, and compute metrics.

### Reference configuration

| Setting | Value |
|---|---:|
| Word | `GSoC` |
| Leader / spellers | 1 / 56 |
| Cell spacing | 4 m |
| Formation altitude | 20 m AGL |
| Position stream | 5 Hz, active cohorts only |
| Coordination stream | 2 Hz, leader and active cohorts |
| Lua control loop | 10 Hz |
| Track logging | 10 Hz |
| CBF design distance | 3 m |
| Overlap threshold | 1 m |
| SITL speedup | 1 |
| Formation timeout | 1000 s |

Use 1× speedup at this scale. Faster simulated time increases host scheduling pressure and reduces the clock budget available to deliver mesh updates and execute each vehicle's CLs.

## Measurement model

Formation scoring is independent of the onboard `arrived` flag. For each airborne speller, the harness computes 3D distance to every target cell and records the nearest error. A cell is occupied when a speller is within 2.5 m. Completion requires the number of unique occupied cells, not merely the number of placed vehicles, to equal the number of glyph cells.

Separation is also measured externally. The harness:

1. logs `GLOBAL_POSITION_INT` at 10 Hz;
2. interpolates each vehicle at common sample times;
3. rejects an interpolation if its surrounding samples are more than 0.6 s apart;
4. evaluates the Euclidean distance for every available vehicle pair;
5. counts distances below 3 m and below 1 m separately.

These are sampled metrics. A zero overlap count does not prove that separation was never below 1 m between samples. Logging gaps also remove affected pairs from a sample instead of inventing positions.

## Reproduction

Install the Python dependency and build Copter SITL:

```bash
python3 -m venv ~/swarm-venv
~/swarm-venv/bin/pip install pymavlink

./waf configure --board sitl
./waf copter
```

Run from the repository root:

```bash
~/swarm-venv/bin/python libraries/AP_SwarmMesh/tools/swarm_spell_test.py \
    --word GSoC --speedup 1 \
    --work-dir ~/spell_run --csv ~/spell_run/track.csv
```

The run writes:

- per-instance SITL state and logs under `~/spell_run/inst_*`;
- vehicle tracks to `~/spell_run/track.csv`;
- target cells to `~/spell_run/track_glyph.csv`.

## Interpretation and limitations

- The experiment validates onboard decentralized setpoint generation and bounded peer state use in SITL, not in practice.
- Inter-cohort safety depends on the deterministic epoch and altitude plan. The CBF sees same cohort peers only.
- The robust guard is tied to 5 Hz state delivery and a bounded acceleration assumption. Packet outages longer than the 600 ms cache window remove the peer constraint rather than growing it indefinitely.
- The fixed formation number mapping is convergent but not fault tolerant. A missing assigned vehicle would leave a missing glyph cell.
- The final 2.03 m minimum is below the 3 m design distance. This is consistent with unmodelled setpoint tracking dynamics and confirms that the measured run is not formally "robust".
- The 254-peer AP_SwarmMesh scale test demonstrates peer table and routing capacity. The glyph experiment additionally runs full multicopter dynamics and 5 Hz state exchange, so its practical host load is different. Bounded active cohorts keep that traffic from becoming fully decentralized across all 57 vehicles.

The next model improvements should add an acceleration constraint to the onboard optimization, retain conservative constraints through longer peer outages, and add decentralized failover for unfilled deterministic slots before moving from SITL to hardware testing.
