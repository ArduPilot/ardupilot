# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

extends Node3D # lockstep SITL physics backend

const Bridge := preload("res://scripts/sitl_bridge.gd")
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const GLTFPhysics := preload("res://scripts/gltf_physics.gd")
const Report := preload("res://scripts/vehicle_report.gd")
const TileGround := preload("res://scripts/tile_ground.gd")
const HUD := preload("res://scripts/hud.gd")
const GyroGraph := preload("res://scripts/gyro_graph.gd")

var options := {
    "vehicle": "res://models/quadx.gltf", # up to MAX_VEHICLES, comma-separated
    "world": "",
    "port": "9002", # UDP port
    "control-port": "", # Control port
    "spacing": "5,0", # east,north between vehicles (m)
    "spawn-height": "0.0",
    "spawn-speed": "0.0", # initial forward speed (m/s)
    "launcher": "", # see README; servo 7 releases
    "wind": "0,0,0", # towards north,east,down (m/s)
    "reverse": "", # servo outputs reversed about 1500
    "water": "", # surface level (m, world Y)
    "home": "-35.363261,149.165230", # lat,lon; default is CMAC
    "view": "", # yaw,pitch,distance (deg, deg, m)
    "tiles": "on", # on, off or XYZ URL
    "tiles-zoom": "16",
    "tiles-radius": "3000", # imagery reach from home (m)
    "lake": "", # east,north from home (m)
    "lake-depth": "", # water depth (m)
    "lake-radius": "",
    "sitl-timeout": "0", # seconds; 0 waits forever
}

const LAKE_LEVEL := -0.4 # default lake surface Y (m)
const WATER_SHADER := """
shader_type spatial;
render_mode blend_mix, cull_disabled, depth_draw_never;

uniform vec4 water : source_color = vec4(0.1, 0.35, 0.55, 0.8);
varying vec3 world;

void vertex() {
    world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

// 1 on a grid line, 0 between them. A line narrower than a pixel is
// drawn one pixel wide and fainter rather than being smeared out to
// several: seen at a shallow angle a pixel covers metres of water, and
// widening the line to match turns a hairline into a road.
float lines(vec2 p, float pitch, float width) {
    vec2 d = abs(fract(p / pitch - 0.5) - 0.5) * pitch;
    vec2 fw = fwidth(p);
    float px = 0.5 * max(fw.x, fw.y);
    float w = max(width, px);
    float a = 1.0 - smoothstep(w - px, w + px, min(d.x, d.y));
    return a * clamp(width / max(px, 0.0001), 0.0, 1.0);
}

void fragment() {
    float range = length(VERTEX);
    // the fine lines blur into a haze far off, so let them go there
    float fine = lines(world.xz, 10.0, 0.012) * clamp(1.0 - range / 350.0, 0.0, 1.0);
    float bold = lines(world.xz, 100.0, 0.035) * clamp(1.0 - range / 2500.0, 0.0, 1.0);
    vec3 c = water.rgb;
    c = mix(c, vec3(0.70, 0.88, 1.0), 0.40 * fine);
    c = mix(c, vec3(1.0), 0.60 * bold);
    ALBEDO = c;
    ALPHA = water.a;
    ROUGHNESS = 0.25;
}
"""

const LAKE_RADIUS := 45.0 # default pond radius (m)
var lake_depth := 2.2
var lake_radius := LAKE_RADIUS
const LAKE_OFFSET := Vector3(-170, 0, -40) # default lake position (m)
const LAKE_BED := Color(0.22, 0.26, 0.22)
const PATCH_SIZE := 300.0 # (m) must exceed lake width
var patch_size := PATCH_SIZE
const MAX_VEHICLES := 5

var bridges: Array[ArduPilotSITLBridge] = []
var vehicles: Array[Vehicle] = []
var focus := 0 # followed vehicle; Tab cycles
var sim_time := 0.0
var last_dt := 0.0
var awaiting: Array[bool] = []
var fresh: Array[bool] = []
var connected: Array[bool] = []
var misses: Array[int] = []
var camera: Camera3D
var hud: Label
var attitude: HUD = null
var gyros: GyroGraph = null
var reversed_servos := PackedInt32Array()
var lake_centre := Vector3.ZERO # XZ only
var tiles: TileGround = null
var _sim: Node3D = null # root of everything rebuilt
var _control := PacketPeerUDP.new() # harness control socket
var flight_mode := "" # from harness via control port
var mission_wp := "" # mission item being steered for
var status_msg := ""
var _wps_text := "" # as sent, for redrawing
var _wp_root: Node3D = null
var _wp_marks := {}
const WP_MARK_HEIGHT := 2.0 # (m)
const WP_COLOUR := Color(1.0, 0.85, 0.2, 0.20)
const WP_ACTIVE := Color(0.3, 1.0, 0.45, 0.38)
var autotest := "" # test name, from harness
var last_impact := ""
var last_impact_time := -10.0
var _beat_ms := 0 # wall-clock log heartbeat
var _steps := 0
var _rate_ms := 0
var _draw_ms := 0
const DRAW_MS := 25 # redraw period, render loop off
var _view_sim := -1.0 # sim time camera last moved
const CAM_TAU := 0.12 # chase time constant (sim s)
const CAM_SLIP := 0.15 # max off-centre, fraction of half-height
const CAM_MIN_Y := 0.2 # keeps camera above ground
const CAM_START_DEPTH := -0.5 # (m) relative to water surface
const CAM_MARGIN := 1.15 # vehicle sphere scale for framing
var _target: Node3D = null
var _target_line: MeshInstance3D = null
var _target_plane: Node3D = null
var sitl_timeout_ms := 0 # 0 waits forever
var _servo_ms := 0 # last servo packet, any SITL
var _rate_steps := 0
var phys_rate := 0.0 # sim seconds per wall second
var step_rate := 0.0
var model_names := PackedStringArray() # one per vehicle, for HUD
var launch_accel := 0.0 # launcher push (m/s^2)
var launch_length := 0.0
var launch_dir := Vector3.ZERO
var launch_from := Vector3.ZERO
var launch_sled: AnimatableBody3D = null
var launch_speed := 0.0
var launch_fired := false
var launch_back := 1.0
var launch_held: ArduPilotVehicle = null # frozen until servo 7 releases
const DROP_HEIGHT := 400.0 # default (m)
const TOSS_HEIGHT := 2.0 # default (m)
const TOSS_UP := 2.0
var cam_yaw := PI # orbit azimuth (rad)
var cam_pitch := 0.35
var cam_dist := 3.0
var cam_focus := Vector3.ZERO
var _orbiting := false
var fleet_view := false # Tab cycles fleet, each vehicle
var fleet_zoom := 1.0 # wheel scales fitted distance

func _ready() -> void:
    _set_options(OS.get_cmdline_user_args())

    if DisplayServer.get_name() != "headless": # draw on clock, not per-step
        RenderingServer.render_loop_enabled = false

    PhysicsServer3D.set_active(false) # stands still until SITL drives
    Engine.max_physics_steps_per_frame = 64

    _build_simulation()

    var cport := int(options["control-port"]) if options["control-port"] != "" else int(options["port"]) + 5
    var cerr := _control.bind(cport)
    if cerr != OK:
        push_warning("cannot listen for control on UDP port %d: %s" % [cport, error_string(cerr)])
    else:
        print("control port %d" % cport)

    if DisplayServer.get_name() != "headless":
        _build_view()

func _teardown_simulation() -> void: # camera, lighting, HUD survive
    PhysicsServer3D.set_active(false)
    for b in bridges:
        b.close()
    bridges.clear()
    vehicles.clear()
    awaiting.clear()
    fresh.clear()
    connected.clear()
    misses.clear()
    model_names = PackedStringArray()
    reversed_servos = PackedInt32Array()
    tiles = null
    focus = 0
    sim_time = 0.0
    last_dt = 0.0
    _servo_ms = 0
    _steps = 0
    _rate_steps = 0
    launch_sled = null
    launch_held = null
    launch_accel = 0.0
    launch_length = 0.0
    launch_speed = 0.0
    launch_fired = false
    launch_dir = Vector3.ZERO
    launch_from = Vector3.ZERO
    lake_centre = Vector3.ZERO
    last_impact = ""
    if _sim != null:
        remove_child(_sim)
        _sim.queue_free()
        _sim = null

func _set_options(args: PackedStringArray) -> void:
    for arg in args:
        var kv := arg.trim_prefix("--").split("=", true, 1)
        if kv.size() == 2:
            options[kv[0]] = kv[1]

func _on_control(args: PackedStringArray) -> void: # new simulation; args are --key=value
    _set_options(args)
    print("control: new simulation, %s" % " ".join(args))
    _teardown_simulation()
    _build_simulation()
    _build_waypoints()
    if camera != null and not vehicles.is_empty():
        cam_focus = vehicles[focus].global_position
        _view_sim = -1.0 # rebuilt world restarts its clock

func _poll_control() -> void:
    while _control.get_available_packet_count() > 0:
        var text := _control.get_packet().get_string_from_utf8().strip_edges()
        _control.set_dest_address(_control.get_packet_ip(), _control.get_packet_port()) # harness waits for this reply
        _control.put_packet("ok".to_ascii_buffer())
        if text.begins_with("test "):
            autotest = text.substr(5).strip_edges()
            _wps_text = "" # new test, new mission
            mission_wp = ""
            status_msg = ""
            _build_waypoints()
        elif text.begins_with("hide "):
            var hide := text.substr(5).strip_edges() == "1" # harness says; guessing flickers
            for v in vehicles:
                v.visible = not hide
        elif text.begins_with("wps "):
            _wps_text = text.substr(4).strip_edges()
            _build_waypoints()
        elif text.begins_with("target "):
            _show_target(text.substr(7).strip_edges())
        elif text.begins_with("wp "):
            mission_wp = text.substr(3).strip_edges()
            _mark_waypoint()
        elif text.begins_with("msg "):
            status_msg = text.substr(4).strip_edges()
        elif text.begins_with("mode "):
            flight_mode = text.substr(5).strip_edges() # HUD only; no world rebuild
            _mark_waypoint()
        elif text != "":
            _on_control(text.split(" ", false))

func _build_simulation() -> void: # must not need earlier state
    print("Godot/Jolt ArduPilot Sim, by <davidbuzz@gmail.com>")
    _sim = Node3D.new()
    _sim.name = "sim"
    add_child(_sim)
    var paths: PackedStringArray = String(options["vehicle"]).split(",", false)
    if paths.size() > MAX_VEHICLES:
        push_warning("at most %d vehicles; ignoring the rest" % MAX_VEHICLES)
        paths = paths.slice(0, MAX_VEHICLES)
    var spacing: PackedFloat64Array = String(options["spacing"]).split_floats(",")
    var home: PackedFloat64Array = String(options["home"]).split_floats(",") # lat, lon, alt, heading
    var models: Array[Node3D] = []
    for i in paths.size():
        var model := _load_model(paths[i])
        model_names.append(paths[i].get_file().get_basename())
        if model == null:
            get_tree().quit(1)
            return
        model.position.y += float(options["spawn-height"]) + _launch_height()
        if i > 0 and spacing.size() >= 2:
            model.position += Vector3(-spacing[0], 0, spacing[1]) * i # east is -X, north +Z
        if home.size() >= 4:
            model.rotate_y(deg_to_rad(-home[3])) # heading is clockwise from north
        var v := _find_vehicle(model)
        if v == null:
            push_error("%s has no ArduPilotVehicle body" % paths[i])
            get_tree().quit(1)
            return
        if options["water"] != "":
            v.water_enabled = true
            v.water_level = float(options["water"])
        models.append(model)
        vehicles.append(v)
        v.impact.connect(_on_vehicle_impact.bind(i))

    if options["lake-depth"] != "":
        lake_depth = float(options["lake-depth"])
    if options["lake-radius"] != "":
        lake_radius = float(options["lake-radius"])
    patch_size = maxf(PATCH_SIZE, lake_radius * 2.6) # shore inside patch, land beyond

    sitl_timeout_ms = int(float(options["sitl-timeout"]) * 1000.0)

    if (options["tiles"] != "off" and options["world"] == ""
            and DisplayServer.get_name() != "headless"):
        if home.size() >= 2:
            tiles = TileGround.new()
            _sim.add_child(tiles)
            tiles.setup(home[0], home[1], int(options["tiles-zoom"]),
                    float(options["tiles-radius"]), String(options["tiles"]))

    if options["world"] != "":
        var world := _load_model(options["world"])
        if world:
            _sim.add_child(world)
    else:
        _build_default_world()
    for ch in String(options["reverse"]).split(",", false):
        reversed_servos.append(int(ch))
    var w: PackedFloat64Array = String(options["wind"]).split_floats(",")

    var base_port := int(options["port"]) # one SITL per vehicle
    for i in vehicles.size():
        _sim.add_child(models[i])
        var v := vehicles[i]
        _add_marker(v)
        if i == 0:
            _build_launcher(v)
        print(Report.report(v))
        var v0 := _launch_velocity(v) # drop, toss apply on release
        if v0 != Vector3.ZERO and _launch_height() == 0.0:
            v.set_initial_velocity(v0)
        if w.size() == 3:
            v.wind = Vehicle.TO_NED.transposed() * Vector3(w[0], w[1], w[2])
        var b := Bridge.new()
        var port := base_port + 10 * i
        var err := b.listen(port)
        if err != OK:
            push_error("cannot listen on UDP port %d: %s" % [port, error_string(err)])
            get_tree().quit(1)
            return
        bridges.append(b)
        awaiting.append(false)
        fresh.append(false)
        connected.append(false)
        misses.append(0)
    print("waiting for SITL on UDP port %s" % options["port"])

    var wv := _water_vehicle()
    if wv != null and options["world"] != "":
        var wmat := _plain_material(Color(0.1, 0.35, 0.55, 0.8)) # custom world: show its water
        wmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
        var wplane := PlaneMesh.new()
        wplane.size = Vector2(2000, 2000)
        _add_mesh(_sim, wplane, Vector3(0, wv.water_level, 0), wmat)

func _load_model(path: String) -> Node3D:
    var ext := path.get_extension().to_lower()
    if ext == "gltf" or ext == "glb":
        var abs_path := ProjectSettings.globalize_path(path) # source read directly, no re-import
        return GLTFPhysics.new().load_gltf(abs_path)
    var res := load(path)
    if res is PackedScene:
        return res.instantiate() as Node3D
    push_error("cannot load %s" % path)
    return null

func _find_vehicle(n: Node) -> Vehicle:
    if n is Vehicle:
        return n
    for c in n.get_children():
        var v := _find_vehicle(c)
        if v:
            return v
    return null

func _physics_process(delta: float) -> void:
    if vehicles.is_empty():
        return
    for i in vehicles.size():
        if awaiting[i]:
            bridges[i].send_state(sim_time, vehicles[i].sitl_state(last_dt))
            awaiting[i] = false

    var all_fresh := true # step once all SITLs sent
    for i in vehicles.size():
        if fresh[i]:
            continue
        var timeout_us := 100000 if connected[i] else 1000
        if not bridges[i].wait_for_servos(timeout_us):
            all_fresh = false
            misses[i] += 1
            if connected[i] and misses[i] >= 10:
                print("SITL %d stopped sending, physics paused" % i)
                connected[i] = false
            continue
        fresh[i] = true
        misses[i] = 0
        _servo_ms = Time.get_ticks_msec()
        if not connected[i]:
            print("SITL %d connected from %s:%d, frame rate %d Hz" % [
                    i, bridges[i].sitl_ip, bridges[i].sitl_port, bridges[i].frame_rate])
            connected[i] = true
        if bridges[i].frame_count == 0 and sim_time > 0.0:
            print("SITL %d restarted, resetting its vehicle" % i)
            vehicles[i].reset()
            if vehicles.size() == 1:
                sim_time = 0.0
    if not all_fresh:
        PhysicsServer3D.set_active(false)
        if (sitl_timeout_ms > 0 and _servo_ms > 0 # dead SITL would hold port
                and Time.get_ticks_msec() - _servo_ms > sitl_timeout_ms):
            print("SITL has sent nothing for %.0fs, quitting" % (sitl_timeout_ms / 1000.0))
            get_tree().quit()
        return

    if bridges[0].frame_rate > 0 and bridges[0].frame_rate != Engine.physics_ticks_per_second:
        Engine.physics_ticks_per_second = bridges[0].frame_rate # takes effect next frame

    for i in vehicles.size():
        var pwm := bridges[i].pwm
        if not reversed_servos.is_empty():
            pwm = pwm.duplicate()
            for ch in reversed_servos:
                if ch >= 1 and ch <= pwm.size() and pwm[ch - 1] >= 500: # disabled output stays at 0
                    pwm[ch - 1] = 3000 - pwm[ch - 1]
        vehicles[i].apply_actuators(pwm, delta)
        if i == 0:
            _launch_step(vehicles[i], pwm, delta)
        fresh[i] = false
        awaiting[i] = true
    PhysicsServer3D.set_active(true)
    sim_time += delta
    last_dt = delta
    _steps += 1
    _rate_steps += 1
    var now := Time.get_ticks_msec()
    if not RenderingServer.render_loop_enabled:
        _maybe_draw()
    if _rate_ms == 0:
        _rate_ms = now
    elif now - _rate_ms >= 1000:
        var secs := (now - _rate_ms) / 1000.0
        step_rate = _rate_steps / secs
        phys_rate = _rate_steps * last_dt / secs
        _rate_steps = 0
        _rate_ms = now
    if now - _beat_ms >= 10000:
        if _beat_ms > 0:
            print("t=%.1fs %d steps in %.1fs wall (%.2fx real time, %.0f steps/s, %.0f fps)" % [
                    sim_time, _steps,
                    (now - _beat_ms) / 1000.0,
                    _steps * last_dt * 1000.0 / (now - _beat_ms),
                    _steps * 1000.0 / (now - _beat_ms),
                    Engine.get_frames_per_second()])
        _beat_ms = now
        _steps = 0

func _launcher_parts() -> PackedStringArray: # kind first, then its arguments
    return String(options["launcher"]).split(",", false)

func _launch_height() -> float: # drop or toss; all vehicles
    var parts := _launcher_parts()
    if parts.is_empty():
        return 0.0
    match parts[0]:
        "drop":
            return float(parts[1]) if parts.size() > 1 else DROP_HEIGHT
        "toss":
            return float(parts[1]) if parts.size() > 1 else TOSS_HEIGHT
    return 0.0

func _launch_velocity(v: ArduPilotVehicle) -> Vector3: # spawn-speed plus toss (m/s)
    var vel := v.global_basis.z * float(options["spawn-speed"])
    var parts := _launcher_parts()
    if parts.is_empty():
        return vel
    match parts[0]:
        "toss":
            vel += Vector3.UP * (float(parts[2]) if parts.size() > 2 else TOSS_UP)
        "drop":
            if parts.size() > 2: # optional forward speed, for airflow
                vel += v.global_basis.z * float(parts[2])
    return vel

func _build_launcher(v: ArduPilotVehicle) -> void: # stationary ramp at the spawn
    var parts := _launcher_parts()
    if parts.is_empty():
        return
    var kind := parts[0]
    if kind == "drop" or kind == "toss": # already airborne, nothing to build
        var v0 := _launch_velocity(v)
        var line := "launcher: %s from %.1f m" % [kind, _launch_height()]
        if v0.length() > 0.0:
            line += ", %s at %.1f m/s" % [
                    "thrown" if kind == "toss" else "moving", v0.length()]
        print(line + ", held at %.1f m until servo 7" % v.global_position.y) # actual height, not requested
        launch_held = v # weightless until triggered
        v.freeze = true
        return
    if parts.size() < 4:
        push_warning("--launcher=%s wants kind,acceleration,length,angle" % kind)
        return
    launch_accel = float(parts[1])
    launch_length = float(parts[2])
    var angle := deg_to_rad(float(parts[3]))
    if launch_accel <= 0.0:
        return
    launch_from = v.global_position
    launch_dir = (v.global_basis.z * cos(angle) + Vector3.UP * sin(angle)).normalized()

    var rail := StaticBody3D.new() # wheels can run up it
    rail.name = "launcher_rail"
    var bs := BoxShape3D.new()
    bs.size = Vector3(0.9, 0.1, launch_length)
    _add_mesh(rail, _box(bs.size), Vector3.ZERO, _plain_material(Color(0.25, 0.25, 0.28)), bs)
    _sim.add_child(rail)
    var mid := launch_from + launch_dir * (0.5 * launch_length) - Vector3.UP * 0.30
    rail.look_at_from_position(mid, mid + launch_dir, Vector3.UP)
    rail.global_position = mid

    if kind == "sled":
        launch_sled = AnimatableBody3D.new() # shuttle stops at rail end
        launch_sled.name = "launcher_sled"
        var sb := BoxShape3D.new()
        sb.size = Vector3(0.5, 0.25, 0.25)
        _add_mesh(launch_sled, _box(sb.size), Vector3.ZERO, _plain_material(Color(0.6, 0.5, 0.2)), sb)
        var aabb := _model_extent(v) # start shuttle clear of vehicle
        if aabb.size.z > 0.0:
            launch_back = 0.5 * aabb.size.z + 0.3
        _sim.add_child(launch_sled)
        launch_sled.global_position = launch_from - launch_dir * launch_back

    print("launcher: %s, %.0f m/s^2 along %.0f m at %.0f deg, leaving at %.1f m/s" % [
            kind, launch_accel, launch_length, float(parts[3]),
            sqrt(2.0 * launch_accel * launch_length)])

func _launch_step(v: ArduPilotVehicle, pwm: PackedInt32Array, delta: float) -> void: # bungee pulls vehicle, catapult pushes
    var triggered := pwm.size() >= 7 and pwm[6] > 1700
    if launch_held != null:
        if not triggered:
            return
        var v0 := _launch_velocity(launch_held) # keeps held height, adds throw
        launch_held.freeze = false
        launch_held.set_initial_velocity(v0)
        print("launcher: released on servo 7 = %d at %.1f m" % [
                pwm[6], launch_held.global_position.y])
        launch_held = null
        return
    if launch_accel <= 0.0:
        return
    if not triggered: # servo 7, as SITL's launcher
        return
    if not launch_fired:
        launch_fired = true
        print("launcher: fired on servo 7 = %d, pushing %.1f N" % [
                pwm[6], v.mass * launch_accel])
    if launch_sled != null:
        if launch_speed * launch_speed >= 2.0 * launch_accel * launch_length:
            return # shuttle ran out of rail
        launch_speed += launch_accel * delta
        launch_sled.position += launch_dir * launch_speed * delta
    if v.global_position.distance_to(launch_from) >= launch_length: # compared in world space
        return
    v.apply_central_force(launch_dir * v.mass * launch_accel)

func _model_extent(model: Node3D) -> AABB: # world-space bounds of visuals
    var out := AABB()
    var first := true
    for n in model.find_children("*", "VisualInstance3D", true, false):
        var vi := n as VisualInstance3D
        var a: AABB = vi.global_transform * vi.get_aabb()
        if first:
            out = a
            first = false
        else:
            out = out.merge(a)
    return out

const MARKER_HEIGHT := 100.0
const MARKER_PX := 2.0 # on-screen width (px)
const MARKER_MIN := 0.01 # world width limits (m)
const MARKER_MAX := 2.0

func _add_marker(v: ArduPilotVehicle) -> void: # tall mark, visible when distant
    var mat := _plain_material(Color(1.0, 0.9, 0.1))
    mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED # an instrument, not scenery
    mat.disable_receive_shadows = true
    var mi := _add_mesh(v, _box(Vector3(1.0, MARKER_HEIGHT, 1.0)), Vector3.ZERO, mat) # metre wide, rescaled every draw
    mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
    mi.name = "marker"

func _on_vehicle_impact(other: Node, speed: float, which: int) -> void:
    var what := String(other.name)
    var hit := vehicles.find(other) if other is Vehicle else -1 # find() throws on non-Vehicle
    if hit >= 0:
        what = "vehicle %d" % hit
    last_impact = "IMPACT: vehicle %d hit %s at %.1f m/s, t=%.1fs" % [which, what, speed, sim_time]
    last_impact_time = sim_time
    print(last_impact)

func _process(_delta: float) -> void:
    _poll_control()
    if RenderingServer.render_loop_enabled:
        _update_view()
    else:
        _maybe_draw() # keeps window responsive between steps

func _maybe_draw() -> void: # view and draw sampled together
    var now := Time.get_ticks_msec()
    if now - _draw_ms < DRAW_MS:
        return
    _draw_ms = now
    _update_view()
    _flush_transforms(camera) # else camera lags a frame
    for v in vehicles:
        _flush_transforms(v)
    RenderingServer.force_draw()

func _flush_transforms(n: Node3D) -> void: # push transforms to renderer now
    n.force_update_transform()
    for c in n.find_children("*", "Node3D", true, false):
        (c as Node3D).force_update_transform()

func _update_view() -> void:
    if camera == null or vehicles.is_empty():
        return
    var v := vehicles[focus]
    var target: Vector3 = _fleet_centre() if fleet_view else v.global_position
    if _view_sim < 0.0: # smoothed in sim time
        _view_sim = sim_time
    var ds := maxf(sim_time - _view_sim, 0.0)
    _view_sim = sim_time
    cam_focus = cam_focus.lerp(target, 1.0 - exp(-ds / CAM_TAU))
    var dist := fleet_zoom * _fleet_fit_distance(cam_focus) if fleet_view else cam_dist # wheel scales around fleet fit
    if not fleet_view:
        var slip := cam_focus - target # lag capped as frame fraction
        var limit := CAM_SLIP * dist * tan(deg_to_rad(camera.fov) * 0.5)
        if slip.length() > limit:
            cam_focus = target + slip.normalized() * limit
    var dir := _orbit_dir()
    _place_camera(cam_focus, dir, dist)
    if not fleet_view:
        var r := _vehicle_radius(v) * CAM_MARGIN # keep vehicle whole in frame
        if not _sphere_in_frame(v.global_position, r):
            cam_focus = v.global_position
            _place_camera(cam_focus, dir,
                    maxf(dist, r / maxf(sin(_half_angle()), 0.001)))
    var ned := Vehicle.TO_NED * cam_focus
    hud.position = Vector2(10, 0.1 * get_viewport().get_visible_rect().size.y + 14) # below the attitude indicator
    var which := ""
    if vehicles.size() > 1:
        which = " fleet (Tab switches)" if fleet_view else " vehicle %d/%d (Tab switches)" % [focus + 1, vehicles.size()]
    hud.text = "%s%s  t=%.2fs  alt=%.2fm  N=%.2f E=%.2f" % [
            "SITL connected" if connected[focus] else "waiting for SITL", which,
            sim_time, -ned.z, ned.x, ned.y]
    var flying: String = model_names[focus] if focus < model_names.size() else "?"
    hud.text += "\n%s  physics %.2fx real  %.0f steps/s  render %.0f fps" % [
            flying, phys_rate, step_rate, Engine.get_frames_per_second()]
    if flight_mode != "" or mission_wp != "" or autotest != "": # mode, waypoint, test from harness
        var bits := PackedStringArray()
        if flight_mode != "":
            bits.append("MODE: %s" % flight_mode)
        if mission_wp != "" and flight_mode.to_upper() == "AUTO": # only steered for in AUTO
            bits.append("WP: %s" % mission_wp)
        if autotest != "":
            bits.append("Autotest: %s" % autotest)
        hud.text += "\n" + "  ".join(bits)
    if status_msg != "":
        hud.text += "\nMSGs: %s" % status_msg
    hud.text += "\nright-drag or ctrl-drag orbits, wheel zooms"
    if sim_time - last_impact_time < 8.0:
        hud.text += "\n" + last_impact
    if tiles != null:
        hud.text += "\n" + tiles.attribution
    _scale_markers()

func _orbit_dir() -> Vector3:
    return Vector3(cos(cam_pitch) * sin(cam_yaw), sin(cam_pitch),
            cos(cam_pitch) * cos(cam_yaw))

func _place_camera(f: Vector3, dir: Vector3, d: float) -> void: # along dir, kept above ground
    var p := f + dir * d
    if p.y < CAM_MIN_Y and f.y > CAM_MIN_Y: # not when subject is submerged
        p.y = CAM_MIN_Y
        var want := sqrt(maxf(d * d - pow(CAM_MIN_Y - f.y, 2.0), 0.0))
        var horiz := Vector2(p.x - f.x, p.z - f.z)
        horiz = horiz.normalized() * want if horiz.length() > 0.001 else Vector2(0.0, want)
        p.x = f.x + horiz.x
        p.z = f.z + horiz.y
        camera.global_position = p # keeps asked direction, vehicle slides
        camera.look_at(p - dir)
        return
    camera.global_position = p
    camera.look_at(f)

func _frustum_depth(p: Vector3) -> float: # least plane distance, positive inwards
    var least := INF
    for pl: Plane in camera.get_frustum():
        least = minf(least, -pl.distance_to(p))
    return least

func _sphere_in_frame(p: Vector3, r: float) -> bool:
    return _frustum_depth(p) >= r

func _half_angle() -> float: # narrower frustum half-angle (rad)
    var size := get_viewport().get_visible_rect().size
    var aspect := size.x / size.y if size.y > 0.0 else 1.0
    var t := tan(deg_to_rad(camera.fov) * 0.5)
    var tv := t
    var th := t * aspect
    if camera.keep_aspect == Camera3D.KEEP_WIDTH:
        th = t
        tv = t / maxf(aspect, 0.001)
    return atan(minf(tv, th))

func _scale_markers() -> void: # call after placing the camera
    var height := get_viewport().get_visible_rect().size.y
    if height <= 0.0:
        return
    var per_metre := 2.0 * tan(deg_to_rad(camera.fov) * 0.5) / height # metres/pixel at one metre
    for v in vehicles:
        var mi := v.get_node_or_null("marker") as Node3D
        if mi == null:
            continue
        var d := camera.global_position.distance_to(v.global_position)
        var w := clampf(d * per_metre * MARKER_PX, MARKER_MIN, MARKER_MAX)
        mi.scale = Vector3(w, 1.0, w)

func _unhandled_input(event: InputEvent) -> void:
    if camera == null:
        return
    if event is InputEventMouseButton:
        if event.button_index in [MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_DOWN] and event.pressed:
            var k := 0.88 if event.button_index == MOUSE_BUTTON_WHEEL_UP else 1.0 / 0.88
            if fleet_view:
                fleet_zoom = clampf(fleet_zoom * k, 0.2, 4.0)
            else:
                cam_dist = clampf(cam_dist * k, 0.4, 150.0)
        elif event.button_index == MOUSE_BUTTON_RIGHT or event.button_index == MOUSE_BUTTON_MIDDLE:
            _orbiting = event.pressed
        elif event.button_index == MOUSE_BUTTON_LEFT:
            _orbiting = event.pressed and event.ctrl_pressed
    elif event is InputEventMouseMotion and _orbiting:
        cam_yaw -= event.relative.x * 0.006
        cam_pitch = clampf(cam_pitch - event.relative.y * 0.006, -1.4, 1.45)
    elif event is InputEventKey and event.pressed and event.keycode == KEY_TAB and vehicles.size() > 1:
        if fleet_view:
            fleet_view = false
            focus = 0
        elif focus + 1 < vehicles.size():
            focus += 1
        else:
            fleet_view = true
            focus = 0
        if attitude != null:
            attitude.vehicle = vehicles[focus]
        if gyros != null:
            gyros.vehicle = vehicles[focus]
        cam_dist = _start_distance(vehicles[focus])

func _fleet_centre() -> Vector3:
    var centre := Vector3.ZERO
    for v in vehicles:
        centre += v.global_position
    return centre / float(vehicles.size())

func _fleet_fit_distance(centre: Vector3) -> float: # closest keeping every vehicle framed
    var dir := _orbit_dir() # from focus towards camera
    var right := (-dir).cross(Vector3.UP).normalized()
    var up := right.cross(-dir)
    var vp := get_viewport().get_visible_rect().size
    var tan_v := tan(deg_to_rad(camera.fov) * 0.5) * 0.85 # 85% of the half view
    var tan_h := tan_v * vp.x / vp.y
    var dist := 1.0
    for v in vehicles:
        var rel: Vector3 = v.global_position - centre
        var toward := rel.dot(dir)
        dist = maxf(dist, toward + (absf(rel.dot(right)) + 1.0) / tan_h)
        dist = maxf(dist, toward + (absf(rel.dot(up)) + 1.0) / tan_v)
    return dist

func _is_submersible(v: Vehicle) -> bool: # judged by displacement versus weight
    if not v.water_enabled:
        return false
    var displaced := 0.0
    for c in v.get_components():
        if "fluid" in c and "volume" in c and String(c.get("fluid")) == "water":
            displaced += float(c.get("volume"))
    if displaced <= 0.0:
        return false
    return v.mass / (displaced * v.water_density) > 0.85

func _from_home(home: PackedFloat64Array, lat: float, lon: float, y: float) -> Vector3:
    var north := (lat - home[0]) * 111319.49
    var east := (lon - home[1]) * 111319.49 * cos(deg_to_rad(home[0]))
    return Vector3(-east, y, north) # north +Z, east -X

func _build_waypoints() -> void: # radius;seq,lat,lon,alt;... alt above home
    if _wp_root != null and is_instance_valid(_wp_root):
        _wp_root.queue_free()
    _wp_root = null
    _wp_marks.clear()
    if _wps_text == "" or _sim == null or not is_instance_valid(_sim):
        return
    var home: PackedFloat64Array = String(options["home"]).split_floats(",")
    if home.size() < 2:
        return
    var parts := _wps_text.split(";", false)
    if parts.size() < 2:
        return
    var radius := maxf(float(parts[0]), 0.2)
    var wv := _water_vehicle() # water surface, else ground
    var base := wv.water_level if wv != null else 0.0
    _wp_root = Node3D.new()
    _wp_root.name = "waypoints"
    _sim.add_child(_wp_root)
    for i in range(1, parts.size()):
        var f := parts[i].split_floats(",")
        if f.size() < 4:
            continue
        var at := _from_home(home, f[1], f[2], base)

        var drum := _cylinder(radius, radius, WP_MARK_HEIGHT)
        drum.radial_segments = 48
        var mat := _plain_material(WP_COLOUR)
        mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
        mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        mat.cull_mode = BaseMaterial3D.CULL_DISABLED
        mat.render_priority = 2 # drawn after the water
        _add_mesh(_wp_root, drum, at + Vector3(0, 0.5 * WP_MARK_HEIGHT - 0.3, 0), mat) # sunk to hide ground gaps
        _wp_marks[int(f[0])] = mat

        if f[3] > 3.0:
            _add_mesh(_wp_root, _cylinder(0.06, 0.06, f[3]), at + Vector3(0, 0.5 * f[3], 0), mat)

        var tag := Label3D.new()
        tag.text = str(int(f[0]))
        tag.billboard = BaseMaterial3D.BILLBOARD_ENABLED
        tag.no_depth_test = true
        tag.render_priority = 3
        tag.fixed_size = true
        tag.pixel_size = 0.0018
        tag.font_size = 40
        tag.outline_size = 10
        tag.position = at + Vector3(0, maxf(f[3], 0.0) + WP_MARK_HEIGHT + 0.6, 0)
        _wp_root.add_child(tag)
    _mark_waypoint()

func _show_target(text: String) -> void: # lat,lon,height,heading in deg and m
    var f := text.split_floats(",")
    var home: PackedFloat64Array = String(options["home"]).split_floats(",")
    if f.size() < 3 or home.size() < 2 or _sim == null or not is_instance_valid(_sim):
        return
    if _target == null or not is_instance_valid(_target):
        _target = Node3D.new()
        _target.name = "target"
        _sim.add_child(_target)
        var red := _plain_material(Color(0.9, 0.15, 0.12))
        red.roughness = 0.5
        var white := _plain_material(Color(0.93, 0.93, 0.95))
        white.roughness = 0.5
        _target_plane = Node3D.new()
        _target.add_child(_target_plane)
        for part in [ # nose +Z; size, position, material
                [Vector3(0.5, 0.5, 4.0), Vector3(0, 0, 0), white],        # fuselage
                [Vector3(6.0, 0.12, 1.0), Vector3(0, 0.15, 0.4), red],    # wing
                [Vector3(2.2, 0.1, 0.6), Vector3(0, 0.1, -1.8), red],     # tailplane
                [Vector3(0.1, 0.9, 0.7), Vector3(0, 0.55, -1.8), red]]:   # fin
            _add_mesh(_target_plane, _box(part[0]), part[1], part[2])
        var thin := _plain_material(Color(0.9, 0.15, 0.12))
        thin.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        _target_line = _add_mesh(_target, _cylinder(0.05, 0.05, 1.0), Vector3.ZERO, thin)
    _target.position = _from_home(home, f[0], f[1], f[2])
    _target_line.scale = Vector3(1.0, maxf(f[2], 0.01), 1.0)
    _target_line.position = Vector3(0, -0.5 * f[2], 0)
    if f.size() >= 4:
        _target_plane.basis = Basis(Vector3.UP, -deg_to_rad(f[3])) \
                * Basis(Vector3.BACK, deg_to_rad(f[4]) if f.size() >= 5 else 0.0) # heading clockwise, +Y turn anticlockwise

func _mark_waypoint() -> void: # lit only while steered for
    var active := int(mission_wp) if mission_wp.is_valid_int() and flight_mode.to_upper() == "AUTO" else -1
    for seq in _wp_marks:
        (_wp_marks[seq] as StandardMaterial3D).albedo_color = WP_ACTIVE if seq == active else WP_COLOUR

func _vehicle_radius(v: Vehicle) -> float:
    var r := 0.3
    for n in v.find_children("*", "MeshInstance3D", true, false):
        var mi := n as MeshInstance3D
        if mi.name == "marker" or mi.mesh == null: # marker bar excluded from size
            continue
        var aabb: AABB = mi.mesh.get_aabb()
        var centre: Vector3 = mi.global_transform * aabb.get_center()
        var ext: float = (mi.global_transform.basis * aabb.size).length() * 0.5
        r = maxf(r, v.global_position.distance_to(centre) + ext)
    return r

func _start_distance(v: Vehicle) -> float:
    return clampf(3.0 * _vehicle_radius(v), 1.0, 15.0)

func _water_vehicle() -> Vehicle: # first wanting water, or null
    for v in vehicles:
        if v.water_enabled:
            return v
    return null

func _build_default_world() -> void: # flat land, lake, trees, buildings
    var floats := _water_vehicle() # floaters must start on water
    var placed := false
    if options["lake"] != "":
        var en: PackedFloat64Array = String(options["lake"]).split_floats(",")
        if en.size() >= 2:
            lake_centre = Vector3(-en[0], 0, en[1]) # +Z north, -X east
            placed = true
        else:
            push_warning("--lake wants east,north in metres")
    if not placed:
        lake_centre = Vector3.ZERO if floats != null else LAKE_OFFSET
    if floats != null and options["water"] == "":
        for v in vehicles:
            if v.water_enabled:
                v.water_level = LAKE_LEVEL

    var mat := PhysicsMaterial.new()
    mat.friction = 0.8
    var grass := _grass_material()

    var h := patch_size * 0.5 # flat ground strips around patch
    var e := 20000.0
    var cx := lake_centre.x
    var cz := lake_centre.z
    for r in [
        Rect2(cx + h, -e, e - (cx + h), 2.0 * e),
        Rect2(-e, -e, (cx - h) - (-e), 2.0 * e),
        Rect2(cx - h, cz + h, patch_size, e - (cz + h)),
        Rect2(cx - h, -e, patch_size, (cz - h) - (-e)),
    ]:
        var ground := StaticBody3D.new()
        ground.name = "ground"
        var cs := CollisionShape3D.new()
        var box := BoxShape3D.new()
        box.size = Vector3(r.size.x, 1, r.size.y)
        cs.shape = box
        cs.position = Vector3(r.position.x + 0.5 * r.size.x, -0.5, r.position.y + 0.5 * r.size.y)
        ground.add_child(cs)
        ground.physics_material_override = mat
        _sim.add_child(ground)
        if tiles != null:
            _add_mesh(_sim, _ground_quad(r), Vector3.ZERO, tiles.ground_material())
        else:
            var plane := PlaneMesh.new()
            plane.size = r.size
            _add_mesh(_sim, plane, Vector3(cs.position.x, 0, cs.position.z), grass)

    _build_terrain(mat, grass)
    _build_scenery()

func _ground_quad(r: Rect2) -> ArrayMesh: # carries the imagery's UVs
    var st := SurfaceTool.new()
    st.begin(Mesh.PRIMITIVE_TRIANGLES)
    var p := [Vector3(r.position.x, 0, r.position.y),
            Vector3(r.position.x, 0, r.end.y),
            Vector3(r.end.x, 0, r.end.y),
            Vector3(r.end.x, 0, r.position.y)]
    for v in [p[0], p[2], p[1], p[0], p[3], p[2]]: # clockwise is front-facing
        st.set_normal(Vector3.UP)
        st.set_uv(tiles.world_to_uv(v.x, v.z))
        st.add_vertex(v)
    return st.commit()

func _build_terrain(mat: PhysicsMaterial, grass: Material) -> void: # patch dished into lake basin
    var st := SurfaceTool.new()
    st.begin(Mesh.PRIMITIVE_TRIANGLES)
    var bed := SurfaceTool.new()
    bed.begin(Mesh.PRIMITIVE_TRIANGLES)
    var bed_cells := 0
    var n := 60
    var step := patch_size / n
    var inner := lake_radius * 0.55
    for j in n:
        for i in n:
            var p := []
            var deepest := 0.0
            for d in [[0, 0], [0, 1], [1, 1], [1, 0]]:
                var x: float = lake_centre.x - 0.5 * patch_size + (i + d[0]) * step
                var z: float = lake_centre.z - 0.5 * patch_size + (j + d[1]) * step
                var r := Vector2(x - lake_centre.x, z - lake_centre.z).length()
                var y := -lake_depth * (1.0 - smoothstep(inner, lake_radius, r))
                deepest = maxf(deepest, -y)
                p.append(Vector3(x, y, z))
            var under := deepest >= -LAKE_LEVEL # cell wholly below waterline
            var into := bed if under else st
            if under:
                bed_cells += 1
            for v in [p[0], p[2], p[1], p[0], p[3], p[2]]: # clockwise is front-facing
                into.set_uv(tiles.world_to_uv(v.x, v.z) if tiles != null else Vector2(v.x, v.z) * 0.5)
                into.add_vertex(v)
    st.generate_normals()
    var mesh := st.commit()
    if bed_cells > 0:
        bed.generate_normals()
        bed.commit(mesh)

    var body := StaticBody3D.new()
    body.name = "terrain"
    var cs := CollisionShape3D.new()
    var shape := mesh.create_trimesh_shape()
    shape.backface_collision = true
    cs.shape = shape
    body.add_child(cs)
    body.physics_material_override = mat
    _sim.add_child(body)

    mesh.surface_set_material(0, tiles.ground_material() if tiles != null else grass)
    if mesh.get_surface_count() > 1:
        mesh.surface_set_material(1, _plain_material(LAKE_BED))
    _add_mesh(_sim, mesh)

    var wv := _water_vehicle() # water drawn even if unused
    var wshader := Shader.new() # world-fixed grid, 10 m lines
    wshader.code = WATER_SHADER
    var wmat := ShaderMaterial.new()
    wmat.shader = wshader
    _add_mesh(_sim, _cylinder(lake_radius, lake_radius, 0.02), Vector3(lake_centre.x, # wider than waterline, meets bank
            wv.water_level if wv != null else LAKE_LEVEL, lake_centre.z), wmat)

func _build_scenery() -> void:
    var out := lake_radius / LAKE_RADIUS # scenery moves out with shore
    for t in [[55.0, 120.0], [62.0, 145.0], [58.0, 170.0], [66.0, 195.0],
            [54.0, 215.0], [60.0, 235.0], [68.0, 255.0], [57.0, 275.0]]:
        var a: float = deg_to_rad(t[1])
        _add_tree(lake_centre + Vector3(cos(a), 0, sin(a)) * (t[0] * out))
    _add_box_building(lake_centre + Vector3(-70, 0, 25) * out, Vector3(10, 6, 8))
    _add_box_building(lake_centre + Vector3(-55, 0, -35) * out, Vector3(4, 3, 5))

func _add_tree(pos: Vector3) -> void:
    var tree := StaticBody3D.new()
    tree.name = "tree"
    tree.position = pos

    var trunk_shape := CylinderShape3D.new()
    trunk_shape.radius = 0.25
    trunk_shape.height = 3.5
    _add_mesh(tree, _cylinder(0.2, 0.3, 3.5), Vector3(0, 1.75, 0),
            _plain_material(Color(0.4, 0.26, 0.13)), trunk_shape)

    var canopy_shape := SphereShape3D.new()
    canopy_shape.radius = 2.2
    var canopy := SphereMesh.new()
    canopy.radius = 2.2
    canopy.height = 4.4
    _add_mesh(tree, canopy, Vector3(0, 4.5, 0),
            _plain_material(Color(0.16, 0.42, 0.14)), canopy_shape)
    _sim.add_child(tree)

func _add_box_building(pos: Vector3, size: Vector3) -> void:
    var building := StaticBody3D.new()
    building.name = "building"
    building.position = pos
    var shape := BoxShape3D.new()
    shape.size = size
    _add_mesh(building, _box(size), Vector3(0, 0.5 * size.y, 0),
            _plain_material(Color(0.65, 0.6, 0.55)), shape)
    _sim.add_child(building)

func _plain_material(color: Color) -> StandardMaterial3D:
    var mat := StandardMaterial3D.new()
    mat.albedo_color = color
    return mat

func _add_mesh(parent: Node, mesh: Mesh, at := Vector3.ZERO, mat: Material = null,
        shape: Shape3D = null) -> MeshInstance3D:
    if shape != null:
        var cs := CollisionShape3D.new()
        cs.shape = shape
        cs.position = at
        parent.add_child(cs)
    var mi := MeshInstance3D.new()
    mi.mesh = mesh
    mi.position = at
    mi.material_override = mat
    parent.add_child(mi)
    return mi

func _box(size: Vector3) -> BoxMesh:
    var box := BoxMesh.new()
    box.size = size
    return box

func _cylinder(top: float, bottom: float, height: float) -> CylinderMesh:
    var mesh := CylinderMesh.new()
    mesh.top_radius = top
    mesh.bottom_radius = bottom
    mesh.height = height
    return mesh

func _grass_material() -> StandardMaterial3D: # 10 m chequerboard, mipmapped
    var cell := 64  # pixels per 10 m square
    var light := Color(0.38, 0.53, 0.31)
    var dark := Color(0.31, 0.45, 0.26)
    var line := Color(0.21, 0.31, 0.18)
    var img := Image.create(2 * cell, 2 * cell, true, Image.FORMAT_RGB8)
    img.fill_rect(Rect2i(0, 0, cell, cell), light)
    img.fill_rect(Rect2i(cell, cell, cell, cell), light)
    img.fill_rect(Rect2i(cell, 0, cell, cell), dark)
    img.fill_rect(Rect2i(0, cell, cell, cell), dark)
    for k in [0, cell]:
        img.fill_rect(Rect2i(0, k, 2 * cell, 1), line)
        img.fill_rect(Rect2i(k, 0, 1, 2 * cell), line)
    img.generate_mipmaps()
    var mat := StandardMaterial3D.new()
    mat.albedo_texture = ImageTexture.create_from_image(img)
    mat.uv1_scale = Vector3(0.05, 0.05, 0.05) # texture repeats every 20 m
    mat.uv1_triplanar = true
    return mat

func _build_view() -> void:
    var env := WorldEnvironment.new()
    env.environment = Environment.new()
    var sky := Sky.new()
    var sky_mat := ProceduralSkyMaterial.new()
    sky_mat.ground_bottom_color = Color(0.30, 0.42, 0.26) # sky's ground, green not tan
    sky_mat.ground_horizon_color = Color(0.42, 0.50, 0.40)
    sky.sky_material = sky_mat
    env.environment.background_mode = Environment.BG_SKY
    env.environment.sky = sky
    env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
    add_child(env)

    var sun := DirectionalLight3D.new()
    sun.rotation_degrees = Vector3(-50, 30, 0)
    sun.shadow_enabled = true
    add_child(sun)

    camera = Camera3D.new()
    add_child(camera)
    camera.make_current()
    cam_focus = vehicles[focus].global_position
    cam_dist = _start_distance(vehicles[focus])
    fleet_view = vehicles.size() > 1
    if fleet_view:
        cam_yaw = PI - PI / 4.0 # 45 deg from broadside
        cam_focus = _fleet_centre()
    if _is_submersible(vehicles[focus]): # submersibles watched from below surface
        var rise := (vehicles[focus].water_level + CAM_START_DEPTH) - cam_focus.y
        cam_pitch = clampf(asin(clampf(rise / maxf(cam_dist, 0.01), -1.0, 1.0)), -1.4, 1.45)

    var view: PackedFloat64Array = String(options["view"]).split_floats(",")
    if view.size() >= 3:
        cam_yaw = deg_to_rad(view[0])
        cam_pitch = deg_to_rad(view[1])
        cam_dist = view[2]
    camera.position = cam_focus + Vector3(0, 0.5 * cam_dist, -cam_dist)

    var layer := CanvasLayer.new()
    attitude = HUD.new()
    attitude.vehicle = vehicles[focus]
    layer.add_child(attitude)
    gyros = GyroGraph.new()
    gyros.vehicle = vehicles[focus]
    layer.add_child(gyros)
    hud = Label.new()
    hud.position = Vector2(10, 10)
    layer.add_child(hud)
    add_child(layer)
