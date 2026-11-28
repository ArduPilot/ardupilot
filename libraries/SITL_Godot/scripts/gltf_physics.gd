# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
class_name ArduPilotGLTFPhysics
extends GLTFDocumentExtension ## glTF physics and ArduPilot parts

const KHR_RB := "KHR_physics_rigid_bodies"
const KHR_SHAPES := "KHR_implicit_shapes"
const AP_EXT := "ARDUPILOT_sitl"
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Component := preload("res://scripts/ap_component.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Surface := preload("res://scripts/ap_surface.gd")
const Wheel := preload("res://scripts/ap_wheel.gd")
const Floater := preload("res://scripts/ap_floater.gd")
const HeliRotor := preload("res://scripts/ap_heli_rotor.gd")
const Streamer := preload("res://scripts/ap_streamer.gd")
const PARTS := {"rotor": Rotor, "surface": Surface, "hinge": Hinge, "wheel": Wheel, ## extension members creating a part
        "floater": Floater, "heliRotor": HeliRotor, "streamer": Streamer}
const OMI_EXTS := ["OMI_physics_body", "OMI_physics_shape", "OMI_collider"]

static var registered: GLTFDocumentExtension = null ## instance registered with GLTFDocument

var _export_shapes: Array = [] # reset by _export_preflight
var _export_materials: Array = []
var _export_vehicle: GLTFNode = null
var _export_nodes := {} # scene node -> GLTFNode
var _export_joints: Array = [] # [joint GLTFNode, reached body]

static func _register(ext: GLTFDocumentExtension) -> bool:
    if registered != null:
        return false
    GLTFDocument.register_gltf_document_extension(ext)
    registered = ext
    return true

static func _unregister(ext: GLTFDocumentExtension) -> void:
    GLTFDocument.unregister_gltf_document_extension(ext)
    registered = null

func load_gltf(path: String) -> Node3D: ## returns scene with physics built
    var temporary := _register(self)
    var doc := GLTFDocument.new()
    var state := GLTFState.new()
    var err := doc.append_from_file(path, state)
    var root: Node3D = null
    if err != OK:
        push_error("failed to load %s: %s" % [path, error_string(err)])
    else:
        root = doc.generate_scene(state) as Node3D
        if root == null:
            push_error("no scene generated from %s" % path)
    if temporary:
        _unregister(self)
    return root

func save_gltf(root: Node, path: String) -> Error:
    var temporary := _register(self)
    var doc := GLTFDocument.new()
    var state := GLTFState.new()
    var copy := root.duplicate() # export omits loader-generated nodes
    var made: Array[Node] = []
    for n in copy.find_children("*", "", true, false):
        if n.has_meta(GENERATED):
            made.append(n)
        elif n.has_meta(JOINT):
            var joint := n.get_node_or_null(n.get_meta(JOINT)) as Joint3D
            if joint != null:
                n.set_meta(JOINT_TO, joint.get_node_or_null(joint.node_b))
    for n in made:
        if is_instance_valid(n):
            n.get_parent().remove_child(n)
            n.free()
    var err := doc.append_from_scene(copy, state)
    if err == OK:
        err = doc.write_to_filesystem(state, path)
    copy.free()
    if temporary:
        _unregister(self)
    return err

func _get_supported_extensions() -> PackedStringArray:
    return PackedStringArray([KHR_RB, KHR_SHAPES, AP_EXT])

func _import_preflight(_state: GLTFState, extensions: PackedStringArray) -> Error:
    if KHR_RB in extensions or AP_EXT in extensions:
        return OK
    return ERR_SKIP

func _import_post(state: GLTFState, root: Node) -> Error:
    build_physics(state, root)
    return OK

func build_physics(state: GLTFState, root: Node) -> void:
    var json: Dictionary = state.json
    var gltf_nodes: Array = json.get("nodes", [])
    var doc_ext: Dictionary = json.get("extensions", {})
    var shapes: Array = doc_ext.get(KHR_SHAPES, {}).get("shapes", [])
    var khr_doc: Dictionary = doc_ext.get(KHR_RB, {})
    var materials: Array = khr_doc.get("physicsMaterials", [])
    var ap_doc: Dictionary = doc_ext.get(AP_EXT, {})
    var visualise: bool = json.get("meshes", []).is_empty() and not Engine.is_editor_hint() # editor has its own gizmos

    var parent_of := {}
    for i in gltf_nodes.size():
        for c in gltf_nodes[i].get("children", []):
            parent_of[int(c)] = i

    var khr := {}
    var ap := {}
    for i in gltf_nodes.size():
        var ext: Dictionary = gltf_nodes[i].get("extensions", {})
        if ext.has(KHR_RB):
            khr[i] = ext[KHR_RB]
        if ext.has(AP_EXT):
            ap[i] = ext[AP_EXT]

    var vehicle_idx := _choose_vehicle(ap_doc, khr, ap, parent_of)
    var replaced := {} # glTF index -> replacement node

    var motion_nodes: Array = khr.keys().filter(func(i): return khr[i].has("motion")) # bodies first, parents before children
    motion_nodes.sort_custom(func(a, b): return _depth(a, parent_of) < _depth(b, parent_of))
    for i in motion_nodes:
        var n := state.get_scene_node(i) as Node3D
        if n == null:
            continue
        var body: RigidBody3D = Vehicle.new() if i == vehicle_idx else RigidBody3D.new()
        body = _replace_node(n, body, root) as RigidBody3D
        replaced[i] = body
        _apply_motion(body, khr[i]["motion"])
        if ap.has(i) and ap[i].has("drag") and body is Vehicle:
            var d: Dictionary = ap[i]["drag"]
            var area: Array = d.get("area", [0, 0, 0])
            body.drag_area = Vector3(area[0], area[1], area[2])
            body.air_density = float(d.get("airDensity", body.air_density))
        if ap.has(i) and ap[i].has("water") and body is Vehicle:
            var w: Dictionary = ap[i]["water"]
            body.water_enabled = true
            body.water_level = float(w.get("level", body.water_level))
            body.water_density = float(w.get("density", body.water_density))

    var drawn_chassis: bool = ap.values().any(func(a): return a.has("chassis")) # drawn chassis replaces collider boxes

    for i in khr.keys():
        var k: Dictionary = khr[i]
        if k.has("trigger"):
            push_warning("%s: trigger on node %d is not supported yet" % [KHR_RB, i])
        if not k.has("collider"):
            continue
        var n := replaced.get(i, state.get_scene_node(i)) as Node3D
        if n == null:
            continue
        _add_collider(state, root, n, k["collider"], shapes, materials,
                visualise and not drawn_chassis, replaced, i)

    var part_nodes: Array = ap.keys().filter(func(i): return _part_key(ap[i]) != "") # parents first, keeps hinge children
    part_nodes.sort_custom(func(a, b): return _depth(a, parent_of) < _depth(b, parent_of))
    for i in part_nodes:
        var key := _part_key(ap[i])
        var n := replaced.get(i, state.get_scene_node(i)) as Node3D
        if n == null:
            continue
        if n is PhysicsBody3D:
            push_warning("%s: %s on body node %s ignored; put it on a child node" % [AP_EXT, key, n.name])
            continue
        var part := _replace_node(n, PARTS[key].new(), root) as Component
        replaced[i] = part
        part.from_gltf(ap[i][key])
        if visualise and not (drawn_chassis and part is Floater):
            _add_part_visual(part, root)

    for i in khr.keys():
        if not khr[i].has("joint"):
            continue
        var jn := replaced.get(i, state.get_scene_node(i)) as Node3D
        var other := int((khr[i]["joint"] as Dictionary).get("connectedNode", -1))
        var on := replaced.get(other, state.get_scene_node(other)) as Node
        var body_a := _body_above(jn)
        var body_b := _body_above(on)
        if jn == null or body_a == null or body_b == null or body_a == body_b:
            push_warning("%s: joint on node %d does not join two bodies" % [KHR_RB, i])
            continue
        var hj := HingeJoint3D.new()
        root.add_child(hj)
        hj.owner = root
        hj.name = "joint_%s" % jn.name
        var t := _xform_to_root(jn, root)
        var b := t.basis.orthonormalized()
        hj.transform = Transform3D(Basis(b.y, b.z, b.x), t.origin) # node X becomes hinge Z
        hj.node_a = hj.get_path_to(body_a)
        hj.node_b = hj.get_path_to(body_b)
        hj.set_meta(GENERATED, true)
        jn.set_meta(JOINT, jn.get_path_to(hj)) # path survives saving as scene

    for i in ap.keys(): # chassis is visual only
        if not ap[i].has("chassis"):
            continue
        var cn := replaced.get(i, state.get_scene_node(i)) as Node3D
        if cn != null:
            cn.set_meta(CHASSIS, ap[i]["chassis"]) # kept for export
            if visualise:
                _build_chassis(cn, root, ap[i]["chassis"])

const GENERATED := &"ap_generated" ## loader-made node, export skips
const JOINT_TO := &"ap_joint_to"
const CHASSIS := &"ap_chassis"
const JOINT := &"ap_joint"

const CHASSIS_FRAME := Color(0.26, 0.28, 0.32)
const CHASSIS_DARK := Color(0.12, 0.13, 0.15)
const CHASSIS_MOTOR := Color(0.55, 0.57, 0.6)

static func _mat(colour: Color, roughness := 0.55, metallic := 0.0) -> StandardMaterial3D:
    var mat := StandardMaterial3D.new()
    mat.albedo_color = colour
    mat.roughness = roughness
    mat.metallic = metallic
    if colour.a < 1.0:
        mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    return mat

func _chassis_box(on: Node3D, root: Node, size: Vector3, pos: Vector3, ## placed in body frame
        colour: Color, yaw := 0.0) -> void:
    var mesh := BoxMesh.new()
    mesh.size = size
    _add_mesh(on, root, mesh, "chassis",
            Transform3D(Basis(Vector3.UP, yaw), pos)).material_override = _mat(colour)

func _chassis_cylinder(on: Node3D, root: Node, radius: float, height: float,
        pos: Vector3, colour: Color) -> void:
    var mesh := CylinderMesh.new()
    mesh.top_radius = radius
    mesh.bottom_radius = radius
    mesh.height = height
    _add_mesh(on, root, mesh, "chassis",
            Transform3D(Basis(), pos)).material_override = _mat(colour, 0.35, 0.6)

func _parts_of(body: Node3D, script: GDScript) -> Array: ## all descendants with that script
    var out: Array = []
    var stack: Array[Node] = [body]
    while not stack.is_empty():
        var n: Node = stack.pop_back()
        for c in n.get_children():
            stack.append(c)
            if c is Node3D and c.get_script() == script:
                out.append(c)
    return out

func _chassis_part(body: Node3D, script: GDScript) -> Node3D: ## first match, or null
    var found := _parts_of(body, script)
    return null if found.is_empty() else found[0] as Node3D

func _chassis_rotors(body: Node3D) -> Array: ## rotor offsets in body frame
    return _parts_of(body, Rotor).map(func(c): return _xform_to_root(c, body).origin)

func _chassis_glass(on: Node3D, root: Node, size: Vector3, pos: Vector3) -> void: ## clear cage panel
    var mesh := BoxMesh.new()
    mesh.size = size
    var mat := _mat(Color(0.55, 0.68, 0.78, 0.17), 0.1, 0.1)
    mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    _add_mesh(on, root, mesh, "chassis", Transform3D(Basis(), pos)).material_override = mat

func _chassis_shroud(on: Node3D, root: Node, radius: float, xform: Transform3D) -> void: ## thruster duct, follows rotor basis
    var mesh := TorusMesh.new()
    mesh.inner_radius = radius
    mesh.outer_radius = radius * 1.22
    mesh.rings = 24
    _add_mesh(on, root, mesh, "chassis", xform).material_override = _mat(CHASSIS_MOTOR, 0.4, 0.5)

func _build_sub_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## open cage keeps thrusters visible
    var w: float = size.x
    var h: float = size.y
    var l: float = size.z
    var bar: float = maxf(0.035 * minf(w, l), 0.004)

    for sx in [-1.0, 1.0]: # twelve edge bars
        for sy in [-1.0, 1.0]:
            _chassis_box(body, root, Vector3(bar, bar, l),
                    at + Vector3(sx * 0.5 * w, sy * 0.5 * h, 0), CHASSIS_FRAME)
    for sx in [-1.0, 1.0]:
        for sz in [-1.0, 1.0]:
            _chassis_box(body, root, Vector3(bar, h, bar),
                    at + Vector3(sx * 0.5 * w, 0, sz * 0.5 * l), CHASSIS_FRAME)
    for sy in [-1.0, 1.0]:
        for sz in [-1.0, 1.0]:
            _chassis_box(body, root, Vector3(w, bar, bar),
                    at + Vector3(0, sy * 0.5 * h, sz * 0.5 * l), CHASSIS_FRAME)

    _chassis_glass(body, root, Vector3(w, 0.004, l), at + Vector3(0, 0.5 * h, 0)) # top, bottom, sides; ends open
    _chassis_glass(body, root, Vector3(w, 0.004, l), at + Vector3(0, -0.5 * h, 0))
    for sx in [-1.0, 1.0]:
        _chassis_glass(body, root, Vector3(0.004, h, l), at + Vector3(sx * 0.5 * w, 0, 0))

    _chassis_box(body, root, Vector3(0.78 * w, 0.17 * h, 0.72 * l), # buoyancy block above electronics tube
            at + Vector3(0, 0.34 * h, 0), Color(0.93, 0.76, 0.18))
    _chassis_cylinder(body, root, 0.17 * h, 0.66 * l,
            at + Vector3(0, -0.06 * h, 0), Color(0.82, 0.85, 0.88))

    for rotor: Node3D in _parts_of(body, Rotor): # duct sized to propeller
        var d: float = 0.1
        if "diameter" in rotor:
            d = float(rotor.get("diameter"))
        _chassis_shroud(body, root, 0.56 * d, _xform_to_root(rotor, body))

func _build_balancebot_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## slab on axle, mass high
    _chassis_box(body, root, size, at, CHASSIS_FRAME) # column, then axle housing
    _chassis_box(body, root, Vector3(2.4 * size.x, 0.16 * size.y, 0.8 * size.z),
            at + Vector3(0, -0.5 * size.y, 0), CHASSIS_DARK)

    _chassis_box(body, root, Vector3(0.8 * size.x, 0.22 * size.y, 0.7 * size.z), # battery low, deck on top
            at + Vector3(0, -0.26 * size.y, 0), CHASSIS_DARK)
    _chassis_box(body, root, Vector3(1.5 * size.x, 0.06 * size.y, 1.3 * size.z),
            at + Vector3(0, 0.5 * size.y, 0), CHASSIS_MOTOR)

func _build_chassis(body: Node3D, root: Node, d: Dictionary) -> void:
    var size := _v3(d.get("size"), Vector3(0.2, 0.1, 0.3))
    var at := _v3(d.get("centre"), Vector3.ZERO) # collider centre, not mass centre
    match String(d.get("style", "")):
        "multirotor":
            _build_multirotor_chassis(body, root, size)
        "quadplane":
            _build_quadplane_chassis(body, root, size, at)
        "helicopter":
            _build_helicopter_chassis(body, root, size, at)
        "rover":
            _build_rover_chassis(body, root, size, at)
        "boat":
            _build_boat_chassis(body, root, size, at)
        "sub":
            _build_sub_chassis(body, root, size, at)
        "balancebot":
            _build_balancebot_chassis(body, root, size, at)
        "solid":
            _build_solid_chassis(body, root, _v3(d.get("colour"), Vector3(0.26, 0.28, 0.32)))
        "blimp":
            _build_blimp_chassis(body, root, size, d)
        "plane":
            _build_plane_chassis(body, root, size, at)

func _build_solid_chassis(body: Node3D, root: Node, colour: Vector3) -> void: ## draws the collision boxes solid
    for c in body.get_children():
        var cs := c as CollisionShape3D
        if cs == null or not (cs.shape is BoxShape3D):
            continue
        var mesh := BoxMesh.new()
        mesh.size = (cs.shape as BoxShape3D).size
        _add_mesh(body, root, mesh, "chassis", cs.transform).material_override = \
                _mat(Color(colour.x, colour.y, colour.z))

func _build_multirotor_chassis(body: Node3D, root: Node, size: Vector3) -> void: ## plates, battery, booms, motors, legs
    var hubs := _chassis_rotors(body)
    if hubs.size() < 3:
        return
    var reach := 0.0 # mean motor distance, not collider
    for p: Vector3 in hubs:
        reach += Vector2(p.x, p.z).length()
    reach /= hubs.size()
    if reach < 0.02:
        return
    var hub := 0.52 * reach
    var h: float = clampf(0.3 * reach, 0.03, maxf(size.y, 0.03))
    var plate := maxf(0.06 * hub, 0.004)
    var boom := 0.1 * reach

    _chassis_box(body, root, Vector3(hub, plate, hub), Vector3(0, -0.45 * h, 0), CHASSIS_FRAME)
    _chassis_box(body, root, Vector3(0.84 * hub, plate, 0.84 * hub), Vector3(0, 0.45 * h, 0), CHASSIS_FRAME)
    _chassis_box(body, root, Vector3(0.6 * hub, 0.6 * h, 0.72 * hub), # battery, then nose canopy
            Vector3(0, -0.05 * h, -0.06 * hub), CHASSIS_DARK)
    _chassis_box(body, root, Vector3(0.44 * hub, 0.5 * h, 0.4 * hub),
            Vector3(0, 0.7 * h, 0.26 * hub), CHASSIS_DARK)

    for p: Vector3 in hubs:
        var flat := Vector3(p.x, 0.0, p.z)
        var out := flat.length()
        if out < 0.02:
            continue
        var yaw := atan2(flat.x, flat.z)
        var start := 0.42 * hub # boom from hub edge
        var span := out - start
        _chassis_box(body, root, Vector3(boom, 0.75 * boom, span),
                flat.normalized() * (start + 0.5 * span) + Vector3(0, 0.2 * h, 0),
                CHASSIS_FRAME, yaw)
        _chassis_cylinder(body, root, 0.95 * boom, 1.7 * boom,
                Vector3(p.x, p.y - 1.2 * boom, p.z), CHASSIS_MOTOR)
        var leg: float = 0.9 * h + 0.18 * reach # leg and foot per boom
        var foot := flat.normalized() * (out - 1.2 * boom)
        _chassis_box(body, root, Vector3(0.45 * boom, leg, 0.45 * boom),
                Vector3(foot.x, 0.2 * h - 0.5 * leg, foot.z), CHASSIS_FRAME, yaw)
        _chassis_box(body, root, Vector3(0.8 * boom, 0.3 * boom, 3.0 * boom),
                Vector3(foot.x, 0.2 * h - leg, foot.z), CHASSIS_DARK, yaw)

func _build_quadplane_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## rotors on fore-aft side booms
    _build_plane_chassis(body, root, size, at)
    var sides := {}
    for p: Vector3 in _chassis_rotors(body):
        if absf(p.x) < 0.05: # centreline rotor: no boom
            continue
        var key := signf(p.x)
        if not sides.has(key):
            sides[key] = []
        sides[key].append(p)
    var boom: float = clampf(0.09 * size.z, 0.02, 0.05)
    for key in sides:
        var hubs: Array = sides[key]
        var x := 0.0
        var y := 0.0
        var front := -1e9
        var back := 1e9
        for p: Vector3 in hubs:
            x += p.x / hubs.size()
            y += p.y / hubs.size()
            front = maxf(front, p.z)
            back = minf(back, p.z)
        if hubs.size() >= 2:
            var length := (front - back) + 4.0 * boom
            _chassis_box(body, root, Vector3(boom, 0.8 * boom, length),
                    Vector3(x, y - 1.6 * boom, 0.5 * (front + back)), CHASSIS_FRAME)
        for p: Vector3 in hubs:
            _chassis_box(body, root, Vector3(0.6 * boom, 1.4 * boom, 0.6 * boom), # pylon from boom to motor
                    Vector3(p.x, p.y - 1.5 * boom, p.z), CHASSIS_FRAME)
            _chassis_cylinder(body, root, 0.85 * boom, 1.5 * boom,
                    Vector3(p.x, p.y - 0.9 * boom, p.z), CHASSIS_MOTOR)

func _build_helicopter_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## cabin, mast, tail boom, skids
    var head := _chassis_part(body, PARTS["heliRotor"])
    if head == null:
        return
    var hub := _xform_to_root(head, body).origin
    var w: float = size.x
    var h: float = size.y
    var l: float = size.z

    _chassis_box(body, root, Vector3(w, h, 0.62 * l), at + Vector3(0, 0, -0.04 * l), CHASSIS_FRAME) # cabin, then tapered nose
    _chassis_box(body, root, Vector3(0.78 * w, 0.66 * h, 0.4 * l),
            at + Vector3(0, -0.12 * h, 0.4 * l), CHASSIS_DARK)
    _chassis_box(body, root, Vector3(0.14 * w, hub.y - 0.3 * h, 0.14 * w), # mast up to head
            Vector3(0, 0.5 * (hub.y + 0.3 * h), 0), CHASSIS_MOTOR)

    var tail := _chassis_part(body, PARTS["rotor"])
    if tail != null:
        var t := _xform_to_root(tail, body).origin
        var boom := 0.16 * w
        var from := -0.3 * l
        _chassis_box(body, root, Vector3(boom, boom, from - t.z),
                Vector3(0, t.y - 0.4 * boom, 0.5 * (from + t.z)), CHASSIS_FRAME)

    var skid := maxf(0.3 * w, 0.02) # side rail on two struts
    for sign: float in [1.0, -1.0]:
        var x: float = sign * 0.75 * w
        _chassis_box(body, root, Vector3(skid, skid, 1.1 * l), Vector3(x, -1.3 * h, 0.05 * l),
                CHASSIS_DARK)
        for z in [-0.3 * l, 0.3 * l]:
            _chassis_box(body, root, Vector3(0.7 * skid, 1.3 * h - 0.5 * h, 0.7 * skid),
                    Vector3(x, -0.9 * h, z), CHASSIS_FRAME)

func _build_rover_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## wheels draw themselves
    _chassis_box(body, root, size, at, CHASSIS_FRAME)
    _chassis_box(body, root, Vector3(0.78 * size.x, 1.4 * size.y, 0.42 * size.z),
            at + Vector3(0, 1.1 * size.y, -0.1 * size.z), CHASSIS_DARK)
    _chassis_box(body, root, Vector3(0.6 * size.x, 0.5 * size.y, 0.3 * size.z),
            at + Vector3(0, 0.6 * size.y, 0.3 * size.z), CHASSIS_FRAME)

func _build_boat_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## hull plus steerable outboard leg
    var w: float = size.x
    var h: float = size.y
    var l: float = size.z
    var mat := _mat(Color(0.85, 0.86, 0.88), 0.5)
    mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    _add_mesh(body, root, _hull_mesh(w, h, l), "chassis",
            Transform3D(Basis(), at)).material_override = mat
    _chassis_box(body, root, Vector3(0.5 * w, 0.9 * h, 0.22 * l), # console, then deck strake
            at + Vector3(0, 0.95 * h, -0.12 * l), CHASSIS_DARK)
    _chassis_box(body, root, Vector3(1.02 * w, 0.12 * h, 0.7 * l),
            at + Vector3(0, 0.5 * h, -0.08 * l), CHASSIS_DARK)

    var leg := _chassis_part(body, PARTS["hinge"]) # outboard swings with steering hinge
    if leg != null:
        _chassis_box(leg, root, Vector3(0.3 * w, 0.5 * h, 0.3 * w),
                Vector3(0, 0.55 * h, 0), CHASSIS_DARK)
        _chassis_box(leg, root, Vector3(0.12 * w, 1.1 * h, 0.14 * w),
                Vector3(0, -0.2 * h, 0), CHASSIS_FRAME)
        _chassis_box(leg, root, Vector3(0.08 * w, 0.4 * h, 0.3 * w),
                Vector3(0, -0.75 * h, -0.04 * w), CHASSIS_FRAME)

func _hull_mesh(w: float, h: float, l: float) -> ArrayMesh: ## planing vee hull, deck, transom
    var st := SurfaceTool.new()
    st.begin(Mesh.PRIMITIVE_TRIANGLES)
    var stations := 9
    var port: Array[Vector3] = []
    var starboard: Array[Vector3] = []
    var keel: Array[Vector3] = []
    for i in stations:
        var t := float(i) / (stations - 1) # 0 stern, 1 bow
        var z := -0.5 * l + t * l
        var half := 0.5 * w * (1.0 - 0.55 * t * t)
        var deck := 0.5 * h + 0.25 * h * t * t
        var bottom := -0.5 * h * (1.0 - 0.85 * t * t) # keel rises towards bow
        port.append(Vector3(half, deck, z))
        starboard.append(Vector3(-half, deck, z))
        keel.append(Vector3(0, bottom, z))

    for i in stations - 1:
        for tri in [[port[i], starboard[i], starboard[i + 1]],
                    [port[i], starboard[i + 1], port[i + 1]],
                    [port[i], keel[i + 1], keel[i]],
                    [port[i], port[i + 1], keel[i + 1]],
                    [starboard[i], keel[i], keel[i + 1]],
                    [starboard[i], keel[i + 1], starboard[i + 1]]]:
            for v: Vector3 in tri:
                st.add_vertex(v)
    for v: Vector3 in [port[0], keel[0], starboard[0]]: # transom closes the stern
        st.add_vertex(v)
    st.generate_normals()
    return st.commit()

func _build_blimp_chassis(body: Node3D, root: Node, d_size: Vector3, d: Dictionary) -> void: ## envelope, gondola, side motor pods
    var lift := _chassis_part(body, PARTS["floater"])
    var centre := Vector3(0, 0.5 * d_size.y, 0)
    if lift != null:
        centre = _xform_to_root(lift, body).origin
    _add_mesh(body, root, _spheroid_mesh(d_size.x, d_size.z), "chassis", # envelope spheroid along the hull
            Transform3D(Basis(), centre)).material_override = _mat(Color(0.85, 0.86, 0.88), 0.8)
    _chassis_box(body, root, Vector3(0.17 * d_size.x, 0.17 * d_size.x, 0.34 * d_size.x),
            Vector3.ZERO, CHASSIS_DARK)

    var pod_l: float = d.get("podLength", 0.1 * d_size.z)
    var pod_d: float = d.get("podDiameter", 0.45 * pod_l)
    var pods := _mat(CHASSIS_DARK, 0.5)
    for p: Vector3 in _chassis_rotors(body):
        if absf(p.x) >= 0.3 * d_size.x:
            _add_mesh(body, root, _spheroid_mesh(pod_d, pod_l), "chassis", # flank pod, propeller behind
                    Transform3D(Basis(), p + Vector3(0, 0, 0.5 * pod_l))).material_override = pods
            continue
        var reach: float = p.y - centre.y # vertical fan on a pylon
        _add_mesh(body, root, _spheroid_mesh(0.6 * pod_d, 0.9 * pod_l), "chassis",
                Transform3D(Basis(), p)).material_override = pods
        _chassis_box(body, root, Vector3(0.25 * pod_d, absf(reach), 0.25 * pod_d),
                Vector3(p.x, centre.y + 0.5 * reach, p.z), CHASSIS_FRAME)

func _spheroid_mesh(diameter: float, length: float) -> ArrayMesh: ## sphere stretched along forward axis
    var ball := SphereMesh.new()
    ball.radius = 0.5 * diameter
    ball.height = diameter
    ball.radial_segments = 24
    ball.rings = 12
    var st := SurfaceTool.new()
    st.create_from(ball, 0)
    var mesh := st.commit()
    var arrays := mesh.surface_get_arrays(0)
    var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
    for i in verts.size():
        var v := verts[i]
        verts[i] = Vector3(v.x, v.z, v.y * length / diameter) # sphere Y becomes fore-aft axis
    arrays[Mesh.ARRAY_VERTEX] = verts
    var out := ArrayMesh.new()
    out.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
    return out

func _build_plane_chassis(body: Node3D, root: Node, size: Vector3, at: Vector3) -> void: ## fuselage only; surfaces draw wings
    _chassis_box(body, root, Vector3(size.x, size.y, 0.5 * size.z),
            at + Vector3(0, 0, 0.05 * size.z), CHASSIS_FRAME)
    _chassis_box(body, root, Vector3(0.72 * size.x, 0.72 * size.y, 0.3 * size.z),
            at + Vector3(0, 0, 0.35 * size.z), CHASSIS_DARK)
    _chassis_box(body, root, Vector3(0.4 * size.x, 0.5 * size.y, 0.4 * size.z),
            at + Vector3(0, 0.05 * size.y, -0.3 * size.z), CHASSIS_FRAME)

func _part_key(d: Dictionary) -> String:
    var keys: Array = d.keys().filter(func(k): return PARTS.has(k))
    if keys.size() > 1:
        push_warning("%s: one part per node, using '%s' and ignoring the rest" % [AP_EXT, keys[0]])
    return keys[0] if keys.size() > 0 else ""

static func _add_mesh(parent: Node3D, root: Node, mesh: Mesh, name: String, xform := Transform3D.IDENTITY) -> MeshInstance3D:
    var mi := MeshInstance3D.new()
    mi.name = name
    mi.mesh = mesh
    mi.transform = xform
    parent.add_child(mi)
    mi.owner = root
    mi.set_meta(GENERATED, true)
    return mi

func _add_part_visual(part: Component, root: Node) -> void: ## stand-in for mesh-less files
    if part is Rotor or part is HeliRotor:
        var radius: float = part.get("diameter") * 0.5
        var disc := CylinderMesh.new()
        disc.top_radius = radius
        disc.bottom_radius = radius
        disc.height = 0.006 if part is HeliRotor else 0.004
        _add_mesh(part, root, disc, "disc").material_override = \
                _mat(Color(0.85, 0.87, 0.9, 0.35), 1.0) # translucent so aircraft stays visible
    elif part is Surface:
        var surface := part as Surface # origin at quarter chord
        var c := surface.chord
        var cf := surface.flap_chord_fraction if surface.flap_servo > 0 else 0.0
        _add_mesh(surface, root, _airfoil_mesh(surface.span, c, c * (1.0 - cf), surface.cl0), "aerofoil")
        if cf > 0.0:
            var pivot := Node3D.new()
            pivot.name = Surface.FLAP_VISUAL
            pivot.position = Vector3(0, 0, -0.75 * c + cf * c)
            surface.add_child(pivot)
            pivot.owner = root
            pivot.set_meta(GENERATED, true)
            var flap := BoxMesh.new()
            flap.size = Vector3(surface.span, 0.012 * c / 0.25, cf * c)
            _add_mesh(pivot, root, flap, "flap", Transform3D(Basis(), Vector3(0, 0, -0.5 * cf * c)))
    elif part is Wheel:
        var wheel := part as Wheel
        var tyre := CylinderMesh.new()
        tyre.top_radius = wheel.radius
        tyre.bottom_radius = wheel.radius
        tyre.height = wheel.radius * 0.6
        _add_mesh(wheel, root, tyre, "tyre", Transform3D(Basis(Vector3.BACK, PI / 2), Vector3.ZERO))
    elif part is Floater:
        var flt := part as Floater
        var box := BoxMesh.new()
        var side: float = sqrt(maxf(flt.volume, 1e-6) / maxf(flt.height, 0.001))
        box.size = Vector3(side, flt.height, side)
        _add_mesh(flt, root, box, "displacement")

static func _airfoil_mesh(span: float, chord: float, kept: float, cl0: float) -> ArrayMesh: ## NACA-style section, span along X
    var thickness := 0.12
    var camber := clampf(cl0 * 0.1, 0.0, 0.06)
    var p := 0.4
    var samples := [0.0, 0.01, 0.03, 0.07, 0.13, 0.22, 0.34, 0.48, 0.63, 0.8, 1.0]
    var upper := PackedVector2Array()
    var lower := PackedVector2Array()
    for s in samples:
        var x: float = s * kept / chord
        var yt := 5.0 * thickness * (0.2969 * sqrt(x) - 0.126 * x - 0.3516 * x * x
                + 0.2843 * x * x * x - 0.1036 * x * x * x * x)
        var yc := camber / (p * p) * (2.0 * p * x - x * x) if x < p \
                else camber / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x)
        var z := 0.25 * chord - x * chord # z aft from leading edge
        upper.append(Vector2(z, (yc + yt) * chord))
        lower.append(Vector2(z, (yc - yt) * chord))
    var outline := PackedVector2Array() # closed: over top, back under
    for i in upper.size():
        outline.append(upper[i])
    for i in range(lower.size() - 1, 0, -1):
        outline.append(lower[i])

    var tris: Array[Vector3] = []
    var hs := 0.5 * span
    var n := outline.size()
    for i in n:
        var a := outline[i]
        var b := outline[(i + 1) % n]
        var al := Vector3(hs, a.y, a.x)
        var ar := Vector3(-hs, a.y, a.x)
        var bl := Vector3(hs, b.y, b.x)
        var br := Vector3(-hs, b.y, b.x)
        tris.append_array([al, bl, br, al, br, ar])
    for i in range(1, n - 1): # end caps, leading-edge fan
        tris.append_array([Vector3(hs, outline[0].y, outline[0].x),
                Vector3(hs, outline[i + 1].y, outline[i + 1].x),
                Vector3(hs, outline[i].y, outline[i].x),
                Vector3(-hs, outline[0].y, outline[0].x),
                Vector3(-hs, outline[i].y, outline[i].x),
                Vector3(-hs, outline[i + 1].y, outline[i + 1].x)])

    var st := SurfaceTool.new()
    st.begin(Mesh.PRIMITIVE_TRIANGLES)
    for i in range(0, tris.size(), 3): # both windings, visible either side
        st.add_vertex(tris[i])
        st.add_vertex(tris[i + 1])
        st.add_vertex(tris[i + 2])
        st.add_vertex(tris[i])
        st.add_vertex(tris[i + 2])
        st.add_vertex(tris[i + 1])
    st.generate_normals()
    return st.commit()

func _choose_vehicle(ap_doc: Dictionary, khr: Dictionary, ap: Dictionary, parent_of: Dictionary) -> int:
    if ap_doc.has("vehicle"):
        return int(ap_doc["vehicle"])
    for i in ap.keys(): # else body carrying first part
        if _part_key(ap[i]) == "":
            continue
        var j: int = i
        while true:
            if khr.has(j) and khr[j].has("motion"):
                return j
            if not parent_of.has(j):
                break
            j = parent_of[j]
    for i in khr.keys():
        if khr[i].has("motion"):
            return i
    return -1

func _depth(i: int, parent_of: Dictionary) -> int:
    var d := 0
    while parent_of.has(i):
        i = parent_of[i]
        d += 1
    return d

static func _body_above(n: Node) -> RigidBody3D: ## n itself or ancestor body
    while n != null:
        if n is RigidBody3D:
            return n
        n = n.get_parent()
    return null

static func _xform_to_root(n: Node, root: Node) -> Transform3D: ## valid outside the SceneTree
    var t := Transform3D.IDENTITY
    while n != null and n != root:
        if n is Node3D:
            t = n.transform * t
        n = n.get_parent()
    return t

func _replace_node(n: Node3D, replacement: Node3D, root: Node) -> Node3D: ## replacement unscaled: Jolt needs that
    var parent := n.get_parent()
    if parent == null:
        push_error("cannot replace scene root %s" % n.name)
        return n
    var local := n.transform
    var node_name := n.name
    replacement.transform = Transform3D(local.basis.orthonormalized(), local.origin)
    var residual := replacement.transform.affine_inverse() * local
    var is_empty := n.get_class() == "Node3D" and n.get_script() == null

    var idx := n.get_index()
    parent.remove_child(n)
    parent.add_child(replacement)
    parent.move_child(replacement, idx)
    replacement.name = node_name
    replacement.owner = root

    if is_empty:
        for c in n.get_children():
            _set_owner_recursive(c, null)
            n.remove_child(c)
            replacement.add_child(c)
            if c is Node3D:
                c.transform = residual * c.transform
            _set_owner_recursive(c, root)
        n.queue_free()
    else:
        _set_owner_recursive(n, null)
        n.name = String(n.name) + "_visual"
        replacement.add_child(n)
        n.transform = residual
        _set_owner_recursive(n, root)
    return replacement

static func _set_owner_recursive(n: Node, owner: Node) -> void:
    n.owner = owner
    for c in n.get_children():
        _set_owner_recursive(c, owner)

static func _v3(a, default: Vector3) -> Vector3:
    if a is Array and a.size() == 3:
        return Vector3(a[0], a[1], a[2])
    return default

func _apply_motion(body: RigidBody3D, m: Dictionary) -> void:
    body.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
    body.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
    body.linear_damp = 0.0
    body.angular_damp = 0.0

    var mass := float(m.get("mass", -1.0))
    if m.get("isKinematic", false) or mass == 0.0:
        body.freeze = true # glTF zero mass means infinite
        body.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
    elif mass > 0.0:
        body.mass = mass
    else:
        push_warning("%s: no mass given for %s, using %.1f kg" % [KHR_RB, body.name, body.mass])

    if m.has("inertiaDiagonal"):
        var inertia := _v3(m["inertiaDiagonal"], Vector3.ZERO)
        body.axis_lock_angular_x = inertia.x == 0.0 # zero inertia locks that axis
        body.axis_lock_angular_y = inertia.y == 0.0
        body.axis_lock_angular_z = inertia.z == 0.0
        body.inertia = Vector3(
                inertia.x if inertia.x > 0.0 else 1.0,
                inertia.y if inertia.y > 0.0 else 1.0,
                inertia.z if inertia.z > 0.0 else 1.0)
    if m.has("inertiaOrientation"):
        var q: Array = m["inertiaOrientation"]
        if not Quaternion(q[0], q[1], q[2], q[3]).is_equal_approx(Quaternion.IDENTITY):
            push_warning("%s: inertiaOrientation is not supported, using principal axes = body axes" % KHR_RB)
    if m.has("centerOfMass"):
        body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
        body.center_of_mass = _v3(m["centerOfMass"], Vector3.ZERO)
    body.gravity_scale = float(m.get("gravityFactor", 1.0))
    var b := body.transform.basis
    body.linear_velocity = b * _v3(m.get("linearVelocity"), Vector3.ZERO)
    body.angular_velocity = b * _v3(m.get("angularVelocity"), Vector3.ZERO)

func _find_body(n: Node, root: Node) -> PhysicsBody3D:
    while n != null and n != root:
        if n is PhysicsBody3D:
            return n
        n = n.get_parent()
    return null

func _add_collider(state: GLTFState, root: Node, n: Node3D, collider: Dictionary,
        shapes: Array, materials: Array, visualise: bool, replaced: Dictionary, index: int) -> void:
    var body := _find_body(n, root)
    if body == null:
        var sb := StaticBody3D.new() # no moving ancestor: static scenery
        sb.name = String(n.name) + "_static"
        root.add_child(sb)
        sb.owner = root
        var t := _xform_to_root(n, root)
        sb.transform = Transform3D(t.basis.orthonormalized(), t.origin)
        body = sb

    var rel := _xform_to_root(body, root).affine_inverse() * _xform_to_root(n, root)
    var scale := rel.basis.get_scale().abs()
    var geometry: Dictionary = collider.get("geometry", {})
    var shape: Shape3D = null
    var shape_mesh: Mesh = null
    if geometry.has("shape"):
        var s: Dictionary = shapes[int(geometry["shape"])]
        var r := _implicit_shape(s, scale)
        shape = r[0]
        shape_mesh = r[1]
    elif geometry.has("mesh"):
        shape = _mesh_shape(state, int(geometry["mesh"]), scale,
                geometry.get("convexHull", false) or body is RigidBody3D)
    if shape == null:
        push_warning("%s: unsupported collider geometry on %s" % [KHR_RB, n.name])
        return

    var cs := CollisionShape3D.new()
    var base := String(n.name)
    cs.shape = shape
    if n.get_parent() == body and n.get_child_count() == 0 \
            and n.get_class() == "Node3D" and n.get_script() == null:
        _replace_node(n, cs, root) # so export writes same node
        replaced[index] = cs
    else:
        cs.name = base + "_collider"
        body.add_child(cs)
        cs.owner = root
        cs.transform = Transform3D(rel.basis.orthonormalized(), rel.origin)

    if collider.has("physicsMaterial") and body.physics_material_override == null:
        var pm: Dictionary = materials[int(collider["physicsMaterial"])]
        var mat := PhysicsMaterial.new()
        mat.friction = float(pm.get("dynamicFriction", pm.get("staticFriction", 0.5)))
        mat.bounce = float(pm.get("restitution", 0.0))
        body.physics_material_override = mat

    if visualise and shape_mesh != null:
        var mi := MeshInstance3D.new()
        mi.name = base + "_mesh"
        mi.mesh = shape_mesh
        var see := _mat(Color(0.8, 0.85, 0.9, 0.08), 1.0) # faint, else ghosts the chassis
        see.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        mi.material_override = see
        cs.add_child(mi)
        mi.owner = root
        mi.set_meta(GENERATED, true)

func _implicit_shape(s: Dictionary, scale: Vector3) -> Array: ## returns [Shape3D, Mesh]
    var radial := maxf(scale.x, scale.z)
    match s.get("type", ""):
        "box":
            var size := _v3(s.get("box", {}).get("size"), Vector3.ONE) * scale
            var shape := BoxShape3D.new()
            shape.size = size
            var mesh := BoxMesh.new()
            mesh.size = size
            return [shape, mesh]
        "sphere":
            var r := float(s.get("sphere", {}).get("radius", 0.5)) * maxf(radial, scale.y)
            var shape := SphereShape3D.new()
            shape.radius = r
            var mesh := SphereMesh.new()
            mesh.radius = r
            mesh.height = 2.0 * r
            return [shape, mesh]
        "capsule", "cylinder":
            var type: String = s["type"]
            var p: Dictionary = s.get(type, {})
            var h := float(p.get("height", 0.5)) * scale.y
            var rt := float(p.get("radiusTop", 0.25)) * radial
            var rb := float(p.get("radiusBottom", 0.25)) * radial
            if type == "capsule":
                var r := maxf(rt, rb)
                if not is_equal_approx(rt, rb):
                    push_warning("%s: tapered capsule approximated with radius %.3f" % [KHR_SHAPES, r])
                var shape := CapsuleShape3D.new()
                shape.radius = r
                shape.height = h + 2.0 * r
                var capsule := CapsuleMesh.new()
                capsule.radius = r
                capsule.height = h + 2.0 * r
                return [shape, capsule]
            var mesh := CylinderMesh.new()
            mesh.top_radius = rt
            mesh.bottom_radius = rb
            mesh.height = h
            if is_equal_approx(rt, rb):
                var shape := CylinderShape3D.new()
                shape.radius = rt
                shape.height = h
                return [shape, mesh]
            var hull := ConvexPolygonShape3D.new() # cone or frustum
            hull.points = mesh.get_mesh_arrays()[Mesh.ARRAY_VERTEX]
            return [hull, mesh]
        "plane":
            var p: Dictionary = s.get("plane", {})
            if p.has("sizeX") and p.has("sizeZ"):
                var size := Vector3(float(p["sizeX"]) * scale.x, 0.001, float(p["sizeZ"]) * scale.z)
                var shape := BoxShape3D.new()
                shape.size = size
                var mesh := PlaneMesh.new()
                mesh.size = Vector2(size.x, size.z)
                return [shape, mesh]
            var shape := WorldBoundaryShape3D.new()
            var mesh := PlaneMesh.new()
            mesh.size = Vector2(200, 200)
            return [shape, mesh]
    push_warning("%s: unknown shape type '%s'" % [KHR_SHAPES, s.get("type", "")])
    return [null, null]

func _mesh_shape(state: GLTFState, mesh_idx: int, scale: Vector3, convex: bool) -> Shape3D:
    var meshes := state.get_meshes()
    if mesh_idx < 0 or mesh_idx >= meshes.size():
        return null
    var mesh := meshes[mesh_idx].mesh.get_mesh()
    if mesh == null:
        return null
    if convex:
        var hull := mesh.create_convex_shape(true, false) as ConvexPolygonShape3D
        var pts := hull.points
        for i in pts.size():
            pts[i] *= scale
        hull.points = pts
        return hull
    var tri := mesh.create_trimesh_shape() as ConcavePolygonShape3D
    var faces := tri.get_faces()
    for i in faces.size():
        faces[i] *= scale
    tri.set_faces(faces)
    return tri

func _export_preflight(_state: GLTFState, _root: Node) -> Error:
    _export_shapes.clear()
    _export_materials.clear()
    _export_vehicle = null
    _export_nodes.clear()
    _export_joints.clear()
    return OK

func _export_node(state: GLTFState, gltf_node: GLTFNode, json: Dictionary, node: Node) -> Error:
    var ext: Dictionary = json.get("extensions", {})
    for omi in OMI_EXTS: # Godot wrote these; avoid duplication
        ext.erase(omi)
    var khr := {}
    var ap := {}

    if node is RigidBody3D:
        khr["motion"] = _motion_to_gltf(node)
        if node is Vehicle:
            _export_vehicle = gltf_node
            if node.drag_area != Vector3.ZERO:
                var a: Vector3 = node.drag_area
                ap["drag"] = {"area": [a.x, a.y, a.z], "airDensity": node.air_density}
            if node.water_enabled:
                ap["water"] = {"level": node.water_level, "density": node.water_density}
    elif node is CollisionShape3D and node.shape != null:
        var collider := _collider_to_gltf(node)
        if not collider.is_empty():
            khr["collider"] = collider
    elif node is Component:
        ap[node.gltf_key()] = node.to_gltf()
        if node is Hinge:
            var r: Transform3D = node.rest_transform() # pivot at zero deflection
            var q := r.basis.get_rotation_quaternion()
            var sc := r.basis.get_scale()
            json.erase("matrix")
            json["translation"] = [r.origin.x, r.origin.y, r.origin.z]
            json["rotation"] = [q.x, q.y, q.z, q.w]
            json["scale"] = [sc.x, sc.y, sc.z]

    _export_nodes[node] = gltf_node
    if node.has_meta(CHASSIS):
        ap["chassis"] = node.get_meta(CHASSIS)
    if node.has_meta(JOINT_TO) and node.get_meta(JOINT_TO) != null:
        _export_joints.append([gltf_node, node.get_meta(JOINT_TO)])

    if not khr.is_empty():
        ext[KHR_RB] = khr
        state.add_used_extension(KHR_RB, false)
    if not ap.is_empty():
        ext[AP_EXT] = ap
        state.add_used_extension(AP_EXT, false)
    if ext.is_empty():
        json.erase("extensions")
    else:
        json["extensions"] = ext
    return OK

func _export_post(state: GLTFState) -> Error:
    var json: Dictionary = state.json
    var ext: Dictionary = json.get("extensions", {})
    if not _export_shapes.is_empty():
        ext[KHR_SHAPES] = {"shapes": _export_shapes}
        var used := Array(json.get("extensionsUsed", []))
        if not KHR_SHAPES in used:
            json["extensionsUsed"] = used + [KHR_SHAPES]
    var khr_doc := {}
    if not _export_materials.is_empty():
        khr_doc["physicsMaterials"] = _export_materials
    var nodes := state.get_nodes() # joints need final node indices
    var json_nodes: Array = json.get("nodes", [])
    for pair in _export_joints:
        var at := nodes.find(pair[0])
        var to := nodes.find(_export_nodes.get(pair[1]))
        if at < 0 or to < 0:
            continue
        var node_ext: Dictionary = json_nodes[at].get("extensions", {})
        var node_khr: Dictionary = node_ext.get(KHR_RB, {})
        node_khr["joint"] = {"connectedNode": to, "joint": 0}
        node_ext[KHR_RB] = node_khr
        json_nodes[at]["extensions"] = node_ext
        khr_doc["physicsJoints"] = [{}]
    if not khr_doc.is_empty():
        ext[KHR_RB] = khr_doc
    var vehicle_idx := state.get_nodes().find(_export_vehicle) if _export_vehicle != null else -1
    if vehicle_idx >= 0:
        ext[AP_EXT] = {"vehicle": vehicle_idx}
    for omi in OMI_EXTS:
        ext.erase(omi)
    if not ext.is_empty():
        json["extensions"] = ext
    var used := Array(json.get("extensionsUsed", []))
    json["extensionsUsed"] = used.filter(func(e): return not e in OMI_EXTS)
    return OK

func _motion_to_gltf(body: RigidBody3D) -> Dictionary:
    var m := {}
    if body.freeze:
        m["isKinematic"] = true
    m["mass"] = body.mass
    if body.inertia != Vector3.ZERO:
        var i := body.inertia
        m["inertiaDiagonal"] = [
            0.0 if body.axis_lock_angular_x else i.x,
            0.0 if body.axis_lock_angular_y else i.y,
            0.0 if body.axis_lock_angular_z else i.z]
    if body.center_of_mass_mode == RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM:
        var c := body.center_of_mass
        m["centerOfMass"] = [c.x, c.y, c.z]
    if body.gravity_scale != 1.0:
        m["gravityFactor"] = body.gravity_scale
    return m

func _collider_to_gltf(cs: CollisionShape3D) -> Dictionary:
    var s := cs.shape
    var d := {}
    if s is BoxShape3D:
        d = {"type": "box", "box": {"size": [s.size.x, s.size.y, s.size.z]}}
    elif s is SphereShape3D:
        d = {"type": "sphere", "sphere": {"radius": s.radius}}
    elif s is CapsuleShape3D:
        d = {"type": "capsule", "capsule": {"height": s.height - 2.0 * s.radius,
                "radiusTop": s.radius, "radiusBottom": s.radius}}
    elif s is CylinderShape3D:
        d = {"type": "cylinder", "cylinder": {"height": s.height,
                "radiusTop": s.radius, "radiusBottom": s.radius}}
    elif s is WorldBoundaryShape3D:
        d = {"type": "plane", "plane": {}}
    else:
        push_warning("%s: %s on %s not exported" % [KHR_SHAPES, s.get_class(), cs.name])
        return {}
    var collider := {"geometry": {"shape": _export_shapes.size()}}
    _export_shapes.append(d)

    var body := cs.get_parent() as PhysicsBody3D
    var pm: PhysicsMaterial = null
    if body is RigidBody3D or body is StaticBody3D:
        pm = body.physics_material_override
    if pm != null:
        var md := {"staticFriction": pm.friction, "dynamicFriction": pm.friction,
                "restitution": pm.bounce}
        var idx := _export_materials.find(md)
        if idx < 0:
            idx = _export_materials.size()
            _export_materials.append(md)
        collider["physicsMaterial"] = idx
    return collider
