# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
extends EditorPlugin # glTF vehicle editor for SITL

const GLTFPhysics := preload("res://scripts/gltf_physics.gd")
const Report := preload("res://scripts/vehicle_report.gd")
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Gizmos := preload("res://addons/ardupilot_sitl/gizmos.gd")

const MENU_REPORT := "ArduPilot: vehicle report"
const MENU_CONVERT := "ArduPilot: convert glTF vehicle to scene..."
const MENU_EXPORT := "ArduPilot: export vehicle as glTF..."

var _ext: GLTFPhysics
var _gizmos: Gizmos
var _open_dialog: EditorFileDialog
var _save_dialog: EditorFileDialog
var _report_dialog: AcceptDialog

func _enter_tree() -> void:
    _ext = GLTFPhysics.new()
    if not GLTFPhysics._register(_ext):
        push_warning("ArduPilot glTF physics extension already registered")
    _gizmos = Gizmos.new()
    add_node_3d_gizmo_plugin(_gizmos)
    add_tool_menu_item(MENU_REPORT, _on_report)
    add_tool_menu_item(MENU_CONVERT, _on_convert)
    add_tool_menu_item(MENU_EXPORT, _on_export)

    var base := EditorInterface.get_base_control()
    _open_dialog = EditorFileDialog.new()
    _open_dialog.file_mode = EditorFileDialog.FILE_MODE_OPEN_FILE
    _open_dialog.filters = PackedStringArray(["*.gltf, *.glb ; glTF vehicle"])
    _open_dialog.file_selected.connect(_on_convert_selected)
    base.add_child(_open_dialog)
    _save_dialog = EditorFileDialog.new()
    _save_dialog.file_mode = EditorFileDialog.FILE_MODE_SAVE_FILE
    _save_dialog.access = EditorFileDialog.ACCESS_FILESYSTEM
    _save_dialog.filters = PackedStringArray(["*.gltf ; glTF JSON", "*.glb ; glTF binary"])
    _save_dialog.file_selected.connect(_on_export_selected)
    base.add_child(_save_dialog)
    _report_dialog = AcceptDialog.new()
    _report_dialog.title = "Vehicle report"
    base.add_child(_report_dialog)

func _exit_tree() -> void:
    remove_tool_menu_item(MENU_REPORT)
    remove_tool_menu_item(MENU_CONVERT)
    remove_tool_menu_item(MENU_EXPORT)
    remove_node_3d_gizmo_plugin(_gizmos)
    if GLTFPhysics.registered == _ext:
        GLTFPhysics._unregister(_ext)
    _open_dialog.queue_free()
    _save_dialog.queue_free()
    _report_dialog.queue_free()

static func _find_vehicle(n: Node) -> Vehicle:
    if n is Vehicle:
        return n
    for c in n.get_children():
        var v := _find_vehicle(c)
        if v:
            return v
    return null

func _on_report() -> void:
    var root := EditorInterface.get_edited_scene_root()
    var vehicle := _find_vehicle(root) if root else null
    if vehicle == null:
        push_error("the open scene has no ArduPilotVehicle")
        return
    var text := Report.report(vehicle)
    print(text)
    _report_dialog.dialog_text = text
    _report_dialog.popup_centered()

func _on_convert() -> void:
    _open_dialog.popup_file_dialog()

func _on_convert_selected(path: String) -> void: # saves an editable scene (.tscn)
    var root := _ext.load_gltf(ProjectSettings.globalize_path(path))
    if root == null:
        return
    var scene := PackedScene.new()
    var err := scene.pack(root)
    var out := path.get_basename() + ".tscn"
    if err == OK:
        err = ResourceSaver.save(scene, out)
    root.free()
    if err != OK:
        push_error("saving %s failed: %s" % [out, error_string(err)])
        return
    EditorInterface.get_resource_filesystem().scan()
    EditorInterface.open_scene_from_path(out)

func _on_export() -> void:
    if EditorInterface.get_edited_scene_root() == null:
        push_error("open the vehicle scene to export first")
        return
    _save_dialog.popup_file_dialog()

func _on_export_selected(path: String) -> void:
    var err := _ext.save_gltf(EditorInterface.get_edited_scene_root(), path)
    if err != OK:
        push_error("export to %s failed: %s" % [path, error_string(err)])
    else:
        print("exported ", path)
