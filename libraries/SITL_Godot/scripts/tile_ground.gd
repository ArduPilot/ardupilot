# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

class_name ArduPilotTileGround
extends Node # satellite tile ground around home

const TILE := 256
const GRASS := Color(0.47, 0.60, 0.40)
const SLOTS := 4
const MAX_GRID := 25
const GRID_M := 10.0 # finest grid through home (m)
const GRID_PX := 40.0 # minimum line spacing (composite pixels)
const GRID_LINE := Color(0.10, 0.10, 0.10) # solid keeps a hard edge
const EQUATOR_MPP := 156543.03392 # zoom 0, 256-pixel tiles (m/pixel)
const ESRI := "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

var texture: ImageTexture
var attribution := ""

var _image: Image
var _url := ""
var _zoom := 18
var _tx0 := 0
var _ty0 := 0
var _grid := Vector2i.ZERO
var _home_px := Vector2.ZERO # home in composite pixels
var _mpp := 1.0 # metres per pixel at home
var _queue: Array[Vector2i] = []
var _cache_dir := ""
var _material: StandardMaterial3D
var _fetched := 0
var _failed := 0
var _retried := {}

func setup(lat_deg: float, lon_deg: float, zoom: int, radius_m: float, source: String) -> void:
    _url = source if source.begins_with("http") else ESRI
    attribution = ("Imagery: Esri World Imagery" if _url == ESRI
            else "Imagery: " + _url.get_slice("/", 2))
    _zoom = clampi(zoom, 1, 22)
    var n := float(1 << _zoom)
    var lat := deg_to_rad(lat_deg)
    var tx := (lon_deg + 180.0) / 360.0 * n
    var ty := (1.0 - log(tan(lat) + 1.0 / cos(lat)) / PI) / 2.0 * n
    _mpp = EQUATOR_MPP * cos(lat) / n
    var r := clampi(int(ceil(radius_m / (_mpp * TILE))), 0, (MAX_GRID - 1) / 2)
    _tx0 = int(floor(tx)) - r
    _ty0 = int(floor(ty)) - r
    _grid = Vector2i(2 * r + 1, 2 * r + 1)
    _home_px = Vector2((tx - _tx0) * TILE, (ty - _ty0) * TILE)

    _image = Image.create(_grid.x * TILE, _grid.y * TILE, false, Image.FORMAT_RGB8)
    _image.fill(GRASS)
    texture = ImageTexture.create_from_image(_image)

    _cache_dir = "user://tiles/%s/%d" % [_url.get_slice("/", 2).validate_filename(), _zoom]
    DirAccess.make_dir_recursive_absolute(_cache_dir)

    var centre := Vector2(tx - _tx0, ty - _ty0)
    for j in _grid.y:
        for i in _grid.x:
            _queue.append(Vector2i(_tx0 + i, _ty0 + j))
    _queue.sort_custom(func(a, b):
        return (Vector2(a.x - _tx0, a.y - _ty0) + Vector2(0.5, 0.5)).distance_to(centre) \
                < (Vector2(b.x - _tx0, b.y - _ty0) + Vector2(0.5, 0.5)).distance_to(centre))

    var misses: Array[Vector2i] = [] # cached tiles land at once
    for t in _queue:
        var bytes := FileAccess.get_file_as_bytes(_cache_path(t))
        if bytes.is_empty() or not _blit(t, bytes):
            misses.append(t)
    _queue = misses
    _grid_overlay(Rect2i(0, 0, _image.get_width(), _image.get_height()))
    texture.update(_image)
    print("satellite imagery: %dx%d tiles at zoom %d, %.2f m/pixel, %.0f m grid, %d cached" %
            [_grid.x, _grid.y, _zoom, _mpp, _grid_spacing(),
            _grid.x * _grid.y - _queue.size()])

    for slot in SLOTS:
        var req := HTTPRequest.new()
        req.timeout = 15.0
        add_child(req)
        req.request_completed.connect(_on_completed.bind(req))
        _start_next(req)

func world_to_uv(x: float, z: float) -> Vector2:
    var px := _home_px.x - x / _mpp # +Z north, -X east
    var py := _home_px.y - z / _mpp # tile rows grow southward
    return Vector2(px / (_grid.x * TILE), py / (_grid.y * TILE))

func ground_material() -> StandardMaterial3D: # shared by the ground pieces
    if _material == null:
        _material = StandardMaterial3D.new()
        _material.albedo_color = Color(1.35, 1.35, 1.35) # lighter, so dark vehicles show
        _material.albedo_texture = texture
        _material.texture_repeat = true # gives motion cues beyond imagery
    return _material

func _cache_path(t: Vector2i) -> String:
    return "%s/%d_%d" % [_cache_dir, t.x, t.y]

func _start_next(req: HTTPRequest) -> void:
    if _queue.is_empty():
        if _failed > 0:
            print("satellite imagery: %d tile(s) not fetched, ground stays green there" % _failed)
            _failed = 0
        elif _fetched > 0: # mipmaps calm the distant ground
            _fetched = 0
            _image.generate_mipmaps()
            texture.set_image(_image)
        return
    var t: Vector2i = _queue.pop_front()
    var url := _url.replace("{z}", str(_zoom)).replace("{x}", str(t.x)).replace("{y}", str(t.y))
    req.set_meta("tile", t)
    if req.request(url) != OK:
        _failed += 1
        _start_next(req)

func _on_completed(result: int, code: int, _headers: PackedStringArray,
        body: PackedByteArray, req: HTTPRequest) -> void:
    var t: Vector2i = req.get_meta("tile")
    if result == HTTPRequest.RESULT_SUCCESS and code == 200 and not body.is_empty() and _blit(t, body):
        _fetched += 1
        _grid_overlay(Rect2i(Vector2i((t.x - _tx0) * TILE, (t.y - _ty0) * TILE), Vector2i(TILE, TILE)))
        texture.update(_image)
        var f := FileAccess.open(_cache_path(t), FileAccess.WRITE)
        if f:
            f.store_buffer(body)
    elif not _retried.has(t): # one retry for slow servers
        _retried[t] = true
        _queue.append(t)
    else:
        _failed += 1
    _start_next(req)

func _blit(t: Vector2i, bytes: PackedByteArray) -> bool:
    var img := Image.new()
    var err := ERR_INVALID_DATA
    if bytes.size() > 2 and bytes[0] == 0xFF and bytes[1] == 0xD8:
        err = img.load_jpg_from_buffer(bytes)
    elif bytes.size() > 2 and bytes[0] == 0x89 and bytes[1] == 0x50:
        err = img.load_png_from_buffer(bytes)
    else:
        err = img.load_webp_from_buffer(bytes)
    if err != OK:
        return false
    if img.get_width() != TILE or img.get_height() != TILE:
        img.resize(TILE, TILE)
    img.convert(Image.FORMAT_RGB8)
    _image.blit_rect(img, Rect2i(0, 0, TILE, TILE),
            Vector2i((t.x - _tx0) * TILE, (t.y - _ty0) * TILE))
    var w := _image.get_width()
    var h := _image.get_height()
    for edge in [Rect2i(0, 0, w, 2), Rect2i(0, h - 2, w, 2), Rect2i(0, 0, 2, h), Rect2i(w - 2, 0, 2, h)]: # keep the border green
        _image.fill_rect(edge, GRASS)
    return true

func _grid_spacing() -> float: # (m), GRID_M stepped 1/2/5
    const STEPS := [1.0, 2.0, 5.0]
    var m := GRID_M
    var i := 0
    while m / _mpp < GRID_PX and i < 12:
        i += 1
        m = GRID_M * STEPS[i % 3] * pow(10.0, floorf(i / 3.0))
    return m

func _grid_overlay(region: Rect2i) -> void: # lines through home, within region
    var step := _grid_spacing() / _mpp
    for axis in 2:
        var origin := _home_px.x if axis == 0 else _home_px.y
        var lo := region.position.x if axis == 0 else region.position.y
        var hi := region.end.x if axis == 0 else region.end.y
        var p := origin + ceilf((lo - origin) / step) * step
        while p < hi:
            var c := int(p)
            if c >= lo and c < hi:
                for q in range(region.position.y if axis == 0 else region.position.x,
                        region.end.y if axis == 0 else region.end.x):
                    var at := Vector2i(c, q) if axis == 0 else Vector2i(q, c)
                    _image.set_pixelv(at, GRID_LINE)
            p += step
