#!/usr/bin/env python3
'''
script to convert a kml file to a lua script with sets of polygons and circles
use in conjunction with AP_Scripting/applets/kml_fence.lua
'''

import argparse
import math
import re
import fnmatch
from xml.etree import ElementTree as ET
from xml.dom.minidom import Document
from statistics import mean
from rdp import rdp

radius_of_earth = 6371000  # meters

def gps_distance(lat1, lon1, lat2, lon2):
    from math import radians, cos, tan, pi, log, sqrt
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    if abs(dlat) < 1e-15:
        q = cos(lat1)
    else:
        q = dlat / log(tan(lat2 / 2 + pi / 4) / tan(lat1 / 2 + pi / 4))
    return radius_of_earth * math.sqrt(dlat**2 + q**2 * dlon**2)

def gps_point_to_segment_distance(p, a, b):
    def to_xy(lat, lon):
        x = math.radians(lon) * radius_of_earth * math.cos(math.radians(lat))
        y = math.radians(lat) * radius_of_earth
        return x, y
    px, py = to_xy(*p)
    ax, ay = to_xy(*a)
    bx, by = to_xy(*b)
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)

def simplify_polygon(coords, tolerance_meters):
    return coords if len(coords) < 3 else rdp(coords, epsilon=tolerance_meters, dist=gps_point_to_segment_distance)

def parse_coords(text):
    return [(float(p[1]), float(p[0])) for l in text.split() if (p := l.split(',')) and len(p) >= 2]

def try_detect_circle(points, tolerance_meters=10.0):
    if len(points) < 10: return None
    clat = mean(p[0] for p in points)
    clon = mean(p[1] for p in points)
    dists = [gps_distance(p[0], p[1], clat, clon) for p in points]
    avg = mean(dists)
    if max(abs(d - avg) for d in dists) < tolerance_meters:
        return (clat, clon, avg)
    return None

def format_circle(name, clat, clon, r): return {"name": name, "type": "circle", "data": (clat, clon, r)}
def format_polygon(name, coords): return {"name": name, "type": "polygon", "data": coords}
def point_in_polygon(polygon, lat, lon):
    inside, n = False, len(polygon)
    for i in range(n):
        j = (i - 1) % n
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        if ((xi > lon) != (xj > lon)) and (lat < (yi - yj) * (lon - xj) / (xi - xj + 1e-15) + yj):
            inside = not inside
    return inside

def is_excluded(name, patterns): return any(fnmatch.fnmatch(name, pat) for pat in patterns)
def is_circle_contained(c1, c2): return gps_distance(*c1[:2], *c2[:2]) + c1[2] < c2[2]
def is_polygon_in_circle(poly, c): return all(gps_distance(lat, lon, c[0], c[1]) <= c[2] for lat, lon in poly)
def is_polygon_inside(p1, p2): return all(point_in_polygon(p2, lat, lon) for lat, lon in p1)

def remove_nested_nfz(zones):
    keep, removed = [], []
    for i, z1 in enumerate(zones):
        if "NFZ" not in z1["name"]: keep.append(z1); continue
        for j, z2 in enumerate(zones):
            if i == j or "NFZ" not in z2["name"]: continue
            if z1["type"] == "circle" and z2["type"] == "circle" and is_circle_contained(z1["data"], z2["data"]): break
            if z1["type"] == "polygon" and z2["type"] == "circle" and is_polygon_in_circle(z1["data"], z2["data"]): break
            if z1["type"] == "polygon" and z2["type"] == "polygon" and is_polygon_inside(z1["data"], z2["data"]): break
            if z1["type"] == "circle" and z2["type"] == "polygon":
                clat, clon, r = z1["data"]
                dlat = r / radius_of_earth * (180 / math.pi)
                test_pt = (clat + dlat, clon)
                if point_in_polygon(z2["data"], clat, clon) and point_in_polygon(z2["data"], *test_pt): break
        else:
            keep.append(z1)
            continue
        removed.append(z1)
    return keep, removed

def generate_kml(path, kept, removed):
    doc = Document()
    kml = doc.createElement("kml")
    kml.setAttribute("xmlns", "http://www.opengis.net/kml/2.2")
    doc.appendChild(kml)
    root = doc.createElement("Document")
    kml.appendChild(root)

    def add_style(id, hexcolor):
        s = doc.createElement("Style"); s.setAttribute("id", id)
        ls, ps = doc.createElement("LineStyle"), doc.createElement("PolyStyle")
        for tag, val in [("color", hexcolor), ("width", "2")]:
            e = doc.createElement(tag); e.appendChild(doc.createTextNode(val))
            ls.appendChild(e) if tag == "width" else ps.appendChild(e)
        s.appendChild(ls); s.appendChild(ps); root.appendChild(s)

    add_style("kept", "0f00ff00"); add_style("excluded", "0f0000ff")

    def add_zone(z, style):
        pm = doc.createElement("Placemark")
        name = doc.createElement("name"); name.appendChild(doc.createTextNode(z["name"]))
        pm.appendChild(name)
        su = doc.createElement("styleUrl"); su.appendChild(doc.createTextNode(f"#{style}")); pm.appendChild(su)
        poly = z["data"]
        if z["type"] == "circle":
            lat, lon, r = poly
            poly = [(lat + (r / radius_of_earth * 180 / math.pi) * math.sin(a),
                     lon + (r / radius_of_earth * 180 / math.pi) * math.cos(a) / math.cos(math.radians(lat)))
                    for a in [2 * math.pi * i / 36 for i in range(36)]]
        coords = " ".join(f"{lon},{lat},0" for lat, lon in poly + [poly[0]])
        poly_el = doc.createElement("Polygon")
        outer = doc.createElement("outerBoundaryIs")
        ring = doc.createElement("LinearRing")
        coord_el = doc.createElement("coordinates")
        coord_el.appendChild(doc.createTextNode(coords))
        ring.appendChild(coord_el)
        outer.appendChild(ring)
        poly_el.appendChild(outer)
        pm.appendChild(poly_el)
        root.appendChild(pm)

    for z in kept: add_zone(z, "kept")
    for z in removed: add_zone(z, "excluded")

    with open(path, "w") as f: f.write(doc.toprettyxml(indent="  "))

def parse_kml_to_lua(path, ct, pt, excludes, remove_nested, kml_output):
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    tree = ET.parse(path)
    root = tree.getroot()
    zones, excluded = [], []
    for placemark in root.findall(".//kml:Placemark", ns):
        name_elem = placemark.find("kml:name", ns)
        if name_elem is None: continue
        name = name_elem.text.strip()
        if is_excluded(name, excludes): continue
        coord_elem = placemark.find(".//kml:Polygon//kml:coordinates", ns)
        if coord_elem is None: continue
        coords = parse_coords(coord_elem.text.strip())
        circ = try_detect_circle(coords, ct)
        zones.append(format_circle(name, *circ) if circ else format_polygon(name, simplify_polygon(coords, pt)))
    if remove_nested:
        zones, excluded = remove_nested_nfz(zones)
    if kml_output:
        generate_kml(kml_output, zones, excluded)
    lua = "local polygons = {"
    for i, z in enumerate(zones):
        sep = "\n," if i < len(zones) - 1 else "\n"
        if z["type"] == "circle":
            lat, lon, r = z["data"]
            lua += f'  \n{{\n name = "{z["name"]}",\n circle = {{ lat = {lat:.7f}, lon = {lon:.7f}, radius = {r:.2f} }}\n }}{sep}'
        else:
            lines = ",\n".join(f"      {{ {lat:.7f}, {lon:.7f} }}" for lat, lon in z["data"])
            lua += f'  \n{{\n name = "{z["name"]}", polygon = {{\n{lines}\n    }} }}{sep}'
    lua += '''}

return polygons
'''
    return lua

def main():
    parser = argparse.ArgumentParser(description="Convert and reduce KML polygons to Lua and/or KML.")
    parser.add_argument("kml_file")
    parser.add_argument("--circle-tolerance", type=float, default=10.0)
    parser.add_argument("--polygon-tolerance", type=float, default=5.0)
    parser.add_argument("--exclude", default="", help="Comma-separated wildcard patterns")
    parser.add_argument("--exclude-nested-nfz", action="store_true")
    parser.add_argument("--generate-kml", type=str)
    parser.add_argument("--output", type=str, default="fence.lua")
    args = parser.parse_args()
    patterns = [x.strip() for x in args.exclude.split(",") if x.strip()]
    result = parse_kml_to_lua(args.kml_file, args.circle_tolerance, args.polygon_tolerance, patterns, args.exclude_nested_nfz, args.generate_kml)
    open(args.output, "w").write(result)
    print(f"Created {args.output}")

if __name__ == "__main__":
    main()
