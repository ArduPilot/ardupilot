# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validated singleton constants used by the AP_Scripting examples."""

import re

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

GROUPS = (
    dict(owner='AP_GPS', rename='gps', depends='AP_GPS_ENABLED', scheduler_semaphore=False,
         constants=(
             ('NO_GPS', 'GPS_NO_GPS', 0, 'NO_GPS'),
             ('NO_FIX', 'GPS_NO_FIX', 1, 'NONE'),
             ('GPS_OK_FIX_2D', 'GPS_OK_FIX_2D', 2, 'FIX_2D'),
             ('GPS_OK_FIX_3D', 'GPS_OK_FIX_3D', 3, 'FIX_3D'),
             ('GPS_OK_FIX_3D_DGPS', 'GPS_OK_FIX_3D_DGPS', 4, 'DGPS'),
             ('GPS_OK_FIX_3D_RTK_FLOAT', 'GPS_OK_FIX_3D_RTK_FLOAT', 5, 'RTK_FLOAT'),
             ('GPS_OK_FIX_3D_RTK_FIXED', 'GPS_OK_FIX_3D_RTK_FIXED', 6, 'RTK_FIXED'),
         )),
    dict(owner='AP_Terrain', rename='terrain',
         depends='defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1', scheduler_semaphore=False,
         constants=(
             ('TerrainStatusDisabled', 'TERRAIN_STATUS_DISABLED', 0, None),
             ('TerrainStatusUnhealthy', 'TERRAIN_STATUS_UNHEALTHY', 1, None),
             ('TerrainStatusOK', 'TERRAIN_STATUS_OK', 2, None),
         )),
    dict(owner='AP_Mission', rename='mission', depends='AP_MISSION_ENABLED', scheduler_semaphore=True,
         constants=(
             ('MISSION_STOPPED', 'MISSION_STOPPED', 0, None),
             ('MISSION_RUNNING', 'MISSION_RUNNING', 1, None),
             ('MISSION_COMPLETE', 'MISSION_COMPLETE', 2, None),
         )),
)


def _without_comments(text):
    return re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.DOTALL)


def _enum_values(text, enum_name):
    text = _without_comments(text)
    blocks = re.findall(rf'\benum(?:\s+class)?\s+{re.escape(enum_name)}(?:\s*:\s*[^{{]+)?\s*{{([^}}]*)}}',
                        text, flags=re.DOTALL)
    if len(blocks) != 1:
        raise ValueError(f'unaudited {enum_name} declaration')
    values = {}
    for declaration in blocks[0].split(','):
        declaration = declaration.strip()
        if not declaration:
            continue
        match = re.fullmatch(r'([A-Za-z_]\w*)\s*=\s*([+-]?\d+)', declaration)
        if match is None or match.group(1) in values:
            raise ValueError(f'unaudited {enum_name} member')
        values[match.group(1)] = int(match.group(2))
    return values


def _descriptor_enums(text, owner):
    text = _without_comments(text)
    matches = re.findall(rf'^\s*singleton\s+{re.escape(owner)}\s+enum\s+([^\n]+?)\s*$',
                         text, flags=re.MULTILINE)
    if len(matches) != 1:
        raise ValueError(f'unaudited {owner} enum descriptor')
    return tuple(matches[0].split())


def _gps_aliases(text):
    text = _without_comments(text)
    matches = re.findall(
        r'\bstatic\s+constexpr\s+uint8_t\s+([A-Za-z_]\w*)\s*=\s*'
        r'\(\s*uint8_t\s*\)\s*AP_GPS_FixType::([A-Za-z_]\w*)\s*;', text)
    aliases = {}
    for name, target in matches:
        if name in aliases:
            raise ValueError('duplicate AP_GPS fix-type alias')
        aliases[name] = target
    return aliases


def _validate_owner(ir, group):
    owners = [item for item in ir['singletons'] if item['name'] == group['owner']]
    if len(owners) != 1:
        raise ValueError(f'missing or duplicate {group["owner"]} singleton')
    owner = owners[0]
    expected = dict(rename=group['rename'], depends=group['depends'], get=None, literal=False, reference=False,
                    semaphore=False, scheduler_semaphore=group['scheduler_semaphore'], semaphore_pointer=False)
    if any(owner[key] != value for key, value in expected.items()):
        raise ValueError(f'unaudited {group["owner"]} owner metadata')


def select(ir, root=ROOT):
    """Validate Lua declarations and their C++ values, then return Rust constants."""
    root = Path(root)
    descriptor = (root / 'libraries/AP_Scripting/generator/description/bindings.desc').read_text()
    sources = {
        'AP_GPS': (root / 'libraries/AP_GPS/AP_GPS.h').read_text(),
        'AP_GPS_FixType': (root / 'libraries/AP_GPS/AP_GPS_FixType.h').read_text(),
        'AP_Terrain': (root / 'libraries/AP_Terrain/AP_Terrain.h').read_text(),
        'AP_Mission': (root / 'libraries/AP_Mission/AP_Mission.h').read_text(),
    }
    enum_names = {'AP_Terrain': 'TerrainStatus', 'AP_Mission': 'mission_state'}
    for group in GROUPS:
        _validate_owner(ir, group)
        declared = tuple(item[0] for item in group['constants'])
        if _descriptor_enums(descriptor, group['owner']) != declared:
            raise ValueError(f'unaudited {group["owner"]} enum constants')
        if group['owner'] == 'AP_GPS':
            aliases = _gps_aliases(sources['AP_GPS'])
            fix_values = _enum_values(sources['AP_GPS_FixType'], 'AP_GPS_FixType')
            for source_name, _, value, target in group['constants']:
                if aliases.get(source_name) != target or fix_values.get(target) != value:
                    raise ValueError(f'unaudited AP_GPS constant {source_name}')
        else:
            values = _enum_values(sources[group['owner']], enum_names[group['owner']])
            for source_name, _, value, _ in group['constants']:
                if values.get(source_name) != value:
                    raise ValueError(f'unaudited {group["owner"]} constant {source_name}')
    return specification()


def specification():
    return dict(constants=[dict(name=rust_name, value=value)
                           for group in GROUPS for _, rust_name, value, _ in group['constants']])


def rust(spec, notice):
    constants = spec['constants']
    expected = specification()['constants']
    if constants != expected:
        raise ValueError('unaudited example singleton constants specification')
    lines = [notice.rstrip(), ''] if notice else []
    lines += ['/// GPS fix-status constants exposed to Lua by `gps`.']
    for item in constants[:7]:
        lines.append(f'pub const {item["name"]}: u8 = {item["value"]};')
    lines += ['', '/// Terrain status constants exposed to Lua by `terrain`.']
    for item in constants[7:10]:
        lines.append(f'pub const {item["name"]}: u8 = {item["value"]};')
    lines += ['', '/// Mission state constants exposed to Lua by `mission`.']
    for item in constants[10:]:
        lines.append(f'pub const {item["name"]}: u8 = {item["value"]};')
    return '\n'.join(lines) + '\n'
