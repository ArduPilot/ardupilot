#!/usr/bin/env python3

'''
Tool to convert parameter names and scales, useful for conversion for cm -> m and cdeg -> deg.

Mission Planner (NAME,VALUE), MAVProxy (NAME VALUE) and QGroundControl
(SYSID COMPID NAME VALUE TYPE) parameter files are supported. Comments, blank
lines, separators and untouched lines are preserved as they are.

This looks for files called *.param, *.params or *.parm.

Example:
  Tools/scripts/convert_param_scale.py --scale 0.1 TRIM_ARSPD_CM AIRSPEED_CRUISE

The parsing and rewriting helpers of this module are also used by other
conversion scripts, e.g. convert_params_4_7.py.

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import os
import re
import shutil
import tempfile

from typing import Callable
from typing import Dict
from typing import Iterator
from typing import List
from typing import Optional
from typing import Union

PARAM_FILE_EXTENSIONS = ('.param', '.params', '.parm')

# Maximum parameter name length accepted by AP_Param.
AP_MAX_NAME_SIZE = 16

# Default number of significant digits kept when a value is rescaled.
DEFAULT_SIG_DIGITS = 3

# MAV_PARAM_TYPE values used in the type column of QGroundControl files.
MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9

# Parameter names are upper case, but ground stations also accept lower case files.
_NAME = r'[A-Za-z][A-Za-z0-9_]*'
_VALUE = r'[-+]?(?:0[xX][0-9A-Fa-f]+|(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)'
_TRAIL = r'(?P<trail>(?:[ \t]+.*|#.*)?)'

# QGroundControl line: "SYSID COMPID NAME VALUE TYPE", tab or space separated.
RE_QGC = re.compile(
    r'^(?P<lead>﻿?[ \t]*)'
    r'(?P<sysid>\d+)(?P<sep1>[ \t]+)'
    r'(?P<compid>\d+)(?P<sep2>[ \t]+)'
    r'(?P<name>' + _NAME + r')(?P<sep3>[ \t]+)'
    r'(?P<value>' + _VALUE + r')(?P<sep4>[ \t]+)'
    r'(?P<ptype>\d+)' + _TRAIL + r'$'
)

# Mission Planner / MAVProxy line: "NAME,VALUE", "NAME VALUE" or "NAME=VALUE".
RE_2COL = re.compile(
    r'^(?P<lead>﻿?[ \t]*)'
    r'(?P<name>' + _NAME + r')'
    r'(?P<sep>[ \t]*[,=][ \t]*|[ \t]+)'
    r'(?P<value>' + _VALUE + r')' + _TRAIL + r'$'
)

# Blank lines, comments and @include directives are passed through untouched.
RE_PASSTHROUGH = re.compile(r'^﻿?[ \t]*(?:#|@include\b|$)')

RE_EOL = re.compile(r'(?s)^(.*?)(\r?\n)?$')


@dataclasses.dataclass
class RawLine:
    '''A line that is not a parameter assignment, reproduced verbatim.'''
    text: str
    name: Optional[str] = dataclasses.field(default=None, init=False)

    def render(self) -> str:
        return self.text

    def first_token(self) -> Optional[str]:
        '''Return the leading parameter-like token of an unparsable line in upper case, if any.'''
        m = re.match(r'﻿?[ \t]*(' + _NAME + r')', self.text)
        return m.group(1).upper() if m else None


@dataclasses.dataclass
class ParamLine:
    '''A parameter assignment split into its parts so it can be rewritten byte-for-byte.

    name is always upper case, name_text keeps the spelling found in the file.
    '''
    lead: str
    name: str
    sep: str
    value: str
    trail: str
    eol: str
    is_qgc: bool = False
    sysid: str = ''
    sep1: str = ''
    compid: str = ''
    sep2: str = ''
    sep4: str = ''
    ptype: str = ''
    name_text: str = ''

    def __post_init__(self):
        if not self.name_text:
            self.name_text = self.name
        self.name = self.name.upper()

    def render(self) -> str:
        if self.is_qgc:
            return (self.lead + self.sysid + self.sep1 + self.compid + self.sep2 + self.name_text +
                    self.sep + self.value + self.sep4 + self.ptype + self.trail + self.eol)
        return self.lead + self.name_text + self.sep + self.value + self.trail + self.eol

    def replace(self, name: Optional[str] = None, value: Optional[str] = None, ptype: Optional[int] = None,
                trail: Optional[str] = None, eol: Optional[str] = None) -> 'ParamLine':
        '''Return a copy with the given fields replaced.'''
        changes = {}
        if name is not None:
            changes['name'] = name
            changes['name_text'] = name
        if value is not None:
            changes['value'] = value
        if ptype is not None and self.is_qgc:
            changes['ptype'] = str(ptype)
        if trail is not None:
            changes['trail'] = trail
        if eol is not None:
            changes['eol'] = eol
        return dataclasses.replace(self, **changes)

    def clone_for(self, name: str, value: str, ptype: Optional[int] = None) -> 'ParamLine':
        '''Return a new assignment line in the same style as this one, without trailing comment.'''
        return self.replace(name=name, value=value, ptype=ptype, trail='')


Line = Union[RawLine, ParamLine]


@dataclasses.dataclass(frozen=True)
class Rename:
    '''Description of a parameter rename, with an optional value conversion.

    The new value is func(old_value) when func is given, old_value * factor
    when factor is not 1.0, and the unchanged value text otherwise. new_type
    replaces the type column of QGroundControl files when set.
    '''
    old: str
    new: str
    factor: float = 1.0
    func: Optional[Callable[[int], int]] = None
    new_type: Optional[int] = None


@dataclasses.dataclass
class Change:
    lineno: int
    old_text: str
    new_text: str

    def __str__(self) -> str:
        if not self.old_text:
            return f"line {self.lineno}: added {self.new_text.strip()}"
        return f"line {self.lineno}: {self.old_text.strip()} -> {self.new_text.strip()}"


def split_lines(content: str) -> List[str]:
    '''Split file content into lines, each keeping its own line ending.'''
    return re.findall(r'[^\n]*\n|[^\n]+\Z', content)


def parse_line(text: str) -> Line:
    '''Parse one line (including its line ending) into a ParamLine or RawLine.'''
    m = RE_EOL.match(text)
    body, eol = m.group(1), m.group(2) or ''
    if RE_PASSTHROUGH.match(body):
        return RawLine(text)
    m = RE_QGC.match(body)
    if m:
        return ParamLine(lead=m.group('lead'), name=m.group('name'), sep=m.group('sep3'),
                         value=m.group('value'), trail=m.group('trail'), eol=eol, is_qgc=True,
                         sysid=m.group('sysid'), sep1=m.group('sep1'), compid=m.group('compid'),
                         sep2=m.group('sep2'), sep4=m.group('sep4'), ptype=m.group('ptype'))
    m = RE_2COL.match(body)
    if m:
        return ParamLine(lead=m.group('lead'), name=m.group('name'), sep=m.group('sep'),
                         value=m.group('value'), trail=m.group('trail'), eol=eol)
    return RawLine(text)


def parse_value(text: str) -> float:
    '''Parse a parameter value, accepting hexadecimal notation.'''
    if text.lower().startswith(('0x', '-0x', '+0x')):
        return float(int(text, 16))
    return float(text)


def is_hex(text: str) -> bool:
    return text.lower().lstrip('+-').startswith('0x')


def round_sig(value: float, sig_digits: int) -> float:
    '''Round a value to the given number of significant digits.'''
    if value == 0 or not math.isfinite(value):
        return value
    digits = sig_digits - 1 - int(math.floor(math.log10(abs(value))))
    return round(value, digits)


def format_value(value: float, sig_digits: int = DEFAULT_SIG_DIGITS) -> str:
    '''Format a rescaled value with limited precision and without exponent notation.'''
    value = round_sig(value, sig_digits)
    if value == int(value):
        return str(int(value))
    decimals = max(0, sig_digits - 1 - int(math.floor(math.log10(abs(value)))))
    return format(value, f'.{decimals}f').rstrip('0').rstrip('.')


def format_int(value: int, like: str) -> str:
    '''Format an integer, keeping hexadecimal notation if the original used it.'''
    if is_hex(like):
        return '0x%X' % value if value >= 0 else '-0x%X' % -value
    return str(value)


def iter_param_files(directory: str) -> Iterator[str]:
    '''Yield all parameter files below a directory, in a stable order.'''
    for root, dirs, files in os.walk(directory):
        dirs.sort()
        for file in sorted(files):
            if file.lower().endswith(PARAM_FILE_EXTENSIONS):
                yield os.path.join(root, file)


def read_param_file(path: str) -> List[Line]:
    with open(path, 'r', encoding='utf-8', errors='surrogateescape', newline='') as f:
        content = f.read()
    return [parse_line(text) for text in split_lines(content)]


def write_param_file(path: str, lines: List[Line], mode_from: Optional[str] = None) -> None:
    '''Write lines to path atomically, keeping the permissions of mode_from (or of path).'''
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix='.' + os.path.basename(path), dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', errors='surrogateescape', newline='') as f:
            for line in lines:
                f.write(line.render())
        mode_source = mode_from or path
        if os.path.exists(mode_source):
            shutil.copymode(mode_source, tmp_path)
        os.replace(tmp_path, path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        raise


def convert_value(line: ParamLine, rename: Rename, sig_digits: int) -> str:
    '''Compute the new value text for a line converted with rename.'''
    if rename.func is not None:
        return format_int(rename.func(int(parse_value(line.value))), line.value)
    if rename.factor != 1.0:
        return format_value(parse_value(line.value) * rename.factor, sig_digits)
    return line.value


def apply_renames(lines: List[Line], renames: Dict[str, Rename], sig_digits: int = DEFAULT_SIG_DIGITS,
                  path: str = '') -> List[Change]:
    '''Rename (and rescale) the parameters listed in renames, in place in lines.

    Lines whose new name already exists in the file are left untouched with a warning.
    '''
    present = {line.name for line in lines if line.name is not None}
    changes = []
    for i, line in enumerate(lines):
        if not isinstance(line, ParamLine) or line.name not in renames:
            continue
        rename = renames[line.name]
        if rename.new in present:
            logging.warning("%s: line %u: %s not converted, %s already present", path, i + 1, line.name, rename.new)
            continue
        new_line = line.replace(name=rename.new, value=convert_value(line, rename, sig_digits), ptype=rename.new_type)
        changes.append(Change(i + 1, line.render(), new_line.render()))
        lines[i] = new_line
        present.add(rename.new)
    return changes


def convert_file(path: str, renames: Dict[str, Rename], sig_digits: int = DEFAULT_SIG_DIGITS,
                 dry_run: bool = False, out_path: Optional[str] = None) -> List[Change]:
    '''Apply renames to one file, writing it back in place or to out_path.'''
    lines = read_param_file(path)
    changes = apply_renames(lines, renames, sig_digits, path)
    if dry_run:
        for change in changes:
            print(f"{path}: {change}")
        return changes
    if out_path is not None:
        write_param_file(out_path, lines, mode_from=path)
    elif changes:
        write_param_file(path, lines)
    return changes


def parse_arguments(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="parameter conversion tool")
    parser.add_argument("--scale", default=1.0, type=float, help="scale factor")
    parser.add_argument("--directory", default=".", help="directory to search")
    parser.add_argument("--sig-digits", default=DEFAULT_SIG_DIGITS, type=int,
                        help="significant digits kept in rescaled values (default %(default)s)")
    parser.add_argument("old_name", help="old parameter name")
    parser.add_argument("new_name", help="new parameter name")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')
    old_name = args.old_name.upper()
    renames = {old_name: Rename(old_name, args.new_name.upper(), args.scale)}
    for path in iter_param_files(args.directory):
        if convert_file(path, renames, args.sig_digits):
            print("Updating %s" % path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
