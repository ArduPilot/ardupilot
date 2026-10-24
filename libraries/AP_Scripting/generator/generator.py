#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Compile bindings.desc into Lua C++ bindings and optional EmmyLua annotations."""

import argparse
import sys

from pathlib import Path

from bindings_parser import parse
from cpp import CppGenerator
from ir import dumps as dump_ir
from model import BindingError
from model import ErrorCode
from templates import Templates

from docs import DocsGenerator


def generate(description, docs=False):
    """Render everything before writing, so bad descriptions produce no files."""
    bindings = parse(description)
    templates = Templates()
    cpp = CppGenerator(bindings, templates)
    outputs = {'.cpp': cpp.source(), '.h': cpp.header()}
    if docs:
        outputs['.lua'] = DocsGenerator(bindings, templates).generate()
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-i', required=True, type=Path, metavar='DESCRIPTION', help='binding description file')
    parser.add_argument('-o', type=Path, metavar='OUTPUT', help='output path without .cpp/.h suffix')
    parser.add_argument('-d', type=Path, metavar='DOCS', help='optional Lua documentation output path')
    parser.add_argument('--dump-ir', type=Path, metavar='IR', help='optional parsed binding contract output')
    args = parser.parse_args()
    if args.o is None and args.dump_ir is None:
        print('Error: An output path must be provided for generated bindings or binding contracts', file=sys.stderr)
        return ErrorCode.GENERAL
    try:
        description = args.i.read_text(encoding='utf-8')
        bindings = parse(description)
        if args.dump_ir is not None:
            args.dump_ir.write_text(dump_ir(bindings), encoding='utf-8')
        if args.o is not None:
            templates = Templates()
            cpp = CppGenerator(bindings, templates)
            outputs = {'.cpp': cpp.source(), '.h': cpp.header()}
            if args.d is not None:
                outputs['.lua'] = DocsGenerator(bindings, templates).generate()
            for suffix in ('.cpp', '.h'):
                Path(str(args.o) + suffix).write_text(outputs[suffix], encoding='utf-8')
            if args.d is not None:
                args.d.write_text(outputs['.lua'], encoding='utf-8')
    except BindingError as error:
        print(error, file=sys.stderr)
        return error.code
    except OSError as error:
        print(f'Error: {error}', file=sys.stderr)
        return ErrorCode.GENERAL
    return 0


if __name__ == '__main__':
    sys.exit(main())
