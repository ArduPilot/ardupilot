#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Compile bindings.desc into Lua C++ bindings and optional EmmyLua annotations."""

import argparse
import sys

from pathlib import Path

from bindings_parser import parse
from cpp import CppGenerator
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
    parser.add_argument('-o', required=True, type=Path, metavar='OUTPUT', help='output path without .cpp/.h suffix')
    parser.add_argument('-d', type=Path, metavar='DOCS', help='optional Lua documentation output path')
    args = parser.parse_args()
    try:
        outputs = generate(args.i.read_text(encoding='utf-8'), docs=args.d is not None)
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
