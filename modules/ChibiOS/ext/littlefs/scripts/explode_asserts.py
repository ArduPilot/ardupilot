#!/usr/bin/env python3

import re
import sys

PATTERN = ['LFS_ASSERT', 'assert']
PREFIX = 'LFS'
MAXWIDTH = 16

ASSERT = "__{PREFIX}_ASSERT_{TYPE}_{COMP}"
FAIL = """
__attribute__((unused))
static void __{prefix}_assert_fail_{type}(
        const char *file, int line, const char *comp,
        {ctype} lh, size_t lsize,
        {ctype} rh, size_t rsize) {{
    printf("%s:%d:assert: assert failed with ", file, line);
    __{prefix}_assert_print_{type}(lh, lsize);
    printf(", expected %s ", comp);
    __{prefix}_assert_print_{type}(rh, rsize);
    printf("\\n");
    fflush(NULL);
    raise(SIGABRT);
}}
"""

COMP = {
    '==': 'eq',
    '!=': 'ne',
    '<=': 'le',
    '>=': 'ge',
    '<':  'lt',
    '>':  'gt',
}

TYPE = {
    'int': {
        'ctype': 'intmax_t',
        'fail': FAIL,
        'print': """
        __attribute__((unused))
        static void __{prefix}_assert_print_{type}({ctype} v, size_t size) {{
            (void)size;
            printf("%"PRIiMAX, v);
        }}
        """,
        'assert': """
        #define __{PREFIX}_ASSERT_{TYPE}_{COMP}(file, line, lh, rh)
        do {{
            __typeof__(lh) _lh = lh;
            __typeof__(lh) _rh = (__typeof__(lh))rh;
            if (!(_lh {op} _rh)) {{
                __{prefix}_assert_fail_{type}(file, line, "{comp}",
                        (intmax_t)_lh, 0, (intmax_t)_rh, 0);
            }}
        }} while (0)
        """
    },
    'bool': {
        'ctype': 'bool',
        'fail': FAIL,
        'print': """
        __attribute__((unused))
        static void __{prefix}_assert_print_{type}({ctype} v, size_t size) {{
            (void)size;
            printf("%s", v ? "true" : "false");
        }}
        """,
        'assert': """
        #define __{PREFIX}_ASSERT_{TYPE}_{COMP}(file, line, lh, rh)
        do {{
            bool _lh = !!(lh);
            bool _rh = !!(rh);
            if (!(_lh {op} _rh)) {{
                __{prefix}_assert_fail_{type}(file, line, "{comp}",
                        _lh, 0, _rh, 0);
            }}
        }} while (0)
        """
    },
    'mem': {
        'ctype': 'const void *',
        'fail': FAIL,
        'print': """
        __attribute__((unused))
        static void __{prefix}_assert_print_{type}({ctype} v, size_t size) {{
            const uint8_t *s = v;
            printf("\\\"");
            for (size_t i = 0; i < size && i < {maxwidth}; i++) {{
                if (s[i] >= ' ' && s[i] <= '~') {{
                    printf("%c", s[i]);
                }} else {{
                    printf("\\\\x%02x", s[i]);
                }}
            }}
            if (size > {maxwidth}) {{
                printf("...");
            }}
            printf("\\\"");
        }}
        """,
        'assert': """
        #define __{PREFIX}_ASSERT_{TYPE}_{COMP}(file, line, lh, rh, size)
        do {{
            const void *_lh = lh;
            const void *_rh = rh;
            if (!(memcmp(_lh, _rh, size) {op} 0)) {{
                __{prefix}_assert_fail_{type}(file, line, "{comp}",
                        _lh, size, _rh, size);
            }}
        }} while (0)
        """
    },
    'str': {
        'ctype': 'const char *',
        'fail': FAIL,
        'print': """
        __attribute__((unused))
        static void __{prefix}_assert_print_{type}({ctype} v, size_t size) {{
            __{prefix}_assert_print_mem(v, size);
        }}
        """,
        'assert': """
        #define __{PREFIX}_ASSERT_{TYPE}_{COMP}(file, line, lh, rh)
        do {{
            const char *_lh = lh;
            const char *_rh = rh;
            if (!(strcmp(_lh, _rh) {op} 0)) {{
                __{prefix}_assert_fail_{type}(file, line, "{comp}",
                        _lh, strlen(_lh), _rh, strlen(_rh));
            }}
        }} while (0)
        """
    }
}

def mkdecls(outf, maxwidth=16):
    outf.write("#include <stdio.h>\n")
    outf.write("#include <stdbool.h>\n")
    outf.write("#include <stdint.h>\n")
    outf.write("#include <inttypes.h>\n")
    outf.write("#include <signal.h>\n")

    for type, desc in sorted(TYPE.items()):
        format = {
            'type': type.lower(), 'TYPE': type.upper(),
            'ctype': desc['ctype'],
            'prefix': PREFIX.lower(), 'PREFIX': PREFIX.upper(),
            'maxwidth': maxwidth,
        }
        outf.write(re.sub('\s+', ' ',
            desc['print'].strip().format(**format))+'\n')
        outf.write(re.sub('\s+', ' ',
            desc['fail'].strip().format(**format))+'\n')

        for op, comp in sorted(COMP.items()):
            format.update({
                'comp': comp.lower(), 'COMP': comp.upper(),
                'op': op,
            })
            outf.write(re.sub('\s+', ' ',
                desc['assert'].strip().format(**format))+'\n')

def mkassert(type, comp, lh, rh, size=None):
    format = {
        'type': type.lower(), 'TYPE': type.upper(),
        'comp': comp.lower(), 'COMP': comp.upper(),
        'prefix': PREFIX.lower(), 'PREFIX': PREFIX.upper(),
        'lh': lh.strip(' '),
        'rh': rh.strip(' '),
        'size': size,
    }
    if size:
        return ((ASSERT + '(__FILE__, __LINE__, {lh}, {rh}, {size})')
            .format(**format))
    else:
        return ((ASSERT + '(__FILE__, __LINE__, {lh}, {rh})')
            .format(**format))


# simple recursive descent parser
LEX = {
    'ws':       [r'(?:\s|\n|#.*?\n|//.*?\n|/\*.*?\*/)+'],
    'assert':   PATTERN,
    'string':   [r'"(?:\\.|[^"])*"', r"'(?:\\.|[^'])\'"],
    'arrow':    ['=>'],
    'paren':    ['\(', '\)'],
    'op':       ['strcmp', 'memcmp', '->'],
    'comp':     ['==', '!=', '<=', '>=', '<', '>'],
    'logic':    ['\&\&', '\|\|'],
    'sep':      [':', ';', '\{', '\}', ','],
}

class ParseFailure(Exception):
    def __init__(self, expected, found):
        self.expected = expected
        self.found = found

    def __str__(self):
        return "expected %r, found %s..." % (
            self.expected, repr(self.found)[:70])

class Parse:
    def __init__(self, inf, lexemes):
        p = '|'.join('(?P<%s>%s)' % (n, '|'.join(l))
            for n, l in lexemes.items())
        p = re.compile(p, re.DOTALL)
        data = inf.read()
        tokens = []
        while True:
            m = p.search(data)
            if m:
                if m.start() > 0:
                    tokens.append((None, data[:m.start()]))
                tokens.append((m.lastgroup, m.group()))
                data = data[m.end():]
            else:
                tokens.append((None, data))
                break
        self.tokens = tokens
        self.off = 0

    def lookahead(self, *pattern):
        if self.off < len(self.tokens):
            token = self.tokens[self.off]
            if token[0] in pattern or token[1] in pattern:
                self.m = token[1]
                return self.m
        self.m = None
        return self.m

    def accept(self, *patterns):
        m = self.lookahead(*patterns)
        if m is not None:
            self.off += 1
        return m

    def expect(self, *patterns):
        m = self.accept(*patterns)
        if not m:
            raise ParseFailure(patterns, self.tokens[self.off:])
        return m

    def push(self):
        return self.off

    def pop(self, state):
        self.off = state

def passert(p):
    def pastr(p):
        p.expect('assert') ; p.accept('ws') ; p.expect('(') ; p.accept('ws')
        p.expect('strcmp') ; p.accept('ws') ; p.expect('(') ; p.accept('ws')
        lh = pexpr(p) ; p.accept('ws')
        p.expect(',') ; p.accept('ws')
        rh = pexpr(p) ; p.accept('ws')
        p.expect(')') ; p.accept('ws')
        comp = p.expect('comp') ; p.accept('ws')
        p.expect('0') ; p.accept('ws')
        p.expect(')')
        return mkassert('str', COMP[comp], lh, rh)

    def pamem(p):
        p.expect('assert') ; p.accept('ws') ; p.expect('(') ; p.accept('ws')
        p.expect('memcmp') ; p.accept('ws') ; p.expect('(') ; p.accept('ws')
        lh = pexpr(p) ; p.accept('ws')
        p.expect(',') ; p.accept('ws')
        rh = pexpr(p) ; p.accept('ws')
        p.expect(',') ; p.accept('ws')
        size = pexpr(p) ; p.accept('ws')
        p.expect(')') ; p.accept('ws')
        comp = p.expect('comp') ; p.accept('ws')
        p.expect('0') ; p.accept('ws')
        p.expect(')')
        return mkassert('mem', COMP[comp], lh, rh, size)

    def paint(p):
        p.expect('assert') ; p.accept('ws') ; p.expect('(') ; p.accept('ws')
        lh = pexpr(p) ; p.accept('ws')
        comp = p.expect('comp') ; p.accept('ws')
        rh = pexpr(p) ; p.accept('ws')
        p.expect(')')
        return mkassert('int', COMP[comp], lh, rh)

    def pabool(p):
        p.expect('assert') ; p.accept('ws') ; p.expect('(') ; p.accept('ws')
        lh = pexprs(p) ; p.accept('ws')
        p.expect(')')
        return mkassert('bool', 'eq', lh, 'true')

    def pa(p):
        return p.expect('assert')

    state = p.push()
    lastf = None
    for pa in [pastr, pamem, paint, pabool, pa]:
        try:
            return pa(p)
        except ParseFailure as f:
            p.pop(state)
            lastf = f
    else:
        raise lastf

def pexpr(p):
    res = []
    while True:
        if p.accept('('):
            res.append(p.m)
            while True:
                res.append(pexprs(p))
                if p.accept('sep'):
                    res.append(p.m)
                else:
                    break
            res.append(p.expect(')'))
        elif p.lookahead('assert'):
            res.append(passert(p))
        elif p.accept('assert', 'ws', 'string', 'op', None):
            res.append(p.m)
        else:
            return ''.join(res)

def pexprs(p):
    res = []
    while True:
        res.append(pexpr(p))
        if p.accept('comp', 'logic', ','):
            res.append(p.m)
        else:
            return ''.join(res)

def pstmt(p):
    ws = p.accept('ws') or ''
    lh = pexprs(p)
    if p.accept('=>'):
        rh = pexprs(p)
        return ws + mkassert('int', 'eq', lh, rh)
    else:
        return ws + lh


def main(args):
    inf = open(args.input, 'r') if args.input else sys.stdin
    outf = open(args.output, 'w') if args.output else sys.stdout

    lexemes = LEX.copy()
    if args.pattern:
        lexemes['assert'] = args.pattern
    p = Parse(inf, lexemes)

    # write extra verbose asserts
    mkdecls(outf, maxwidth=args.maxwidth)
    if args.input:
        outf.write("#line %d \"%s\"\n" % (1, args.input))

    # parse and write out stmt at a time
    try:
        while True:
            outf.write(pstmt(p))
            if p.accept('sep'):
                outf.write(p.m)
            else:
                break
    except ParseFailure as f:
        pass

    for i in range(p.off, len(p.tokens)):
        outf.write(p.tokens[i][1])

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Cpp step that increases assert verbosity")
    parser.add_argument('input', nargs='?',
        help="Input C file after cpp.")
    parser.add_argument('-o', '--output', required=True,
        help="Output C file.")
    parser.add_argument('-p', '--pattern', action='append',
        help="Patterns to search for starting an assert statement.")
    parser.add_argument('--maxwidth', default=MAXWIDTH, type=int,
        help="Maximum number of characters to display for strcmp and memcmp.")
    main(parser.parse_args())
