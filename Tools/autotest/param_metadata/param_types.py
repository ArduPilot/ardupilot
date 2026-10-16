'''
Work out the type each parameter has on the wire by reading the source:
the AP_Param table entry declaring a documented parameter names the
class member holding it, and that member's declaration gives its type.

The parameter documentation does not carry types, but consumers of the
MAVLink component metadata parameter schema require them.

AP_FLAKE8_CLEAN
'''

import os
import re

# the MAVLink parameter-metadata type for each AP_Param type; unsigned
# parameters go over the wire as the signed type of the same size
WIRE_TYPES = {
    'AP_Int8': 'Int8',
    'AP_UInt8': 'Int8',
    'AP_Enum': 'Int8',
    'AP_Int16': 'Int16',
    'AP_UInt16': 'Int16',
    'AP_Enum16': 'Int16',
    'AP_Int32': 'Int32',
    'AP_UInt32': 'Int32',
    'AP_Float': 'Float',
    'AP_Vector3f': 'Float',
}

# directories holding the classes and tables parameters can live in
SOURCE_DIRS = [
    'libraries',
    'ArduCopter',
    'ArduPlane',
    'ArduSub',
    'AntennaTracker',
    'Blimp',
    'Rover',
    os.path.join('Tools', 'AP_Periph'),
]

# a definition of a parameter table, e.g.
# const AP_Param::GroupInfo AP_Baro::var_info[] = {
prog_table = re.compile(r"AP_Param::(?:GroupInfo|Info)\s+((?:\w+::)*)(\w+)\s*\[\s*\w*\s*\]\s*=?\s*{")

# an entry in a parameter table; see the macros in AP_Param.h
prog_entry = re.compile(r"\b(AP_GROUPINFO\w*|AP_SUBGROUP\w*|AP_NESTEDGROUPINFO|GSCALAR|GARRAY|ASCALAR|GGROUP|GOBJECT\w*)\s*\(")

# a define of one identifier as another, e.g.
# #define RC_CHANNELS_SUBCLASS RC_Channels_Copter
prog_define = re.compile(r"^[ \t]*#[ \t]*define[ \t]+(\w+)[ \t]+([A-Za-z_]\w*)[ \t]*$", re.MULTILINE)

# a class or struct definition
prog_class = re.compile(r"\b(?:class|struct)\s+(\w+)(?:\s+final)?\s*(?::([^;{]*))?{")

# a declaration of one or more parameter members
prog_param_member = re.compile(r"\b(" + '|'.join(WIRE_TYPES.keys()) + r")\b(?:\s*<[^;{}]*?>)?\s+([^;()]+);")


def blank_comments(text):
    '''replace comments with spaces, keeping offsets and line breaks intact'''
    def blank(m):
        comment = m.group(0)
        if '\n' not in comment:
            return ' ' * len(comment)
        return re.sub(r"[^\n]", " ", comment)
    return re.sub(r"//[^\n]*|/\*.*?\*/", blank, text, flags=re.DOTALL)


def matching_close(text, start, open_char, close_char):
    '''return the offset of the bracket closing the one at start'''
    depth = 0
    for i in range(start, len(text)):
        c = text[i]
        if c == open_char:
            depth += 1
        elif c == close_char:
            depth -= 1
            if depth == 0:
                return i
    return None


def matching_open(text, end, open_char, close_char):
    '''return the offset of the bracket opening the one at end'''
    depth = 0
    for i in range(end, -1, -1):
        c = text[i]
        if c == close_char:
            depth += 1
        elif c == open_char:
            depth -= 1
            if depth == 0:
                return i
    return None


def split_args(text):
    '''split a macro's arguments on top-level commas'''
    args = []
    depth = 0
    current = ''
    for c in text:
        if c in '([{<':
            depth += 1
        elif c in ')]}>':
            depth -= 1
        if c == ',' and depth == 0:
            args.append(current.strip())
            current = ''
        else:
            current += c
    args.append(current.strip())
    return args


def unquote(name):
    '''return a string literal's contents, or None if it is not one'''
    m = re.fullmatch(r'"([^"]*)"', name)
    if m is None:
        return None
    return m.group(1)


class Entry(object):
    '''a parameter (or a group of parameters) in a parameter table'''
    def __init__(self, name, clazz, element=None, table=None):
        self.name = name
        self.clazz = clazz
        # set for parameters:
        self.element = element
        # set for groups; None for one whose table is only known at runtime:
        self.table = table


class ParamTypes(object):
    def __init__(self, apm_path):
        self.apm_path = os.path.normpath(apm_path)
        self.texts = {}
        self.tables_in_file = {}
        self.class_index = None
        self.table_index = None
        self.defines = None
        self.member_cache = {}
        self.entries_cache = {}
        self.param_members_cache = {}

    def text(self, path):
        '''return the comment-free text of a source file'''
        path = os.path.normpath(path)
        if path not in self.texts:
            with open(path, errors='replace') as f:
                self.texts[path] = blank_comments(f.read())
        return self.texts[path]

    def build_index(self):
        '''find every class and parameter table in the source'''
        self.class_index = {}
        self.table_index = {}
        self.defines = {}
        for d in SOURCE_DIRS:
            for root, dirs, files in os.walk(os.path.join(self.apm_path, d)):
                for f in files:
                    if not f.endswith(('.h', '.cpp')):
                        continue
                    path = os.path.join(root, f)
                    with open(path, errors='replace') as fh:
                        raw = fh.read()
                    if 'class' not in raw and 'struct' not in raw and 'AP_Param::' not in raw:
                        continue
                    text = blank_comments(raw)
                    self.texts[os.path.normpath(path)] = text
                    for m in prog_define.finditer(text):
                        self.defines.setdefault(m.group(1), []).append((path, m.group(2)))
                    for m in prog_class.finditer(text):
                        close = matching_close(text, m.end() - 1, '{', '}')
                        if close is None:
                            continue
                        bases = []
                        if m.group(2) is not None:
                            for base in m.group(2).split(','):
                                base = re.sub(r"\b(public|protected|private|virtual)\b", "", base).strip()
                                base = re.sub(r"<.*", "", base).split('::')[-1].strip()
                                if base:
                                    bases.append(base)
                        self.class_index.setdefault(m.group(1), []).append(
                            (path, text[m.end():close], bases))
                    for (clazz, tablename, start, end) in self.tables(path):
                        self.table_index.setdefault((clazz, tablename), []).append((path, start, end))

    def tables(self, path):
        '''return (class, table name, start, end) for each table in a file'''
        path = os.path.normpath(path)
        if path not in self.tables_in_file:
            text = self.text(path)
            tables = []
            for m in prog_table.finditer(text):
                close = matching_close(text, m.end() - 1, '{', '}')
                if close is None:
                    continue
                qualifier = m.group(1).rstrip(':').split('::')
                tables.append((qualifier[-1] or None, m.group(2), m.end(), close))
            self.tables_in_file[path] = tables
        return self.tables_in_file[path]

    def entries(self, path, start, end):
        '''return the Entry objects in the table at path[start:end]'''
        key = (os.path.normpath(path), start)
        if key not in self.entries_cache:
            self.entries_cache[key] = self.parse_entries(path, start, end)
        return self.entries_cache[key]

    def parse_entries(self, path, start, end):
        text = self.text(path)
        ret = []
        for m in prog_entry.finditer(text, start, end):
            close = matching_close(text, m.end() - 1, '(', ')')
            if close is None:
                continue
            macro = m.group(1)
            args = split_args(text[m.end():close])
            try:
                if macro.startswith('AP_GROUPINFO'):
                    ret.append(Entry(unquote(args[0]), args[2], element=args[3]))
                elif macro in ('GSCALAR', 'GARRAY'):
                    # these are members of the vehicle's Parameters class
                    name = args[1] if macro == 'GSCALAR' else args[2]
                    ret.append(Entry(unquote(name), 'Parameters', element=args[0]))
                elif macro == 'ASCALAR':
                    # these are members of Plane's aparm, an AP_FixedWing
                    ret.append(Entry(unquote(args[1]), 'AP_FixedWing', element=args[0]))
                elif macro in ('AP_SUBGROUPINFO', 'AP_SUBGROUPPTR'):
                    ret.append(Entry(unquote(args[1]), args[4], table='var_info'))
                elif macro == 'AP_SUBGROUPEXTENSION':
                    ret.append(Entry(unquote(args[0]), args[2], table=args[3]))
                elif macro == 'AP_NESTEDGROUPINFO':
                    ret.append(Entry('', args[0], table='var_info'))
                elif macro == 'AP_SUBGROUPVARPTR':
                    ret.append(Entry(unquote(args[1]), None))
                elif macro in ('GGROUP', 'GOBJECT', 'GOBJECTPTR'):
                    ret.append(Entry(unquote(args[1]), args[2], table='var_info'))
                elif macro == 'GOBJECTN':
                    ret.append(Entry(unquote(args[2]), args[3], table='var_info'))
                elif macro == 'GOBJECTVARPTR':
                    ret.append(Entry(unquote(args[1]), None))
            except IndexError:
                continue
        return ret

    def candidates(self, index, name, context_path, suffix=()):
        '''return the entries of index for a class name, which may be a
        define standing in for one (as in a table shared by several
        vehicles), nearest to context_path first'''
        ret = self.nearest(index.get((name,) + suffix if suffix else name, []), context_path)
        if not ret:
            for (path, value) in self.nearest(self.defines.get(name, []), context_path):
                key = (value,) + suffix if suffix else value
                ret.extend(self.nearest(index.get(key, []), context_path))
        return ret

    def nearest(self, candidates, context_path):
        '''order candidate (path, ...) tuples by closeness to context_path'''
        context = os.path.dirname(os.path.normpath(context_path))

        def closeness(candidate):
            return len(os.path.commonpath([context, os.path.dirname(candidate[0])]))
        return sorted(candidates, key=closeness, reverse=True)

    def member_type(self, clazz, element, context_path, depth=0):
        '''return the AP_Param type name of clazz's member element'''
        if self.class_index is None:
            self.build_index()
        clazz = clazz.split('::')[-1]
        key = (clazz, element, os.path.dirname(context_path))
        if key in self.member_cache:
            return self.member_cache[key]
        ret = None
        parts = [re.sub(r"\[.*?\]", "", p).strip() for p in element.split('.')]
        for (path, body, bases) in self.candidates(self.class_index, clazz, context_path):
            ret = self.member_type_in_body(body, parts, path, depth)
            if ret is None and depth < 5:
                for base in bases:
                    ret = self.member_type(base, element, path, depth + 1)
                    if ret is not None:
                        break
            if ret is not None:
                break
        self.member_cache[key] = ret
        return ret

    def member_type_in_body(self, body, parts, path, depth):
        if len(parts) > 1:
            if depth >= 5:
                return None
            # a member of a member, e.g. params.enable; the name may be
            # declared more than once, e.g. also in a nested struct, so
            # try each declaration
            name = re.escape(parts[0]) + r"\s*(?:\[[^\]]*\])?\s*(?:{[^;]*}|=[^;]*)?;"
            for m in re.finditer(r"\b" + name, body):
                before = body[max(0, m.start()-200):m.start()].rstrip()
                if before.endswith('}'):
                    # ... where the member is a struct defined in place
                    close = body.rindex('}', 0, m.start())
                    open_brace = matching_open(body, close, '{', '}')
                    if open_brace is None:
                        continue
                    ret = self.member_type_in_body(body[open_brace+1:close], parts[1:], path, depth + 1)
                    if ret is not None:
                        return ret
                    continue
                decl = re.search(r"\b([\w:]+)\s*(?:<([^;{}]*)>)?\s*&?$", before)
                if decl is None:
                    continue
                if decl.group(2) is not None:
                    # ... where the member is a container of its first
                    # template argument, e.g. RestrictIDTypeArray, whose
                    # elements are the member after next
                    element_type = split_args(decl.group(2))[0].split('::')[-1]
                    if len(parts) == 2:
                        if element_type in WIRE_TYPES:
                            return element_type
                        continue
                    ret = self.member_type(element_type, '.'.join(parts[2:]), path, depth + 1)
                else:
                    ret = self.member_type(decl.group(1), '.'.join(parts[1:]), path, depth + 1)
                if ret is not None:
                    return ret
            return None
        return self.param_members(body).get(parts[0])

    def param_members(self, body):
        '''return a map from name to type of the parameters declared in body'''
        if body not in self.param_members_cache:
            members = {}
            for m in prog_param_member.finditer(body):
                for declarator in split_args(m.group(2)):
                    name = re.split(r"[\[={]", declarator)[0].strip()
                    members.setdefault(name, m.group(1))
            self.param_members_cache[body] = members
        return self.param_members_cache[body]

    def find_table(self, clazz, tablename, context_path):
        if self.table_index is None:
            self.build_index()
        candidates = self.candidates(self.table_index, clazz.split('::')[-1], context_path, (tablename,))
        if not candidates:
            return None
        return candidates[0]

    def resolve_in_table(self, path, start, end, short_name, depth=0):
        '''return the set of wire types short_name could have in a table'''
        found = set()
        for entry in self.entries(path, start, end):
            if entry.name is None:
                continue
            if entry.element is not None:
                if short_name == entry.name:
                    t = self.member_type(entry.clazz, entry.element, path)
                    if t is not None:
                        found.add(WIRE_TYPES[t])
                elif short_name[:-2] == entry.name and short_name[-2:] in ('_X', '_Y', '_Z'):
                    if self.member_type(entry.clazz, entry.element, path) == 'AP_Vector3f':
                        found.add('Float')
            elif entry.clazz is not None and short_name.startswith(entry.name) and depth < 10:
                table = self.find_table(entry.clazz, entry.table, path)
                if table is not None:
                    (tpath, tstart, tend) = table
                    found |= self.resolve_in_table(tpath, tstart, tend, short_name[len(entry.name):], depth + 1)
        return found

    def resolve(self, path, offset, short_name):
        '''return the wire type of the parameter documented at offset in path,
        whose name relative to the table it lives in is short_name'''
        if path.endswith('.lua'):
            # scripting parameters are all AP_Float, see AP_Param::add_param
            return 'Float'
        tables = self.tables(path)
        # the table the documentation sits in is the most likely home;
        # then try the others in the same file
        preceding = [t for t in tables if t[2] <= offset]
        ordered = preceding[-1:] + [t for t in tables if t not in preceding[-1:]]
        for (clazz, tablename, start, end) in ordered:
            found = self.resolve_in_table(path, start, end, short_name)
            if len(found) == 1:
                return found.pop()
            if len(found) > 1:
                return None
        return None
