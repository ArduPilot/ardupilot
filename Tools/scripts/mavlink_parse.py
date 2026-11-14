#!/usr/bin/env python3

# flake8: noqa

import re, sys
from bisect import bisect_right
from enum import StrEnum # requires Python >= 3.11
from pathlib import Path
from itertools import chain
from importlib import import_module
from dataclasses import dataclass, astuple

# MAVLink message sets included in ArduPilot, determined manually via recursion
# through https://mavlink.io/en/messages/ardupilotmega.html#mavlink-include-files
class MAVLinkDialect(StrEnum):
    # in subset, superset, unknown order, for correct links
    # supported values must match imported dialect names
    MINIMAL = 'minimal'
    CSAIRLINK = 'csAirLink'
    ICAROUS = 'icarous'
    LOWEHEISER = 'loweheiser'
    STANDARD = 'standard'
    COMMON = 'common'
    CUBEPILOT = 'cubepilot'
    UAVIONIX = 'uAvionix'
    ARDUPILOTMEGA = 'ardupilotmega'
    DEVELOPMENT = 'development'
    ALL = 'all'
    UNKNOWN = 'UNKNOWN'

for dialect in MAVLinkDialect:
    if dialect == MAVLinkDialect.UNKNOWN: continue
    globals()[dialect] = import_module(f'pymavlink.dialects.v20.{dialect}')


@dataclass(slots=True, order=True)
class MAVLinkMessage:
    name: str
    source: str
    dialect: MAVLinkDialect = MAVLinkDialect.UNKNOWN
    id_number: int | None = None
    # (condition, files) for each distinct set of compile-time conditions
    # the message was found under, with the files that use that condition
    requires: tuple = ()

    PREFIX = 'MAVLINK_MSG_ID_'

    # Get message sets, for quick containment checks.
    #  Function is required because of Python's class scoping rules.
    #  See https://stackoverflow.com/questions/13905741.
    def _get_known_messages(prefix):
        ''' Returns a dictionary of {dialect: {message: id}} given 'prefix'. '''
        dialects = {}
        for dialect in MAVLinkDialect:
            if dialect == MAVLinkDialect.UNKNOWN: continue
            mavlink_module = globals()[dialect]
            dialects[dialect] = {
                message: getattr(mavlink_module, message)
                for message in dir(mavlink_module)
                if message.startswith(prefix)
            }
        return dialects
    KNOWN_DIALECTS = _get_known_messages(PREFIX)

    # Try to determine dialect and ID number if initialised without them specified.
    def __post_init__(self):
        if self.dialect == MAVLinkDialect.UNKNOWN:
            self.determine_dialect()
        if self.id_number is None:
            self.determine_id()

    @property
    def id_name(self):
        return self.PREFIX + self.name

    def determine_dialect(self):
        for dialect, message_set in self.KNOWN_DIALECTS.items():
            if self.id_name in message_set:
                self.dialect = dialect
                self.id_number = message_set[self.id_name]
                break # dialect found, no need to continue searching
        else:
            self.dialect = MAVLinkDialect.UNKNOWN

    def determine_id(self):
        self.id_number = self.KNOWN_DIALECTS.get(self.dialect, {}).get(self.id_name, None)

    def as_tuple(self):
        return astuple(self)

    def describe_requires(self, quote='', limit=None):
        if len(self.requires) == 1:
            return f'{quote}{self.requires[0][0]}{quote}'
        # several places may be alternatives or steps of one path, which
        # the guards cannot tell apart, so name the files and leave it open.
        # The separators are safe in csv, markdown and rst table cells.
        places = [f'{quote}{condition}{quote} ({" / ".join(files)})'
                  for condition, files in self.requires]
        if limit and len(places) > limit:
            # keeps a table readable, the csv export has the full list
            places[limit:] = [f'and {len(places) - limit} more']
        return '; '.join(places)

    def __str__(self):
        return (f'{self.name:<45}{self.source:<55}{self.dialect:<16}'
                f'{self.describe_requires()}').rstrip()

    @classmethod
    def get_unsupported(cls, supported: set, remove_prefix=True):
        ''' Yields known messages that are not included in 'supported'. '''
        offset = len(cls.PREFIX) if remove_prefix else 0
        known_missing = set() # don't double-count for supersets
        for dialect, messages in cls.KNOWN_DIALECTS.items():
            missing_names = set(messages) - supported - known_missing
            for name in missing_names:
                yield cls(name[offset:], 'UNSUPPORTED', dialect)
            known_missing |= missing_names


class MAVLinkCommand(MAVLinkMessage):
    PREFIX = 'MAV_CMD_'
    KNOWN_DIALECTS = MAVLinkMessage._get_known_messages(PREFIX)

    @property
    def id_name(self):
        return self.name # commands are registered with their prefix

    @classmethod
    def get_unsupported(cls, supported: set, remove_prefix=False):
        ''' Yields known commands that are not included in 'supported'. '''
        # avoid accidentally treating enum values as commands
        enums = [f'{e}_' for e in ardupilotmega.enums
                 if e.startswith(cls.PREFIX)]
        for command in super().get_unsupported(supported, remove_prefix):
            if not any(command.name.startswith(e) for e in enums):
                yield command


class PreprocessorGuards:
    ''' Tracks which preprocessor conditions each line of a source file sits within. '''
    DIRECTIVE = re.compile(r'[ \t]*#[ \t]*(if|ifdef|ifndef|elif|else|endif)\b(.*)')
    DEFINE = re.compile(r'[ \t]*#[ \t]*define[ \t]+(\w+)')
    COMMENT = re.compile(r'//.*|/\*.*?\*/')
    DEFINED = re.compile(r'\bdefined\s*(?:\(\s*(\w+)\s*\)|(\w+))')
    # a single name or a single macro call such as APM_BUILD_TYPE(APM_BUILD_ArduSub)
    ATOM = re.compile(r'\w+(\([\w\s,]*\))?')
    ALWAYS, NEVER = '1', '0'

    def __init__(self, text: str):
        lines = text.split('\n')
        self._line_starts = []
        self._line_guards = []
        # one list per open #if, of the conditions of its branches so far
        stack = []
        guard = ()
        start = 0
        for number, line in enumerate(lines):
            self._line_starts.append(start)
            self._line_guards.append(guard)
            start += len(line) + 1

            match = self.DIRECTIVE.match(line)
            if not match:
                continue
            directive, condition = match.groups()
            following = number
            while condition.rstrip().endswith('\\') and following + 1 < len(lines):
                following += 1
                condition = condition.rstrip()[:-1] + ' ' + lines[following]
            condition = self.normalise(condition)

            if directive == 'ifdef':
                directive, condition = 'if', f'defined({condition})'
            elif directive == 'ifndef':
                directive, condition = 'if', f'!defined({condition})'

            if directive == 'if':
                # an include guard is not something a feature requires and
                # neither is a default for an unset option
                define = None
                if condition.startswith('!defined('):
                    next_lines = (later for later in lines[following + 1:following + 5]
                                  if later.strip())
                    define = self.DEFINE.match(next(next_lines, ''))
                is_default = define and condition == f'!defined({define[1]})'
                stack.append([self.ALWAYS if is_default else condition])
            # unbalanced directives are skipped, old branches should not stop the parse
            elif directive == 'elif' and stack:
                stack[-1].append(condition)
            elif directive == 'else' and stack:
                stack[-1].append(self.ALWAYS)
            elif directive == 'endif' and stack:
                stack.pop()
            guard = self.combine(stack)

    def at(self, position: int):
        ''' Returns the conditions around the character at 'position', as a
        tuple of terms that must all hold. Returns None if it is never compiled. '''
        return self._line_guards[bisect_right(self._line_starts, position) - 1]

    @classmethod
    def normalise(cls, condition: str):
        condition = cls.COMMENT.sub(' ', condition)
        condition = cls.DEFINED.sub(lambda m: f'defined({m[1] or m[2]})', condition)
        return ' '.join(condition.split())

    @classmethod
    def combine(cls, stack):
        terms = []
        for branches in stack:
            # reaching a later branch means every earlier one was false
            for earlier in branches[:-1]:
                terms.extend(cls.negate(earlier))
            terms.extend(cls.split(branches[-1], '&&'))
        if cls.NEVER in terms:
            return None
        # dict.fromkeys removes repeats while keeping outer conditions first
        return tuple(term for term in dict.fromkeys(terms) if term != cls.ALWAYS)

    @staticmethod
    def strip_brackets(condition: str):
        ''' Removes brackets that wrap the whole of 'condition'. '''
        condition = condition.strip()
        while condition.startswith('('):
            depth = 0
            for index, character in enumerate(condition):
                depth += (character == '(') - (character == ')')
                if depth == 0:
                    break
            if index != len(condition) - 1:
                break # the first bracket closes early, e.g. (A) && (B)
            condition = condition[1:-1].strip()
        return condition

    @classmethod
    def split(cls, condition: str, operator: str):
        ''' Splits 'condition' at each 'operator' that is not inside brackets. '''
        condition = cls.strip_brackets(condition)
        parts = []
        depth = start = 0
        for index, character in enumerate(condition):
            depth += (character == '(') - (character == ')')
            if depth == 0 and condition.startswith(operator, index):
                parts.append(condition[start:index])
                start = index + len(operator)
        parts.append(condition[start:])
        return [cls.strip_brackets(part) for part in parts]

    @classmethod
    def negated(cls, term: str):
        ''' Returns what 'term' negates if it is a single negation, else None. '''
        if not term.startswith('!') or term.startswith('!='):
            return None
        inner = term[1:].strip()
        if cls.ATOM.fullmatch(inner) or cls.strip_brackets(inner) != inner:
            return cls.strip_brackets(inner)
        return None

    @classmethod
    def negate(cls, condition: str):
        ''' Returns the terms that must all hold for 'condition' to be false. '''
        condition = cls.strip_brackets(condition)
        if condition in (cls.ALWAYS, cls.NEVER):
            return [cls.NEVER if condition == cls.ALWAYS else cls.ALWAYS]
        options = cls.split(condition, '||')
        if len(options) > 1:
            return [term for option in options for term in cls.negate(option)]
        inner = cls.negated(condition)
        if inner is not None:
            return cls.split(inner, '&&')
        if cls.ATOM.fullmatch(condition):
            return [f'!{condition}']
        return [f'!({condition})']

    @staticmethod
    def describe(guard):
        ''' Returns the terms of 'guard' as a single condition. '''
        if len(guard) == 1:
            return guard[0]
        return ' && '.join(f'({term})' if '||' in term else term for term in guard)


class MAVLinkDetector:
    # file paths
    BASE_DIR = Path(__file__).parent.parent.parent
    COMMON_FILE = BASE_DIR / 'libraries/GCS_MAVLink/GCS_Common.cpp'
    # stream groups were defined per vehicle up to 4.6 and are shared from 4.7
    STREAM_GROUP_FILE = 'GCS_MAVLink.cpp'
    COMMON_STREAM_GROUP_FILE = BASE_DIR / 'libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp'
    BUILD_TYPE_FILE = BASE_DIR / 'libraries/AP_Vehicle/AP_Vehicle_Type.h'

    # regex for messages handled by the autopilot
    INCOMING_MESSAGES = re.compile(r'case MAVLINK_MSG_ID_([A-Z0-9_]*)')
    # regex for commands handled by the autopilot
    INCOMING_COMMANDS = re.compile(r'case (MAV_CMD_[A-Z0-9_]*)')
    # regex for messages that can be requested from the autopilot
    REQUESTABLE_REGION = re.compile(r' map\[\]([^;]*);')
    REQUESTABLE_MAP = re.compile(r'MAVLINK_MSG_ID_([A-Z0-9_]*),\s*MSG_([A-Z0-9_]*)')
    # regex for messages the autopilot might send, but cannot be requested
    OUTGOING_MESSAGES = re.compile(r'mavlink_msg_([a-z0-9_]*)_send\(')
    # regex for extracting messages in stream groups
    STREAM_GROUPS = re.compile(r'ap_message STREAM_([A-Z0-9_]*)_msgs\[\] = \{([^\}]*)')
    # the word boundary avoids matching options like AP_MAVLINK_MSG_WIND_ENABLED
    AP_MESSAGE = re.compile(r'\bMSG_([A-Z0-9_]*)')
    # regex for the parameter that sets the rate of each stream group
    STREAM_PARAMETER = re.compile(
        r'AP_GROUPINFO\("(\w+)"[^\n]*streamRates\[GCS_MAVLINK::STREAM_(\w+)\]')
    GCS_PARAMETERS = re.compile(r'GOBJECT\(_gcs,\s*"(\w+)",\s*GCS\)')
    # regex for vehicle type checks and how they are defined for each build
    BUILD_TYPE = re.compile(r'APM_BUILD_TYPE\(APM_BUILD_(\w+)\)')
    BUILD_MACRO = re.compile(r'#define (APM_BUILD_\w+) (\(APM_BUILD_TYPE.*\))')
    EXTRA_BUILD_TYPE = re.compile(r"ap_build_type='(\w+)'")
    # nothing here exists without MAVLink support, so it is not worth listing
    IMPLIED_GUARD = 'HAL_GCS_ENABLED'
    # so the output says it once instead
    IMPLIED_NOTE = ('Everything listed needs MAVLink support ({guard}),'
                    ' which is left out of the Requires column. '
                    'A name in brackets there is the source file an option was found in.')
    # most places to list in a markdown or rst table cell
    PLACE_LIMIT = 3
    # regex for a case that does nothing but call one method and for method definitions
    CASE_LABELS = re.compile(r'\s*:(?:\s*(?:case\s+[\w:]+\s*:|//[^\n]*|/\*.*?\*/))*\s*', re.S)
    CASE_CALL = re.compile(r'(?:return\s+)?(\w+)\s*\([^;{}#]*\)\s*;\s*(?://[^\n]*\s*)*'
                           r'(?:break\s*;|case\s|default\s*:|\})')
    METHOD = re.compile(r'^\w[^\n;(]*?\b(\w+)::(\w+)\s*\([^;{}]*\{', re.M)
    METHOD_END = re.compile(r'^\}', re.M)
    # regex for named values
    NAMED_FLOAT = re.compile(r'send_named_float\("([\w]*)"')
    NAMED_INT = re.compile(r'send_named_int\("([\w]*)"')

    TYPE_DESCRIPTIONS = {
        'incoming_messages':
            'Messages the autopilot handles when received.',
        'requestable_messages':
            'Messages that can be requested/streamed from the autopilot.',
        'outgoing_messages':
            'Messages the autopilot will send automatically (unrequested).',
        'named_floats':
            'Breakout of named floating-point (numerical) values sent by the autopilot.',
        'named_ints':
            'Breakout of named integer values sent by the autopilot.',
    }
    STREAM_PARAMETER_PREFIX = 'SRn_'
    EXTRA_DESCRIPTIONS = {
        'stream_groups':
            f'Message groups with stream rates requestable by `{STREAM_PARAMETER_PREFIX}*` parameters.'
            ' Messages in a group are only sent if the corresponding feature'
            ' is active.',
        'missing_messages':
            'Unsupported / unhandled messages.',
        'incoming_commands':
            TYPE_DESCRIPTIONS['incoming_messages'].replace('Messages','Commands'),
    }
    EXTRA_DESCRIPTIONS['missing_commands'] = \
        EXTRA_DESCRIPTIONS['missing_messages'].replace('messages', 'commands')

    TYPE_OPTIONS = {
        'messages': MAVLinkMessage,
        'commands': MAVLinkCommand,
    }
        
    MAVLINK_URL = 'https://mavlink.io/en/messages/{dialect}.html#{message_name}'
    ARDUPILOT_URL = 'https://github.com/ArduPilot/ardupilot/tree/{branch}/{source}'
    EXPORT_FILETYPES = {
        'csv': 'csv',
        'markdown': 'md',
        'rst': 'rst',
    }

    MARKDOWN_INTRO = (
        'The [MAVLink](https://mavlink.io/en/) protocol supports a variety'
        ' of features and functionalities, but not all'
        ' [messages](https://mavlink.io/en/messages/) or'
        ' [commands](https://mavlink.io/en/services/command.html)'
        ' are implemented by the ArduPilot ecosystem, or relevant to a'
        ' particular autopilot firmware.\n\n'
        'This page is auto-generated from analysing the {vehicle} source'
        ' code, and provides an indication of which messages{commands} are'
        ' handled by, requestable from, and sent from the firmware. '
        'A message being handled does not guarantee full support, but at'
        ' least shows that the autopilot is aware it exists, and will try'
        ' to do something meaningful with it.{requires}{unsupported}{stream_groups}'
    )

    # Convert markdown hyperlinks into rst syntax
    RST_INTRO = MARKDOWN_INTRO.replace('[', '`').replace('](', ' <').replace(')', '>`_')

    VEHICLES = ('AntennaTracker', 'ArduCopter', 'ArduPlane', 'ArduSub', 'Blimp', 'Rover')

    def __init__(self, common_files, vehicle='ALL',
                 exclude_libraries=['SITL', 'AP_Scripting']):
        self.vehicle = vehicle
        vehicles = [vehicle] if vehicle != 'ALL' else self.VEHICLES
        files = chain(*((self.BASE_DIR / vehicle).glob('**/*.cpp') 
                        for vehicle in vehicles),
                      common_files)
        self.incoming_messages = {}
        self.incoming_commands = {}
        self.outgoing_messages = {}
        self.requestable_messages = {}
        self._ap_to_mavlink = {
            'NAMED_FLOAT': 'NAMED_VALUE_FLOAT', # manual inclusion
        }
        self.named_floats = {}
        self.named_ints = {}
        # {(message type, name): (guard, file name) of each place the name was found}
        self._guards = {}
        self._build_types, self._build_macros = self.get_build_types(vehicle)
        self._stream_parameter_prefix = self.STREAM_PARAMETER_PREFIX

        for file in files:
            folder = file.parent.stem
            if folder in exclude_libraries:
                continue

            try:
                text = file.read_text()
            except FileNotFoundError as e:  # Broken symlink
                print(e, file=sys.stderr)
                continue

            source = f'{folder}/{file.name}'
            guards = PreprocessorGuards(text)
            if file == self.COMMON_FILE:
                for mavlink, ap_message in self.find_requestable_messages(text):
                    self.requestable_messages[mavlink] = \
                        MAVLinkMessage(mavlink, source)
                    if ap_message != mavlink:
                        self._ap_to_mavlink[ap_message] = mavlink
                region = self.REQUESTABLE_REGION.search(text)
                self.find_guards('requestable_messages', self.REQUESTABLE_MAP,
                                 region.group(), guards, file.stem, region.start())

            named_types = ('float', 'int') if folder in vehicles else ()
            for type_ in named_types:
                message = f'NAMED_VALUE_{type_.upper()}'
                mavlink_name = MAVLinkMessage.PREFIX + message
                id_number = MAVLinkMessage.KNOWN_DIALECTS['common'][mavlink_name]
                substring = f'named_{type_}s'
                method = getattr(self, f'find_{substring}')
                names = getattr(self, substring)
                new_names = set(method(text)) - names.keys()
                for name in new_names:
                    names[name] = MAVLinkMessage(f'{message}:{name}', source,
                                                 MAVLinkDialect.COMMON, id_number)
                pattern = getattr(self, f'NAMED_{type_.upper()}')
                self.find_guards(substring, pattern, text, guards, file.stem)
            
            for method, data, type_ in (
                (self.find_incoming_messages, self.incoming_messages, 'messages'),
                (self.find_incoming_commands, self.incoming_commands, 'commands'),
                (self.find_outgoing_messages, self.outgoing_messages, 'messages'),
            ):
                new_data = set(method(text)) - data.keys()
                cls = self.TYPE_OPTIONS[type_]
                for datum in new_data:
                    data[datum] = cls(datum, source)

            # a name is only listed with the first file it is found in, but
            # the guards are needed from every file that uses it
            methods = list(self.METHOD.finditer(text))
            for message_type in ('incoming_messages', 'incoming_commands'):
                pattern = getattr(self, message_type.upper())
                self.find_guards(message_type, pattern, text, guards, file.stem,
                                 methods=methods, folder=file.parent)
            self.find_guards('outgoing_messages', self.OUTGOING_MESSAGES, text,
                             guards, file.stem)

            match = self.GCS_PARAMETERS.search(text) if folder in vehicles else None
            if match:
                self._stream_parameter_prefix = f'{match[1]}n_'

            self._supported_names = {'messages': None, 'commands': None}
            self._unsupported = self._supported_names.copy()

        for (message_type, name), found in self._guards.items():
            message = getattr(self, message_type).get(name)
            if message is not None:
                message.requires = self.get_requirements(found)

        self._stream_groups = self.get_stream_groups(vehicle) if len(vehicles) == 1 else []

    def get_description(self, query):
        description = (self.TYPE_DESCRIPTIONS.get(query)
                       or self.EXTRA_DESCRIPTIONS.get(query, ''))
        # the parameter group was renamed when stream groups became shared
        return description.replace(self.STREAM_PARAMETER_PREFIX,
                                   self._stream_parameter_prefix)

    @classmethod
    def find_incoming_messages(cls, text: str):
        return cls.INCOMING_MESSAGES.findall(text)

    @classmethod
    def find_incoming_commands(cls, text: str):
        return cls.INCOMING_COMMANDS.findall(text)

    @classmethod
    def find_outgoing_messages(cls, text: str):
        return (msg.upper() for msg in 
                cls.OUTGOING_MESSAGES.findall(text))

    @classmethod
    def find_requestable_messages(cls, text: str):
        region = cls.REQUESTABLE_REGION.search(text).group()
        return cls.REQUESTABLE_MAP.findall(region)
        
    @classmethod
    def find_named_floats(cls, text: str):
        return cls.NAMED_FLOAT.findall(text)

    @classmethod
    def find_named_ints(cls, text: str):
        return cls.NAMED_INT.findall(text)

    def get_build_types(self, vehicle):
        ''' Returns the APM_BUILD_* types 'vehicle' can be built as, and
            the macros that are shorthand for several vehicle type checks. '''
        if vehicle not in self.VEHICLES:
            return None, {}
        build_types = {vehicle}
        build_macros = {}
        try:
            # helicopters are built from the ArduCopter folder as their own type
            wscript = (self.BASE_DIR / vehicle / 'wscript').read_text()
            build_types.update(self.EXTRA_BUILD_TYPE.findall(wscript))
            build_macros.update(self.BUILD_MACRO.findall(self.BUILD_TYPE_FILE.read_text()))
        except FileNotFoundError as e:  # older branches are laid out differently
            print(e, file=sys.stderr)
        return build_types, build_macros

    def find_guards(self, message_type: str, pattern, text: str, guards,
                    file_name: str, offset=0, methods=None, folder=None):
        ''' Records the preprocessor guards around each match of 'pattern'. '''
        for match in pattern.finditer(text):
            name = match[1]
            if message_type == 'outgoing_messages':
                name = name.upper()
            guard = guards.at(offset + match.start())
            if methods and guard is not None:
                guard += self.find_method_guard(text, guards, match.end(), methods, folder)
            self._guards.setdefault((message_type, name), set()).add((guard, file_name))

    def find_method_guard(self, text: str, guards, position: int, methods, folder):
        ''' Returns the guards of the method a case hands over to.

        Only followed when the case at 'position' does nothing but call one
        method of its own class that is defined once in the same file and is
        not virtual. The guards are the ones around that definition and the
        ones that every line of its body sits within. '''
        labels = self.CASE_LABELS.match(text, position)
        call = self.CASE_CALL.match(text, labels.end()) if labels else None
        owners = [method[1] for method in methods if method.start() < position]
        if not call or not owners:
            return ()
        found = [method for method in methods
                 if method[1] == owners[-1] and method[2] == call[1]]
        end = self.METHOD_END.search(text, found[0].end()) if len(found) == 1 else None
        if not end:
            return ()
        body = [guards.at(start)
                for start, line in self.get_lines(text, found[0].end(), end.start())
                if line.strip() and not line.lstrip().startswith(('#', '//'))]
        body = [guard for guard in body if guard is not None]
        if not body:
            return ()
        shared = tuple(term for term in body[0]
                       if not any(term not in guard for guard in body))
        # a subclass could replace a virtual method, so its guards say nothing
        virtual = re.compile(rf'\bvirtual\b[^;{{]*\b{call[1]}\s*\(')
        if shared and any(virtual.search(header.read_text(errors='replace'))
                          for header in folder.glob('*.h')):
            return ()
        return shared

    @staticmethod
    def get_lines(text: str, start: int, end: int):
        ''' Yields the (position, line) of each line of 'text' after the line
            'start' is on, up to 'end'. '''
        position = text.find('\n', start) + 1
        for line in text[position:end].split('\n'):
            yield position, line
            position += len(line) + 1

    def resolve_vehicle(self, term: str):
        ''' Returns True or False if this vehicle decides 'term', and
            otherwise what is left of it once the vehicle is accounted for. '''
        inner = PreprocessorGuards.negated(term)
        if inner is not None:
            state = self.resolve_vehicle(inner)
            return term if isinstance(state, str) else not state

        options = PreprocessorGuards.split(self._build_macros.get(term, term), '||')
        if len(options) > 1:
            states = [self.resolve_vehicle(option) for option in options]
            undecided = [state for state in states if isinstance(state, str)]
            build_types = {match[1] for state in undecided
                           if (match := self.BUILD_TYPE.fullmatch(state))}
            # true for every way of building this vehicle is also decided
            if True in states or build_types >= self._build_types:
                return True
            if not undecided:
                return False
            if len(undecided) == len(options):
                return term
            return ' || '.join(f'({state})' if '&&' in state else state
                               for state in undecided)

        match = self.BUILD_TYPE.fullmatch(term)
        if not match:
            return term
        if match[1] not in self._build_types:
            return False
        return True if len(self._build_types) == 1 else term

    def resolve_guard(self, guard):
        ''' Returns the terms of 'guard' this vehicle leaves open. Returns
            None if the guarded code is never part of this vehicle. '''
        if guard is None:
            return None
        states = [term for term in guard if term != self.IMPLIED_GUARD]
        if self._build_types is not None:
            states = [self.resolve_vehicle(term) for term in states]
        if False in states:
            return None
        return tuple(dict.fromkeys(state for state in states if state is not True))

    @staticmethod
    def is_stricter(guard, other):
        ''' Returns True if 'guard' holding means that 'other' holds too. '''
        def options(term):
            return set(PreprocessorGuards.split(term, '||'))

        def is_unmet(needed):
            return not any(options(term) <= options(needed) for term in guard)
        # the built-in all() is hidden by the 'all' dialect module
        return not any(is_unmet(needed) for needed in other)

    def get_requirements(self, places):
        ''' Returns the most general requirements from 'places', the
            (guard, file name) of each use of a name, as (condition, files)
            for each distinct guard that is left. '''
        files = {}
        for guard, file_name in places:
            guard = self.resolve_guard(guard)
            if guard is not None:
                files.setdefault(guard, set()).add(file_name)
        if not files:
            # every use is in code this vehicle never compiles, so report
            # the conditions as written instead of claiming there are none
            for guard, file_name in places:
                guard = tuple(term for term in guard or (PreprocessorGuards.NEVER,)
                              if term != self.IMPLIED_GUARD)
                files.setdefault(guard, set()).add(file_name)
        # the order of the terms does not matter, so keep one guard per set
        unique = {}
        for guard in sorted(files):
            unique.setdefault(frozenset(guard), (guard, set()))[1].update(files[guard])
        unique = sorted(unique.values())
        # a use that needs less makes the stricter ones redundant,
        # which also means an unguarded use leaves no requirements
        general = [
            (guard, found) for index, (guard, found) in enumerate(unique)
            if not any(self.is_stricter(guard, other)
                       and (other_index < index or not self.is_stricter(other, guard))
                       for other_index, (other, _) in enumerate(unique) if other is not guard)
        ]
        return tuple((PreprocessorGuards.describe(guard), tuple(sorted(found)))
                     for guard, found in general if guard)

    def get_stream_groups(self, vehicle):
        stream_groups = ['stream_groups']

        # the vehicle file's name has changed case between releases
        files = [file for file in (self.BASE_DIR / vehicle).glob('*.cpp')
                 if file.name.lower() == self.STREAM_GROUP_FILE.lower()]
        file = (files or [self.COMMON_STREAM_GROUP_FILE])[0]
        try:
            text = file.read_text()
        except FileNotFoundError:  # No stream groups
            print('Could not find stream groups for', vehicle, file=sys.stderr)
            return []

        guards = PreprocessorGuards(text)
        # the shared file declares each rate parameter by its stream name, so
        # the real parameter names are known. The vehicle files keep the
        # labels they have always had, so stored output does not change.
        parameters = {} if files else {
            stream: self._stream_parameter_prefix + name.lstrip('_')
            for name, stream in self.STREAM_PARAMETER.findall(text)
        }
        for group in self.STREAM_GROUPS.finditer(text):
            group_name, message_data = group.groups()
            parameter = parameters.get(group_name, f'SRn_{group_name}')
            messages = {}
            for match in self.AP_MESSAGE.finditer(message_data):
                guard = self.resolve_guard(guards.at(group.start(2) + match.start()))
                if guard is None:
                    continue # not in this vehicle's group
                name = self._ap_to_mavlink.get(match[1], match[1])
                messages.setdefault(name, set()).add((guard, file.stem))
            stream_groups.extend(sorted(
                MAVLinkMessage(name, parameter, requires=self.get_requirements(found))
                for name, found in messages.items()
            ))

        return stream_groups

    def get_supported(self, type: str, inject_commands=False):
        if type == 'messages':
            for message_type in self.TYPE_DESCRIPTIONS:
                values = getattr(self, message_type).values()
                if not values:
                    continue
                yield message_type
                yield from sorted(values)
                # add in incoming_commands right after incoming_messages
                if inject_commands and message_type == 'incoming_messages':
                    yield from self.get_supported('commands')
        elif type == 'commands':
            yield 'incoming_commands'
            yield from sorted(self.incoming_commands.values())

    def get_supported_names(self, type: str):
        if self._supported_names[type] is None:
            self._supported_names[type] = set(
                m.id_name for m in self.get_supported(type)
                if isinstance(m, MAVLinkMessage)
            )
        return self._supported_names[type]

    def get_unsupported(self, type='messages'):
        if self._unsupported[type] is None:
            supported_messages = self.get_supported_names(type)
            cls = self.TYPE_OPTIONS[type]
            self._unsupported[type] = sorted(
                cls.get_unsupported(supported_messages)
            )

        if self._unsupported[type]:
            yield f'missing_{type}'
            yield from self._unsupported[type]

    def get_iterable(self, include_commands=False, include_stream_groups=False,
                     include_unsupported=False):
        iterables = [self.get_supported('messages', include_commands)]
        if include_stream_groups:
            iterables.append(self._stream_groups)
        if include_unsupported:
            iterables.append(self.get_unsupported('messages'))
            if include_commands:
                iterables.append(self.get_unsupported('commands'))
        return chain(*iterables)

    def printout(self, **iter_options):
        print(self.IMPLIED_NOTE.format(guard=self.IMPLIED_GUARD))
        for data in self.get_iterable(**iter_options):
            match data: # requires Python >= 3.10
                case str() as type_:
                    print(f'\n{type_}:',
                          self.get_description(type_),
                          sep='\n')
                case MAVLinkMessage() as message:
                    print(message)

    def export(self, filename: Path, type='csv', include_commands=False,
               include_stream_groups=False, include_unsupported=False,
               **export_options):
        export_method = getattr(self, f'export_{type}')
        # ensure export_method and get_iterable have the same options specified
        iter_options = dict(
            include_commands = include_commands,
            include_stream_groups = include_stream_groups,
            include_unsupported = include_unsupported,
        )
        with open(filename, 'w') as file:
            export_method(file, self.get_iterable(**iter_options),
                          **export_options, **iter_options)

    def export_csv(self, file, iterable, **ignore):
        file.write('MAVLinkID,MessageName,CodeSource,MAVLinkDialect,MessageType,Requires\n')
        for data in iterable:
            match data:
                case str():
                    current_type = data
                case MAVLinkMessage() as message:
                    *values, _ = message.as_tuple()
                    requires = message.describe_requires()
                    if ',' in requires:
                        requires = f'"{requires}"'
                    print(*values, current_type, requires, sep=',', file=file)

    def export_markdown(self, file, iterable, branch='master', header=None, 
                        use_intro=True, **extra_kwargs):
        if header == 'ArduSub':
            import time
            now = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            date = f'{now[:-2]}:{now[-2:]}' # add colon to the timezone
            header = '\n'.join((
                '+++',
                'title = "MAVLink Support"',
                'description = "MAVLink message support details."',
                f'{date = }', 'template = "docs/page.html"',
                'sort_by = "weight"', 'weight = 20', 'draft = false',
                '[extra]', 'toc = true', 'top = false',
                '+++'
            ))
        if header:
            print(header, file=file)

        if use_intro:
            commands = stream_groups = unsupported = ''
            if extra_kwargs['include_commands']:
                commands = ' (and commands)'
            if extra_kwargs['include_unsupported']:
                unsupported = (
                    '\n\nKnown [unsupported messages](#missing-messages)'
                    f'{commands} are shown at the end.'
                )
            if extra_kwargs['include_stream_groups']:
                stream_groups = (
                    '\n\nThe autopilot includes a set of [stream groups]'
                    '(#stream-groups) for convenience, which allow'
                    ' configuring the stream rates of groups of'
                    ' requestable messages by setting parameter values. '
                    'It is also possible to manually request messages,'
                    ' and request individual messages be streamed at a'
                    ' specified rate.'
                )
            vehicle = self.vehicle.replace('ALL', 'ArduPilot')
             
            requires = '\n\n' + self.IMPLIED_NOTE.format(guard=f'`{self.IMPLIED_GUARD}`')
            print(self.MARKDOWN_INTRO.format(
                vehicle=vehicle, commands=commands, requires=requires,
                stream_groups=stream_groups, unsupported=unsupported
            ), file=file)
            
        for data in iterable:
            match data:
                case str() as type_:
                    heading = type_.title().replace('_', ' ')
                    source_header = (
                        'Code Source' if type_ != 'stream_groups' else
                        'Stream Group Parameter'
                    )
                    print(f'## {heading}',
                          self.get_description(type_),
                          f'\nID | MAVLink Message | {source_header} | MAVLink Dialect | Requires',
                          '--- | --- | --- | --- | ---', sep='\n', file=file)
                case MAVLinkMessage() as message:
                    name, source, dialect, id_number, _ = message.as_tuple()
                    # a pipe ends a table cell, even inside a code span
                    requires = message.describe_requires('`', self.PLACE_LIMIT).replace('|', '\\|')
                    if dialect != MAVLinkDialect.UNKNOWN:
                        msg_url = self.MAVLINK_URL.format(dialect=dialect,
                                                          message_name=name.split(':')[0])
                        name = f'[{name}]({msg_url})'
                    if source != 'UNSUPPORTED' and type_ != 'stream_groups':
                        folder = source.split('/')[0]
                        base = 'libraries/' if folder not in self.VEHICLES else '' 
                        code_url = self.ARDUPILOT_URL.format(branch=branch,
                                                             source=base+source)
                        source = f'[{source}]({code_url})'
                    if id_number is None:
                        id_number = '??'

                    print(' | '.join((f'#{id_number}', name, source, dialect, requires)).rstrip(),
                          file=file)

    def export_rst(self, file, iterable, branch='master', header=None, 
                        use_intro=True, **extra_kwargs):
        if header == 'ardupilot_wiki':
            header = '\n'.join((
                '.. _mavlink_support:',
                '',
                '===============',
                'MAVLink Support',
                '===============',
                '\n',
            ))

        if header:
            print(header, file=file)

        if use_intro:
            commands = stream_groups = unsupported = ''
            if extra_kwargs['include_commands']:
                commands = ' (and commands)'
            if extra_kwargs['include_unsupported']:
                unsupported = (
                    '\n\nKnown :ref:`unsupported messages <mavlink_missing_messages>`'
                    f'{commands} are shown at the end.'
                )
            if extra_kwargs['include_stream_groups']:
                stream_groups = (
                    '\n\nThe autopilot includes a set of :ref:`mavlink_stream_groups`'
                    ' for convenience, which allow configuring the stream rates of'
                    ' groups of requestable messages by setting parameter values. '
                    'It is also possible to manually request messages, and request'
                    ' individual messages be streamed at a specified rate.'
                )
            vehicle = self.vehicle.replace('ALL', 'ArduPilot')
             
            requires = '\n\n' + self.IMPLIED_NOTE.format(guard=f'``{self.IMPLIED_GUARD}``')
            print(self.RST_INTRO.format(
                vehicle=vehicle, commands=commands, requires=requires,
                stream_groups=stream_groups, unsupported=unsupported
            ), '\n', file=file)
            
        for data in iterable:
            match data:
                case str() as type_:
                    reference = f'mavlink_{type_}'
                    heading = type_.title().replace('_', ' ')
                    source_header = (
                        'Code Source' if type_ != 'stream_groups' else
                        'Stream Group Parameter'
                    )
                    print(f'\n.. _{reference}:\n\n{heading}\n{"="*len(heading)}\n',
                          self.get_description(type_).replace('`','``'),
                          '\n.. csv-table::',
                          f'  :header: ID, MAVLink Message, {source_header}, MAVLink Dialect, Requires\n\n',
                          sep='\n', file=file)
                case MAVLinkMessage() as message:
                    name, source, dialect, id_number, _ = message.as_tuple()
                    requires = message.describe_requires('``', self.PLACE_LIMIT)
                    if ',' in requires:
                        requires = f'"{requires}"'
                    if dialect != MAVLinkDialect.UNKNOWN:
                        msg_url = self.MAVLINK_URL.format(dialect=dialect,
                                                          message_name=name.split(':')[0])
                        name = f'`{name} <{msg_url}>`_'
                    if source != 'UNSUPPORTED' and type_ != 'stream_groups':
                        folder = source.split('/')[0]
                        base = 'libraries/' if folder not in self.VEHICLES else '' 
                        code_url = self.ARDUPILOT_URL.format(branch=branch,
                                                             source=base+source)
                        source = f'`{source} <{code_url}>`_'
                    if id_number is None:
                        id_number = '??'

                    print(', '.join((f'  #{id_number}', name, source, dialect, requires)).rstrip(),
                          file=file)


if __name__ == '__main__':
    from inspect import signature
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    detector_init_params = signature(MAVLinkDetector.__init__).parameters
    default_vehicle = detector_init_params['vehicle'].default
    vehicle_options = [default_vehicle, *MAVLinkDetector.VEHICLES]
    default_exclusions = detector_init_params['exclude_libraries'].default
    format_options = [*MAVLinkDetector.EXPORT_FILETYPES, 'none']

    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parse_opts = parser.add_argument_group('parsing options')
    parse_opts.add_argument('-v', '--vehicle', default=default_vehicle,
                            choices=vehicle_options, help='Vehicle folder, or ALL.')
    parse_opts.add_argument('-e', '--exclude-library', action='append',
                            default=default_exclusions,
                            help='Libraries to exclude from the search.')
    parse_opts.add_argument('-c', '--include-commands', action='store_true',
                            help='Include MAVLink commands as well as messages.')
    parse_opts.add_argument('-g', '--include-stream-groups', action='store_true',
                            help='Include stream group message sets in the output.')
    parse_opts.add_argument('-u', '--include-unsupported', action='store_true',
                            help='Include unsupported messages in the output.')
    export_opts = parser.add_argument_group('export options')
    export_opts.add_argument('-q', '--quiet', action='store_true',
                             help='Disable printout, only export a file.')
    export_opts.add_argument('-f', '--format', default='markdown',
                             choices=format_options,
                             help='Desired format for the exported file.')
    export_opts.add_argument('-b', '--branch',
                             help=('The branch to link to in markdown/rst mode.'
                                   ' Defaults to the branch in the working directory.'))
    export_opts.add_argument('--filename', help='Override default filename.')
    export_opts.add_argument('--header', help='Header for the exported markdown/rst file.')
    export_opts.add_argument('--no-intro', action='store_true',
                             help="Flag to not use the automatic markdown/rst intro.")
    
    args = parser.parse_args()

    assert (args.vehicle in MAVLinkDetector.VEHICLES
            or not args.include_stream_groups), \
        'Determining stream groups requires a single vehicle to be specified.'

    common_files = (MAVLinkDetector.BASE_DIR / 'libraries').glob('**/*.cpp')
    messages = MAVLinkDetector(common_files, args.vehicle, args.exclude_library)

    include_options = dict(
        include_commands = args.include_commands,
        include_stream_groups = args.include_stream_groups,
        include_unsupported = args.include_unsupported,
    )

    if not args.quiet:
        messages.printout(**include_options)
    if args.format != 'none':
        ext = messages.EXPORT_FILETYPES[args.format]
        branch = args.branch
        if not branch:
            import subprocess
            pattern = re.compile(r'On branch ([\S]*)')
            result = subprocess.run(['git', 'status'], capture_output=True).stdout
            try:
                branch, = pattern.search(result.decode()).groups()
            except AttributeError as e:
                raise Exception(
                    'No --branch specified, and "git status" failed to find one.'
                    'Please manually specify an ardupilot firmware branch for '
                    'code source hyperlinks (e.g. Sub-4.1) or ensure this '
                    'repository copy is managed by git.'
                )

        filename = (
            args.filename or
            f'{args.vehicle}_{branch}_MAVLink_Messages.{ext}'
        )
        
        messages.export(filename, type=args.format, branch=branch, header=args.header,
                        use_intro=not args.no_intro, **include_options)
