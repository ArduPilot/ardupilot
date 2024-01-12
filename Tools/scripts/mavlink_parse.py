#!/usr/bin/env python

import re
from enum import StrEnum # requires Python >= 3.11
from pathlib import Path
from itertools import chain
from dataclasses import dataclass, astuple

from pymavlink.dialects.v20 import (
    common, icarous, cubepilot, uAvionix, ardupilotmega
)

class MAVLinkDialect(StrEnum):
    # in subset, superset, unknown order, for correct links
    # supported values must match imported dialect names
    COMMON = 'common'
    ICAROUS = 'icarous'
    CUBEPILOT = 'cubepilot'
    UAVIONIX = 'uAvionix'
    ARDUPILOTMEGA = 'ardupilotmega'
    UNKNOWN = 'UNKNOWN'


@dataclass(slots=True, order=True)
class MAVLinkMessage:
    name: str
    source: str
    dialect: MAVLinkDialect = MAVLinkDialect.UNKNOWN

    PREFIX = 'MAVLINK_MSG_ID_'

    # Get message sets, for quick containment checks.
    #  Function is required because of Python's class scoping rules.
    #  See https://stackoverflow.com/questions/13905741.
    def _get_known_messages(prefix):
        ''' Returns a dictionary of {dialect: {messages}} given 'prefix'. '''
        return {
            dialect: set(m for m in dir(globals()[dialect])
                         if m.startswith(prefix))
            for dialect in MAVLinkDialect
            if dialect != MAVLinkDialect.UNKNOWN
        }
    KNOWN_DIALECTS = _get_known_messages(PREFIX)

    # Try to determine dialect if initialised without one specified.
    def __post_init__(self):
        if self.dialect == MAVLinkDialect.UNKNOWN:
            self.determine_dialect()

    @property
    def id_name(self):
        return self.PREFIX + self.name

    def determine_dialect(self):
        for dialect, message_set in self.KNOWN_DIALECTS.items():
            if self.id_name in message_set:
                self.dialect = dialect
                break # dialect found, no need to continue searching
        else:
            self.dialect = MAVLinkDialect.UNKNOWN

    def as_tuple(self):
        return astuple(self)

    def __str__(self):
        return f'{self.name:<45}{self.source:<55}{self.dialect}'

    @classmethod
    def get_unsupported(cls, supported: set, remove_prefix=True):
        ''' Yields known messages that are not included in 'supported'. '''
        offset = len(cls.PREFIX) if remove_prefix else 0
        known_missing = set() # don't double-count for supersets
        for dialect, message_set in cls.KNOWN_DIALECTS.items():
            missing_names = message_set - supported - known_missing
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


class MAVLinkDetector:
    # file paths
    BASE_DIR = Path(__file__).parent / '../..'
    COMMON_FILE = BASE_DIR / 'libraries/GCS_MAVLink/GCS_Common.cpp'
    STREAM_GROUP_FILE = 'GCS_MAVLink.cpp'

    # regex for messages handled by the autopilot
    INCOMING_MESSAGES = re.compile(r'case MAVLINK_MSG_ID_([A-Z0-9_]*)')
    # regex for commands handled by the autopilot
    INCOMING_COMMANDS = re.compile(r'case (MAV_CMD_[A-Z0-9_]*)')
    # regex for messages that can be requested from the autopilot
    REQUESTABLE_REGION = re.compile(' map\[\]([^;]*);')
    REQUESTABLE_MAP = re.compile(r'MAVLINK_MSG_ID_([A-Z0-9_]*),\s*MSG_([A-Z0-9_]*)')
    # regex for messages the autopilot might send, but cannot be requested
    OUTGOING_MESSAGES = re.compile(r'mavlink_msg_([a-z0-9_]*)_send\(')
    # regex for extracting messages in stream groups
    STREAM_GROUPS = re.compile(r'ap_message STREAM_([A-Z0-9_]*)_msgs\[\] = \{([^\}]*)')
    AP_MESSAGE = re.compile(r'MSG_([A-Z0-9_]*)')
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
    EXTRA_DESCRIPTIONS = {
        'stream_groups':
            'Message groups with stream rates requestable by `SRn_*` parameters.'
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
        'markdown': 'md'
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
        ' to do something meaningful with it.{unsupported}{stream_groups}'
    )

    VEHICLES = ('AntennaTracker', 'ArduCopter', 'ArduPlane', 'ArduSub', 'Rover')

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

        for file in files:
            folder = file.parent.stem
            if folder in exclude_libraries:
                continue
            text = file.read_text()
            source = f'{folder}/{file.name}'
            if file == self.COMMON_FILE:
                for mavlink, ap_message in self.find_requestable_messages(text):
                    self.requestable_messages[mavlink] = \
                        MAVLinkMessage(mavlink, source)
                    if ap_message != mavlink:
                        self._ap_to_mavlink[ap_message] = mavlink

            named_types = ('float', 'int') if folder in vehicles else ()
            for type_ in named_types:
                substring = f'named_{type_}s'
                method = getattr(self, f'find_{substring}')
                names = getattr(self, substring)
                new_names = set(method(text)) - names.keys()
                for name in new_names:
                    names[name] = MAVLinkMessage(f'NAMED_VALUE_{type_.upper()}:{name}',
                                                 source, MAVLinkDialect.COMMON)
            
            for method, data, type_ in (
                (self.find_incoming_messages, self.incoming_messages, 'messages'),
                (self.find_incoming_commands, self.incoming_commands, 'commands'),
                (self.find_outgoing_messages, self.outgoing_messages, 'messages'),
            ):
                new_data = set(method(text)) - data.keys()
                cls = self.TYPE_OPTIONS[type_]
                for datum in new_data:
                    data[datum] = cls(datum, source)

            self._supported_names = {'messages': None, 'commands': None}
            self._unsupported = self._supported_names.copy()

        self._stream_groups = self.get_stream_groups(vehicle) if len(vehicles) == 1 else []

    @classmethod
    def get_description(cls, query):
        return cls.TYPE_DESCRIPTIONS.get(query,
            cls.EXTRA_DESCRIPTIONS.get(query, '')
        )

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

    def get_stream_groups(self, vehicle):
        stream_groups = ['stream_groups']

        text = (self.BASE_DIR / vehicle / self.STREAM_GROUP_FILE).read_text()
        for group_name, message_data in self.STREAM_GROUPS.findall(text):
            stream_groups.extend(sorted(
                MAVLinkMessage(self._ap_to_mavlink.get(ap_message, ap_message),
                               f'SRn_{group_name}')
                for ap_message in self.AP_MESSAGE.findall(message_data)
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
        file.write('MAVLinkMessage,CodeSource,MAVLinkDialect,MessageType\n')
        for data in iterable:
            match data:
                case str():
                    current_type = data
                case MAVLinkMessage() as message:
                    print(*message.as_tuple(), current_type, sep=',', file=file)

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
             
            print(self.MARKDOWN_INTRO.format(
                vehicle=vehicle, commands=commands, 
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
                          f'\nMAVLink Message | {source_header} | MAVLink Dialect',
                          '--- | --- | ---', sep='\n', file=file)
                case MAVLinkMessage() as message:
                    name, source, dialect = message.as_tuple()
                    if dialect != MAVLinkDialect.UNKNOWN:
                        msg_url = self.MAVLINK_URL.format(dialect=dialect,
                                                          message_name=name.split(':')[0])
                        name = f'[{name}]({msg_url})'
                    if source != 'UNSUPPORTED' and not source.startswith('SRn'):
                        folder = source.split('/')[0]
                        base = 'libraries/' if folder not in self.VEHICLES else '' 
                        code_url = self.ARDUPILOT_URL.format(branch=branch,
                                                             source=base+source)
                        source = f'[{source}]({code_url})'

                    print(name, source, dialect, sep=' | ', file=file)


if __name__ == '__main__':
    from inspect import signature
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    detector_init_params = signature(MAVLinkDetector.__init__).parameters
    default_vehicle = detector_init_params['vehicle'].default
    vehicle_options = [default_vehicle, *MAVLinkDetector.VEHICLES]
    default_exclusions = detector_init_params['exclude_libraries'].default

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
                             choices=['csv', 'markdown', 'none'],
                             help='Desired format for the exported file.')
    export_opts.add_argument('-b', '--branch',
                             help=('The branch to link to in markdown mode.'
                                   ' Defaults to the branch in the working directory.'))
    export_opts.add_argument('--filename', help='Override default filename.')
    export_opts.add_argument('--header', help='Header for the markdown file.')
    export_opts.add_argument('--no-intro', action='store_true',
                             help="Flag to not use the automatic markdown intro.")
    
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
