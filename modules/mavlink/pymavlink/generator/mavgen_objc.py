#!/usr/bin/env python3
'''
parse a MAVLink protocol XML file and generate an Objective-C implementation

Copyright John Boiles 2013
Released under GNU GPL version 3 or later
'''
import os
from . import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_mavlink(directory, xml):
    '''generate MVMavlink header and implementation'''
    f = open(os.path.join(directory, "MVMavlink.h"), mode='w')
    t.write(f,'''
//
//  MVMavlink.h
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created on ${parse_time} by mavgen_objc.py
//  https://mavlink.io/en/
//

#import "MVMessage.h"
${{message_definition_files:#import "MV${name_camel_case}Messages.h"
}}

@class MVMavlink;
@protocol MVMessage;

@protocol MVMavlinkDelegate <NSObject>

/*!
 Method called on the delegate when a full message has been received. Note that this may be called multiple times when parseData: is called, if the data passed to parseData: contains multiple messages.

 @param mavlink The MVMavlink object calling this method
 @param message The id<MVMessage> class containing the parsed message
 */
- (void)mavlink:(MVMavlink *)mavlink didGetMessage:(id<MVMessage>)message;

/*!
 Method called on the delegate when data should be sent.

 @param mavlink The MVMavlink object calling this method
 @param data NSData object containing the bytes to be sent
 */
- (MAV_BOOL)mavlink:(MVMavlink *)mavlink shouldWriteData:(NSData *)data;

@end

/*!
 Class for parsing and sending instances of id<MVMessage>

 @discussion MVMavlink receives a stream of bytes via the parseData: method and calls the delegate method mavlink:didGetMessage: each time a message is fully parsed. Users of MVMavlink can call parseData: anytime they get new data, even if that data does not contain a complete message.
 */
@interface MVMavlink : NSObject
@property (weak, nonatomic) id<MVMavlinkDelegate> delegate;

/*!
 Parse byte data received from a MAVLink byte stream.

 @param data NSData containing the received bytes
 */
- (void)parseData:(NSData *)data;

/*!
 Compile MVMessage object into a bytes and pass to the delegate for sending.

 @param message Object conforming to the MVMessage protocol that represents the data to be sent
 @return YES if message sending was successful
 */
- (MAV_BOOL)sendMessage:(id<MVMessage>)message;

@end
''', xml)
    f.close()
    f = open(os.path.join(directory, "MVMavlink.m"), mode='w')
    t.write(f,'''
//
//  MVMavlink.m
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created by mavgen_objc.py
//  https://mavlink.io/en/
//

#import "MVMavlink.h"

@implementation MVMavlink

- (void)parseData:(NSData *)data {
  mavlink_message_t msg;
  mavlink_status_t status;
  char *bytes = (char *)[data bytes];

  for (NSInteger i = 0; i < [data length]; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &msg, &status)) {
      // Packet received
      id<MVMessage> message = [MVMessage messageWithCMessage:msg];
      [_delegate mavlink:self didGetMessage:message];
    }
  }
}

- (MAV_BOOL)sendMessage:(id<MVMessage>)message {
  return [_delegate mavlink:self shouldWriteData:[message data]];
}

@end
''', xml)
    f.close()

def generate_base_message(directory, xml):
    '''Generate base MVMessage header and implementation'''
    f = open(os.path.join(directory, 'MVMessage.h'), mode='w')
    t.write(f, '''
//
//  MVMessage.h
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created by mavgen_objc.py
//  https://mavlink.io/en/
//

#import "mavlink.h"

@protocol MVMessage <NSObject>
- (id)initWithCMessage:(mavlink_message_t)message;
- (NSData *)data;
@property (readonly, nonatomic) mavlink_message_t message;
@end

@interface MVMessage : NSObject <MVMessage> {
  mavlink_message_t _message;
}

/*!
 Create an MVMessage subclass from a mavlink_message_t.

 @param message Struct containing the details of the message
 @result MVMessage or subclass representing the message
 */
+ (id<MVMessage>)messageWithCMessage:(mavlink_message_t)message;

//! System ID of the sender of the message.
- (uint8_t)systemId;

//! Component ID of the sender of the message.
- (uint8_t)componentId;

//! Message ID of this message.
- (uint8_t)messageId;

@end
''', xml)
    f.close()
    f = open(os.path.join(directory, 'MVMessage.m'), mode='w')
    t.write(f, '''
//
//  MVMessage.m
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created by mavgen_objc.py
//  https://mavlink.io/en/
//

#import "MVMessage.h"
${{message_definition_files:#import "MV${name_camel_case}Messages.h"
}}

@implementation MVMessage

@synthesize message=_message;

+ (id)messageWithCMessage:(mavlink_message_t)message {
  static NSDictionary *messageIdToClass = nil;
  if (!messageIdToClass) {
    messageIdToClass = @{
${{message:      @${id} : [MVMessage${name_camel_case} class],
}}
    };
  }

  Class messageClass = messageIdToClass[@(message.msgid)];
  // Store unknown messages to MVMessage
  if (!messageClass) {
    messageClass = [MVMessage class];
  }

  return [[messageClass alloc] initWithCMessage:message];
}

- (id)initWithCMessage:(mavlink_message_t)message {
  if ((self = [super init])) {
    self->_message = message;
  }
  return self;
}

- (NSData *)data {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  NSInteger length = mavlink_msg_to_send_buffer(buffer, &self->_message);

  return [NSData dataWithBytes:buffer length:length];
}

- (uint8_t)systemId {
  return self->_message.sysid;
}

- (uint8_t)componentId {
  return self->_message.compid;
}

- (uint8_t)messageId {
  return self->_message.msgid;
}

- (NSString *)description {
  return [NSString stringWithFormat:@"%@, systemId=%d, componentId=%d", [self class], self.systemId, self.componentId];
}

@end
''', xml)
    f.close()

def generate_message_definitions_h(directory, xml):
    '''generate headerfile containing includes for all messages'''
    f = open(os.path.join(directory, "MV" + camel_case_from_underscores(xml.basename) + "Messages.h"), mode='w')
    t.write(f, '''
//
//  MV${basename_camel_case}Messages.h
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created by mavgen_objc.py
//  https://mavlink.io/en/
//

${{message:#import "MVMessage${name_camel_case}.h"
}}
''', xml)
    f.close()

def generate_message(directory, m):
    '''generate per-message header and implementation file'''
    f = open(os.path.join(directory, 'MVMessage%s.h' % m.name_camel_case), mode='w')
    t.write(f, '''
//
//  MVMessage${name_camel_case}.h
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created by mavgen_objc.py
//  https://mavlink.io/en/
//

#import "MVMessage.h"

/*!
 Class that represents a ${name} Mavlink message.

 @discussion ${description}
 */
@interface MVMessage${name_camel_case} : MVMessage

- (id)initWithSystemId:(uint8_t)systemId componentId:(uint8_t)componentId${{arg_fields: ${name_lower_camel_case}:(${arg_type}${array_prefix})${name_lower_camel_case}}};

${{fields://! ${description}
- (${return_type})${name_lower_camel_case}${get_arg_objc};

}}
@end
''', m)
    f.close()
    f = open(os.path.join(directory, 'MVMessage%s.m' % m.name_camel_case), mode='w')
    t.write(f, '''
//
//  MVMessage${name_camel_case}.m
//  MAVLink communications protocol built from ${basename}.xml
//
//  Created by mavgen_objc.py
//  https://mavlink.io/en/
//

#import "MVMessage${name_camel_case}.h"

@implementation MVMessage${name_camel_case}

- (id)initWithSystemId:(uint8_t)systemId componentId:(uint8_t)componentId${{arg_fields: ${name_lower_camel_case}:(${arg_type}${array_prefix})${name_lower_camel_case}}} {
  if ((self = [super init])) {
    mavlink_msg_${name_lower}_pack(systemId, componentId, &(self->_message)${{arg_fields:, ${name_lower_camel_case}}});
  }
  return self;
}

${{fields:- (${return_type})${name_lower_camel_case}${get_arg_objc} {
  ${return_method_implementation}
}

}}
- (NSString *)description {
  return [NSString stringWithFormat:@"%@${{fields:, ${name_lower_camel_case}=${print_format}}}", [super description]${{fields:, ${get_message}}}];
}

@end
''', m)
    f.close()

def camel_case_from_underscores(string):
    """generate a CamelCase string from an underscore_string."""
    components = string.split('_')
    string = ''
    for component in components:
        string += component[0].upper() + component[1:]
    return string

def lower_camel_case_from_underscores(string):
    """generate a lower-cased camelCase string from an underscore_string.
    For example: my_variable_name -> myVariableName"""
    components = string.split('_')
    string = components[0]
    for component in components[1:]:
        string += component[0].upper() + component[1:]
    return string

def generate_shared(basename, xml_list):
    # Create a dictionary to hold all the values we want to use in the templates
    template_dict = {}
    template_dict['parse_time'] = xml_list[0].parse_time
    template_dict['message'] = []
    template_dict['message_definition_files'] = []

    print("Generating Objective-C implementation in directory %s" % basename)
    mavparse.mkdir_p(basename)

    for xml in xml_list:
        template_dict['message'].extend(xml.message)
        basename_camel_case = camel_case_from_underscores(xml.basename)
        template_dict['message_definition_files'].append({'name_camel_case': basename_camel_case})
        if not template_dict.get('basename', None):
            template_dict['basename'] = xml.basename
        else:
            template_dict['basename'] = template_dict['basename'] + ', ' + xml.basename

    # Sort messages by ID
    template_dict['message'] = sorted(template_dict['message'], key = lambda message : message.id)

    # Add name_camel_case to each message object 
    for message in template_dict['message']:
        message.name_camel_case = camel_case_from_underscores(message.name_lower)

    generate_mavlink(basename, template_dict)
    generate_base_message(basename, template_dict)

def generate_message_definitions(basename, xml):
    '''generate files for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating Objective-C implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    xml.basename_camel_case = camel_case_from_underscores(xml.basename)

    # Add some extra field attributes for convenience
    for m in xml.message:
        m.basename = xml.basename
        m.parse_time = xml.parse_time
        m.name_camel_case = camel_case_from_underscores(m.name_lower)
        for f in m.fields:
            f.name_lower_camel_case = lower_camel_case_from_underscores(f.name);
            f.get_message = "[self %s]" % f.name_lower_camel_case
            f.return_method_implementation = ''
            f.array_prefix = ''
            f.array_return_arg = ''
            f.get_arg = ''
            f.get_arg_objc = ''
            if f.enum:
                f.return_type = f.enum
                f.arg_type = f.enum
            else:
                f.return_type = f.type
                f.arg_type = f.type
            if f.print_format is None:
                if f.array_length != 0:
                    f.print_format = "%@"
                elif f.type.startswith('uint64_t'):
                    f.print_format = "%lld"
                elif f.type.startswith('uint') or f.type.startswith('int'):
                    f.print_format = "%d"
                elif f.type.startswith('float'):
                    f.print_format = "%f"
                elif f.type.startswith('char'):
                    f.print_format = "%c"
                else:
                    print("print_format unsupported for type %s" % f.type)
            if f.array_length != 0:
                f.get_message = '@"[array of %s[%d]]"' % (f.type, f.array_length)
                f.array_prefix = ' *'
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s' % (f.name)
                f.get_arg_objc = ':(%s *)%s' % (f.type, f.name)
                if f.type == 'char':
                    # Special handling for strings (assumes all char arrays are strings)
                    f.return_type = 'NSString *'
                    f.get_arg_objc = ''
                    f.get_message = "[self %s]" % f.name_lower_camel_case
                    f.return_method_implementation = \
"""char string[%(array_length)d];
  mavlink_msg_%(message_name_lower)s_get_%(name)s(&(self->_message), (char *)&string);
  return [[NSString alloc] initWithBytes:string length:%(array_length)d encoding:NSASCIIStringEncoding];""" % {'array_length': f.array_length, 'message_name_lower': m.name_lower, 'name': f.name}

            if not f.return_method_implementation:
                f.return_method_implementation = \
"""return mavlink_msg_%(message_name_lower)s_get_%(name)s(&(self->_message)%(get_arg)s);""" % {'message_name_lower': m.name_lower, 'name': f.name, 'get_arg': f.get_arg}

    for m in xml.message:
        m.arg_fields = []
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
 
    generate_message_definitions_h(directory, xml)
    for m in xml.message:
        generate_message(directory, m)


def generate(basename, xml_list):
    '''generate complete MAVLink Objective-C implemenation'''

    generate_shared(basename, xml_list)
    for xml in xml_list:
        generate_message_definitions(basename, xml)
