#!/usr/bin/env python
'''
tool to manipulate ArduPilot firmware files, changing default parameters
'''

import os, sys, struct, json, base64, zlib, hashlib

import argparse

class embedded_defaults(object):
    '''class to manipulate embedded defaults in a firmware'''
    def __init__(self, filename):
        self.filename = filename
        self.offset = 0
        self.max_len = 0
        self.extension = os.path.splitext(filename)[1]
        if self.extension.lower() in ['.apj', '.px4']:
            self.load_apj()
        elif self.extension.lower() in ['.abin']:
            self.load_abin()
        else:
            self.load_binary()

    def load_binary(self):
        '''load firmware from binary file'''
        f = open(self.filename,'rb')
        self.firmware = f.read()
        f.close()
        print("Loaded binary file of length %u" % len(self.firmware))

    def load_abin(self):
        '''load firmware from abin file'''
        f = open(self.filename,'r')
        self.headers = []
        while True:
            line = f.readline().rstrip()
            if line == '--':
                break
            self.headers.append(line)
            if len(self.headers) > 50:
                print("Error: too many abin headers")
                sys.exit(1)
        self.firmware = f.read()
        f.close()
        print("Loaded abin file of length %u" % len(self.firmware))

    def load_apj(self):
        '''load firmware from a json apj or px4 file'''
        f = open(self.filename,'r')
        self.fw_json = json.load(f)
        f.close()
        self.firmware = zlib.decompress(base64.b64decode(self.fw_json['image']))
        print("Loaded apj file of length %u" % len(self.firmware))

    def save_binary(self):
        '''save binary file'''
        f = open(self.filename, 'w')
        f.write(self.firmware)
        f.close()
        print("Saved binary of length %u" % len(self.firmware))

    def save_apj(self):
        '''save apj file'''
        self.fw_json['image'] = base64.b64encode(zlib.compress(self.firmware, 9))
        f = open(self.filename,'w')
        json.dump(self.fw_json,f,indent=4)
        f.truncate()
        f.close()
        print("Saved apj of length %u" % len(self.firmware))

    def save_abin(self):
        '''save abin file'''
        f = open(self.filename,'w')
        for i in range(len(self.headers)):
            line = self.headers[i]
            if line.startswith('MD5: '):
                h = hashlib.new('md5')
                h.update(self.firmware)
                f.write('MD5: %s\n' % h.hexdigest())
            else:
                f.write(line+'\n')
        f.write('--\n')
        f.write(self.firmware)
        f.close()
        print("Saved abin of length %u" % len(self.firmware))

    def find(self):
        '''find defaults in firmware'''
        # these are the magic headers from AP_Param.cpp
        magic_str = "PARMDEF"
        param_magic = [ 0x55, 0x37, 0xf4, 0xa0, 0x38, 0x5d, 0x48, 0x5b ]
        while True:
            i = self.firmware[self.offset:].find(magic_str)
            if i == -1:
                print("No param area found")
                return None
            matched = True
            for j in range(len(param_magic)):
                if ord(self.firmware[self.offset+i+j+8]) != param_magic[j]:
                    matched = False
                    break
            if not matched:
                self.offset += i+8
                continue
            self.offset += i
            self.max_len, self.length = struct.unpack("<HH", self.firmware[self.offset+16:self.offset+20])
            return True
    
    def contents(self):
        '''return current contents'''
        contents = self.firmware[self.offset+20:self.offset+20+self.length]
        # remove carriage returns
        contents = contents.replace('\r','')
        return contents

    def set_contents(self, contents):
        '''set new defaults as a string'''
        length = len(contents)
        if length > self.max_len:
            print("Error: Length %u larger than maximum %u" % (length, self.max_len))
            sys.exit(1)
        new_fw = self.firmware[:self.offset+18]
        new_fw += struct.pack("<H", length)
        new_fw += contents
        new_fw += self.firmware[self.offset+20+length:]
        self.firmware = new_fw
        self.length = len(contents)

    def set_file(self, filename):
        '''set defaults to contents of a file'''
        print("Setting defaults from %s" % filename)
        f = open(filename, 'r')
        contents = f.read()
        f.close()
        # remove carriage returns from the file
        contents = contents.replace('\r','')
        self.set_contents(contents)

    def split_multi(self, str, separators):
        '''split a string, handling multiple separators'''
        for sep in separators:
            str = str.replace(sep, ' ')
        return str.split()

    def set_one(self, set):
        '''set a single parameter'''
        v = set.split('=')
        if len(v) != 2:
            print("Error: set takes form NAME=VALUE")
            sys.exit(1)
        param_name = v[0].upper()
        param_value = v[1]
        
        contents = self.contents()
        lines = contents.strip().split('\n')
        changed = False
        for i in range(len(lines)):
            a = self.split_multi(lines[i], ", =\t")
            if len(a) != 2:
                continue
            if a[0].upper() == param_name:
                separator=lines[i][len(param_name)]
                print("Changing %s from %s to %s" % (param_name, a[1], param_value))
                lines[i] = '%s%s%s' % (param_name, separator, param_value)
                changed = True
        if not changed:
            print("Adding %s=%s" % (param_name, param_value))
            lines.append('%s=%s' % (param_name, param_value))
        contents = '\n'.join(lines)
        contents = contents.lstrip() + '\n'
        self.set_contents(contents)

    def save(self):
        '''save new firmware'''
        if self.extension.lower() in ['.apj', '.px4']:
            self.save_apj()
        elif self.extension.lower() in ['.abin']:
            self.save_abin()
        else:
            self.save_binary()

    def extract(self):
        '''extract firmware image to *.bin'''
        a = os.path.splitext(self.filename)
        if len(a) == 1:
            a.append('.bin')
        else:
            a = (a[0], '.bin')
            binfile = ''.join(a)
            print("Extracting firmware to %s" % binfile)
            f = open(binfile,'w')
            f.write(self.firmware)
            f.close()
    

def defaults_contents(firmware, ofs, length):
    '''return current defaults contents'''
    return firmware

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='manipulate parameter defaults in an ArduPilot firmware')

    parser.add_argument('firmware_file')
    parser.add_argument('--set-file', type=str, default=None, help='replace parameter defaults from a file')
    parser.add_argument('--set', type=str, default=None, help='replace one parameter default, in form NAME=VALUE')
    parser.add_argument('--show', action='store_true', default=False, help='show current parameter defaults')
    parser.add_argument('--extract', action='store_true', default=False, help='extract firmware image to *.bin')

    args = parser.parse_args()

    defaults = embedded_defaults(args.firmware_file)

    if not defaults.find():
        print("Error: Param defaults support not found in firmware")
        sys.exit(1)
    
    print("Found param defaults max_length=%u length=%u" % (defaults.max_len, defaults.length))

    if args.set_file:
        # load new defaults from a file
        defaults.set_file(args.set_file)
        defaults.save()

    if args.set:
        # set a single parameter
        defaults.set_one(args.set)
        defaults.save()

    if args.show:
        # show all defaults
        print(defaults.contents())

    if args.extract:
        defaults.extract()
