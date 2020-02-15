#!/usr/bin/env python
'''
 create alternate function tables 

This assumes a csv file extracted from the datasheet using tablula:

 https://github.com/tabulapdf/tabula
'''

import sys, csv, os

def is_pin(str):
    '''see if a string is a valid pin name'''
    if len(str) < 3:
        return False
    if str[0] != 'P':
        return False
    if str[1] not in "ABCDEFGHIJK":
        return False
    try:
        p = int(str[2:])
        if p < 0 or p > 15:
            return False
        return True
    except ValueError:
        return False

def pin_compare(p1, p2):
    '''control pin sort order'''
    (p1,f1) = p1.split(':')
    (p2,f2) = p2.split(':')
    port1 = p1[:2]
    port2 = p2[:2]
    pin1 = int(p1[2:])
    pin2 = int(p2[2:])
    #print(port1, pin1, port2, pin2)
    if port1 == port2:
        if pin1 == pin2:
            if f1 < f2:
                return -1
            return 1
        if pin1 < pin2:
            return -1
        return 1
    if port1 < port2:
        return -1
    return 1

def parse_af_table(fname, table):
    csvt = csv.reader(open(fname,'r'))
    i = 0
    aflist = []
    for row in csvt:
        if len(row) > 2 and row[1] == 'AF0':
            # it is a AF list
            aflist = []
            for s in row[1:]:
                if s:
                    aflist.append(int(s[2:]))
        if not is_pin(row[0]):
            if len(row) < 2 or not is_pin(row[1]):
                continue
            row = row[1:]
        pin = row[0]
        for i in range(len(aflist)):
            if len(row) <= i+1:
                break
            af = aflist[i]
            s = row[i+1]
            s = s.replace('_\r', '_')
            s = s.replace('\r_', '_')
            s = s.replace('\r', '')
            s = s.replace(' ', '')
            if s == '-' or len(s) == 0:
                continue
            functions = s.split('/')
            for f in functions:
                table[pin+':'+f.upper()] = af

table = {}

if len(sys.argv) != 2:
    print("Error: expected 1 CSV file")
    sys.exit(1)

parse_af_table(sys.argv[1], table)

sys.stdout.write("AltFunction_map = {\n");
sys.stdout.write('\t# format is PIN:FUNCTION : AFNUM\n')
sys.stdout.write('\t# extracted from %s\n' % os.path.basename(sys.argv[1]))
for k in sorted(table.keys(), cmp=pin_compare):
    s = '"' + k + '"'
    sys.stdout.write('\t%-20s\t:\t%s,\n' % (s, table[k]))
sys.stdout.write("}\n");
