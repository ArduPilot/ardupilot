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
    if str[1] not in "ABCDEFGH":
        return False
    try:
        p = int(str[2:])
        if p < 0 or p > 15:
            return False
        return True
    except ValueError:
        return False

def parse_af_table(fname, table):
    csvt = csv.reader(open(fname,'rb'))
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
            continue
        pin = row[0]
        for i in range(len(aflist)):
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
for k in sorted(table.keys()):
    s = '"' + k + '"'
    sys.stdout.write('\t%-20s\t:\t%s,\n' % (s, table[k]))
sys.stdout.write("}\n");
