#!/usr/bin/env python3

# flake8: noqa
'''
create additional functions ADC table, used for mapping analog pins to ADC channel numbers

Currently only extracts ADC1 channels

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

def parse_adc_table(fname, table):
    csvt = csv.reader(open(fname,'r'))
    i = 0
    for row in csvt:
        for col in row:
            if is_pin(col) and row[-1].startswith('ADC1'):
                chan = row[-1]
                chan = chan.replace('\r', '')
                chan = chan.replace(' ', '')
                chan = chan.split('/')
                chan = chan[0]
                chan = chan.split(',')
                chan = chan[0]
                a = chan.split('_')
                if len(a) != 2:
                    continue
                chan_num = a[1]
                if chan_num.startswith('IN'):
                    chan_num = chan_num[2:]
                try:
                    chan_num = int(chan_num)
                except Exception:
                    continue
                table[col] = chan_num

table = {}

if len(sys.argv) != 2:
    print("Error: expected 1 CSV file")
    sys.exit(1)

parse_adc_table(sys.argv[1], table)

sys.stdout.write("ADC1_map = {\n");
sys.stdout.write('\t# format is PIN : ADC1_CHAN\n')
sys.stdout.write('\t# extracted from %s\n' % os.path.basename(sys.argv[1]))
for k in sorted(table.keys()):
    s = '"' + k + '"'
    sys.stdout.write('\t%-4s\t:\t%s,\n' % (s, table[k]))
sys.stdout.write("}\n");
