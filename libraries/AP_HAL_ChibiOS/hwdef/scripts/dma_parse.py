#!/usr/bin/env python
'''
extra DMA mapping tables from a stm32 datasheet

This assumes a csv file extracted from the datasheet using tablula:
 https://github.com/tabulapdf/tabula

'''

import sys, csv, os

def parse_dma_table(fname, table):
    dma_num = 1
    csvt = csv.reader(open(fname,'rb'))
    i = 0
    last_channel = -1
    for row in csvt:
        if len(row) > 1 and row[1].startswith('Channel '):
            row = row[1:]
        if not row[0].startswith('Channel '):
            continue
        channel = int(row[0].split(' ')[1])
        if channel < last_channel:
            dma_num += 1
        last_channel = channel
        for stream in range(8):
            s = row[stream+1]
            s = s.replace('_\r', '_')
            s = s.replace('\r_', '_')
            if s == '-':
                continue
            keys = s.split()
            for k in keys:
                brace = k.find('(')
                if brace != -1:
                    k = k[:brace]
                if k not in table:
                    table[k] = []
                table[k] += [(dma_num, stream)]

table = {}

if len(sys.argv) != 2:
    print("Error: expected a CSV files and output file")
    sys.exit(1)

parse_dma_table(sys.argv[1], table)

sys.stdout.write("DMA_Map = {\n");
sys.stdout.write('\t# format is [DMA_TABLE, StreamNum]\n')
sys.stdout.write('\t# extracted from %sn\n' % os.path.basename(sys.argv[1]))

for k in sorted(table.iterkeys()):
    s = '"%s"' % k
    sys.stdout.write('\t%-10s\t:\t[' % s)
    for i in range(len(table[k])):
        sys.stdout.write("(%u,%u)" % (table[k][i][0], table[k][i][1]))
        if i < len(table[k])-1:
            sys.stdout.write(",")
    sys.stdout.write("],\n")
sys.stdout.write("}\n");
