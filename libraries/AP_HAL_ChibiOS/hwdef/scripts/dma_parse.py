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
                table[k] += [(dma_num, stream, channel)]

def error(str):
    '''show an error and exit'''
    print("Error: " + str)
    sys.exit(1)

def check_full_table(table):
    '''check the table is not missing rows or columns
       we should have at least one entry in every row and one entry in every colum of each dma table
    '''
    stream_mask = [0,0]
    channel_mask = [0,0]
    for k in table:
        for v in table[k]:
            (engine,stream,channel) = v
            if engine > 2 or engine < 1:
                error("Bad entry for %s: %s" % (k, v))
            stream_mask[engine-1] |= 1<<stream
            channel_mask[engine-1] |= 1<<channel
    for i in range(2):
        for c in range(8):
            if not ((1<<c) & channel_mask[i]):
                error("Missing channel %u for dma table %u" % (c, i))
            if not ((1<<c) & stream_mask[i]):
                error("Missing stream %u for dma table %u" % (c, i))
    

table = {}

if len(sys.argv) != 2:
    print("Error: expected a CSV files and output file")
    sys.exit(1)

parse_dma_table(sys.argv[1], table)

check_full_table(table)

sys.stdout.write("DMA_Map = {\n");
sys.stdout.write('\t# format is (DMA_TABLE, StreamNum, Channel)\n')
sys.stdout.write('\t# extracted from %s\n' % os.path.basename(sys.argv[1]))

for k in sorted(table.iterkeys()):
    s = '"%s"' % k
    sys.stdout.write('\t%-10s\t:\t[' % s)
    for i in range(len(table[k])):
        sys.stdout.write("(%u,%u,%u)" % (table[k][i][0], table[k][i][1], table[k][i][2]))
        if i < len(table[k])-1:
            sys.stdout.write(",")
    sys.stdout.write("],\n")
sys.stdout.write("}\n");
