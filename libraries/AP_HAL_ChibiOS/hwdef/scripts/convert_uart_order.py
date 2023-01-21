#!/usr/bin/env python
'''
convert UART_ORDER in a hwdef.dat into a SERIAL_ORDER
'''

import sys, shlex

def convert_file(fname):
    lines = open(fname, 'r').readlines()
    for i in range(len(lines)):
        if lines[i].startswith('SERIAL_ORDER'):
            print("Already has SERIAL_ORDER: %s" % fname)
            return

    for i in range(len(lines)):
        line = lines[i]
        if not line.startswith('UART_ORDER'):
            continue
        a = shlex.split(line, posix=False)
        if a[0] != 'UART_ORDER':
            continue
        uart_order = a[1:]
        if not fname.endswith('-bl.dat'):
            while len(uart_order) < 4:
                uart_order += ['EMPTY']
                a += ['EMPTY']
            map = [ 0, 2, 3, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12 ]
            for j in range(len(uart_order)):
                a[j+1] = uart_order[map[j]]
        a[0] = 'SERIAL_ORDER'
        print("%s new order " % fname, a)
        lines[i] = ' '.join(a) + '\n'
    open(fname, 'w').write(''.join(lines))

files=sys.argv[1:]
for fname in files:
    convert_file(fname)
