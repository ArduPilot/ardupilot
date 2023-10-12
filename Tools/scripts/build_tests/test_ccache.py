#!/usr/bin/env python3
# test ccache efficiency building two similar boards
# AP_FLAKE8_CLEAN

import subprocess
import re
import argparse
import sys
import os

parser = argparse.ArgumentParser(description='test ccache performance')
parser.add_argument('--boards', default='MatekF405,MatekF405-bdshot', help='boards to test')
parser.add_argument('--min-cache-pct', type=int, default=75, help='minimum acceptable ccache hit rate')
parser.add_argument('--display', action='store_true', help='parse and show ccache stats')

args = parser.parse_args()


def ccache_stats():
    '''return hits/misses from ccache -s'''
    hits = 0
    miss = 0
    stats = str(subprocess.Popen(["ccache", "-s"], stdout=subprocess.PIPE).communicate()[0], encoding='ascii')
    for line in stats.split('\n'):
        m = re.match(r"cache.hit\D*(\d+)$", line)
        if m is not None:
            hits += int(m.group(1))

        m = re.match(r"cache.miss\D*(\d+)", line)
        if m is not None:
            miss += int(m.group(1))

        m = re.match(r"\s*Hits:\s*(\d+)", line)
        if m is not None:
            hits += int(m.group(1))

        m = re.match(r"\s*Misses:\s*(\d+)", line)
        if m is not None:
            miss += int(m.group(1))

        if line.startswith("Primary"):
            break
    return (hits, miss)


def build_board(boardname):
    subprocess.run(["./waf", "configure", "--board", boardname])
    subprocess.run(["./waf", "clean", "copter"])


if args.display:
    (hits, misses) = ccache_stats()
    print("Hits=%u misses=%u" % (hits, misses))
    sys.exit(0)

boards = args.boards.split(",")
if len(boards) != 2:
    print(boards)
    print("Must specify exactly 2 boards (comma separated)")
    sys.exit(1)

os.environ['CCACHE_DIR'] = os.path.join(os.getcwd(), 'build', 'ccache')
subprocess.run(["ccache", "--version"])
subprocess.run(["ccache", "-C", "-z"])
build_board(boards[0])
subprocess.run(["ccache", "-z"])
build_board(boards[1])
subprocess.run(["ccache", "-s"])

post = ccache_stats()
hit_pct = 100 * post[0] / float(post[0]+post[1])
print("ccache hit percentage: %.1f%%  %s" % (hit_pct, post))
if hit_pct < args.min_cache_pct:
    print("ccache hits too low, need %d%%" % args.min_cache_pct)
    sys.exit(1)
else:
    print("ccache hits good")
