#!/usr/bin/env python
# encoding: utf-8
import subprocess
import sys

ar_p=sys.argv[1]
objcopy_p=sys.argv[2]
functions_lst_p=sys.argv[3]
out_arch_p=sys.argv[4]
in_arch_p=sys.argv[5:]

def parseFunctions(functions_lst):
    f=open(functions_lst, "r")
    funcs=f.read()
    f.close()
    mapping = dict()
    for func in funcs.splitlines():
        place=func.split(":")[0]
        symbol=func.split(":")[1]
        if not mapping.has_key(place):
            mapping[place]=list()
        mapping[place].append(symbol)
    return mapping

def appendArchive(mapping, in_arch, out_arch):
    lines = subprocess.check_output([ar_p,"t", in_arch])
    counting=dict()
    for line in lines.splitlines():
        if not counting.has_key(line):
            counting[line]=0
        counting[line]=counting[line]+1
        subprocess.call([ar_p,"xN", str(counting[line]), in_arch,line])
        fname=line.split(".")[0]
        if mapping.has_key(fname):
            cmd=[objcopy_p]
            for sym in mapping[fname]:
                cmd.append("--rename-section")
                cmd.append(".literal."+sym+"=.iram1.literal."+sym)
                cmd.append("--rename-section")
                cmd.append(".text."+sym+"=.iram1.text."+sym)
                cmd.append("--rename-section")
                cmd.append(".rodata."+sym+"=.dram1."+sym)
            cmd.append(line)
            subprocess.call(cmd)
        subprocess.call([ar_p,"q", out_arch, line])
        subprocess.call(["rm", line])

m=parseFunctions(functions_lst_p)
subprocess.call(["rm", "-f", out_arch_p])
for a in in_arch_p:
    appendArchive(m, a, out_arch_p)
subprocess.call([ar_p,"s", out_arch_p])



