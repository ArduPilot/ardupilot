#! /usr/bin/env python
# encoding: utf-8

import sys
import os
import codecs
import bottle
import shutil

def stpl(tsk):
    ps = tsk.inputs[0].abspath()
    pt = tsk.outputs[0].abspath()
    bld = tsk.generator.bld
    lookup,name=os.path.split(ps)
    st=bottle.template(name,template_lookup=[lookup], company = bld.env.company, guiname=bld.env.guiname, version=bld.env.version,
            dllname=bld.env.dllname, maxfuni=bld.env.maxfuni)
    with codecs.open(pt,mode='w',encoding="utf-8") as f: f.write(st)
    os.chmod(pt, 493)

# copy files that already exist
def src2bld(self, filename):
    self(features='subst', source=filename, target=filename, is_copy=True)

def build(bld):
    # clean initialization
    bld.src2bld = src2bld
    bld.stpl = stpl

