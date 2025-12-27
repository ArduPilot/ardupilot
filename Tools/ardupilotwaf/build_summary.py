# encoding: utf-8

# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

# flake8: noqa

'''
Waf tool for printing build summary. To be used, this must be loaded in the
options(), configure() and build() functions.

This tool expects toolchain tool to be already loaded.

The environment variable BUILD_SUMMARY_HEADER can be used to change the default
header for the targets' summary table.

Extra information can be printed by creating assigning a function to
bld.extra_build_summary. That function must receive bld as the first argument
and this module as the second one.

If one target's task generator (tg) doesn't have a link_task or places the ELF
file at a place different from link_task.outputs[0], then
tg.build_summary['binary'] should be set as the Node object or a path relative
to bld.bldnode for the binary file. Otherwise, size information won't be
printed for that target.
'''
import sys

from waflib import Context, Logs, Node
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature

MAX_TARGETS = 20

header_text = {
    'target': 'Target',
    'binary_path': 'Binary',
    'size_text': 'Text (B)',
    'size_data': 'Data (B)',
    'size_bss': 'BSS (B)',
    'size_total': 'Total Flash Used (B)',
    'size_free_flash': 'Free Flash (B)',
    'ext_flash_used': 'External Flash Used (B)',
}

def text(label, text=''):
    text = text.strip()
    if text:
        Logs.info('%s%s%s%s%s' % (
            Logs.colors.NORMAL,
            Logs.colors.BOLD,
            label,
            Logs.colors.NORMAL,
            text))
    else:
        Logs.info('%s%s%s' % (
            Logs.colors.NORMAL,
            Logs.colors.BOLD,
            label
        ))

def print_table(summary_data_list, header):
    max_widths = []
    table = [[] for _ in range(len(summary_data_list))]

    header_row = []
    for h in header:
        txt = header_text.get(h, h)
        header_row.append(txt)
        max_width = len(txt)
        for i, row_data in enumerate(summary_data_list):
            data = row_data.get(h, '-')

            # Output if a piece of reporting data is not applicable, example: free_flash in SITL
            if data is None:
                data = "Not Applicable"

            txt = str(data)
            table[i].append(txt)

            w = len(txt)
            if w > max_width:
                max_width = w
        max_widths.append(max_width)

    sep = '  '
    fmts = ['{:<%d}' % w for w in max_widths]
    header_row = sep.join(fmts).format(*header_row)
    text(header_row)

    line = ('-' * len(sep)).join('-' * w for w in max_widths)
    print(line)

    for row in table:
        fmts = []
        for j, v in enumerate(row):
            w = max_widths[j]
            try:
                float(v)
            except ValueError:
                fmts.append('{:<%d}' % w)
            else:
                fmts.append('{:>%d}' % w)
        row = sep.join(fmts).format(*row)
        print(row)

def _build_summary(bld):
    Logs.info('')
    text('BUILD SUMMARY')
    text('Build directory: ', bld.bldnode.abspath())

    targets_suppressed = False
    if bld.targets == '*':
        taskgens = bld.get_all_task_gen()
        if len(taskgens) > MAX_TARGETS and not bld.options.summary_all:
            targets_suppressed = True
            taskgens = taskgens[:MAX_TARGETS]
    else:
        targets = bld.targets.split(',')
        if len(targets) > MAX_TARGETS and not bld.options.summary_all:
            targets_suppressed = True
            targets = targets[:MAX_TARGETS]
        taskgens = [bld.get_tgen_by_name(t) for t in targets]

    nodes = []
    filtered_taskgens = []
    for tg in taskgens:
        if not hasattr(tg, 'build_summary'):
            tg.init_summary_data()

        n = tg.build_summary.get('binary', None)
        if not n:
            t = getattr(tg, 'link_task', None)
            if not t:
                continue
            n = t.outputs[0]
            tg.build_summary['binary'] = str(n)

        nodes.append(n)
        filtered_taskgens.append(tg)
    taskgens = filtered_taskgens

    if nodes:
        l = bld.size_summary(nodes)
        for i, data in enumerate(l):
            taskgens[i].build_summary.update(data)

        summary_data_list = [tg.build_summary for tg in taskgens]
        print_table(summary_data_list, bld.env.BUILD_SUMMARY_HEADER)

        if targets_suppressed:
            Logs.info('')
            Logs.pprint(
                'NORMAL',
                '\033[0;31;1mNote: Some targets were suppressed. Use --summary-all if you want information of all targets.',
            )

    if hasattr(bld, 'extra_build_summary'):
        bld.extra_build_summary(bld, sys.modules[__name__])

# totals=True means relying on -t flag to give us a "(TOTALS)" output
def _parse_size_output(s, s_all, totals=False):

    # Get the size of .crash_log and .heap to remove them from .bss reporting as
    # binutils size includes in bss the size of any section that has the ALLOC
    # flag but neither CODE nor HAS_CONTENTS (as reported by objdump -h). also
    # get external flash size if applicable.
    crash_log_size = None
    heap_size = 0
    ext_flash_used = 0
    if s_all is not None:
        lines = s_all.splitlines()[1:]
        for line in lines:
            if ".crash_log" in line:
                row = line.strip().split()
                crash_log_size = int(row[1])
            if ".heap" in line:
                row = line.strip().split()
                heap_size = int(row[1])
            if ".extflash" in line:
                row = line.strip().split()
                if int(row[1]) > 0:
                    ext_flash_used = int(row[1])

    import re
    pattern = re.compile("^.*TOTALS.*$")
    lines = s.splitlines()[1:]
    l = []
    for line in lines:
      if pattern.match(line) or totals is False:
        row = line.strip().split()

        # check if crash_log wasn't found
        # this will be the case for none arm boards: sitl, linux, etc.
        if crash_log_size is None:
            size_bss = int(row[2])
            size_free_flash = None
        else:
            # BSS: remove the portion occupied by crash_log as the command `size binary.elf`
            # reports BSS with crash_log included
            size_bss = int(row[2]) - crash_log_size
            size_free_flash = crash_log_size
        size_bss -= heap_size # remove also-included default heap section size from bss

        l.append(dict(
            size_text=int(row[0]),
            size_data=int(row[1]),
            size_bss=size_bss,
            # Total Flash Cost = Data + Text
            size_total=int(row[0]) + int(row[1]) - ext_flash_used,
            size_free_flash=size_free_flash,
            ext_flash_used= ext_flash_used if ext_flash_used else None,
        ))
    return l

@conf
def size_summary(bld, nodes):
    l = []
    for n in nodes:
        path = n
        if isinstance(n, Node.Node):
            path = n.path_from(bld.bldnode)
        l.append(dict(binary_path=path))

    for d in l:
        if bld.env.SIZE:
            if bld.env.get_flat('SIZE').endswith("xtensa-esp32-elf-size"):
                cmd = [bld.env.get_flat('SIZE')] + ["-t"] + [d['binary_path']]
            else:
                cmd = [bld.env.get_flat('SIZE')] + [d['binary_path']]

                if bld.env.get_flat('SIZE').endswith("arm-none-eabi-size"):
                    cmd2 = [bld.env.get_flat('SIZE')] + ["-A"] + [d['binary_path']]
                    out2 = bld.cmd_and_log(cmd2,
                                        cwd=bld.bldnode.abspath(),
                                        quiet=Context.BOTH,
                                        )
                else:
                    out2 = None

            out = bld.cmd_and_log(
                cmd,
                cwd=bld.bldnode.abspath(),
                quiet=Context.BOTH,
            )
            if bld.env.get_flat('SIZE').endswith("xtensa-esp32-elf-size"):
                parsed = _parse_size_output(out, out2, True)
            else:
                parsed = _parse_size_output(out, out2, False)
            for i, data in enumerate(parsed):
                try:
                    d.update(data)
                except:
                    print("build summary debug: "+str(i)+"->"+str(data))

    return l

@conf
def build_summary_post_fun(bld):
    if not bld.env.AP_PROGRAM_AS_STLIB:
        bld.add_post_fun(_build_summary)

@feature('cprogram', 'cxxprogram')
@before_method('process_rule')
def init_summary_data(self):
    self.build_summary = dict(target=self.name)

def options(opt):
    g = opt.ap_groups['build']

    g.add_option('--summary-all',
        action='store_true',
        help='''Print build summary for all targets. By default, only
information about the first %d targets will be printed.
''' % MAX_TARGETS)

def configure(cfg):
    size_name = 'size'

    if cfg.env.TOOLCHAIN != 'native':
        size_name = cfg.env.TOOLCHAIN + '-' + size_name

    cfg.find_program(size_name, var='SIZE', mandatory=False)

    if not cfg.env.BUILD_SUMMARY_HEADER:
        cfg.env.BUILD_SUMMARY_HEADER = [
            'target',
            'size_text',
            'size_data',
            'size_bss',
            'size_total',
            'size_free_flash',
            'ext_flash_used',
        ]
