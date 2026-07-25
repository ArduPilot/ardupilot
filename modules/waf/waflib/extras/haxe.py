import re

from waflib import Utils, Task, Errors, Logs
from waflib.Configure import conf
from waflib.TaskGen import extension, taskgen_method

HAXE_COMPILERS = {
    'JS': {'tgt': '--js', 'ext_out': ['.js']},
    'LUA': {'tgt': '--lua', 'ext_out': ['.lua']},
    'SWF': {'tgt': '--swf', 'ext_out': ['.swf']},
    'NEKO': {'tgt': '--neko', 'ext_out': ['.n']},
    'PHP': {'tgt': '--php', 'ext_out': ['.php']},
    'CPP': {'tgt': '--cpp', 'ext_out': ['.h', '.cpp']},
    'CPPIA': {'tgt': '--cppia', 'ext_out': ['.cppia']},
    'CS': {'tgt': '--cs', 'ext_out': ['.cs']},
    'JAVA': {'tgt': '--java', 'ext_out': ['.java']},
    'JVM': {'tgt': '--jvm', 'ext_out': ['.jar']},
    'PYTHON': {'tgt': '--python', 'ext_out': ['.py']},
    'HL': {'tgt': '--hl', 'ext_out': ['.hl']},
    'HLC': {'tgt': '--hl', 'ext_out': ['.h', '.c']},
}

@conf
def check_haxe_pkg(self, **kw):
    self.find_program('haxelib')
    libs = kw.get('libs')
    if not libs or not (type(libs) == str or (type(libs) == list and all(isinstance(s, str) for s in libs))):
        self.fatal('Specify correct libs value in ensure call')
        return
    fetch = kw.get('fetch')
    if not fetch is None and not type(fetch) == bool:
        self.fatal('Specify correct fetch value in ensure call')

    libs = [libs] if type(libs) == str else libs
    halt = False
    for lib in libs:
        try:
            self.start_msg('Checking for library %s' % lib)
            output = self.cmd_and_log(self.env.HAXELIB + ['list', lib])
        except Errors.WafError:
            self.end_msg(False)
            self.fatal('Can\'t run haxelib list, ensuring halted')
            return

        if lib in output:
            self.end_msg(lib in output)
        else:
            if not fetch:
                self.end_msg(False)
                halt = True
                continue
            try:
                status = self.exec_command(self.env.HAXELIB + ['install', lib])
                if status:
                    self.end_msg(False)
                    self.fatal('Can\'t get %s with haxelib, ensuring halted' % lib)
                    return
                else:
                    self.end_msg('downloaded', color='YELLOW')
            except Errors.WafError:
                self.end_msg(False)
                self.fatal('Can\'t run haxelib install, ensuring halted')
                return
        postfix = kw.get('uselib_store') or lib.upper()
        self.env.append_unique('LIB_' + postfix, lib)

    if halt:
        self.fatal('Can\'t find libraries in haxelib list, ensuring halted')
        return

class haxe(Task.Task):
    vars = ['HAXE_VERSION', 'HAXE_FLAGS']
    ext_in = ['.hx']

    def run(self):
        cmd = self.env.HAXE + self.env.HAXE_FLAGS_DEFAULT + self.env.HAXE_FLAGS
        return self.exec_command(cmd)

for COMP in HAXE_COMPILERS:
    # create runners for each compile target
    type("haxe_" + COMP, (haxe,), {'ext_out': HAXE_COMPILERS[COMP]['ext_out']})

@taskgen_method
def init_haxe(self):
    errmsg = '%s not found, specify correct value'
    try:
        compiler = HAXE_COMPILERS[self.compiler]
        comp_tgt = compiler['tgt']
        comp_mod = '/main.c' if self.compiler == 'HLC' else ''
    except (AttributeError, KeyError):
        self.bld.fatal(errmsg % 'COMPILER' + ': ' + ', '.join(HAXE_COMPILERS.keys()))
        return

    self.env.append_value(
        'HAXE_FLAGS',
        [comp_tgt, self.path.get_bld().make_node(self.target + comp_mod).abspath()])
    if hasattr(self, 'use'):
        if not (type(self.use) == str or type(self.use) == list):
            self.bld.fatal(errmsg % 'USE')
            return
        self.use = [self.use] if type(self.use) == str else self.use

        for dep in self.use:
            if self.env['LIB_' + dep]:
                for lib in self.env['LIB_' + dep]:
                    self.env.append_value('HAXE_FLAGS', ['-lib', lib])

    if hasattr(self, 'res'):
        if not type(self.res) == str:
            self.bld.fatal(errmsg % 'RES')
            return
        self.env.append_value('HAXE_FLAGS', ['-D', 'resourcesPath=%s' % self.res])

@extension('.hx')
def haxe_hook(self, node):
    if len(self.source) > 1:
        self.bld.fatal('Use separate task generators for multiple files')
        return

    src = node
    tgt = self.path.get_bld().find_or_declare(self.target)

    self.init_haxe()
    self.create_task('haxe_' + self.compiler, src, tgt)

@conf
def check_haxe(self, mini=None, maxi=None):
    self.start_msg('Checking for haxe version')
    try:
        curr = re.search(
            r'(\d+.?)+',
            self.cmd_and_log(self.env.HAXE + ['-version'])).group()
    except Errors.WafError:
        self.end_msg(False)
        self.fatal('Can\'t get haxe version')
        return

    if mini and Utils.num2ver(curr) < Utils.num2ver(mini):
        self.end_msg('wrong', color='RED')
        self.fatal('%s is too old, need >= %s' % (curr, mini))
        return
    if maxi and Utils.num2ver(curr) > Utils.num2ver(maxi):
        self.end_msg('wrong', color='RED')
        self.fatal('%s is too new, need <= %s' % (curr, maxi))
        return
    self.end_msg(curr, color='GREEN')
    self.env.HAXE_VERSION = curr

def configure(self):
    self.env.append_value(
        'HAXE_FLAGS_DEFAULT',
        ['-D', 'no-compilation', '-cp', self.path.abspath()])
    Logs.warn('Default flags: %s' % ' '.join(self.env.HAXE_FLAGS_DEFAULT))
    self.find_program('haxe')
