# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Build native modules selected with --module=PATH and embed them in ROMFS.

PATH is relative to libraries/AP_Module and must contain a wscript using:

    def configure(cfg):
        cfg.ap_module_check_rust()

    def build(bld):
        bld.ap_module(name='example', source='main.rs', stack_size=16384, heap_size=32768)
"""

import os
import re
import subprocess

from waflib import Configure, Task, Utils
from waflib.Configure import conf

MODULE_ROOT = 'libraries/AP_Module'
# host-call stack margin added by the loader, see AP_Module README
STACK_MARGIN = {'thumbv7em-none-eabihf': 4096, 'x86_64-unknown-linux-gnu': 16384}
# AP_Module parameter defaults when no module raises them
DEFAULT_HEAP_MAX = 32768
DEFAULT_STACK_MAX = 32768
DEFAULT_CODE_MAX = 131072


def module_target(env):
    '''rust target and CPU profile for the board being configured'''
    if env.BOARD_CLASS == 'ChibiOS':
        cpu = 'cortex-m7' if env.CORTEX == 'cortex-m7' else 'cortex-m4'
        return 'thumbv7em-none-eabihf', cpu
    if env.BOARD_CLASS in ('SITL', 'LINUX') and env.DEST_CPU in ('x86_64', 'amd64', None, ''):
        return 'x86_64-unknown-linux-gnu', None
    return None, None


def min_rust_version(cfg):
    path = cfg.srcnode.find_node('Tools/modules/rust/rust-toolchain-version')
    return path.read().strip()


def install_help(target):
    return (
        "\nInstall the Rust tools needed for native modules with:\n"
        "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh\n"
        "  rustup update stable\n"
        f"  rustup target add {target}\n"
        "  python3 -m pip install pyelftools\n"
        "or set RUSTC to a suitable rustc. See libraries/AP_Module/README.md\n"
    )


@conf
def ap_module_check_rust(cfg):
    '''check for rustc, the board's rust target and pyelftools'''
    if cfg.env.AP_MODULE_RUST_CHECKED:
        return
    target, _ = module_target(cfg.env)
    if target is None:
        cfg.fatal('native modules are not supported on board %s' % cfg.env.BOARD)
    rustc = os.environ.get('RUSTC', 'rustc')
    minimum = min_rust_version(cfg)
    cfg.start_msg('Checking for rustc >= %s' % minimum)
    try:
        version = subprocess.check_output([rustc, '--version'], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        cfg.end_msg('not found', color='RED')
        cfg.fatal('rustc not found' + install_help(target))
    m = re.match(r'rustc ([0-9]+)\.([0-9]+)\.([0-9]+)', version)
    if m is None or tuple(map(int, m.groups())) < tuple(map(int, minimum.split('.'))):
        cfg.end_msg(version, color='RED')
        cfg.fatal('rustc %s or newer is required' % minimum + install_help(target))
    cfg.end_msg(version)

    cfg.start_msg('Checking for rust target %s' % target)
    sysroot = subprocess.check_output([rustc, '--print', 'sysroot'], text=True).strip()
    if not os.path.isdir(os.path.join(sysroot, 'lib', 'rustlib', target)):
        cfg.end_msg('not installed', color='RED')
        cfg.fatal('rust target %s is not installed' % target + install_help(target))
    cfg.end_msg('yes')

    cfg.start_msg('Checking for pyelftools')
    try:
        import elftools  # noqa: F401
    except ImportError:
        cfg.end_msg('not found', color='RED')
        cfg.fatal('python pyelftools is not installed' + install_help(target))
    cfg.end_msg('yes')
    cfg.env.AP_MODULE_RUST_CHECKED = True


@conf
def ap_module(ctx, name, stack_size, heap_size, source='main.rs', thread_priority=None):
    '''declare a native module; at configure time only its limits are recorded'''
    if isinstance(ctx, Configure.ConfigurationContext):
        ctx.env.AP_MODULE_SETTINGS += [(name, stack_size, heap_size)]
        return
    target, cpu = module_target(ctx.env)
    src = ctx.path.find_node(source)
    if src is None:
        ctx.fatal('native module %s: %s not found' % (name, source))
    out = ctx.bldnode.find_or_declare('modules/%s.apmod' % name)
    cmd = ['${PYTHON}', ctx.srcnode.find_node('Tools/modules/build.py').abspath(),
           src.abspath(), out.abspath(), '--target', target,
           '--stack-size', str(stack_size), '--heap-size', str(heap_size)]
    if cpu:
        cmd += ['--cpu', cpu]
    if thread_priority is not None:
        cmd += ['--thread-priority', str(thread_priority)]
    deps = ctx.path.ant_glob('**/*.rs', excl=['**/target/**'])
    root = ctx.srcnode.find_node(MODULE_ROOT)
    deps += root.ant_glob('sdk/rust/src/*.rs generator/*.py generator/*.json generator/templates/**')
    deps += ctx.srcnode.ant_glob('Tools/modules/*.py Tools/modules/rust/*.py Tools/modules/module.ld '
                                 'libraries/AP_Scripting/generator/description/bindings.desc')
    tg = ctx(
        rule=' '.join(cmd),
        source=deps,
        target=out,
        name='ap_module_' + name,
        cwd=ctx.srcnode.abspath(),
        cls_str=lambda task: 'Native module',
        cls_keyword=lambda task: 'Building',
    )
    tg.post()
    # module builds share the generated SDK under build/modules
    for t in tg.tasks:
        t.semaphore = module_semaphore
    ctx.ap_module_outputs.append(('modules/%s.apmod' % name, out))


module_semaphore = Task.TaskSemaphore(1)


def configure(cfg):
    '''called from the top wscript with the --module list'''
    paths = [p.strip('/') for opt in cfg.options.module for p in opt.split(',') if p.strip('/')]
    if not paths:
        return
    if cfg.options.disable_modules:
        cfg.fatal('--module cannot be used with --disable-modules')
    cfg.env.AP_MODULES = paths
    cfg.env.AP_MODULE_SETTINGS = []
    for p in paths:
        d = cfg.srcnode.find_node(os.path.join(MODULE_ROOT, p))
        if d is None or d.find_node('wscript') is None:
            cfg.fatal('--module=%s: no wscript in %s' % (p, os.path.join(MODULE_ROOT, p)))
        cfg.msg('Native module', p)
        cfg.recurse(d.abspath())
        # record stack/heap limits declared by the build rule
        cfg.recurse(d.abspath(), name='build')
        cfg.files.append(d.find_node('wscript').abspath())
        cfg.hash = Utils.h_list((cfg.hash, d.find_node('wscript').read('rb')))
    target, _ = module_target(cfg.env)
    margin = STACK_MARGIN[target]
    heap = max([DEFAULT_HEAP_MAX] + [s[2] for s in cfg.env.AP_MODULE_SETTINGS])
    stack = max([DEFAULT_STACK_MAX] + [s[1] + margin for s in cfg.env.AP_MODULE_SETTINGS])
    # built-in modules run by default and their declared limits are admitted
    for name, value in (('AP_MODULE_ENABLED', 1),
                        ('AP_MODULE_ENABLE_DEFAULT', 1),
                        ('AP_MODULE_HEAP_MAX_DEFAULT', heap),
                        ('AP_MODULE_STACK_MAX_DEFAULT', stack),
                        ('AP_MODULE_CODE_MAXSIZE_DEFAULT', max(DEFAULT_CODE_MAX, 524288))):
        cfg.undefine(name)
        cfg.define(name, value)


def embed_rule(task):
    import embed
    files = task.generator.romfs_files
    if not embed.create_embedded_h(task.outputs[0].abspath(), files, task.env.ROMFS_UNCOMPRESSED,
                                   prelink_apmods=task.generator.prelink_apmods):
        return 1
    return 0


def build_modules(bld, prelink_apmods):
    '''build the --module list and generate ap_romfs_embedded.h after them'''
    bld.ap_module_outputs = []
    for p in bld.env.AP_MODULES:
        bld.recurse(os.path.join(bld.srcnode.abspath(), MODULE_ROOT, p))
    files = list(bld.env.ROMFS_FILES) + [(name, node.abspath()) for name, node in bld.ap_module_outputs]
    sources = [node for _, node in bld.ap_module_outputs]
    for _, path in bld.env.ROMFS_FILES:
        node = bld.srcnode.find_node(path) if not os.path.isabs(path) else bld.root.find_node(path)
        if node is not None:
            sources.append(node)
    bld(
        rule=embed_rule,
        source=sources,
        target=bld.bldnode.find_or_declare('ap_romfs_embedded.h'),
        romfs_files=files,
        prelink_apmods=prelink_apmods,
        vars=['ROMFS_UNCOMPRESSED'],
        cls_str=lambda task: 'ROMFS',
        cls_keyword=lambda task: 'Embedding',
    )
    return files
