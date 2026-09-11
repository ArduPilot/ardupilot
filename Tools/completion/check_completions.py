#!/usr/bin/env python3

"""
Check that the shell completion scripts stay in sync with the options
actually accepted by the tools they complete.

For each tool the "ground truth" long options are extracted from its help
output and compared with the long options declared in the bash and zsh
completion scripts:

  waf            -> ./waf --help              vs bash/_waf, zsh/_waf
  sim_vehicle.py -> sim_vehicle.py --help     vs bash/_sim_vehicle, zsh/_sim_vehicle
  autotest.py    -> autotest.py --help        vs bash/_ap_autotest, zsh/_ap_autotest
  SITL binaries  -> arducopter --help         vs bash/_ap_bin, zsh/_ap_bin

Stale entries (declared in a completion script but unknown to the tool) are
always errors. Missing entries (accepted by the tool but not completed) are
errors too, except for waf where the completion intentionally offers a
curated subset, so they are only reported as warnings.

Run from the ArduPilot root directory:

  ./Tools/completion/check_completions.py

Exits non-zero if any check fails, so it can be used in CI.

AP_FLAKE8_CLEAN
"""

import os
import re
import subprocess
import sys

COMPLETION_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(COMPLETION_DIR))

LONG_OPT_RE = re.compile(r'(?<![\w\-])--[a-zA-Z][a-zA-Z0-9\-]*')


def get_help_long_options(cmd):
    """Run cmd and extract the long options from its help output.

    Only option-declaration lines (starting with whitespace then a dash,
    or a tab-indented --option for the SITL binary help format) are
    considered, so options mentioned inside descriptions are ignored.
    """
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120,
            cwd=ROOT_DIR,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        raise RuntimeError(f"failed to run {' '.join(cmd)}: {e}")
    options = set()
    for line in (proc.stdout + proc.stderr).splitlines():
        m = re.match(r'^\s+(-.*?)(?:\s{2,}|$)', line)
        if m is None:
            continue
        options.update(LONG_OPT_RE.findall(m.group(1)))
    if not options:
        raise RuntimeError(f"no options found in help output of {' '.join(cmd)}")
    return options


def get_bash_long_options(path):
    """Extract the long options statically declared in a bash completion script.

    Options are collected from the opts="..."/opts+="..." accumulation lines
    and from the case patterns handling option arguments.
    """
    options = set()
    with open(path) as f:
        for line in f:
            line = line.split('#')[0]
            m = re.match(r'^\s*opts\+?="(.*)"', line)
            if m is not None:
                options.update(LONG_OPT_RE.findall(m.group(1)))
                continue
            # case patterns such as: -M | --model)
            m = re.match(r'^\s*(-[^)]*)\)\s*$', line)
            if m is not None:
                options.update(LONG_OPT_RE.findall(m.group(1)))
    return options


def get_zsh_long_options(path):
    """Extract the long options declared in a zsh completion script.

    Descriptions inside [...] and action specs after ':' are stripped first
    so that options mentioned in help text are not picked up.
    """
    options = set()
    with open(path) as f:
        for line in f:
            line = line.split('#')[0]
            # only _arguments specs declare options: they all carry a [description]
            if '[' not in line:
                continue
            line = re.sub(r'\[[^]]*\]', '', line)
            for opt in LONG_OPT_RE.findall(line):
                options.add(opt.rstrip('='))
    return options


def check(name, truth, declared, allow_missing, allow_stale, missing_is_error):
    """Compare declared completion options against the tool's real options."""
    errors = 0
    stale = sorted(declared - truth - allow_stale)
    missing = sorted(truth - declared - allow_missing)
    if stale:
        print(f"ERROR: {name}: stale completion entries (not accepted by the tool): {' '.join(stale)}")
        errors += 1
    if missing:
        if missing_is_error:
            print(f"ERROR: {name}: options not completed: {' '.join(missing)}")
            errors += 1
        else:
            # keep warnings readable: waf has hundreds of generated build options
            shown = ' '.join(missing[:15])
            more = f" (and {len(missing) - 15} more)" if len(missing) > 15 else ""
            print(f"WARNING: {name}: {len(missing)} options not completed: {shown}{more}")
    if errors == 0:
        print(f"OK: {name} ({len(declared & truth)} options in sync)")
    return errors


def find_sitl_binary():
    for binary in ['arducopter', 'arduplane', 'ardurover', 'ardusub']:
        path = os.path.join(ROOT_DIR, 'build', 'sitl', 'bin', binary)
        if os.path.exists(path):
            return path
    return None


def main():
    errors = 0

    # waf: the completion intentionally offers a curated subset of options,
    # so missing options are only warnings; stale entries remain errors.
    truth = get_help_long_options(['./waf', '--help'])
    # wscript's add_build_options also registers an all-lower-case alias of
    # every generated --enable-X/--disable-X option with SUPPRESS_HELP, so
    # they are accepted but invisible in the help output. Some of them shadow
    # options explicitly defined in wscript (e.g. --disable-networking).
    truth |= {opt.lower().replace('_', '-') for opt in truth
              if opt.startswith('--enable-') or opt.startswith('--disable-')}
    for shell, parser in [('bash/_waf', get_bash_long_options),
                          ('zsh/_waf', get_zsh_long_options)]:
        declared = parser(os.path.join(COMPLETION_DIR, shell))
        errors += check(
            f"waf vs {shell}",
            truth,
            declared,
            allow_missing=set(),
            # --targets is spelled --targets= in the bash script and is real;
            # --target is the zsh spelling kept for compatibility
            allow_stale={'--target', '--targets'},
            missing_is_error=False,
        )

    # sim_vehicle.py: full parity expected, completion helpers excluded
    truth = get_help_long_options([sys.executable, 'Tools/autotest/sim_vehicle.py', '--help'])
    helpers = {'--list-vehicle', '--list-frame', '--list-locations'}
    for shell, parser in [('bash/_sim_vehicle', get_bash_long_options),
                          ('zsh/_sim_vehicle', get_zsh_long_options)]:
        declared = parser(os.path.join(COMPLETION_DIR, shell))
        errors += check(
            f"sim_vehicle.py vs {shell}",
            truth,
            declared,
            allow_missing=helpers,
            allow_stale=set(),
            missing_is_error=True,
        )

    # autotest.py: full parity expected, completion helpers excluded.
    # the bash script has no static option list (it relies on _parse_help),
    # so only the zsh script is checked.
    truth = get_help_long_options([sys.executable, 'Tools/autotest/autotest.py', '--help'])
    helpers = {'--list', '--list-subtests', '--list-vehicles',
               '--list-vehicles-test', '--list-subtests-for-vehicle'}
    declared = get_zsh_long_options(os.path.join(COMPLETION_DIR, 'zsh/_ap_autotest'))
    errors += check(
        "autotest.py vs zsh/_ap_autotest",
        truth,
        declared,
        allow_missing=helpers,
        allow_stale=set(),
        missing_is_error=True,
    )

    # SITL binaries: full parity expected; needs a built binary
    binary = find_sitl_binary()
    if binary is None:
        print("WARNING: no SITL binary found in build/sitl/bin, skipping binary completion check")
    else:
        truth = get_help_long_options([binary, '--help'])
        helpers = {'--list-models', '--uartA'}  # --uartA is a deprecated alias
        for shell, parser in [('bash/_ap_bin', get_bash_long_options),
                              ('zsh/_ap_bin', get_zsh_long_options)]:
            declared = parser(os.path.join(COMPLETION_DIR, shell))
            errors += check(
                f"{os.path.basename(binary)} vs {shell}",
                truth,
                declared,
                allow_missing=helpers,
                allow_stale=set(),
                missing_is_error=True,
            )

    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
