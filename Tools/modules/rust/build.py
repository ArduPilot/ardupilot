#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build a standalone no_std Rust module with the experimental ArduPilot SDK."""

import argparse
import json
import os
import re
import subprocess
import sys

from pathlib import Path

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pack  # noqa: E402
import targets  # noqa: E402

from targets import TARGETS  # noqa: E402

HERE = Path(__file__).resolve().parents[1]
MIN_RUST_VERSION = (HERE / 'rust/rust-toolchain-version').read_text().strip()
SDK_ROOT = HERE.parents[1] / 'libraries/AP_Module'
GENERATED_ROOT = HERE.parents[1] / 'build/modules/libraries/AP_Module'


def parse_rust_version(output):
    match = re.match(r'^rustc ([0-9]+)\.([0-9]+)\.([0-9]+)(?:[-+][^ ]+)?(?: |$)', output)
    if match is None:
        raise ValueError(f'unrecognised rustc version: {output.strip()}')
    return tuple(int(component) for component in match.groups())


def check_rust_version(output):
    minimum = tuple(int(component) for component in MIN_RUST_VERSION.split('.'))
    actual = parse_rust_version(output)
    if actual < minimum:
        raise ValueError(f'requires Rust {MIN_RUST_VERSION} or newer, got {output.strip()}')


def generate_bindings():
    subprocess.run([
        os.environ.get('PYTHON', 'python3'),
        str(SDK_ROOT / 'generator/generate.py'),
        '--output-dir', str(GENERATED_ROOT),
    ], check=True)
    return GENERATED_ROOT


def sdk_environment():
    generated = generate_bindings()
    return dict(os.environ, AP_MODULE_SDK_GENERATED=str(generated / 'sdk/rust/src'))


def build(source, output, target, stack_size, base=0x10000, cfg=(), debug=False, opt_level='s', heap_size=4096, cpu=None,
          thread_priority=None):
    profile = targets.select(target, cpu)
    thread_priority = profile.default_thread_priority if thread_priority is None else thread_priority
    compiler = os.environ.get('RUSTC', 'rustc')
    version = subprocess.check_output([compiler, '--version'], text=True)
    check_rust_version(version)
    environment = sdk_environment()
    contract = json.loads((GENERATED_ROOT / 'sdk/abi.json').read_text())
    contract.update(cpu_profile=profile.id, stack_size=stack_size, heap_size=heap_size, thread_priority=thread_priority)
    pack.validate_contract(profile.machine, contract['abi_version'], profile.id,
                           stack_size, bytes.fromhex(contract['fingerprint']), heap_size, thread_priority)
    output.parent.mkdir(parents=True, exist_ok=True)
    library = output.with_name(f'lib{output.stem}_sdk.rlib')
    elf = output.with_suffix('.elf')
    arm = profile.machine == 40
    relocation = 'ropi-rwpi' if arm else 'pic'
    common = [compiler, '--edition=2021', '--target', target, '-C', f'target-cpu={profile.cpu}',
              '-C', f'opt-level={opt_level}', '-C', 'panic=abort', '-C', f'relocation-model={relocation}',
              '-C', 'codegen-units=1']
    if debug:
        # Host-only CFI permits debugger backtraces without retaining runtime
        # exception tables or introducing an unwinding personality dependency.
        common += ['-C', 'debuginfo=2', '-C', 'force-frame-pointers=yes',
                   '-C', 'llvm-args=--force-dwarf-frame-section']
    subprocess.run(common + [str(SDK_ROOT / 'sdk/rust/src/lib.rs'), '--crate-name', 'ap_rust_sdk', '--crate-type', 'rlib',
                             '-C', 'embed-bitcode=yes', '-o', str(library)], check=True, env=environment)
    command = common + [str(source), '--extern', f'ap_rust_sdk={library}', '-C', 'lto=fat', '-o', str(elf),
                        '--check-cfg', 'cfg(test)', '--check-cfg', 'cfg(ap_rust_aerobatics_faults)']
    for option in cfg:
        command += ['--cfg', option]
    link_args = [f'-T{HERE / "module.ld"}', '--emit-relocs', f'--defsym=MODULE_BASE={base}']
    if arm:
        link_args += ['--defsym=MODULE_DATA_ALIGNMENT=4']
    if not arm:
        command += ['-C', 'link-arg=-nostdlib', '-C', 'link-arg=-static', '-C', 'link-arg=-no-pie']
        link_args = ['-Wl,' + arg for arg in link_args]
    for argument in link_args:
        command += ['-C', f'link-arg={argument}']
    subprocess.run(command, check=True, env=environment)
    if arm and debug:
        # Debug builds can retain an unused EHABI personality symbol from
        # compiler-builtins. objcopy refuses this if a relocation needs it.
        subprocess.run([os.environ.get('ARM_OBJCOPY', 'arm-none-eabi-objcopy'),
                        '--strip-symbol=__aeabi_unwind_cpp_pr0', str(elf)], check=True, env=environment)
    if not arm:
        # Some prebuilt compiler-builtins retain this undefined symbol after
        # their unwind sections are discarded. Strip it only if unreferenced;
        # objcopy and the packer still reject actual unresolved dependencies.
        subprocess.run([os.environ.get('OBJCOPY', 'objcopy'),
                        '--strip-symbol=rust_eh_personality', str(elf)], check=True, env=environment)
    output.write_bytes(pack.pack_elf(elf.read_bytes(), contract))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', choices=TARGETS, required=True)
    parser.add_argument('--cpu', choices=('cortex-m4', 'cortex-m7', 'x86-64'),
                        help='module CPU requirement (default depends on target)')
    parser.add_argument('--stack-size', type=int, required=True, help='declared module stack requirement in bytes')
    parser.add_argument('--heap-size', type=int, required=True, help='initial module heap in bytes, capped by MOD_HEAP_MAX')
    parser.add_argument('--thread-priority', type=int, help='native thread priority (default depends on target)')
    parser.add_argument('--debug', action='store_true', help='keep full DWARF in the host ELF and use frame pointers')
    parser.add_argument('--opt-level', choices=('0', '1', '2', '3', 's', 'z'), default='s', help='Rust optimisation level')
    args = parser.parse_args()
    build(args.source, args.output, args.target, args.stack_size, debug=args.debug,
          opt_level=args.opt_level, heap_size=args.heap_size, cpu=args.cpu, thread_priority=args.thread_priority)


if __name__ == '__main__':
    main()
