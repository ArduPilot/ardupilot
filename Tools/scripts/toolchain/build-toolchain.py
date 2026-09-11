#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Build and package the ArduPilot arm-none-eabi toolchain."""

from __future__ import annotations

import argparse
import hashlib
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys

from pathlib import Path
from typing import Iterable
from typing import Mapping
from typing import Sequence

ROOT_DIR = Path(__file__).resolve().parent
TARGET = "arm-none-eabi"
DEFAULT_SOURCE_DATE_EPOCH = "1785542400"
BUILD_RECIPE_VERSION = "3"
PROFILE_ASSIGNMENT = re.compile(r"^([A-Z][A-Z0-9_]*)=(.*)$")


class BuildError(RuntimeError):
    """An expected build failure with a user-facing diagnostic."""


def note(message: str) -> None:
    print(f"==> {message}", flush=True)


def fail(message: str) -> None:
    raise BuildError(message)


def format_command(argv: Sequence[str]) -> str:
    if len(argv) <= 16:
        return shlex.join(argv)
    return f"{shlex.join(argv[:8])} ... ({len(argv) - 8} more arguments)"


def run(
    command: Sequence[os.PathLike[str] | str],
    *,
    cwd: Path | None = None,
    env: Mapping[str, str] | None = None,
    capture: bool = False,
    stdin=None,
) -> str:
    argv = [os.fspath(item) for item in command]
    printable = format_command(argv)
    print(f"+ {printable}", flush=True)
    result = subprocess.run(
        argv,
        cwd=cwd,
        env=env,
        stdin=stdin,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=False,
    )
    if result.returncode:
        if capture and result.stdout:
            print(result.stdout, end="", file=sys.stderr)
        fail(f"command failed with status {result.returncode}: {printable}")
    return result.stdout or ""


def parse_profile(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, source_line in enumerate(path.read_text().splitlines(), 1):
        line = source_line.strip()
        if not line or line.startswith("#"):
            continue
        match = PROFILE_ASSIGNMENT.fullmatch(line)
        if not match:
            fail(f"invalid profile line {path}:{line_number}: {source_line}")
        key, encoded = match.groups()
        try:
            words = shlex.split(encoded, comments=True, posix=True)
        except ValueError as error:
            fail(f"invalid profile value {path}:{line_number}: {error}")
        if len(words) > 1:
            fail(f"profile values must be quoted when they contain spaces: {path}:{line_number}")
        values[key] = words[0] if words else ""
    return values


def profiles() -> list[str]:
    return sorted(path.stem for path in (ROOT_DIR / "versions").glob("*.conf"))


def positive_integer(value: str) -> int:
    try:
        number = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("must be a positive integer") from None
    if number < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build an ArduPilot arm-none-eabi GCC toolchain from source."
    )
    parser.add_argument(
        "--profile", default="gcc-16.1", help="version profile (default: gcc-16.1)"
    )
    parser.add_argument("--work-dir", type=Path, help="source, object, and state directory")
    parser.add_argument("--prefix", type=Path, help="final installation directory")
    parser.add_argument("--jobs", type=positive_integer, help="parallel make jobs")
    parser.add_argument(
        "--python",
        dest="python_bin",
        default=os.environ.get("PYTHON", "python3"),
        help="Python interpreter embedded in GDB (default: python3)",
    )
    parser.add_argument(
        "--download-only", action="store_true", help="fetch and verify source archives, then stop"
    )
    parser.add_argument(
        "--newlib-only",
        "--rebuild-library",
        "--rebuild-newlib",
        dest="newlib_only",
        action="store_true",
        help="rebuild, reinstall, and verify only full and nano newlib",
    )
    package_group = parser.add_mutually_exclusive_group()
    package_group.add_argument(
        "--package", action="store_true", help="create an x86_64-linux release archive"
    )
    package_group.add_argument(
        "--package-only",
        action="store_true",
        help="verify and package an existing installation without building",
    )
    package_group.add_argument(
        "--no-package",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--package-dir",
        type=Path,
        help="package output directory (default: _build/packages)",
    )
    parser.add_argument("--list-profiles", action="store_true", help="list available profiles")
    parser.add_argument("--show-profile", metavar="NAME", help="print a profile without building")
    return parser


def require_program(name: str, *, path: str | None = None) -> str:
    resolved = shutil.which(name, path=path)
    if not resolved:
        fail(f"required command not found: {name}")
    return resolved


def sha512(path: Path) -> str:
    digest = hashlib.sha512()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def host_package_tag() -> str:
    machine = platform.machine().lower()
    if platform.system() != "Linux" or machine not in {"x86_64", "amd64"}:
        fail(
            "packaging currently supports only x86_64 Linux hosts "
            f"(this host is {platform.system()} {platform.machine()})"
        )
    return "x86_64-linux"


class ToolchainBuilder:
    def __init__(self, args: argparse.Namespace, profile_path: Path, config: dict[str, str]):
        self.args = args
        self.profile_path = profile_path
        self.config = config
        self.profile = args.profile
        self.release = self.required("TOOLCHAIN_RELEASE")
        self.multilib_list = config.get("MULTILIB_LIST", "rmprofile")
        self.package_base = f"ardupilot-arm-none-eabi-{self.release}"
        self.work_dir = (args.work_dir or ROOT_DIR / "_build" / self.profile).resolve()
        self.prefix = (args.prefix or ROOT_DIR / "_build" / "install" / self.package_base).resolve()
        self.package_dir = (args.package_dir or ROOT_DIR / "_build" / "packages").resolve()
        self.download_dir = ROOT_DIR / "downloads"
        self.source_dir = self.work_dir / "src"
        self.object_dir = self.work_dir / "obj"
        self.state_dir = self.work_dir / "state"
        self.nano_prefix = self.work_dir / "nano-install"
        self.policy = ROOT_DIR / "config" / "forbidden-newlib-symbols.txt"
        self.newlib_patch_dir = (
            ROOT_DIR / "patches" / f"newlib-{self.required('NEWLIB_VERSION')}"
        )
        self.jobs = args.jobs or (os.cpu_count() or 1)
        self.python_bin = args.python_bin
        self.source_date_epoch = os.environ.get(
            "SOURCE_DATE_EPOCH", DEFAULT_SOURCE_DATE_EPOCH
        )
        if any(character.isspace() for character in os.fspath(self.work_dir)):
            fail("the work path must not contain whitespace")
        if any(character.isspace() for character in os.fspath(self.prefix)):
            fail("the install prefix must not contain whitespace")
        self.env = os.environ.copy()
        # Configure scripts may select bash via CONFIG_SHELL. Bash evaluates
        # BASH_ENV for non-interactive shells, so user aliases can corrupt old
        # Autoconf probes (for example, an `ls --color` alias changes the
        # output expected by newlib 3.3's timestamp sanity check).
        self.env.pop("BASH_ENV", None)
        self.env.pop("ENV", None)
        self.env["PATH"] = f"{self.prefix / 'bin'}{os.pathsep}{self.env.get('PATH', '')}"
        self.env["LC_ALL"] = "C"
        self.env["SOURCE_DATE_EPOCH"] = self.source_date_epoch
        # A toolchain build is large enough that an implicit host ccache is
        # usually counterproductive, and its cache may not be writable in
        # containers or CI. Users can explicitly set CCACHE_DISABLE=0.
        self.env.setdefault("CCACHE_DISABLE", "1")

    def required(self, key: str) -> str:
        try:
            return self.config[key]
        except KeyError:
            fail(f"profile {self.profile_path} does not define {key}")

    def target_program(self, suffix: str) -> str:
        return os.fspath(self.prefix / "bin" / f"{TARGET}-{suffix}")

    def target_make_overrides(self) -> list[str]:
        tools = {
            "AR_FOR_TARGET": "ar",
            "AS_FOR_TARGET": "as",
            "CC_FOR_TARGET": "gcc",
            "GCC_FOR_TARGET": "gcc",
            "CXX_FOR_TARGET": "g++",
            "RAW_CXX_FOR_TARGET": "g++",
            "LD_FOR_TARGET": "ld",
            "NM_FOR_TARGET": "nm",
            "OBJCOPY_FOR_TARGET": "objcopy",
            "OBJDUMP_FOR_TARGET": "objdump",
            "RANLIB_FOR_TARGET": "ranlib",
            "READELF_FOR_TARGET": "readelf",
            "STRIP_FOR_TARGET": "strip",
        }
        return [f"{variable}={self.target_program(suffix)}" for variable, suffix in tools.items()]

    def make(
        self,
        build: Path,
        *targets: str,
        build_env: Mapping[str, str] | None = None,
        jobs: int | None = None,
    ) -> None:
        parallel_jobs = self.jobs if jobs is None else jobs
        command = ["make", "-C", build, f"-j{parallel_jobs}", *targets]
        run(command, env=build_env or self.env)

    def run_stage(self, name: str, function) -> None:
        marker = self.state_dir / f"{name}.done"
        if marker.exists():
            note(f"Skipping completed stage: {name}")
            return
        note(f"Stage: {name}")
        function()
        marker.touch()

    def input_fingerprint(self) -> str:
        digest = hashlib.sha256()
        inputs = [
            self.profile_path,
            self.policy,
            *sorted(self.newlib_patch_dir.glob("*.patch")),
        ]
        for path in inputs:
            digest.update(os.fspath(path.relative_to(ROOT_DIR)).encode())
            digest.update(path.read_bytes())
        settings = (
            f"recipe={BUILD_RECIPE_VERSION}\n"
            f"prefix={self.prefix}\n"
            f"python={self.python_bin}\n"
            f"multilib_list={self.multilib_list}\n"
            f"source_date_epoch={self.source_date_epoch}\n"
        )
        digest.update(settings.encode())
        return digest.hexdigest()

    def installed_profile_matches(self) -> bool:
        installed = self.prefix / "share" / "ardupilot-toolchain" / "version-profile.conf"
        if not installed.exists():
            return False
        try:
            values = parse_profile(installed)
        except BuildError:
            return False
        keys = (
            "TOOLCHAIN_RELEASE",
            "GCC_VERSION",
            "BINUTILS_VERSION",
            "GDB_VERSION",
            "NEWLIB_VERSION",
            "MULTILIB_LIST",
        )
        return all(values.get(key) == self.config.get(key) for key in keys)

    def write_fingerprint(self) -> None:
        self.state_dir.mkdir(parents=True, exist_ok=True)
        (self.state_dir / "input-fingerprint-v2").write_text(
            self.input_fingerprint() + "\n"
        )

    def check_fingerprint(
        self, *, allow_legacy: bool = False, allow_changed: bool = False
    ) -> None:
        self.state_dir.mkdir(parents=True, exist_ok=True)
        fingerprint_path = self.state_dir / "input-fingerprint-v2"
        expected = self.input_fingerprint()
        if fingerprint_path.exists():
            actual = fingerprint_path.read_text().strip()
            if actual != expected:
                if allow_changed:
                    note(
                        "Build inputs changed; the library-only rebuild will "
                        "record them after verification"
                    )
                    return
                fail("profile or build inputs changed; choose a new --work-dir")
            return
        legacy = self.state_dir / "input-fingerprint"
        if legacy.exists() and not (allow_legacy or self.installed_profile_matches()):
            fail(
                "this work directory was created by the shell builder and cannot be "
                "validated; use a new --work-dir"
            )
        self.write_fingerprint()
        if legacy.exists():
            note("Migrated existing build state to the Python builder")

    def download_archive(self, url: str, output: Path, expected: str) -> None:
        if output.exists():
            actual = sha512(output)
            if actual != expected:
                fail(f"SHA-512 mismatch for {output} (got {actual})")
            note(f"Using cached {output.name}")
            return
        output.parent.mkdir(parents=True, exist_ok=True)
        partial = output.with_name(output.name + ".part")
        note(f"Downloading {url}")
        run(
            [
                "curl",
                "--fail",
                "--location",
                "--retry",
                "3",
                "--continue-at",
                "-",
                "--output",
                partial,
                url,
            ]
        )
        actual = sha512(partial)
        if actual != expected:
            fail(f"SHA-512 mismatch for {partial} (got {actual})")
        partial.replace(output)

    def fetch_sources(self) -> None:
        if self.required("SOURCE_MODE") == "bundle":
            self.download_archive(
                self.required("BUNDLE_URL"),
                self.download_dir / self.required("BUNDLE_ARCHIVE"),
                self.required("BUNDLE_SHA512"),
            )
            if self.config.get("GDB_ARCHIVE"):
                self.download_archive(
                    self.required("GDB_URL"),
                    self.download_dir / self.required("GDB_ARCHIVE"),
                    self.required("GDB_SHA512"),
                )
            return
        for component in ("GCC", "BINUTILS", "GDB", "NEWLIB", "GMP", "MPFR", "MPC", "ISL", "GETTEXT"):
            self.download_archive(
                self.required(f"{component}_URL"),
                self.download_dir / self.required(f"{component}_ARCHIVE"),
                self.required(f"{component}_SHA512"),
            )

    @staticmethod
    def extract_stripped(archive: Path, destination: Path) -> None:
        destination.mkdir(parents=True, exist_ok=True)
        run(["tar", "-xf", archive, "-C", destination, "--strip-components=1"])

    @staticmethod
    def replace_symlink(target: str, link: Path) -> None:
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(target)

    def prepare_sources(self) -> None:
        self.source_dir.mkdir(parents=True, exist_ok=True)
        if self.required("SOURCE_MODE") == "bundle":
            bundle_dir = self.source_dir / "arm-source-bundle"
            self.extract_stripped(
                self.download_dir / self.required("BUNDLE_ARCHIVE"), bundle_dir
            )
            for component in ("gcc", "binutils", "newlib"):
                run(["tar", "-xf", bundle_dir / "src" / f"{component}.tar.bz2", "-C", self.source_dir])
            if self.config.get("GDB_ARCHIVE"):
                self.extract_stripped(
                    self.download_dir / self.required("GDB_ARCHIVE"), self.source_dir / "gdb"
                )
            else:
                run(["tar", "-xf", bundle_dir / "src" / "gdb.tar.bz2", "-C", self.source_dir])
            for prerequisite in ("gmp-6.1.0", "mpfr-3.1.4", "mpc-1.0.3", "isl-0.18"):
                archives = sorted((bundle_dir / "src").glob(f"{prerequisite}.tar.*"))
                if not archives:
                    fail(f"missing {prerequisite} in Arm source bundle")
                run(["tar", "-xf", archives[0], "-C", self.source_dir])
                self.replace_symlink(
                    f"../{prerequisite}", self.source_dir / "gcc" / prerequisite.split("-", 1)[0]
                )
        else:
            for component in ("GCC", "BINUTILS", "GDB", "NEWLIB"):
                self.extract_stripped(
                    self.download_dir / self.required(f"{component}_ARCHIVE"),
                    self.source_dir / component.lower(),
                )
            prerequisites = self.source_dir / "gcc-prerequisites"
            prerequisites.mkdir(parents=True, exist_ok=True)
            for dependency in ("GMP", "MPFR", "MPC", "ISL", "GETTEXT"):
                archive_name = self.required(f"{dependency}_ARCHIVE")
                source_name = archive_name.split(".tar.", 1)[0]
                link_name = source_name.split("-", 1)[0]
                self.extract_stripped(
                    self.download_dir / archive_name, prerequisites / source_name
                )
                self.replace_symlink(
                    f"../gcc-prerequisites/{source_name}", self.source_dir / "gcc" / link_name
                )
        self.apply_newlib_patches()

    def apply_newlib_patches(self) -> None:
        patches = sorted(self.newlib_patch_dir.glob("*.patch"))
        if not patches:
            fail(
                f"no source patches found for newlib {self.required('NEWLIB_VERSION')} "
                f"in {self.newlib_patch_dir}"
            )
        for patch_path in patches:
            note(f"Applying newlib patch: {patch_path.name}")
            with patch_path.open("r") as patch:
                run(
                    ["patch", "-d", self.source_dir / "newlib", "-p1", "--forward"],
                    stdin=patch,
                )

    def reset_newlib_source(self) -> None:
        """Restore pristine newlib sources and apply versioned patches."""
        source = self.source_dir / "newlib"
        if source.exists():
            shutil.rmtree(source)
        if self.required("SOURCE_MODE") == "bundle":
            archive = self.source_dir / "arm-source-bundle" / "src" / "newlib.tar.bz2"
            run(["tar", "-xf", archive, "-C", self.source_dir])
        else:
            self.extract_stripped(
                self.download_dir / self.required("NEWLIB_ARCHIVE"), source
            )
        self.apply_newlib_patches()

    def gcc_options(self) -> list[str]:
        return [
            f"--target={TARGET}",
            "--disable-decimal-float",
            "--disable-libffi",
            "--disable-libgomp",
            "--disable-libquadmath",
            "--disable-libssp",
            "--disable-libstdcxx-pch",
            "--disable-nls",
            "--disable-shared",
            "--disable-threads",
            "--disable-tls",
            "--enable-lto",
            "--enable-plugins",
            "--with-gnu-as",
            "--with-gnu-ld",
            f"--with-multilib-list={self.multilib_list}",
            "--with-newlib",
            f"--with-pkgversion=ArduPilot toolchain {self.release}",
        ]

    @staticmethod
    def newlib_common_options() -> list[str]:
        return [
            f"--target={TARGET}",
            "--disable-newlib-supplied-syscalls",
            "--disable-newlib-reent-check-verify",
            "--enable-newlib-retargetable-locking",
        ]

    def build_binutils(self) -> None:
        build = self.object_dir / "binutils"
        build.mkdir(parents=True, exist_ok=True)
        run(
            [
                self.source_dir / "binutils" / "configure",
                f"--target={TARGET}",
                f"--prefix={self.prefix}",
                f"--with-sysroot={self.prefix / TARGET}",
                "--disable-nls",
                "--disable-werror",
                "--disable-gdb",
                "--disable-sim",
                "--enable-plugins",
                f"--with-pkgversion=ArduPilot toolchain {self.release}",
            ],
            cwd=build,
            env=self.env,
        )
        # Avoid old bundled GDB/Readline host libraries in the Arm GCC 10 bundle.
        self.make(build, "all-binutils", "all-gas", "all-ld", "all-gprof")
        self.make(build, "install-binutils", "install-gas", "install-ld", "install-gprof")

    def build_gcc_stage1(self) -> None:
        build = self.object_dir / "gcc-stage1"
        build.mkdir(parents=True, exist_ok=True)
        run(
            [
                self.source_dir / "gcc" / "configure",
                *self.gcc_options(),
                f"--prefix={self.prefix}",
                f"--with-sysroot={self.prefix / TARGET}",
                "--without-headers",
                "--enable-languages=c",
            ],
            cwd=build,
            env=self.env,
        )
        self.make(build, "all-gcc", "all-target-libgcc")
        self.make(build, "install-gcc", "install-target-libgcc")

    def build_newlib(self) -> None:
        build = self.object_dir / "newlib"
        build.mkdir(parents=True, exist_ok=True)
        build_env = self.env.copy()
        build_env.update(
            {
                "CFLAGS_FOR_TARGET": "-O2 -g -DNDEBUG -ffunction-sections -fdata-sections",
                "CC_FOR_TARGET": self.target_program("gcc"),
            }
        )
        run(
            [
                self.source_dir / "newlib" / "configure",
                *self.newlib_common_options(),
                f"--prefix={self.prefix}",
                "--enable-newlib-io-long-long",
                "--enable-newlib-io-c99-formats",
                "--enable-newlib-register-fini",
                "--enable-newlib-mb",
            ],
            cwd=build,
            env=build_env,
        )
        overrides = self.target_make_overrides()
        self.make(build, *overrides, build_env=build_env)
        (self.prefix / TARGET / "lib").mkdir(parents=True, exist_ok=True)
        self.make(build, "install", *overrides, build_env=build_env, jobs=1)

    def multilib_directories(self) -> Iterable[str]:
        output = run([self.target_program("gcc"), "-print-multi-lib"], env=self.env, capture=True)
        for line in output.splitlines():
            if line:
                yield line.split(";", 1)[0]

    def build_newlib_nano(self) -> None:
        build = self.object_dir / "newlib-nano"
        build.mkdir(parents=True, exist_ok=True)
        build_env = self.env.copy()
        build_env.update(
            {
                "CFLAGS_FOR_TARGET": "-Os -g -DNDEBUG -ffunction-sections -fdata-sections",
                "CXXFLAGS_FOR_TARGET": "-Os -g -DNDEBUG -ffunction-sections -fdata-sections -fno-exceptions",
                "CC_FOR_TARGET": self.target_program("gcc"),
            }
        )
        run(
            [
                self.source_dir / "newlib" / "configure",
                *self.newlib_common_options(),
                f"--prefix={self.nano_prefix}",
                "--enable-newlib-nano-malloc",
                "--disable-newlib-unbuf-stream-opt",
                "--enable-newlib-reent-small",
                "--disable-newlib-fseek-optimization",
                "--enable-newlib-nano-formatted-io",
                "--disable-newlib-fvwrite-in-streamio",
                "--disable-newlib-wide-orient",
                "--enable-lite-exit",
                "--enable-newlib-global-atexit",
            ],
            cwd=build,
            env=build_env,
        )
        overrides = self.target_make_overrides()
        self.make(build, *overrides, build_env=build_env)
        (self.nano_prefix / TARGET / "lib").mkdir(parents=True, exist_ok=True)
        self.make(build, "install", *overrides, build_env=build_env, jobs=1)
        nano_header = self.prefix / TARGET / "include" / "newlib-nano"
        nano_header.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.nano_prefix / TARGET / "include" / "newlib.h", nano_header / "newlib.h")
        for multidir in self.multilib_directories():
            source = self.nano_prefix / TARGET / "lib" / multidir
            destination = self.prefix / TARGET / "lib" / multidir
            destination.mkdir(parents=True, exist_ok=True)
            for old_name, new_name in (("libc.a", "libc_nano.a"), ("libg.a", "libg_nano.a")):
                shutil.copy2(source / old_name, destination / new_name)
            rdimon = source / "librdimon.a"
            if rdimon.exists():
                shutil.copy2(rdimon, destination / "librdimon_nano.a")
            for name in ("nano.specs", "rdimon.specs", "nosys.specs"):
                candidate = source / name
                if candidate.exists():
                    shutil.copy2(candidate, destination / name)
            for startup in source.glob("*crt0.o"):
                shutil.copy2(startup, destination / startup.name)

    def build_gcc_final(self) -> None:
        build = self.object_dir / "gcc-final"
        build.mkdir(parents=True, exist_ok=True)
        run(
            [
                self.source_dir / "gcc" / "configure",
                *self.gcc_options(),
                f"--prefix={self.prefix}",
                f"--with-sysroot={self.prefix / TARGET}",
                "--with-native-system-header-dir=/include",
                "--enable-languages=c,c++",
            ],
            cwd=build,
            env=self.env,
        )
        self.make(build, "all-gcc", "all-target-libgcc", "all-target-libstdc++-v3")
        self.make(build, "install-gcc", "install-target-libgcc", "install-target-libstdc++-v3")

    def build_gcc_nano(self) -> None:
        build = self.object_dir / "gcc-nano"
        build.mkdir(parents=True, exist_ok=True)
        run(
            [
                self.source_dir / "gcc" / "configure",
                *self.gcc_options(),
                f"--prefix={self.nano_prefix}",
                f"--with-sysroot={self.nano_prefix / TARGET}",
                "--with-native-system-header-dir=/include",
                "--enable-languages=c,c++",
            ],
            cwd=build,
            env=self.env,
        )
        build_env = self.env.copy()
        build_env.update(
            {
                "CFLAGS_FOR_TARGET": "-Os -g -ffunction-sections -fdata-sections",
                "CXXFLAGS_FOR_TARGET": "-Os -g -ffunction-sections -fdata-sections -fno-exceptions",
            }
        )
        self.make(build, "all-gcc", "all-target-libgcc", "all-target-libstdc++-v3", build_env=build_env)
        # Do not install a non-standalone GCC driver in the nano staging prefix.
        self.make(build, "install-target-libstdc++-v3", build_env=build_env)
        for multidir in self.multilib_directories():
            source = self.nano_prefix / TARGET / "lib" / multidir
            destination = self.prefix / TARGET / "lib" / multidir
            shutil.copy2(source / "libstdc++.a", destination / "libstdc++_nano.a")
            shutil.copy2(source / "libsupc++.a", destination / "libsupc++_nano.a")

    def build_gdb(self) -> None:
        build = self.object_dir / "gdb"
        build.mkdir(parents=True, exist_ok=True)
        python = require_program(self.python_bin, path=self.env.get("PATH"))
        run(
            [
                self.source_dir / "gdb" / "configure",
                f"--target={TARGET}",
                f"--prefix={self.prefix}",
                "--disable-binutils",
                "--disable-gas",
                "--disable-gold",
                "--disable-gprof",
                "--disable-ld",
                "--disable-nls",
                "--disable-sim",
                "--disable-werror",
                "--with-expat",
                f"--with-python={python}",
                "--with-system-zlib",
                f"--with-pkgversion=ArduPilot toolchain {self.release}",
            ],
            cwd=build,
            env=self.env,
        )
        self.make(build, "all-gdb")
        self.make(build, "install-gdb")

    def write_manifest(self) -> None:
        manifest_dir = self.prefix / "share" / "ardupilot-toolchain"
        manifest_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.profile_path, manifest_dir / "version-profile.conf")
        shutil.copy2(self.policy, manifest_dir / "forbidden-newlib-symbols.txt")
        python_version = run([self.python_bin, "--version"], env=self.env, capture=True).strip()
        sections = [
            f"profile={self.profile}",
            f"description={self.required('PROFILE_DESCRIPTION')}",
            f"target={TARGET}",
            f"gcc={self.required('GCC_VERSION')}",
            f"binutils={self.required('BINUTILS_VERSION')}",
            f"gdb={self.required('GDB_VERSION')}",
            f"newlib={self.required('NEWLIB_VERSION')}",
            f"multilib_list={self.multilib_list}",
            f"python={python_version}",
            f"source_date_epoch={self.source_date_epoch}",
            "",
            run([self.target_program("gcc"), "-v"], env=self.env, capture=True).rstrip(),
            "",
            run([self.target_program("gdb"), "--configuration"], env=self.env, capture=True).rstrip(),
            "",
            run([self.target_program("gcc"), "-print-multi-lib"], env=self.env, capture=True).rstrip(),
            "",
        ]
        (manifest_dir / "manifest.txt").write_text("\n".join(sections))

    def verify_newlib_symbols(self) -> None:
        run(
            [
                ROOT_DIR / "scripts" / "verify-newlib-symbols.sh",
                self.prefix,
                self.policy,
            ],
            env=self.env,
        )

    def verify_toolchain(self, prefix: Path | None = None) -> None:
        run(
            [ROOT_DIR / "scripts" / "verify-toolchain.sh", prefix or self.prefix, self.policy],
            env=self.env,
        )

    def require_library_rebuild_inputs(self) -> None:
        if self.required("SOURCE_MODE") == "bundle":
            source_archive = (
                self.source_dir / "arm-source-bundle" / "src" / "newlib.tar.bz2"
            )
        else:
            source_archive = self.download_dir / self.required("NEWLIB_ARCHIVE")
        if not source_archive.is_file():
            fail(
                f"newlib source archive not found at {source_archive}; "
                "run a full build with this --work-dir first"
            )
        if not Path(self.target_program("gcc")).is_file():
            fail(
                f"installed compiler not found at {self.target_program('gcc')}; "
                "run a full build with this --prefix first"
            )
        if not self.installed_profile_matches():
            fail("the installed toolchain does not match the selected version profile")

    def build_newlib_only(self) -> None:
        self.require_library_rebuild_inputs()
        note("Rebuilding full and nano newlib only")
        for name in ("newlib", "newlib-nano"):
            build = self.object_dir / name
            if build.exists():
                shutil.rmtree(build)
        for name in (
            "newlib",
            "newlib-nano",
            "newlib-symbols",
            "manifest",
            # Clear state left by builds made before source-level omission.
            "sanitize",
            "verify",
        ):
            marker = self.state_dir / f"{name}.done"
            if marker.exists():
                marker.unlink()
        self.reset_newlib_source()
        self.build_newlib()
        self.build_newlib_nano()
        self.verify_newlib_symbols()
        self.write_manifest()
        self.verify_toolchain()
        self.write_fingerprint()
        for name in ("newlib", "newlib-nano", "newlib-symbols", "manifest", "verify"):
            (self.state_dir / f"{name}.done").touch()

    @staticmethod
    def elf_machine(path: Path) -> int | None:
        if path.is_symlink() or not path.is_file():
            return None
        try:
            with path.open("rb") as stream:
                header = stream.read(20)
        except OSError:
            return None
        if len(header) < 20 or header[:4] != b"\x7fELF":
            return None
        byte_order = "little" if header[5] == 1 else "big"
        return int.from_bytes(header[18:20], byte_order)

    @staticmethod
    def batches(paths: list[Path], size: int = 64) -> Iterable[list[Path]]:
        for offset in range(0, len(paths), size):
            yield paths[offset : offset + size]

    def strip_package_tree(self, root: Path) -> None:
        note("Stripping debug data from package staging copy")
        host_strip = require_program("strip")
        target_strip = root / "bin" / f"{TARGET}-strip"
        if not target_strip.is_file():
            fail(f"target strip program not found: {target_strip}")
        # ELF e_machine values: EM_ARM=40, EM_X86_64=62. Host programs and
        # target startup objects coexist in the installation, so the ELF
        # magic alone is not enough to select the correct strip program.
        host_files = [path for path in root.rglob("*") if self.elf_machine(path) == 62]
        target_files: list[Path] = []
        target_trees = [root / TARGET, root / "lib" / "gcc" / TARGET]
        for tree in target_trees:
            if not tree.exists():
                continue
            target_files.extend(
                path
                for path in tree.rglob("*")
                if not path.is_symlink()
                and path.is_file()
                and (path.suffix == ".a" or self.elf_machine(path) == 40)
            )
        for batch in self.batches(host_files):
            run([host_strip, "--strip-debug", *batch])
        for batch in self.batches(target_files):
            run([target_strip, "--strip-debug", *batch])

    def package_toolchain(self) -> Path:
        host_tag = host_package_tag()
        if not self.prefix.is_dir():
            fail(f"install prefix does not exist: {self.prefix}")
        if not self.installed_profile_matches():
            fail("the installed toolchain does not match the selected version profile")
        distribution_name = f"{self.package_base}-{host_tag}"
        staging_parent = self.work_dir / "package-staging"
        staging_root = staging_parent / distribution_name
        if staging_root.exists():
            shutil.rmtree(staging_root)
        staging_root.mkdir(parents=True)
        note(f"Staging package: {distribution_name}")
        try:
            run(["cp", "-a", "--reflink=auto", f"{self.prefix}/.", staging_root])
            self.strip_package_tree(staging_root)
            self.verify_toolchain(staging_root)
            self.package_dir.mkdir(parents=True, exist_ok=True)
            archive = self.package_dir / f"{distribution_name}.tar.xz"
            temporary = archive.with_name(archive.name + ".part")
            if temporary.exists():
                temporary.unlink()
            run(
                [
                    "tar",
                    "--sort=name",
                    f"--mtime=@{self.source_date_epoch}",
                    "--owner=0",
                    "--group=0",
                    "--numeric-owner",
                    "-C",
                    staging_parent,
                    "-cJf",
                    temporary,
                    distribution_name,
                ]
            )
            temporary.replace(archive)
            checksum = self.package_dir / f"{archive.name}.sha256"
            checksum.write_text(f"{sha256(archive)}  {archive.name}\n")
            note(f"Package: {archive}")
            note(f"Checksum: {checksum}")
            return archive
        finally:
            if staging_root.exists():
                shutil.rmtree(staging_root)

    def full_build(self) -> None:
        self.check_fingerprint()
        self.fetch_sources()
        if self.args.download_only:
            note("All source archives downloaded and verified")
            return
        self.run_stage(
            "host-check",
            lambda: run(
                ["env", f"PYTHON={self.python_bin}", ROOT_DIR / "scripts" / "check-host.sh"],
                env=self.env,
            ),
        )
        self.run_stage("sources", self.prepare_sources)
        self.run_stage("binutils", self.build_binutils)
        self.run_stage("gcc-stage1", self.build_gcc_stage1)
        self.run_stage("newlib", self.build_newlib)
        self.run_stage("newlib-nano", self.build_newlib_nano)
        self.run_stage("newlib-symbols", self.verify_newlib_symbols)
        self.run_stage("gcc-final", self.build_gcc_final)
        self.run_stage("gcc-nano", self.build_gcc_nano)
        self.run_stage("gdb", self.build_gdb)
        self.run_stage("manifest", self.write_manifest)
        self.run_stage("verify", self.verify_toolchain)
        note(f"Toolchain ready: {self.prefix}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argument_parser()
    args = parser.parse_args(argv)
    if args.list_profiles:
        print("\n".join(profiles()))
        return 0
    if args.show_profile:
        path = ROOT_DIR / "versions" / f"{args.show_profile}.conf"
        if not path.is_file():
            fail(f"unknown profile: {args.show_profile}")
        values = parse_profile(path)
        for key, value in values.items():
            print(f"{key}={value}")
        return 0
    profile_path = ROOT_DIR / "versions" / f"{args.profile}.conf"
    if not profile_path.is_file():
        fail(f"unknown profile '{args.profile}' (use --list-profiles)")
    if args.download_only and (args.newlib_only or args.package_only or args.package):
        parser.error("--download-only cannot be combined with rebuild or packaging options")
    if args.newlib_only and args.package_only:
        parser.error("--newlib-only cannot be combined with --package-only")
    builder = ToolchainBuilder(args, profile_path, parse_profile(profile_path))
    note(builder.required("PROFILE_DESCRIPTION"))
    note(f"Work directory: {builder.work_dir}")
    note(f"Install prefix: {builder.prefix}")
    if args.package_only:
        builder.verify_toolchain()
        builder.package_toolchain()
        return 0
    if args.newlib_only:
        builder.check_fingerprint(allow_legacy=True, allow_changed=True)
        run(
            ["env", f"PYTHON={builder.python_bin}", ROOT_DIR / "scripts" / "check-host.sh"],
            env=builder.env,
        )
        builder.build_newlib_only()
    else:
        builder.full_build()
    if args.package:
        builder.package_toolchain()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BuildError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1) from None
