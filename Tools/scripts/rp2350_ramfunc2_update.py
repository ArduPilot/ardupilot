#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
rp2350_ramfunc2_update.py -- bring the RP2350 SRAM placement registries back in
step with the source after a rebase, keeping in SRAM what was in SRAM.

When master renames, splits or removes a function a registry names, an RP2350
copter build stops with "no symbol match". For each such entry this script
finds the commit that removed the function and moves into the same registry
what took its place:

- functions that commit added in the same class or file (a rename or a split),
- existing functions that called it there and now carry its code.

They go in only as far as they fit in the bytes the removed entries held, so
SRAM use does not grow. Toolchain archive picks whose functions the build now
supplies from its own objects (Picolibc's libc) are moved to those objects.

With --build it then builds, and when upstream growth overflows Scratch X or
Y it moves the smallest entry that covers the overflow from that bank to main
SRAM, and links again. A build that links is checked: every registered
function, and every function the port defines in SRAM, must have landed there.

It needs only git, waf and the cross binutils. Placement is by size, not by
measurement: every change carries a "ramfunc2-update:" comment, and should be
profiled before it is relied on (rp2350/DEVELOPMENT.md, "Acting on a profile").

An entry marked for programs, as [arduplane], is checked only when --vehicle
builds one of them, as waf checks it, and what replaces it keeps the marker.

Usage, after `./waf copter` has failed on "no symbol match" or a scratch bank:
    Tools/scripts/rp2350_ramfunc2_update.py --board RPI_UAVFC           # print the plan
    Tools/scripts/rp2350_ramfunc2_update.py --board RPI_UAVFC --apply   # edit the registries
    Tools/scripts/rp2350_ramfunc2_update.py --board RPI_UAVFC --build   # edit, build, repeat
and with --vehicle plane, rover and so on after that vehicle's build failed.
"""

import argparse
import difflib
import re
import subprocess
import sys

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REG_DIR = ROOT / "libraries/AP_HAL_ChibiOS/rp2350"
SECTIONS_SH = REG_DIR / "rp2350_ramfunc2_sections.sh"
RAMFUNC2 = "rp2350_ramfunc2_registry.txt"
REGISTRIES = (RAMFUNC2, "rp2350_scratchx_registry.txt", "rp2350_scratchy_registry.txt")
BANK_OF_REGION = {"ram4": "rp2350_scratchx_registry.txt", "ram5": "rp2350_scratchy_registry.txt"}
SRAM_BASE = 0x20000000
TOOL_LIBS = ("libm.a", "libc_nano.a", "libc.a")
SOURCE_EXTS = (".cpp", ".cc", ".c")
TAG = "ramfunc2-update:"
NOT_FUNCTIONS = {"if", "for", "while", "switch", "return", "sizeof", "defined",
                 "static_assert", "else", "do", "case"}
KIND_ORDER = {"new": 0, "absorbed": 1, "replaces": 2, "moved": 3}
MAX_BUILDS = 8
# waf target -> (program, vehicle directory), as waf passes them to the generator
PROGRAMS = {
    "copter": ("arducopter", "ArduCopter"),
    "heli": ("arducopter-heli", "ArduCopter"),
    "plane": ("arduplane", "ArduPlane"),
    "rover": ("ardurover", "Rover"),
    "sub": ("ardusub", "ArduSub"),
    "antennatracker": ("antennatracker", "AntennaTracker"),
    "blimp": ("blimp", "Blimp"),
}


def run(cmd, check=True):
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if check and r.returncode != 0:
        sys.exit(f"{' '.join(str(c) for c in cmd[:3])} ... failed: {r.stderr.strip()}")
    return r


def git(*args):
    return run(("git",) + args).stdout


def git_file(rev, path):
    r = run(["git", "show", f"{rev}:{path}"], check=False)
    return r.stdout if r.returncode == 0 else ""


def same(a, b):
    """Two function names agree, allowing for a namespace one of them leaves out
    (the registry has ChibiOS::RCOutput::x where the source says RCOutput::x)."""
    return a == b or a.endswith("::" + b) or b.endswith("::" + a)


def last(name):
    return name.rsplit("::", 1)[-1]


def owner(name):
    parts = name.split("::")
    return parts[-2] if len(parts) > 1 else ""


def normalise(dem):
    """A demangled name as rp2350_ramfunc2_sections.sh matches it: no decltype
    return type, cut at the first "(" outside template brackets, no spaces."""
    dem = dem.strip()
    if dem.startswith("decltype"):
        depth = 0
        for i in range(8, len(dem)):
            if dem[i] == "(":
                depth += 1
            elif dem[i] == ")":
                depth -= 1
                if depth == 0:
                    dem = dem[i + 1:].lstrip()
                    break
    depth = 0
    cut = -1
    for i, c in enumerate(dem):
        if c == "<":
            depth += 1
        elif c == ">" and depth > 0:
            depth -= 1
        elif c == "(" and depth == 0:
            cut = i
            break
    if cut >= 0:
        dem = dem[:cut]
    elif "(" in dem:
        dem = dem[:dem.index("(")]
    return re.sub(r"\s+", "", dem)


def demangle(names, prefix):
    names = sorted(set(names))
    if not names:
        return {}
    out = subprocess.run([f"{prefix}-c++filt"], input="\n".join(names) + "\n",
                         capture_output=True, text=True, check=True).stdout
    return dict(zip(names, out.splitlines()))


class Sym:
    def __init__(self, name, mangled, size, typ, obj, src):
        self.name = name
        self.mangled = mangled
        self.size = size
        self.typ = typ
        self.obj = obj
        self.src = src


def load_symbols(buildroot, prefix):
    """Every function the build's objects and archives define."""
    files = sorted(str(p) for p in buildroot.rglob("*")
                   if p.suffix in (".o", ".a") and p.is_file())
    rows = []
    for i in range(0, len(files), 400):
        out = run([f"{prefix}-nm", "-A", "-S", "--defined-only"] + files[i:i + 400], check=False).stdout
        for line in out.splitlines():
            head, _, rest = line.rpartition(":")
            f = rest.split()
            if len(f) == 4:
                size, typ, mangled = int(f[1], 16), f[2], f[3]
            elif len(f) == 3:
                size, typ, mangled = 0, f[1], f[2]
            else:
                continue
            if typ in "TtWw" and not mangled.startswith("."):
                rows.append((head, size, typ, mangled))
    if not rows:
        sys.exit(f"no symbols under {buildroot}: build the board first")
    dem = demangle((r[3] for r in rows), prefix)
    return [Sym(normalise(dem.get(m, m)), m, size, typ, head, source_of(head, buildroot))
            for head, size, typ, m in rows]


def source_of(obj, buildroot):
    """The source file a loose object was compiled from, if it is in the tree."""
    try:
        rel = Path(obj).resolve().relative_to(buildroot.resolve()).as_posix()
    except ValueError:
        return None
    m = re.fullmatch(r"(.+\.(?:cpp|cc|c|S))\.\d+\.o", rel)
    if m and (ROOT / m.group(1)).is_file():
        return m.group(1)
    return None


def sym_size(syms, name):
    return max((s.size for s in syms if s.name == name), default=0)


class Entry:
    def __init__(self, reg, idx, path, name, size, active, programs):
        self.reg = reg
        self.idx = idx
        self.path = path
        self.name = name
        self.size = size
        self.active = active
        # the programs a "[<program>,...]" entry is placed in, or None for all
        self.programs = programs


def load_registries():
    lines = {}
    entries = []
    for reg in REGISTRIES:
        lines[reg] = (REG_DIR / reg).read_text(encoding="utf-8").splitlines(keepends=True)
        for idx, line in enumerate(lines[reg]):
            text = line.strip()
            active = not text.startswith("#")
            body = text.lstrip("#").strip()
            if "|" not in body:
                continue
            path, rest = body.split("|", 1)
            path = path.strip()
            if not re.fullmatch(r"\(archive\)|[\w./+-]+", path):
                continue
            sym, _, comment = rest.partition("#")
            sym = re.sub(r"\s+", "", sym)
            m = re.search(r"(\d+)\s*B\b", comment)
            # as the generator reads them: every bracketed name list in the
            # comment other than [inlinable]
            programs = {p for tag in re.findall(r"\[([A-Za-z_][\w,-]*)\]", comment) if tag != "inlinable"
                        for p in tag.split(",")}
            if sym:
                entries.append(Entry(reg, idx, path, sym, int(m.group(1)) if m else None, active,
                                     programs or None))
    return lines, entries


def strict_misses(buildroot, prefix, vehicle):
    """The entries the build's own check rejects, found by running that check as
    waf runs it for vehicle: with its program, so entries marked for it are
    checked, and strict only for Copter. The check writes its section lists into
    the build directory, so the build's own are put back: lists made for another
    set of programs would drop those programs' picks at the next link."""
    program, vehicle_dir = PROGRAMS[vehicle]
    cmd = ["bash", str(SECTIONS_SH), str(buildroot), "--toolchain", prefix, "--program", f"{program}:{vehicle_dir}"]
    if program == "arducopter":
        cmd.append("--strict")
    saved = {p: p.read_bytes() for p in buildroot.glob("rp2350_*_sections.ld")}
    try:
        r = run(cmd, check=False)
    finally:
        for p in buildroot.glob("rp2350_*_sections.ld"):
            if p in saved:
                p.write_bytes(saved[p])
            else:
                p.unlink()
    misses = []
    for line in r.stderr.splitlines():
        m = re.search(r": (rp2350_\w+\.txt): no (symbol match for|archive member) '(.*)'$", line)
        if m:
            misses.append((m.group(1), m.group(3), m.group(2) == "archive member"))
    if r.returncode not in (0, 1) or (r.returncode == 1 and not misses):
        sys.exit(f"{SECTIONS_SH.name} failed:\n{r.stderr}")
    return misses


def def_name(sig):
    depth = 0
    for i, c in enumerate(sig):
        if c == "<":
            depth += 1
        elif c == ">" and depth:
            depth -= 1
        elif c == "(" and depth == 0:
            head = sig[:i].rstrip()
            break
    else:
        return None
    if "=" in head or "{" in head:
        return None
    tok = re.split(r"[\s*&]+", head)[-1]
    if tok in NOT_FUNCTIONS or not re.fullmatch(r"(?:[A-Za-z_]\w*(?:<[^()]*>)?::)*~?[A-Za-z_]\w*", tok):
        return None
    return tok


def parse_defs(text):
    """(name, first line, last line) of each function defined at column 0, in
    ArduPilot's style: the body ends at the next "}" in column 0."""
    lines = text.splitlines()
    defs = []
    i = 0
    while i < len(lines):
        if not re.match(r"[A-Za-z_~]", lines[i]) or "(" not in lines[i]:
            i += 1
            continue
        j = i
        sig = lines[i]
        while "{" not in lines[j] and not lines[j].rstrip().endswith(";") and j + 1 < len(lines) and j - i < 12:
            j += 1
            sig += " " + lines[j]
        name = def_name(sig)
        if name is None or "{" not in lines[j]:
            i = j + 1 if lines[j].rstrip().endswith(";") else i + 1
            continue
        k = j
        if not (lines[j].count("{") == lines[j].count("}") and lines[j].rstrip().endswith("}")):
            k = j + 1
            while k < len(lines) and not lines[k].startswith("}"):
                k += 1
        defs.append((name, i + 1, k + 1))
        i = k + 1
    return defs


class Removal:
    """The commit that removed one or more registered functions."""

    def __init__(self, commit, subject):
        self.commit = commit
        self.subject = subject
        self.files = [f for f in git("show", "--format=", "--name-only", commit).split()
                      if f.endswith(SOURCE_EXTS)]
        self.pre = {f: parse_defs(git_file(commit + "^", f)) for f in self.files}
        self.post = {f: parse_defs(git_file(commit, f)) for f in self.files}

    def removes(self, name):
        before = any(same(name, d[0]) for f in self.files for d in self.pre[f])
        after = any(same(name, d[0]) for f in self.files for d in self.post[f])
        return before and not after

    def candidates(self, stale):
        """(name, source file, kind) of what may now carry the removed code."""
        out = []
        pre_names = {d[0] for f in self.files for d in self.pre[f]}
        post_names = {d[0] for f in self.files for d in self.post[f]}
        stale_files = {f for f in self.files for d in self.pre[f] if any(same(s, d[0]) for s in stale)}
        stale_owners = {owner(s) for s in stale}
        for f in self.files:
            for name, _, _ in self.post[f]:
                if name not in pre_names and ((owner(name) and owner(name) in stale_owners) or f in stale_files):
                    out.append((name, f, "new"))
        calls = [re.compile(r"\b" + re.escape(last(s)) + r"\s*\(") for s in stale]
        for f in self.files:
            old = 0
            for line in git("show", "--format=", "-U0", self.commit, "--", f).splitlines():
                m = re.match(r"@@ -(\d+)", line)
                if m:
                    old = int(m.group(1))
                    continue
                if line.startswith("-") and not line.startswith("---"):
                    if any(c.search(line) for c in calls):
                        for name, start, end in self.pre[f]:
                            if start <= old <= end and name in post_names \
                                    and not any(same(name, s) for s in stale):
                                out.append((name, f, "absorbed"))
                    old += 1
        seen = set()
        return [c for c in out if not (c[0] in seen or seen.add(c[0]))]


def find_removal(entry, cache):
    """The most recent commit that removed entry's function from the source."""
    key = "::".join(entry.name.split("::")[-2:])
    specs = ([entry.path] if entry.path.endswith(SOURCE_EXTS) else []) + ["."]
    for spec in specs:
        # the registries name the function too, so they must not count as its source
        out = git("log", "-n20", "--format=%H%x09%s", "-S" + key, "HEAD", "--", spec,
                  ":!libraries/AP_HAL_ChibiOS/rp2350")
        for line in out.splitlines():
            commit, _, subject = line.partition("\t")
            if commit not in cache:
                cache[commit] = Removal(commit, subject)
            if cache[commit].removes(entry.name):
                return cache[commit]
    return None


def still_defined(entry):
    if not entry.path.endswith(SOURCE_EXTS) or not (ROOT / entry.path).is_file():
        return False
    text = (ROOT / entry.path).read_text(encoding="utf-8", errors="replace")
    return any(same(entry.name, d[0]) for d in parse_defs(text))


def lookup(syms, name, src):
    """Build symbols for a function, preferring the object of its own source file."""
    exact = [s for s in syms if same(s.name, name) and last(s.name) == last(name)]
    own = [s for s in exact if s.src == src]
    return own or [s for s in exact if s.typ in "TW"]


def toolchain_members(prefix):
    members = {}
    for lib in TOOL_LIBS:
        path = run([f"{prefix}-gcc", f"-print-file-name={lib}"]).stdout.strip()
        if not Path(path).is_file():
            continue
        cur = None
        for line in run([f"{prefix}-nm", "--defined-only", path], check=False).stdout.splitlines():
            if line.endswith(".o:"):
                cur = line[:-1]
                members.setdefault(cur, set())
            elif cur:
                f = line.split()
                if len(f) == 3 and f[1] == "T":
                    members[cur].add(f[2])
    return members


class Plan:
    """Entries taken out of one registry, and what goes in at the same place."""

    def __init__(self, reg, stale, why):
        self.reg = reg
        self.stale = stale          # registry entries the plan takes out
        self.why = why              # what happened to them
        self.add = []               # (name, path, size, kind)
        self.left = []              # (name, size, reason)
        self.budget = sum(e.size or 0 for e in stale)
        # the programs what goes in is placed for, or None for all
        self.programs = None

    def take_programs(self):
        """Place what goes in for the programs of what it replaces: all of them if
        any replaced entry was unmarked, else every program they were marked for."""
        sets = [e.programs for e in self.stale]
        self.programs = None if not sets or any(s is None for s in sets) else set().union(*sets)

    def marker(self):
        return f" [{','.join(sorted(self.programs))}]" if self.programs else ""

    def fill(self, cands, slack):
        used = 0
        for c in sorted(cands, key=lambda c: (KIND_ORDER[c[3]], -similarity(c[0], self.stale), c[2])):
            if used + c[2] <= self.budget + slack:
                self.add.append(c)
                used += c[2]
            else:
                self.left.append((c[0], c[2], "does not fit"))

    def note(self):
        names = ", ".join(e.name for e in self.stale)
        text = f"{TAG} {names + ': ' if names else ''}{self.why}."
        if self.add:
            text += " Placed by size, not profiled:"
        elif self.stale and not self.why.startswith("moved"):
            text += " Nothing that replaced it fitted in its place."
        left = [f"{n} ({s} B)" for n, s, r in self.left if r == "does not fit"]
        if left:
            text += " Too big: " + ", ".join(left) + "."
        return text


def similarity(name, stale):
    return max((difflib.SequenceMatcher(None, last(name), last(e.name)).ratio() for e in stale), default=0)


def placed_for(entries, name, programs):
    """Is name already an active entry for every program in programs (None: all)?"""
    return any(e.active and e.name == name and
               (e.programs is None or (programs is not None and programs <= e.programs))
               for e in entries)


def plan_removals(misses, entries, syms, slack, program):
    plans = {}
    manual = []
    cache = {}
    evicted = {e.name for e in entries if not e.active}
    for reg, name, archive in misses:
        if archive:
            manual.append(f"{reg}: no archive member '{name}' in this toolchain")
            continue
        entry = next((e for e in entries if e.active and e.reg == reg and e.name == name
                      and (e.programs is None or program in e.programs)), None)
        if entry is None:
            manual.append(f"{reg}: '{name}' reported missing but not found in the registry")
            continue
        if still_defined(entry):
            manual.append(f"{reg}: {name} is still defined in {entry.path}, so it was compiled out or"
                          " inlined: mark it [needs DEFINE] or [inlinable]")
            continue
        removal = find_removal(entry, cache)
        if removal is None:
            manual.append(f"{reg}: {name}: no commit found that removed it from {entry.path}")
            continue
        # one plan per set of programs too: a replacement serves the programs of
        # what it replaces, and a commit can remove entries marked differently
        key = (reg, removal.commit, frozenset(entry.programs) if entry.programs else None)
        if key not in plans:
            why = f"removed upstream in {removal.commit[:10]} (\"{removal.subject}\")"
            plans[key] = (Plan(reg, [], why), removal)
        plans[key][0].stale.append(entry)
        plans[key][0].budget += entry.size or 0
    out = []
    for plan, removal in plans.values():
        plan.take_programs()
        cands = []
        for cname, src, kind in removal.candidates([e.name for e in plan.stale]):
            found = lookup(syms, cname, src)
            if not found:
                plan.left.append((cname, 0, "not in this build (inlined or compiled out)"))
                continue
            sym = max(found, key=lambda s: s.size)
            if placed_for(entries, sym.name, plan.programs):
                plan.left.append((sym.name, sym.size, "already placed"))
            elif sym.name in evicted:
                plan.left.append((sym.name, sym.size, "commented out in a registry; left out on purpose"))
            elif sym.size == 0:
                plan.left.append((sym.name, 0, "no size in the object"))
            else:
                cands.append((sym.name, sym.src or src, sym.size, kind))
        plan.fill(cands, slack)
        out.append(plan)
    return out, manual


def plan_archives(entries, syms, prefix, slack):
    """Archive picks whose functions now come from the build's own objects."""
    members = None
    plans = []
    defined = {}
    for s in syms:
        if s.typ == "T" and s.src:
            defined.setdefault(s.name, s)
    for e in entries:
        if not (e.active and e.path == "(archive)"):
            continue
        if members is None:
            members = toolchain_members(prefix)
        base = re.sub(r"^libm?_a-", "", e.name)
        funcs = members.get("lib_a-" + base) or members.get("libm_a-" + base) or set()
        shadowed = sorted(f for f in funcs if f in defined)
        if shadowed:
            plan = Plan(e.reg, [e], f"no longer linked, as the build defines {', '.join(shadowed)} itself")
            plan.fill([(f, defined[f].src, defined[f].size, "replaces") for f in shadowed], slack)
            plans.append(plan)
    return plans


def plan_bank_overflow(reg, over, entries, syms):
    """Move the smallest entry that covers an overflow out of a scratch bank,
    into main SRAM, where it still runs from RAM."""
    sized = [(sym_size(syms, e.name), e) for e in entries if e.active and e.reg == reg]
    sized = [(s, e) for s, e in sized if s > 0]
    covering = sorted((s, e.name) for s, e in sized if s >= over)
    if not covering:
        return []
    size, name = covering[0]
    entry = next(e for e in entries if e.active and e.reg == reg and e.name == name)
    entry.size = size
    out_plan = Plan(reg, [entry], f"moved to {RAMFUNC2}, as this bank overflowed by {over} B")
    in_plan = Plan(RAMFUNC2, [], f"moved here from {reg}, which overflowed by {over} B")
    in_plan.programs = entry.programs
    in_plan.add.append((name, entry.path, size, "moved"))
    return [out_plan, in_plan]


def show(plans, manual):
    for p in plans:
        print(f"\n{p.reg}: {p.why}")
        for e in p.stale:
            print(f"  - {e.name}  {e.size if e.size is not None else '?'} B")
        for name, _, size, kind in p.add:
            print(f"  + {name}  {size} B  ({kind})")
        for name, size, reason in p.left:
            print(f"    not added: {name}  {size} B  ({reason})")
    for m in manual:
        print(f"\nneeds a hand: {m}")


def wrap(text, width=96):
    out, line = [], "#"
    for word in text.split():
        if len(line) + 1 + len(word) > width:
            out.append(line + "\n")
            line = "#"
        line += " " + word
    return out + [line + "\n"]


def apply(plans, lines):
    for reg in REGISTRIES:
        mine = [p for p in plans if p.reg == reg]
        if not mine:
            continue
        drop = {e.idx for p in mine for e in p.stale}
        insert = {}
        for p in mine:
            at = min(e.idx for e in p.stale) if p.stale else len(lines[reg])
            new = wrap(p.note()) + [f"{path}|{name}  # {size} B{p.marker()}\n" for name, path, size, _ in p.add]
            insert.setdefault(at, []).extend(new)
        out = []
        for idx, line in enumerate(lines[reg] + [""]):
            out.extend(insert.get(idx, []))
            if idx not in drop:
                out.append(line)
        (REG_DIR / reg).write_text("".join(out), encoding="utf-8")
        print(f"updated {reg}")


def fix_names(buildroot, prefix, slack, write, vehicle):
    """Plan, and if asked write, the replacements for entries that match nothing."""
    misses = strict_misses(buildroot, prefix, vehicle)
    lines, entries = load_registries()
    syms = load_symbols(buildroot, prefix)
    plans, manual = plan_removals(misses, entries, syms, slack, PROGRAMS[vehicle][0])
    plans += plan_archives(entries, syms, prefix, slack)
    show(plans, manual)
    if write and plans:
        apply(plans, lines)
    return plans, manual


def configured_board():
    cache = ROOT / "build/c4che/_cache.py"
    m = re.search(r"^BOARD = '([^']+)'", cache.read_text(), re.M) if cache.is_file() else None
    return m.group(1) if m else None


def verify(elf, buildroot, prefix):
    """Report registered functions, and functions the port defines in SRAM, that
    the link put in flash. Returns the number found."""
    _, entries = load_registries()
    # entries marked for other programs are not placed in this one
    wanted = {e.name for e in entries if e.active and e.path != "(archive)"
              and (e.programs is None or elf.name in e.programs)}
    out = run([f"{prefix}-nm", "-S", "--defined-only", str(elf)]).stdout
    rows = [line.split() for line in out.splitlines()]
    # a weak label with no size is a default handler in vectors.S, such as an
    # interrupt vector the board does not use; nothing runs it
    rows = [(int(r[0], 16), r[-1]) for r in rows
            if (len(r) == 4 and r[2] in "TtWw") or (len(r) == 3 and r[1] in "Tt")]
    dem = demangle((r[1] for r in rows), prefix)
    where = {}
    for addr, m in rows:
        where.setdefault(normalise(dem.get(m, m)), []).append(addr)
    # functions some object of the port places in .ramtext, whichever definition won
    ramtext = set()
    objs = [str(p) for p in buildroot.rglob("*.o")]
    for i in range(0, len(objs), 400):
        for line in run([f"{prefix}-objdump", "-t"] + objs[i:i + 400], check=False).stdout.splitlines():
            f = line.split()
            if len(f) >= 6 and f[-3].startswith(".ramtext") and "F" in f[1:-3]:
                ramtext.add(f[-1])
    ramtext = {normalise(d) for d in demangle(ramtext, prefix).values()}
    bad = []
    for name in sorted(wanted):
        if name in where and not any(a >= SRAM_BASE for a in where[name]):
            bad.append(f"{name} (registered)")
    for name in sorted(ramtext):
        if name in where and not any(a >= SRAM_BASE for a in where[name]):
            bad.append(f"{name} (the port's copy is in .ramtext; another definition was linked)")
    for b in bad:
        print(f"in flash: {b}")
    if not bad:
        print(f"every registered function, and every .ramtext function, is in SRAM in {elf.name}")
    return len(bad)


def build_loop(args, buildroot):
    board = configured_board()
    if board != args.board:
        sys.exit(f"waf is configured for {board}: run ./waf configure --board {args.board} first")
    for _ in range(MAX_BUILDS):
        print(f"\n./waf {args.vehicle}")
        r = run([str(ROOT / "waf"), args.vehicle], check=False)
        log = r.stdout + r.stderr
        if r.returncode == 0:
            return verify(buildroot / "bin" / PROGRAMS[args.vehicle][0], buildroot, args.toolchain)
        if "matched no symbol" in log or "no archive member" in log:
            plans, manual = fix_names(buildroot, args.toolchain, args.slack, True, args.vehicle)
            if manual or not plans:
                return 1
            continue
        m = re.search(r"region `(ram[45])' overflowed by (\d+) bytes", log)
        if m:
            lines, entries = load_registries()
            plans = plan_bank_overflow(BANK_OF_REGION[m.group(1)], int(m.group(2)),
                                       entries, load_symbols(buildroot, args.toolchain))
            if not plans:
                print(f"nothing in {BANK_OF_REGION[m.group(1)]} covers {m.group(2)} B")
                return 1
            show(plans, [])
            apply(plans, lines)
            continue
        print("\n".join(log.splitlines()[-25:]))
        return 1
    print(f"still failing after {MAX_BUILDS} builds")
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", required=True, help="board whose build failed, e.g. RPI_UAVFC")
    ap.add_argument("--build-dir", help="build directory (default build/<board>)")
    ap.add_argument("--toolchain", default="arm-none-eabi", help="cross toolchain prefix")
    ap.add_argument("--slack", type=int, default=0,
                    help="bytes a replacement may exceed what it replaces by (default 0)")
    ap.add_argument("--apply", action="store_true", help="edit the registries; otherwise print the plan")
    ap.add_argument("--build", action="store_true",
                    help="edit, run ./waf VEHICLE, and repeat until it links; then check placement")
    ap.add_argument("--vehicle", default="copter", choices=sorted(PROGRAMS),
                    help="waf target whose build failed (default copter): its program's marked entries are"
                    " checked, strictly for copter as waf does; --build and --verify use it too")
    ap.add_argument("--verify", action="store_true", help="only check the placement in the built ELF")
    args = ap.parse_args()

    buildroot = Path(args.build_dir) if args.build_dir else ROOT / "build" / args.board
    if not (buildroot / "hwdef.h").is_file():
        sys.exit(f"{buildroot} has no hwdef.h: configure and build {args.board} first")
    if args.verify:
        return 1 if verify(buildroot / "bin" / PROGRAMS[args.vehicle][0], buildroot, args.toolchain) else 0
    if args.build:
        return build_loop(args, buildroot)
    version = buildroot / "ap_version.h"
    head = git("rev-parse", "--short=8", "HEAD").strip()
    if version.is_file():
        m = re.search(r'GIT_VERSION "([0-9a-f]+)"', version.read_text())
        if m and not head.startswith(m.group(1)):
            print(f"warning: {buildroot} was built at {m.group(1)}, HEAD is {head}; rebuild before trusting this")
    plans, manual = fix_names(buildroot, args.toolchain, args.slack, args.apply, args.vehicle)
    if not plans and not manual:
        print("the registries match this build")
    elif not args.apply:
        print("\nnothing written; --apply edits the registries, --build also builds")
    return 1 if manual else 0


if __name__ == "__main__":
    sys.exit(main())
