#!/usr/bin/env python3

'''
Lint the path filters (paths / paths-ignore) of the GitHub Actions
workflows.  A path filter is a promise that changes to those paths do or
do not require running the workflow, and nothing else validates it: a
misspelled pattern silently matches nothing, a pattern kept after its
file is deleted is dead weight, and a push list that drifts from its
pull_request twin means a branch and its PR run different CI.

AP_FLAKE8_CLEAN

How to use?
~/ardupilot $ python Tools/scripts/check_workflow_path_filters.py
~/ardupilot $ python Tools/scripts/check_workflow_path_filters.py --self-test

Exits non-zero if anything is reported.

Patterns are matched against the files git knows about, so a pattern is
judged dead only when it matches no tracked path at all.

The "ignores a path it uses" check reads what the workflow names -- a
composite action, a script in a run: block -- and then one hop further:
what those shell scripts themselves run.  build_ci.sh is sliced per
CI_BUILD_TARGET, so a workflow owns only the branches for its own
targets; that is how it knows the navigator build needs
firmware_version_decoder.py.

It stays blind to content coupling: a python import, a parameter file, a
header the build pulls in transitively.  It is a net, not a proof.
'''

import argparse
import functools
import os
import re
import subprocess
import sys

try:
    import yaml
except ImportError:
    print("check_workflow_path_filters.py: PyYAML required (python3 -m pip install pyyaml)", file=sys.stderr)
    sys.exit(1)

# every build needs these, and no workflow spells them out, so the
# reference scan below cannot find them on its own
ALWAYS_RELEVANT = ['waf', 'wscript', 'Tools/ardupilotwaf', 'modules/waf']

# tokens in a workflow body that could name something in the repo
REF_TOKEN = re.compile(r"[A-Za-z0-9_./-]+")

# the job list, anchored: a "jobs:" inside a comment or a string must not
# be mistaken for the start of the real one
JOBS_KEY = re.compile(r"^jobs:", re.M)

# only the wildcards ArduPilot's filters actually use are modelled; '?',
# '+', '[...]' and '\' are reported as unsupported rather than guessed at
UNSUPPORTED = re.compile(r"[?+[\\]")

# A script being run, rather than any path-looking token: the token form reads
# example paths out of comments and test data as if they were dependencies.
# The quote exclusion is what does that work -- a quoted path is data -- so
# widening this character class silently brings the noise back.
EXEC = re.compile(r"(?:^|[\s(`$|&;])\.?/?((?:Tools|libraries|modules)/[A-Za-z0-9_./-]+\.(?:sh|py))")

# build_ci.sh dispatches on CI_BUILD_TARGET, one branch per target, so a
# workflow depends only on the branches for the targets it names
DISPATCHER = 'Tools/scripts/build_ci.sh'
BRANCH = re.compile(r'^\s*(?:el)?if \[+ "\$t" ==+ "([^"]+)" \]')
FUNCTION = re.compile(r'^\s*(?:function\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*\(\)\s*\{')
ASSIGNMENT = re.compile(r'^\s*([A-Za-z_][A-Za-z0-9_]*)=([A-Za-z0-9_./-]+)\s*$', re.M)
WORD = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')
LOOP = re.compile(r'^\s*for t in ')

# package markers carry no behaviour, so a filter treating them differently
# means nothing; counting them only pads a drift report and makes its example
# the least informative path in the set
UNINTERESTING = ('__init__.py',)
# 54 branches today; well under that means the shape changed and the slicing
# stopped working, which is reported rather than silently passing
MIN_BRANCHES = 40


@functools.lru_cache(maxsize=None)
def translate(pattern):
    '''GitHub Actions path filter pattern -> anchored regex'''
    out = []
    i = 0
    while i < len(pattern):
        for glob, regex in (('/**/', '/(?:.*/)?'),   # zero or more directories
                            ('**/', '(?:.*/)?'),
                            ('/**', '/.*'),          # everything below a directory
                            ('**', '.*'),            # crosses / boundaries
                            ('*', '[^/]*')):         # stays inside one path segment
            if pattern.startswith(glob, i):
                out.append(regex)
                i += len(glob)
                break
        else:
            out.append(re.escape(pattern[i]))
            i += 1
    return re.compile('^' + ''.join(out) + '$')


def selected(path, patterns):
    '''last-match-wins over an ordered filter list; True if path ends up matched'''
    state = False
    for pat in patterns:
        neg = pat.startswith('!')
        if translate(pat[1:] if neg else pat).match(path):
            state = not neg
    return state


def ignores(kind, patterns, path):
    '''True if a change to path would not run the workflow'''
    matched = selected(path, patterns)
    return matched if kind == 'paths-ignore' else not matched


def repo_root():
    return subprocess.run(["git", "rev-parse", "--show-toplevel"], check=True,
                          stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()


def tracked_files(root):
    '''every path git tracks, submodule gitlinks included: GitHub reports a
    submodule bump as a change to the gitlink, so those paths are filterable'''
    out = subprocess.run(["git", "-c", "core.quotePath=false", "ls-files"],
                         check=True, cwd=root, stdout=subprocess.PIPE,
                         universal_newlines=True).stdout
    return out.splitlines()


def shell_functions(lines):
    '''name -> body, for the functions defined in a shell script'''
    funcs, kept, i = {}, [], 0
    while i < len(lines):
        match = FUNCTION.match(lines[i])
        if not match:
            kept.append(lines[i])
            i += 1
            continue
        depth, body = lines[i].count('{') - lines[i].count('}'), []
        i += 1
        while i < len(lines) and depth > 0:
            depth += lines[i].count('{') - lines[i].count('}')
            if depth > 0:
                body.append(lines[i])
            i += 1
        funcs[match.group(1)] = body
    return funcs, kept


def exec_paths(lines, funcs, variables, resolve, seen=()):
    '''tracked paths these lines run, following calls into funcs'''
    found = set()
    for line in lines:
        for name, value in variables.items():
            line = line.replace('${%s}' % name, value).replace('$' + name, value)
        for hit in EXEC.findall(line):
            path = resolve(hit)
            if path:
                found.add(path)
        for word in WORD.findall(line):
            if word in funcs and word not in seen:
                found |= exec_paths(funcs[word], funcs, variables, resolve, seen + (word,))
    return found


def dispatcher_deps(text, targets, resolve):
    '''(paths build_ci.sh runs for these targets, number of branches parsed)

    Everything outside a branch runs for every target; a target with no branch
    of its own falls through to the catch-all at the end.
    '''
    funcs, lines = shell_functions(text.splitlines())
    variables = dict(ASSIGNMENT.findall(text))
    branches, common, catch_all, current, in_loop = {}, [], [], None, False
    for line in lines:
        # inside the per-target loop, a line outside every branch runs for the
        # targets that have no branch of their own; outside it, for all of them
        if LOOP.match(line):
            in_loop = True
            continue
        if in_loop and re.match(r'^\s*done\s*$', line):
            in_loop = False
            continue
        match = BRANCH.match(line)
        if match:
            current = match.group(1)
            branches.setdefault(current, [])
            continue
        if current is not None:
            if re.match(r'^\s*continue\s*$', line):
                current = None
            else:
                branches[current].append(line)
        elif in_loop:
            catch_all.append(line)
        else:
            common.append(line)
    deps = exec_paths(common, funcs, variables, resolve)
    for target in targets or [None]:
        deps |= exec_paths(branches.get(target, catch_all), funcs, variables, resolve)
    return deps, len(branches)


_SCRIPT_DEPS = {}


def script_deps(path, resolve):
    '''tracked paths a script runs, read once per script'''
    if path not in _SCRIPT_DEPS:
        try:
            with open(path, errors='replace') as fh:
                text = fh.read()
        except OSError:
            # a script inside a submodule that is not checked out
            _SCRIPT_DEPS[path] = set()
            return _SCRIPT_DEPS[path]
        funcs, lines = shell_functions(text.splitlines())
        _SCRIPT_DEPS[path] = exec_paths(lines, funcs, dict(ASSIGNMENT.findall(text)), resolve)
    return _SCRIPT_DEPS[path]


def workflow_deps(root, body, targets, resolve, dispatcher_text):
    '''tracked paths the workflow reaches through the scripts it runs'''
    jobs = JOBS_KEY.split(body, maxsplit=1)[-1]
    deps, branches = set(), None
    for script in sorted(set(EXEC.findall(jobs))):
        # shell only: a python caller quotes its arguments, so this finds
        # nothing there but its own test data
        if not script.endswith('.sh') or resolve(script) != script:
            continue
        if script == DISPATCHER and dispatcher_text is not None:
            found, branches = dispatcher_deps(dispatcher_text, targets, resolve)
            deps |= found
            continue
        deps |= script_deps(os.path.join(root, script), resolve)
    return deps, branches


def matrix_values(matrix, key):
    '''the values a matrix key takes, include: entries counted'''
    values = {str(v) for v in (matrix.get(key) or []) if not isinstance(v, (dict, list))}
    for entry in matrix.get('include', []) or []:
        if isinstance(entry, dict) and key in entry:
            values.add(str(entry[key]))
    return values


def build_targets(doc, body):
    '''the CI_BUILD_TARGET values a workflow uses

    Read from the assignment itself rather than guessed from a matrix key:
    the value can be a literal, a matrix reference, or text around one --
    CI_BUILD_TARGET: dds-${{matrix.config}} -- and it is written both as a
    yaml key and inline in a run: line.
    '''
    templates = set(re.findall(r'CI_BUILD_TARGET[:=]\s*"?([^"\s]+)"?', body))
    targets = set()
    for job in (doc.get('jobs') or {}).values():
        if not isinstance(job, dict):
            continue
        matrix = ((job.get('strategy') or {}).get('matrix') or {})
        for template in templates:
            expanded = {template}
            for key in set(re.findall(r'\$\{\{\s*matrix\.([A-Za-z0-9_]+)\s*\}\}', template)):
                values = matrix_values(matrix, key)
                placeholder = re.compile(r'\$\{\{\s*matrix\.%s\s*\}\}' % key)
                expanded = {placeholder.sub(value, text)
                            for text in expanded for value in values}
            targets |= expanded
    return targets


def gitlinks(root):
    '''tracked submodule entries'''
    out = subprocess.run(["git", "-c", "core.quotePath=false", "ls-files", "-s"],
                         check=True, cwd=root, stdout=subprocess.PIPE,
                         universal_newlines=True).stdout
    return {line.split('\t', 1)[1] for line in out.splitlines() if line.startswith('160000')}


def resolver(tracked, links):
    '''path a script runs -> the path a filter could match, or None'''
    def resolve(path):
        if path in tracked:
            return path
        for link in links:
            # inside a submodule: what a filter sees change is the gitlink
            if path.startswith(link + '/'):
                return link
        return None
    return resolve


def load_workflows(wf_dir):
    '''yield (name, body, doc, events, error) per workflow file

    events maps each declared event to its (kind, patterns), or to None when
    the event is declared with no filter at all.
    '''
    for name in sorted(f for f in os.listdir(wf_dir) if f.endswith((".yml", ".yaml"))):
        with open(os.path.join(wf_dir, name)) as fh:
            body = fh.read()
        try:
            doc = yaml.safe_load(body)
        except yaml.YAMLError as err:
            yield name, body, {}, {}, "cannot be parsed (%s)" % str(err).replace("\n", " ")
            continue
        if not isinstance(doc, dict):
            yield name, body, {}, {}, "is not a YAML mapping"
            continue
        # YAML 1.1 parses a bare "on" key as boolean True
        triggers = doc.get('on', doc.get(True, {}))
        if isinstance(triggers, str):
            triggers = {triggers: None}
        elif isinstance(triggers, list):
            triggers = {event: None for event in triggers}
        elif not isinstance(triggers, dict):
            triggers = {}
        events = {}
        for event in ('push', 'pull_request'):
            if event not in triggers:
                continue
            ev = triggers[event]
            events[event] = None
            if not isinstance(ev, dict):
                continue
            for kind in ('paths', 'paths-ignore'):
                if kind in ev:
                    events[event] = (kind, [str(p) for p in ev[kind] or []])
        yield name, body, doc, events, None


def referenced(body, kind, files, tracked):
    '''repo paths the workflow names, as (display name, files to test)

    A filter pattern only ever matches a file, so a referenced directory
    is tested through the files under it.
    '''
    jobs = JOBS_KEY.split(body, maxsplit=1)[-1]
    tokens = (t[2:] if t.startswith('./') else t for t in REF_TOKEN.findall(jobs))
    # a bare word is a waf target or an English word, not a path
    names = {t for t in tokens if '/' in t}
    if kind == 'paths-ignore':
        # a denylist claims everything it does not list is relevant, so the
        # build's own machinery has to be relevant; an allowlist is instead an
        # explicit statement of the narrow set that matters, and is left alone
        names.update(ALWAYS_RELEVANT)
    for name in sorted(names):
        if name in tracked:
            yield name, [name]
        else:
            under = [f for f in files if f.startswith(name + '/')]
            if under:
                yield name + "/", under


def deciding(patterns, path):
    '''the pattern that settles this path: last match wins, so the last one'''
    winner = None
    for pat in patterns:
        bare = pat[1:] if pat.startswith('!') else pat
        if translate(bare).match(path):
            winner = pat
    return winner


def drift(events, files):
    '''yield messages when push and pull_request would not run on the same changes'''
    if 'push' not in events or 'pull_request' not in events:
        return
    push, pr = events['push'], events['pull_request']
    if push == pr:
        return
    if push is None or pr is None:
        filtered = "push" if pr is None else "pull_request"
        unfiltered = "pull_request" if pr is None else "push"
        yield ("%s filters which changes run it, %s does not, so they do not run "
               "on the same changes" % (filtered, unfiltered))
        return
    # the lists differ on paper; only a difference in what they select matters
    runs = {}
    interesting = [f for f in files if not f.endswith(UNINTERESTING)]
    for event, (kind, patterns) in (('push', push), ('pull_request', pr)):
        runs[event] = {f for f in interesting if not ignores(kind, patterns, f)}
    events_by_name = {'push': push, 'pull_request': pr}
    for which, other in (("push", "pull_request"), ("pull_request", "push")):
        paths = sorted(runs[which] - runs[other])
        if not paths:
            continue
        # a filter is written as patterns, so name the pattern that settles
        # the difference rather than a file it happens to match
        kind, patterns = events_by_name[other]
        theirs = set(events_by_name[which][1])
        groups = {}
        for path in paths:
            groups.setdefault(deciding(patterns, path), []).append(path)
        for pattern, group in sorted(groups.items(), key=lambda item: (item[0] or '', item[1])):
            detail = "%d path(s) run %s only, such as %s" % (len(group), which, group[0])
            if pattern is None:
                yield detail
            elif pattern not in theirs:
                yield "%s %r is in %s only, so %s" % (kind, pattern, other, detail)
            else:
                yield "%s %r comes last in %s only, so %s" % (kind, pattern, other, detail)


def check(name, body, events, files, tracked, deps=()):
    '''yield one message per problem found in a workflow's filters

    deps are paths the workflow reaches through the scripts it runs, as
    opposed to the ones it names itself.
    '''
    for message in drift(events, files):
        yield message

    own = ".github/workflows/%s" % name
    # push and pull_request normally carry the same list; check it once
    filters = {(kind, tuple(patterns)) for kind, patterns in filter(None, events.values())}
    for kind, patterns in sorted(filters):
        if not patterns:
            yield "%s is empty, so the workflow never runs" % kind
            continue

        # a workflow must run when it is itself edited
        if ignores(kind, patterns, own):
            yield "%s does not run on changes to itself (%s)" % (kind, own)

        # per-pattern sanity: unsupported syntax, duplicates, dead patterns
        for pat in sorted(set(patterns)):
            if patterns.count(pat) > 1:
                yield "%s duplicates pattern %r" % (kind, pat)
            bare = pat[1:] if pat.startswith('!') else pat
            if UNSUPPORTED.search(bare):
                yield "%s pattern %r uses syntax this linter does not model" % (kind, pat)
            elif not any(translate(bare).match(f) for f in files):
                yield "%s pattern %r matches no tracked file" % (kind, pat)

        # a workflow must run when something it uses changes
        for ref, paths in referenced(body, kind, files, tracked):
            hidden = [p for p in paths if ignores(kind, patterns, p)]
            if hidden:
                detail = " (e.g. %s)" % hidden[0] if len(paths) > 1 else ""
                yield "%s does not run on changes to %s, which it uses%s" % (kind, ref, detail)

        # the same, for what it reaches through the scripts it runs
        for dep in sorted(deps):
            if ignores(kind, patterns, dep):
                yield "%s does not run on changes to %s, which it uses" % (kind, dep)


# ---------------- self test ----------------

MATCH_CASES = [
    # (pattern, path, expected) -- from the GitHub filter pattern cheat sheet
    ("*", "README.md", True),
    ("*", "docs/README.md", False),
    ("**", "all/the/files.md", True),
    ("**.js", "index.js", True),
    ("**.js", "src/js/app.js", True),
    ("docs/*", "docs/README.md", True),
    ("docs/*", "docs/mona/octocat.md", False),
    ("docs/**", "docs/mona/octocat.md", True),
    ("docs/**/*.md", "docs/README.md", True),
    ("docs/**/*.md", "docs/mona/hello-world.md", True),
    ("**/docs/**", "docs/hello.md", True),
    ("**/docs/**", "dir/docs/my-file.txt", True),
    ("**/docs/**", "mydocs/x", False),
    ("**/README.md", "README.md", True),
    ("**/README.md", "js/README.md", True),
    ("**/*src/**", "a-src/app.js", True),
    ("**/*-post.md", "my-post.md", True),
    ("Tools/scripts/**.py", "Tools/scripts/build_tests/x.py", True),
    ("Tools/scripts/**.py", "Tools/scripts/x.py", True),
    # a bare directory name matches only a file of that name
    (".github", ".github/workflows/x.yml", False),
    (".github", ".github", True),
]

STATE_CASES = [
    # (patterns, path, expected final state)
    (["*.md", "!README.md"], "README.md", False),
    (["*.md", "!README.md"], "docs.md", True),
    (["*.md", "!README.md", "README*"], "README.md", True),
    (["Tools/**", "!Tools/ardupilotwaf/**"], "Tools/ardupilotwaf/boards.py", False),
    (["Tools/**", "!Tools/ardupilotwaf/**"], "Tools/scripts/build_ci.sh", True),
    ([".github/**", "!.github/actions/**"], ".github/actions/setup-ccache/action.yml", False),
    ([".github/**", "!.github/actions/**"], ".github/workflows/other.yml", True),
]

SELF_TEST_FILES = [
    ".github/workflows/w.yml", "Tools/scripts/x.py", "Tools/scripts/__init__.py",
    "Blimp/mode.cpp", "Rover/mode.cpp",
]

# (description, events, body, expected substrings of the messages)
CHECK_CASES = [
    ("a reordering that selects the same files is not drift",
     {'push': ('paths-ignore', ['Blimp/**', 'Rover/**']),
      'pull_request': ('paths-ignore', ['Rover/**', 'Blimp/**'])},
     "jobs:\n", []),
    ("a package marker on one side only is not drift",
     {'push': ('paths-ignore', ['Blimp/**', 'Tools/scripts/__init__.py']),
      'pull_request': ('paths-ignore', ['Blimp/**'])},
     "jobs:\n", []),
    ("a divergence names the pattern behind it, not a file it matched",
     {'push': ('paths-ignore', ['Blimp/**']),
      'pull_request': ('paths-ignore', ['Blimp/**', 'Rover/**'])},
     "jobs:\n", ["'Rover/**' is in pull_request only, so 1 path(s) run push only"]),
    ("an order that changes the outcome names the pattern that comes last",
     {'push': ('paths-ignore', ['Tools/**', '!Tools/scripts/**']),
      'pull_request': ('paths-ignore', ['!Tools/scripts/**', 'Tools/**'])},
     "jobs:\n", ["'Tools/**' comes last in pull_request only"]),
    ("one event filtered and the other not",
     {'push': ('paths-ignore', ['Blimp/**']), 'pull_request': None},
     "jobs:\n", ["does not, so they do not run"]),
    ("a denylist on one side and an allowlist on the other",
     {'push': ('paths-ignore', ['Blimp/**']),
      'pull_request': ('paths', ['Blimp/**'])},
     "jobs:\n", ["run push only", "run pull_request only", "changes to itself"]),
    ("an empty list is reported once",
     {'push': ('paths', [])}, "jobs:\n", ["is empty"]),
    ("a dead pattern is reported",
     {'push': ('paths', ['.github/workflows/w.yml', 'Tools/scripts/nosuch.py'])},
     "jobs:\n", ["matches no tracked file"]),
    ("a workflow that would not run on its own edit",
     {'push': ('paths', ['Tools/scripts/x.py'])}, "jobs:\n", ["changes to itself"]),
    ("a jobs: inside the on: block does not feed the filters back in",
     {'push': ('paths-ignore', ['Tools/**'])},
     "on:\n  push:\n    # skip slow jobs: here\n    paths-ignore: [Tools/**]\njobs:\n  b:\n",
     []),
]


# a dispatcher in the shape of build_ci.sh, so the cases below do not break
# when the real one changes
FAKE_DISPATCHER = '''\
waf=modules/waf/waf-light

run_autotest() {
    Tools/autotest/autotest.py "$1"
}

for t in $CI_BUILD_TARGET; do
    if [ "$t" == "navigator" ]; then
        $waf configure --board navigator
        ./Tools/scripts/firmware_version_decoder.py -f build/navigator/bin/ardusub
        continue
    fi

    if [ "$t" == "sitltest-copter" ]; then
        run_autotest "Copter"
        continue
    fi

    $waf configure --board "$t"
    Tools/scripts/catch_all.py
done
'''

FAKE_TRACKED = {
    'Tools/autotest/autotest.py', 'Tools/scripts/firmware_version_decoder.py',
    'Tools/scripts/catch_all.py', 'Tools/scripts/x.py', '.github/workflows/w.yml',
}
FAKE_LINKS = {'modules/waf'}

EXEC_CASES = [
    # (line, expected paths) -- what counts as running a script
    ("        Tools/scripts/x.py --flag", ['Tools/scripts/x.py']),
    ("        ./Tools/scripts/x.py", ['Tools/scripts/x.py']),
    ("        out=$(Tools/scripts/x.py)", ['Tools/scripts/x.py']),
    ("        cd x && Tools/scripts/x.py", ['Tools/scripts/x.py']),
    # a quoted path is data, not a command: this is what stops the linter
    # reading example paths out of comments and test fixtures
    ('        print("Tools/scripts/x.py")', []),
    ("        NAME = 'Tools/scripts/x.py'", []),
    ("        xTools/scripts/x.py", []),
    ("        Tools/autotest/locations.txt", []),
    # an argument counts as well; narrowing that has not been worth the misses
    ("        grep -f Tools/scripts/x.py file", ['Tools/scripts/x.py']),
]

DISPATCH_CASES = [
    # (description, targets, expected paths)
    ("a literal branch",
     {'navigator'}, ['Tools/scripts/firmware_version_decoder.py']),
    ("a branch that only calls a function",
     {'sitltest-copter'}, ['Tools/autotest/autotest.py']),
    ("a target with no branch falls through to the catch-all",
     {'fmuv3'}, ['Tools/scripts/catch_all.py']),
]

TARGET_CASES = [
    # (description, workflow text, expected targets)
    ("a literal", "jobs:\n  b:\n    env:\n      CI_BUILD_TARGET: sitltest-sub\n", {'sitltest-sub'}),
    ("a matrix value", "jobs:\n  b:\n    strategy:\n      matrix:\n        config: [navigator, linux]\n"
     "    env:\n      CI_BUILD_TARGET: ${{matrix.config}}\n", {'navigator', 'linux'}),
    ("text around a matrix value", "jobs:\n  b:\n    strategy:\n      matrix:\n        config: [sitl]\n"
     "    env:\n      CI_BUILD_TARGET: dds-${{matrix.config}}\n", {'dds-sitl'}),
    ("an include: entry", "jobs:\n  b:\n    strategy:\n      matrix:\n        config: [linux]\n"
     "        include:\n          - config: navigator64\n"
     "    env:\n      CI_BUILD_TARGET: ${{matrix.config}}\n", {'linux', 'navigator64'}),
    ("an inline assignment in a run: line", 'jobs:\n  b:\n    steps:\n      - run: |\n'
     '          CI_BUILD_TARGET="clang_scan_build" Tools/scripts/build_ci.sh\n', {'clang_scan_build'}),
]


def expect(condition, message):
    '''assert that survives python -O, which strips the statement'''
    if not condition:
        raise SystemExit("self-test failed: " + message)


def run_self_test():
    for pattern, path, want in MATCH_CASES:
        got = bool(translate(pattern).match(path))
        expect(got == want, "match(%r, %r) = %s, want %s" % (pattern, path, got, want))
    for patterns, path, want in STATE_CASES:
        got = selected(path, patterns)
        expect(got == want, "selected(%r, %r) = %s, want %s" % (path, patterns, got, want))
    # paths: (an allowlist) inverts the sense of a match
    expect(ignores('paths', ['Tools/autotest/**'], 'libraries/AP_HAL/HAL.h'), "paths ignores")
    expect(not ignores('paths', ['Tools/autotest/**'], 'Tools/autotest/arducopter.py'), "paths runs")
    expect(ignores('paths-ignore', ['Tools/autotest/**'], 'Tools/autotest/arducopter.py'),
           "paths-ignore ignores")
    expect(not ignores('paths-ignore', ['Tools/autotest/**'], 'libraries/AP_HAL/HAL.h'),
           "paths-ignore runs")
    tracked = set(SELF_TEST_FILES)
    for description, events, body, wanted in CHECK_CASES:
        got = list(check("w.yml", body, events, SELF_TEST_FILES, tracked))
        expect(len(got) == len(wanted), "%s: got %r, want %d message(s)"
               % (description, got, len(wanted)))
        for want in wanted:
            expect(any(want in message for message in got),
                   "%s: no message mentions %r in %r" % (description, want, got))

    for line, want in EXEC_CASES:
        got = EXEC.findall(line)
        expect(got == want, "exec(%r) = %r, want %r" % (line, got, want))

    resolve = resolver(FAKE_TRACKED, FAKE_LINKS)
    for description, targets, want in DISPATCH_CASES:
        got, branches = dispatcher_deps(FAKE_DISPATCHER, targets, resolve)
        expect(branches == 2, "%s: parsed %d branches, want 2" % (description, branches))
        expect(sorted(got) == sorted(want), "%s: got %r, want %r" % (description, sorted(got), want))
    # a rewritten dispatcher parses as no branches at all, which is what the
    # count in main() reports rather than passing every workflow silently
    _, branches = dispatcher_deps('case "$t" in\n  navigator) x.py ;;\nesac\n', {'navigator'}, resolve)
    expect(branches == 0, "a case statement should parse as no branches, got %d" % branches)

    for description, body, want in TARGET_CASES:
        got = build_targets(yaml.safe_load(body), body)
        expect(got == want, "%s: got %r, want %r" % (description, got, want))

    # a dependency reached through a script is reported like a named one
    events = {'push': ('paths-ignore', ['Tools/**'])}
    got = list(check("w.yml", "jobs:\n", events, SELF_TEST_FILES, tracked,
                     ['Tools/scripts/x.py']))
    expect(any('Tools/scripts/x.py' in m for m in got), "a hidden dependency: %r" % got)
    got = list(check("w.yml", "jobs:\n", events, SELF_TEST_FILES, tracked,
                     ['Blimp/mode.cpp']))
    expect(not any('Blimp' in m for m in got), "a visible dependency should be silent: %r" % got)

    print("self-test OK (%d match, %d state, %d check, %d exec, %d dispatch, %d target cases)"
          % (len(MATCH_CASES), len(STATE_CASES), len(CHECK_CASES), len(EXEC_CASES),
             len(DISPATCH_CASES), len(TARGET_CASES)))


def main():
    parser = argparse.ArgumentParser(
        description="lint the path filters of the GitHub Actions workflows")
    parser.add_argument("--self-test", action="store_true",
                        help="run the unit tests and exit")
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return 0

    root = repo_root()
    files = tracked_files(root)
    tracked = set(files)
    resolve = resolver(tracked, gitlinks(root))
    dispatcher = None
    if os.path.exists(os.path.join(root, DISPATCHER)):
        with open(os.path.join(root, DISPATCHER), errors='replace') as fh:
            dispatcher = fh.read()
    found = 0
    seen = 0
    filtered = 0
    parsed_branches = None
    for name, body, doc, events, error in load_workflows(os.path.join(root, ".github/workflows")):
        seen += 1
        if error:
            print("%s: %s" % (name, error))
            found += 1
            continue
        if not any(events.values()):
            continue
        filtered += 1
        targets = build_targets(doc, body)
        deps, branches = workflow_deps(root, body, targets, resolve, dispatcher)
        if branches is not None:
            parsed_branches = branches
            if not targets:
                # the assignment moved or changed shape; without it the
                # dependencies of this workflow are unknown rather than empty
                print("%s: runs %s but no CI_BUILD_TARGET could be read" % (name, DISPATCHER))
                found += 1
        for message in sorted(set(check(name, body, events, files, tracked, deps))):
            print("%s: %s" % (name, message))
            found += 1
    if parsed_branches is not None and parsed_branches < MIN_BRANCHES:
        print("%s: only %d target branches parsed, expected %d or more; the "
              "per-target dependency check is not working"
              % (DISPATCHER, parsed_branches, MIN_BRANCHES))
        found += 1
    print("%d workflows, %d with path filters, %d problems" % (seen, filtered, found))
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
