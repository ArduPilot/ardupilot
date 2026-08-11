'''Out-of-tree ArduPilot builds, keyed for reuse.

 AP_FLAKE8_CLEAN
'''

import fcntl
import os
import subprocess


class WafBuild(object):
    '''encapsulates an out-of-tree waf build.

    An instance describes a (key, board, targets, configure-arguments)
    tuple.  The build lives in build-<key>/ at the top of the source
    tree, with waf's lockfile confined to that directory: waf is run
    with its working directory inside the build directory and with
    NO_LOCK_IN_TOP set, so the source tree's own default build - and
    any other keyed build - is never disturbed.  Builds with the same
    key share a build directory and so are incremental across runs;
    an advisory lock in the build directory serialises concurrent
    builders using the same key.

    The intent is that tests which need binaries built declare one of
    these rather than running waf themselves.  A future test runner
    can collect the builds declared by the tests selected for a run,
    deduplicate them by key, and build exactly - and lazily - what the
    run requires, replacing the up-front build.Vehicle step.
    '''

    def __init__(self, key, board, targets, topdir, configure_args=None):
        self.key = key
        self.board = board
        self.targets = targets
        self.topdir = os.path.abspath(topdir)
        self.configure_args = list(configure_args or [])

    @property
    def out_dir(self):
        return os.path.join(self.topdir, "build-%s" % self.key)

    @property
    def waf(self):
        return os.path.join(self.topdir, "waf")

    def binary_path(self, target=None):
        '''path a built target lands at; target defaults to the first
        target this build was declared with'''
        if target is None:
            target = self.targets[0]
        return os.path.join(self.out_dir, self.board, target)

    def is_configured(self):
        return os.path.exists(os.path.join(self.out_dir, "c4che", "_cache.py"))

    def _env(self):
        env = dict(os.environ)
        # confine waf's lockfile to the build directory; without this
        # waf also writes its lock into the source tree, repointing the
        # tree's default build and racing other builders:
        env["NO_LOCK_IN_TOP"] = "1"
        return env

    def _run_waf(self, args):
        cmd = [self.waf, "--top", self.topdir, "--out", self.out_dir] + args
        subprocess.check_call(cmd, cwd=self.out_dir, env=self._env())

    def _holding_build_lock(self):
        '''advisory lock so concurrent builders sharing a key serialise
        rather than corrupting the build directory.  Returns an open
        file object; the lock is held until it is closed.'''
        lockfile = open(os.path.join(self.out_dir, ".builder-lock"), "w")
        fcntl.flock(lockfile, fcntl.LOCK_EX)
        return lockfile

    def configure(self):
        os.makedirs(self.out_dir, exist_ok=True)
        # submodules are shared with the source tree; concurrent
        # configures must not race to update them:
        args = ["configure", "--board", self.board, "--no-submodule-update"]
        args.extend(self.configure_args)
        self._run_waf(args)

    def build(self):
        self._run_waf(["build", "--targets", ",".join(self.targets)])

    def run(self):
        '''configure if required, then build.  Serialised against other
        builders using the same key.'''
        os.makedirs(self.out_dir, exist_ok=True)
        lock = self._holding_build_lock()
        try:
            if not self.is_configured():
                self.configure()
            self.build()
        finally:
            lock.close()
        return self.binary_path()

    def assert_config_matches(self, other_build_dir):
        '''raise unless this build's generated configuration is
        identical to another build directory's (e.g. the tree's default
        build/sitl).  Guards uses - Replay in particular - where any
        compile-time difference from the binary under test silently
        invalidates the result rather than failing to build.'''
        mine = os.path.join(self.out_dir, self.board, "ap_config.h")
        theirs = os.path.join(other_build_dir, "ap_config.h")
        with open(mine) as f:
            mine_content = f.read()
        with open(theirs) as f:
            theirs_content = f.read()
        if mine_content != theirs_content:
            raise ValueError(
                "WafBuild(%s): configuration does not match %s" %
                (self.key, other_build_dir))
