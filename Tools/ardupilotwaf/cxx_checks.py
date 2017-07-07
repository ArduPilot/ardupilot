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
"""
WAF Tool that checks cxx parameters, creating the ap_config.h
header file.

This tool needs compiler_cxx to be loaded, make sure you
load them before this tool.

Example::
    def configure(cfg):
        cfg.load('cxx_checks')
"""

from waflib.Configure import conf

@conf
def ap_common_checks(cfg):
    cfg.check(
        compiler='cxx',
        fragment='''
        #include <cmath>

        int main() {
          return std::isfinite(1.0f);
        }''',
        define_name="HAVE_CMATH_ISFINITE",
        msg="Checking for HAVE_CMATH_ISFINITE",
        mandatory=False,
    )

    cfg.check(
        compiler='cxx',
        fragment='''
        #include <cmath>

        int main() {
          return std::isinf(1.0f);
        }''',
        define_name="HAVE_CMATH_ISINF",
        msg="Checking for HAVE_CMATH_ISINF",
        mandatory=False,
    )

    cfg.check(
        compiler='cxx',
        fragment='''
        #include <cmath>

        int main() {
          return std::isnan(1.0f);
        }''',
        define_name="HAVE_CMATH_ISNAN",
        msg="Checking for HAVE_CMATH_ISNAN",
        mandatory=False,
    )

    # NEED_CMATH_FUNCTION_STD_NAMESPACE checks are needed due to
    # new gcc versions being more restrictive.
    #
    # Here we check if we need to add 'using std::function' to
    # the function.
    #
    # Without these checks, in some cases, gcc points this as
    # overloads or function duplication in scope.

    cfg.check(
        compiler='cxx',
        fragment='''
        #include <math.h>
        #include <cmath>

        using std::isfinite;

        int main() {
          return isfinite((double)1);
        }''',
        define_name="NEED_CMATH_ISFINITE_STD_NAMESPACE",
        msg="Checking for NEED_CMATH_ISFINITE_STD_NAMESPACE",
        mandatory=False,
    )

    cfg.check(
        compiler='cxx',
        fragment='''
        #include <math.h>
        #include <cmath>

        using std::isinf;

        int main() {
          return isinf((double)1);
        }''',
        define_name="NEED_CMATH_ISINF_STD_NAMESPACE",
        msg="Checking for NEED_CMATH_ISINF_STD_NAMESPACE",
        mandatory=False,
    )

    cfg.check(
        compiler='cxx',
        fragment='''
        #include <math.h>
        #include <cmath>

        using std::isnan;

        int main() {
          return isnan((double)1);
        }''',
        define_name="NEED_CMATH_ISNAN_STD_NAMESPACE",
        msg="Checking for NEED_CMATH_ISNAN_STD_NAMESPACE",
        mandatory=False,
    )

    cfg.check(header_name='endian.h', mandatory=False)

    cfg.check(header_name='byteswap.h', mandatory=False)

@conf
def check_librt(cfg, env):
    if cfg.env.DEST_OS == 'darwin':
        return True

    ret = cfg.check(
        compiler='cxx',
        fragment='''
        #include <time.h>

        int main() {
            clock_gettime(CLOCK_REALTIME, NULL);
        }''',
        msg='Checking for need to link with librt',
        okmsg='not necessary',
        errmsg='necessary',
        mandatory=False,
    )

    if ret:
        return ret

    ret = cfg.check(compiler='cxx', lib='rt')
    if ret:
        env.LIB += cfg.env['LIB_RT']

    return ret

@conf
def check_package(cfg, env, libname):
    '''use pkg-config to look for an installed library that has a LIBNAME.pc file'''
    capsname = libname.upper()

    # we don't want check_cfg() changing the global environment during
    # this test, in case it fails in the 2nd link step
    cfg.env.stash()

    cfg.check_cfg(package=libname, mandatory=False, global_define=True,
                  args=['--libs', '--cflags'], uselib_store=capsname)

    # we need to also check that we can link against the lib. We may
    # have a pc file for the package, but its the wrong
    # architecture. This can happen as PKG_CONFIG_PATH is not
    # architecture specific
    cfg.env.LIB += cfg.env['LIB_%s' % capsname]
    cfg.env.INCLUDES += cfg.env['INCLUDES_%s' % capsname]
    cfg.env.CFLAGS += cfg.env['CFLAGS_%s' % capsname]
    cfg.env.LIBPATH += cfg.env['LIBPATH_%s' % capsname]

    ret = cfg.check(
        compiler='cxx',
        fragment='''int main() { return 0; }''',
        msg='Testing link with %s' % libname,
        mandatory=False,
        lib='dl'
    )

    if ret:
        env.LIB += cfg.env['LIB_%s' % capsname]
        env.INCLUDES += cfg.env['INCLUDES_%s' % capsname]
        env.CFLAGS += cfg.env['CFLAGS_%s' % capsname]
        env.LIBPATH += cfg.env['LIBPATH_%s' % capsname]

    cfg.env.revert()

@conf
def check_lttng(cfg, env):
    if cfg.env.STATIC_LINKING:
        # lttng-ust depends on libdl which means it can't be used in a static build
        cfg.msg("Checking for 'lttng-ust':", 'disabled for static build', color='YELLOW')
        return False
    if cfg.options.disable_lttng:
        cfg.msg("Checking for 'lttng-ust':", 'disabled', color='YELLOW')
        return False

    check_package(cfg, env, 'lttng-ust')
    return True

@conf
def check_libiio(cfg, env):
    if cfg.env.STATIC_LINKING:
        # libiio depends on libdl which means it can't be used in a static build
        cfg.msg("Checking for 'libiio':", 'disabled for static build', color='YELLOW')
        return False
    if cfg.options.disable_libiio:
        cfg.msg("Checking for 'libiio':", 'disabled', color='YELLOW')
        return False

    check_package(cfg, env, 'libiio')
    return True

@conf
def check_libdl(cfg, env):
    if cfg.env.STATIC_LINKING:
        # using loadable modules for a static build is not recommended
        cfg.msg("Checking for 'libdl':", 'disabled for static build', color='YELLOW')
        return False
    ret = cfg.check(compiler='cxx', lib='dl', mandatory=False, global_define=True, define_name='HAVE_LIBDL')
    if ret:
        env.LIB += cfg.env['LIB_DL']
    return ret
