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

@conf
def check_librt(cfg, env):
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
def check_lttng(cfg, env):
    cfg.check_cfg(package='lttng-ust', mandatory=False, global_define=True,
                  args=['--libs', '--cflags'])
    env.LIB += cfg.env['LIB_LTTNG-UST']
    return True

@conf
def check_libiio(cfg, env):
    cfg.check_cfg(package='libiio', mandatory=False, global_define=True,
                  args=['--libs', '--cflags'])
    env.LIB += cfg.env['LIB_LIBIIO']
    # workaround bug in libiio 0.6 not including -ldl
    if cfg.env['LIB_LIBIIO']:
        env.LIB += ['dl']
    return True
