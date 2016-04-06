"""
WAF Tool that checks cxx parameters, creating the ap_config.h
header file.

This tool needs compiler_cxx to be loaded, make sure you
load them before this tool.

Example::
    def configure(cfg):
        cfg.load('cxx_checks')
"""

def configure(cfg):
    cfg.check_cxx(fragment='''
                  #include <cmath>

                  int main() {
                    return std::isfinite(1.0f);
                  }''',
                  define_name="HAVE_CMATH_ISFINITE",
                  msg="Checking for HAVE_CMATH_ISFINITE",
                  mandatory=False)

    cfg.check_cxx(fragment='''
                  #include <cmath>

                  int main() {
                    return std::isinf(1.0f);
                  }''',
                  define_name="HAVE_CMATH_ISINF",
                  msg="Checking for HAVE_CMATH_ISINF",
                  mandatory=False)

    cfg.check_cxx(fragment='''
                  #include <cmath>

                  int main() {
                    return std::isnan(1.0f);
                  }''',
                  define_name="HAVE_CMATH_ISNAN",
                  msg="Checking for HAVE_CMATH_ISNAN",
                  mandatory=False)

    # NEED_CMATH_FUNCTION_STD_NAMESPACE checks are needed due to
    # new gcc versions being more restrictive.
    #
    # Here we check if we need to add 'using std::function' to
    # the function.
    #
    # Without these checks, in some cases, gcc points this as
    # overloads or function duplication in scope.

    cfg.check_cxx(fragment='''
                  #include <math.h>
                  #include <cmath>

                  using std::isfinite;

                  int main() {
                    return isfinite((double)1);
                  }''',
                  define_name="NEED_CMATH_ISFINITE_STD_NAMESPACE",
                  msg="Checking for NEED_CMATH_ISFINITE_STD_NAMESPACE",
                  mandatory=False)

    cfg.check_cxx(fragment='''
                  #include <math.h>
                  #include <cmath>

                  using std::isinf;

                  int main() {
                    return isinf((double)1);
                  }''',
                  define_name="NEED_CMATH_ISINF_STD_NAMESPACE",
                  msg="Checking for NEED_CMATH_ISINF_STD_NAMESPACE",
                  mandatory=False)

    cfg.check_cxx(fragment='''
                  #include <math.h>
                  #include <cmath>

                  using std::isnan;

                  int main() {
                    return isnan((double)1);
                  }''',
                  define_name="NEED_CMATH_ISNAN_STD_NAMESPACE",
                  msg="Checking for NEED_CMATH_ISNAN_STD_NAMESPACE",
                  mandatory=False)
