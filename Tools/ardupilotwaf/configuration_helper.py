"""
WAF Tool that helps on configuration parameter, creating the AP_Config
header file.

This tool needs compiler_c and compiler_cxx to be loaded, make sure
you load them before this tool.

Example::
    def configure(cfg):
        cfg.load('configuration_helper')
"""

def configure(cfg):
    cfg.check_cxx(fragment='#include<cmath>\n int main() { std::isfinite(1); }\n',
                  define_name="HAVE_CMATH_ISFINITE", mandatory=False)

    cfg.check_cxx(fragment='#include<math.h>\n int main() { isinf(1); }\n',
                  define_name="HAVE_MATH_ISINF", mandatory=False)

    cfg.check_cxx(fragment='#include<cmath>\n int main() { std::isinf(1); }\n',
                  define_name="HAVE_CMATH_ISINF", mandatory=False)

    cfg.check_cxx(fragment='#include<math.h>\n int main() { isnan(1); }\n',
                  define_name="HAVE_MATH_ISNAN", mandatory=False)

    cfg.check_cxx(fragment='#include<cmath>\n int main() { std::isnan(1); }\n',
                  define_name="HAVE_CMATH_ISNAN", mandatory=False)

