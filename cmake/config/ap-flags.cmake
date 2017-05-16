# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

set(DBG_FLAGS 
   "-g \
    -ggdb \
    -O0"
)

set(REQ_FLAGS 
   "-std=gnu++11 \
    -fsigned-char"
)
    
set(OPT_FLAGS 
   "-Wformat \
    -Wall \
    -Wshadow  \
    -Wpointer-arith \
    -Wcast-align \
    -Wwrite-strings \
    -Wformat=2"
)

set(WRN_FLAGS 
   "-Wno-reorder \
    -Wno-unused-parameter \
    -Wno-missing-field-initializers \
    -Wno-unused-function \
    -Wno-unused-parameter \
    -Wno-missing-field-initializers \
    -Werror=unused-but-set-variable \
    -Werror=format-security \
    -Werror=array-bounds \
    -Werror=unused-but-set-variable \
    -Werror=uninitialized \
    -Werror=init-self"
)