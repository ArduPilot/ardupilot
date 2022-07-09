cmake_minimum_required(VERSION 3.10.2)

project(sfml_display)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)

add_executable(sfml_display
    main.cpp
)

target_link_libraries(sfml_display
  sfml-window
)

# uncomment to match the compiler flags used in ./waf
# target_compile_options(sfml_display
#   PUBLIC
#   -std=gnu++14
#   # -std=gnu++11
#   -fdata-sections
#   -ffunction-sections
#   -fno-exceptions
#   -fsigned-char
#   -Wall
#   -Wextra
#   -Wpointer-arith
#   -Wno-unused-parameter
#   -Wno-missing-field-initializers
#   -Wno-reorder
#   -Wno-redundant-decls
#   -Wno-unknown-pragmas
#   -Wno-expansion-to-defined
#   -Werror=cast-align
#   -Werror=attributes
#   -Werror=format-security
#   -Werror=format-extra-args
#   -Werror=enum-compare
#   -Werror=format
#   -Werror=array-bounds
#   -Werror=uninitialized
#   -Werror=init-self
#   -Werror=narrowing
#   -Werror=return-type
#   -Werror=switch
#   -Werror=sign-compare
#   -Werror=type-limits
#   -Werror=undef
#   -Werror=unused-result
#   -Werror=shadow
#   -Werror=unused-value
#   -Werror=unused-variable
#   -Werror=delete-non-virtual-dtor
#   -Wfatal-errors
#   -Wno-trigraphs
#   -Werror=parentheses
#   -Wuninitialized
#   -Warray-bounds
#   -fcolor-diagnostics
#   -Werror=address-of-packed-member
#   -Werror=inconsistent-missing-override
#   -Werror=overloaded-virtual
#   -Werror=bitfield-enum-conversion
#   -Werror=bool-conversion
#   -Werror=constant-conversion
#   -Werror=enum-conversion
#   -Werror=int-conversion
#   -Werror=literal-conversion
#   -Werror=non-literal-null-conversion
#   -Werror=null-conversion
#   -Werror=objc-literal-conversion
#   -Werror=string-conversion
#   -Wno-gnu-designator
#   -Wno-mismatched-tags
#   -Wno-gnu-variable-sized-type-not-at-end
#   -Werror=implicit-fallthrough
#   -Werror=float-equal
#   -g
#   -O0
#   # -MMD
#   -fno-slp-vectorize
# )

# get_target_property(TARGET_CXX_FLAGS sfml_display COMPILE_OPTIONS)

# message("COMPILE_OPTIONS: ${TARGET_CXX_FLAGS}")