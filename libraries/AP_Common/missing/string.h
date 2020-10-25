#include_next <string.h>

// Necessary for toolchains that does not provide `strndupa`, such as musl.
#if !defined(HAVE_DECL_STRNDUPA) && !defined(strndupa)
// The last value of the GCC extension "statement exprs" will be
// evaluated and returned, E.g: `#define foo(n) ({ n; })` is equivalent for
// `auto foo(auto n) { return n; }`
#define strndupa(old_string, len)                                     \
({                                                                    \
    const size_t string_len = strnlen(old_string, len);               \
    char *new_string = static_cast<char*>(alloca(string_len + 1));    \
    new_string[string_len] = '\0';                                    \
    static_cast<char*>(memcpy(new_string, old_string, len));          \
})
#endif
