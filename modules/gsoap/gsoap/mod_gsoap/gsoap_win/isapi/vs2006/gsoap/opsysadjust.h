/** windoze adaption header file, because that one is missing posix.
    @file opsysadjust.h
 */

#ifdef _WIN32

#if _MSC_VER > 1000
    #pragma warning (disable:4786)
#endif
/** C-standard interface for case insensitive compare */
inline int strcasecmp(const char *psz1, const char *psz2) {
    return stricmp(psz1, psz2);
}

#endif //_WIN32