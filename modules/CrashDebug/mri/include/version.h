

#ifndef MRI_VERSION_STRING

#define MRI_BRANCH "https://github.com/adamgreen/mri/tree/version_1.0"

#define MRI_VERSION_MAJOR       1
#define MRI_VERSION_MINOR       0
#define MRI_VERSION_BUILD       20170707
#define MRI_VERSION_SUBBUILD    1

#define MRI_STR(X) MRI_STR2(X)
#define MRI_STR2(X) #X

#define MRI_VERSION_STRING MRI_STR(MRI_VERSION_MAJOR) "." MRI_STR(MRI_VERSION_MINOR) "-" MRI_STR(MRI_VERSION_BUILD) "." MRI_STR(MRI_VERSION_SUBBUILD)

#endif
