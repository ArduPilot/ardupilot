/*
 * Portions Copyright (c) 1987, 1993, 1994
 * The Regents of the University of California.  All rights reserved.
 *
 * Portions Copyright (c) 2003-2010, PostgreSQL Global Development
 * Group
 *
 * Simple conversion to C++ by Andrew Tridgell for ArduPilot. Based on
 * getopt_long.h from ccache
 */
#pragma once

#include <stdbool.h>

class GetOptLong {
public:
    struct option {
	const char *name;
	bool        has_arg;
	int        *flag;
	int         val;
    };

    GetOptLong(int argc, char *const argv[], const char *optstring, const option * longopts);

    int   opterr;
    int   optind;
    int   optopt;
    int   longindex;
    const char *optarg;

    enum error_return {
        BADCH='?',
        BADARG=':'
    };


    int getoption(void);

private:
    int argc;
    char *const *argv;
    const char *optstring;
    const struct option *longopts;
    const char *place;
};
