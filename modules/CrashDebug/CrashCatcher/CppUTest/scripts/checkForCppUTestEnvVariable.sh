#!/bin/bash
checkForCppUTestEnvVariable() {
	if [ -z "$CPPUTEST_HOME" ] ; then
	   echo "CPPUTEST_HOME not set"
	   exit 1
	fi
	if [ ! -d "$CPPUTEST_HOME" ] ; then
	   echo "CPPUTEST_HOME not set to a directory"
	   exit 2
	fi
}
