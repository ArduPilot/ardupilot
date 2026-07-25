#!/bin/bash
package=$1

for dir in src include tests ; do
    packageDir=${dir}/${package}
    if [ ! -d "$packageDir" ] ; then
       echo "creating $packageDir"
       mkdir $packageDir
    fi
done

