#!/bin/bash
NEW_SCRIPTS=" \
    NewClass \
    NewInterface \
    NewCModule \
    NewCmiModule \
    NewCInterface \
    NewCFunction" 

for file in $NEW_SCRIPTS ; do
    echo ${file} name package
   rm -f ${EXE_DIR}/${file}
   ln -s ${CPPUTEST_HOME}/scripts/${file}.sh ${EXE_DIR}/${file}
   chmod a+x ${EXE_DIR}/${file}
done

