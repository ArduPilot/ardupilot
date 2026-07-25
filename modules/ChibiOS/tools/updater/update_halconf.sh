#!/bin/bash
if [ $# -eq 2 ]
  then
  if [ $1 = "rootpath" ]
  then
    find $2 -name "halconf.h" -exec bash update_halconf.sh "{}" \;
  else
    echo "Usage: update_halconf.sh [rootpath <path>]"
  fi
elif [ $# -eq 1 ]
then
  declare conffile=$(<$1)
#  if egrep -q "" <<< "$conffile"
#  then
    echo Processing: $1
    egrep -e "\#define\s+[a-zA-Z0-9_]*\s+[a-zA-Z0-9_]" <<< "$conffile" | sed 's/\#define //g; s/  */=/g' > ./values.txt
    if ! fmpp -q -C conf.fmpp -S ../ftl/processors/conf/halconf
    then
      echo
      echo "aborted"
      exit 1
    fi
    cp ./halconf.h $1
    rm ./halconf.h ./values.txt
#  fi
else
 echo "Usage: update_halconf.sh [rootpath <root path>]"
 echo "       update_halconf.sh <configuration file>]"
fi
