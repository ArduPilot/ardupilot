#!/bin/bash
if [ $# -eq 2 ]
  then
  if [ $1 = "rootpath" ]
  then
    find $2 -name "vfsconf.h" -exec bash update_vfsconf.sh "{}" \;
  else
    echo "Usage: vfsconf.sh [rootpath <path>]"
  fi
elif [ $# -eq 1 ]
then
  declare conffile=$(<$1)
#  if egrep -q "" <<< "$conffile"
#  then
    echo Processing: $1
    egrep -e "\#define\s+[a-zA-Z0-9_]*\s+[a-zA-Z0-9_]" <<< "$conffile" | sed 's/\#define //g; s/  */=/g' > ./values.txt
    if ! fmpp -q -C conf.fmpp -S ../ftl/processors/conf/vfsconf
    then
      echo
      echo "aborted"
      exit 1
    fi
    cp ./vfsconf.h $1
    rm ./vfsconf.h ./values.txt
#  fi
else
 echo "Usage: update_vfsconf.sh [rootpath <root path>]"
 echo "       update_vfsconf.sh <configuration file>]"
fi
