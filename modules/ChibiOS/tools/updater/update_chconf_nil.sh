#!/bin/bash
if [ $# -eq 2 ]
  then
  if [ $1 = "rootpath" ]
  then
    find $2 -name "chconf.h" -exec bash update_chconf_nil.sh "{}" \;
  else
    echo "Usage: update_chconf_nil.sh [rootpath <root path>]"
  fi
elif [ $# -eq 1 ]
then
  declare conffile=$(<$1)
  if egrep -q "_CHIBIOS_NIL_CONF_" <<< "$conffile"
  then
    echo Processing: $1
    egrep -e "\#define\s+[a-zA-Z0-9_]*\s+[a-zA-Z0-9_]" <<< "$conffile" | sed 's/\#define //g; s/  */=/g' > ./values.txt
    if ! fmpp -q -C conf.fmpp -S ../ftl/processors/conf/chconf_nil
    then
      echo
      echo "aborted"
      exit 1
    fi
    cp ./chconf.h $1
    rm ./chconf.h ./values.txt
  fi
else
 echo "Usage: update_chconf_nil.sh [rootpath <root path>]"
 echo "       update_chconf_nil.sh <configuration file>]"
fi
