#!/bin/sh

# searches for mavlink_msg_*_* function calls and prints
# the corresponding message names to stdout
#
# param1: path to ardupilot directory
# param2: method-name suffix - e.g. "send", "decode" (may be a regex)
#
get_used_mavlink_message_types() {
  REGEX="mavlink_msg_(\w+)_$2\s*\("
  find "$1" \
    `# 'exclude mavlink headers, because the regex matches function declarations as well as calls'` \
    -wholename "$1/libraries/GCS_MAVLink/include/mavlink" -prune -o \
    \( \
      -wholename "$1/APMrover2/*" -o \
      -wholename "$1/ArduCopter/*" -o \
      -wholename "$1/ArduPlane/*" -o \
      -wholename "$1/libraries/*" \
    \) \
    -iregex '.*\.\(c\|cpp\|h\|hpp\|pde\)' \
    `# 'use gcc to remove all comments'`\
    -exec gcc -fpreprocessed -dD -E -w -xc++ {} \; \
      | grep -o -E "$REGEX" \
      | sed -r "s#$REGEX#\1#g" \
      | sort \
      | uniq
}

# searches for mavlink_msg_*_{send,decode} function calls and prints
# the corresponding message names and their payload-lengths to stdout
#
# param1: path to ardupilot directory
#
get_used_mavlink_message_lengths() {
  get_used_mavlink_message_types "$1" '(send|decode)' | while read msg;
  do
    MSG=$(echo "${msg}" | tr '[:lower:]' '[:upper:]');
    REGEX="#define\s+MAVLINK_MSG_ID_${MSG}_LEN\s+([[:digit:]]+)";
    LENGTH=$(find "${1}/libraries/GCS_MAVLink/include/mavlink/v1.0/" -name "mavlink_msg_${msg}.h" -exec cat {} \; \
      | grep -o -E "$REGEX" \
      | sed -r "s@$REGEX@\1@")
    printf '%-40s %3d\n' "$msg" "$LENGTH"
  done
}

###############################################################################

if [ $# -ge 1 ]
  then
    ARDU_PATH="$1"
  else
    SCRIPT_PATH="$( cd "$( dirname "$0" )" && pwd )"
    ARDU_PATH=`readlink -m "${SCRIPT_PATH}/../../"`
    echo "no ardupilot path supplied, using default one: $ARDU_PATH"
fi

echo
echo 'MESSAGE PAYLOAD LENGTHS:'
echo '-----------------------------------------------------------'
get_used_mavlink_message_lengths "$ARDU_PATH"
echo '-----------------------------------------------------------'
