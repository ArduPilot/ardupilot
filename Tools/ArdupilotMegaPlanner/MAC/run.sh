#!/bin/sh
#get the bundle's MacOS directory full path
DIR=$(cd "$(dirname "$0")"; pwd)
 
#change these values to match your app
EXE_PATH="$DIR/ArdupilotMegaPlanner10.exe"
PROCESS_NAME=ArdupilotMegaPlanner10
APPNAME="ArdupilotMegaPlanner10"
 
#set up environment
MONO_FRAMEWORK_PATH=/Library/Frameworks/Mono.framework/Versions/Current
export DYLD_FALLBACK_LIBRARY_PATH="$DIR:$MONO_FRAMEWORK_PATH/lib:/lib:/usr/lib"
export PATH="$MONO_FRAMEWORK_PATH/bin:$PATH"

#export MONO_MWF_MAC_FORCE_X11=true
 
#mono version check
REQUIRED_MAJOR=2
REQUIRED_MINOR=8
 
VERSION_TITLE="Cannot launch $APPNAME"
VERSION_MSG="$APPNAME requires the Mono Framework version $REQUIRED_MAJOR.$REQUIRED_MINOR or later."
DOWNLOAD_URL="http://www.go-mono.com/mono-downloads/download.html"
 
MONO_VERSION="$(mono --version | grep 'Mono JIT compiler version ' |  cut -f5 -d\ )"
MONO_VERSION_MAJOR="$(echo $MONO_VERSION | cut -f1 -d.)"
MONO_VERSION_MINOR="$(echo $MONO_VERSION | cut -f2 -d.)"
if [ -z "$MONO_VERSION" ] \
	|| [ $MONO_VERSION_MAJOR -lt $REQUIRED_MAJOR ] \
	|| [ $MONO_VERSION_MAJOR -eq $REQUIRED_MAJOR -a $MONO_VERSION_MINOR -lt $REQUIRED_MINOR ] 
then
	osascript \
	-e "set question to display dialog \"$VERSION_MSG\" with title \"$VERSION_TITLE\" buttons {\"Cancel\", \"Download...\"} default button 2" \
	-e "if button returned of question is equal to \"Download...\" then open location \"$DOWNLOAD_URL\""
	echo "$VERSION_TITLE"
	echo "$VERSION_MSG"
	exit 1
fi
 
#get an exec command that will work on the current OS version
OSX_VERSION=$(uname -r | cut -f1 -d.)
if [ $OSX_VERSION -lt 9 ]; then  # If OSX version is 10.4
	MONO_EXEC="exec mono"
else
	MONO_EXEC="exec -a \"$PROCESS_NAME\" mono"
fi
 
#create log file directory if it doesn't exist
LOG_FILE="$HOME/Library/Logs/$APPNAME/$APPNAME.log"
mkdir -p "`dirname \"$LOG_FILE\"`"
 
#run app using mono
$MONO_EXEC $MONO_OPTIONS "$EXE_PATH" $* 2>&1 1> "$LOG_FILE"

