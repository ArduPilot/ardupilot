# Windows Powershell script, installs and builds Ardupilot Wiki in cygwin 
# Most of the heavy lifting is done within cygwin shell via installdoc.sh
# Can be run on top of existing cygwin install
# Right click file and select "Run with Powershell"
# Windows 10 only

Import-Module BitsTransfer

Write-Output "- Downloading cygwin"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "- Installing or updating Cygwin x64"
Start-Process -NoNewWindow -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages git,make,python2,zip,mercurial --quiet-mode"

#Run in cygwin:
@'
#!/bin/bash

# Install and build Ardupilot wiki in cygwin environment
# 
#
# Process can be lengthy, and default ouput from commands overwhelming 
# (and -quiet options too silent) so we add a simple sh progress bar and spinner
# and log to /tmp.  
# 

#set -o errexit
#set -x

function spinner {

	i=1
	sp="/-\|"
	echo -n ' '
	while true
		do
		sleep .3
		printf "\b${sp:i++%${#sp}:1}"
	done
}

function ProgressBar {
# Modified from github @fearside thanks

	let progress=${1}
	let done=(${progress}*4)/10
	let left=40-$done

	fill=$(printf "%${done}s")
	empty=$(printf "%${left}s")

	printf "\r$2: [${fill// /\#}${empty// /-}] ${progress}%%  "

}

logfile=/tmp/wiki-install.`date +%Y-%m-%d.%H:%M:%S`.log

spinner & >> $logfile 2>&1
SPID=$! && disown

printf '\nInstalling Ardupilot Wiki and build environment within Cygwin\n'
printf '[Open 2nd Cygwin terminal and use tail -f /tmp/*.log for verbose progress output]\n\n'
printf 'This may take several minutes ...\n\n'

# Get pip
ProgressBar 3 "Getting pip           "
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py > $logfile 2>&1
python get-pip.py  >> $logfile 2>&1
rm -f get-pip.py  >> $logfile 2>&1

# Clone wiki 
ProgressBar 10 "Downloading wiki     "
git clone https://github.com/ArduPilot/ardupilot_wiki.git >>$logfile 2>&1

# Install sphinx  
ProgressBar 30 "Installing Sphinx    " 
pip -q install -U sphinx==1.8.3 >> $logfile 2>&1

# Install sphinx theme from ArduPilot repository
ProgressBar 40 "Installing theme     " 
pip install git+https://github.com/ArduPilot/sphinx_rtd_theme.git -UI >> $logfile 2>&1

# Install youtube and vimeo plugins:
ProgressBar 60 "Installing plugins   " 
pip  install git+https://github.com/sphinx-contrib/youtube.git -UI >> $logfile 2>&1
hg clone https://bitbucket.org/jdouglass/sphinxcontrib.vimeo /tmp/sphinxcontrib.vimeo >> $logfile 2>&1
pushd /tmp/sphinxcontrib.vimeo   >> $logfile  2>&1
python setup.py build >> $logfile   2>&1
python setup.py install >> $logfile  2>&1
popd   >>  $logfile  2>&1

ProgressBar 100 "Completed           "         

rm -rf /tmp/sphinxcontrib.vimeo
kill -SIGKILL $SPID   >> $logfile  2>&1

printf '\n\nArdupilot Wiki local Installation and Build environment Completed\n'
printf '\n\nRendering copter and plane wiki as example ...\n\n'

# We only make copter and plane wiki sections, and leave it optional for the user  
# to render the entire wiki later as this can take a lot of time ...

spinner &     >> $logfile 2>&1
SPID=$!
disown

ProgressBar 3 "Rendering copter html"
make -C ~/ardupilot_wiki/copter html >> $logfile   2>&1

ProgressBar 50 "Rendering plane html"
make -C ~/ardupilot_wiki/plane html >> $logfile   2>&1

ProgressBar 100 "Completed           "

printf '\n\n\nCopter and plane sections rendered\n'
printf 'Use make -C ~/ardupilot_wiki/xxxxx to render other sections,\n'
printf 'where xxxxx is rover/antennatracker/dev/planner/planner2/ardupilot.\n\n'

printf 'RST files are in ~/ardupilot_wiki/xxxx/source/docs\n'
printf "and can also be edited with any windows editor pointing to C:\cygwin\\home\%s\\\ardupilot_wiki\\\xxxx\\\filename.rst\n\n" $USER

printf 'Or make -C ~/ardupilot_wiki all to render all files (Takes some time).\n\n'

kill -SIGKILL $SPID   >> $logfile  2>&1

# Ugly hack: launch new shell to keep terminal open
# Todo: Detach cygwin terminal from within powershell if possible

printf '\n\nClose this window for auto launch of browser with rendered copter and plane sections\n'
/bin/bash

'@ > 'C:\cygwin\tmp\installdoc.sh'

#Convert character encoding and line endings
Start-Process -NoNewWindow -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c 
'iconv -f UTF-16 -t UTF-8 /tmp/installdoc.sh  > /tmp/installdoc.sh- && mv /tmp/installdoc.sh- /tmp/installdoc.sh'"   
Start-Process -NoNewWindow -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c
 'sed -i ""s/\r$//"" /tmp/installdoc.sh && chmod 755 /tmp/installdoc.sh'"

Write-Output "- Continuing download and setup in cygwin ..."
Write-Output "  Please do not close this window ..."

Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c '/tmp/installdoc.sh'"

Write-Output "- Launching browser to local APcopter wiki page as an example"
Start-Process "file:///C:/cygwin/home/$([Environment]::UserName)/ardupilot_wiki/copter/build/html/index.html"

Write-Host "Press any key to close this window"
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
