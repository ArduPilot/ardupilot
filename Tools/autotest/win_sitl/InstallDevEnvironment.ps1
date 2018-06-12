#Powershell script to download and configure the APM SITL environment

Import-Module BitsTransfer

Write-Output "Starting Downloads"

Write-Output "Downloading MAVProxy (1/7)"
Start-BitsTransfer -Source "http://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe" -Destination "$PSScriptRoot\MAVProxySetup-latest.exe"

Write-Output "Downloading Cygwin x64 (2/7)"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "Installing Cygwin x64 (3/7)"
Start-Process -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel,libxml2-devel,python2,python2-future,python2-libxml2,python2-pip,libxslt-devel,python2-devel,procps-ng,zip,gdb,ddd --quiet-mode"

Write-Output "Copying JSBSim install script to Cygwin (4/7)"
Start-BitsTransfer -Source "https://github.com/ArduPilot/ardupilot/raw/master/Tools/autotest/win_sitl/jsbsim_install.sh" -Destination "C:\cygwin\home\jsbsim_install.sh"

Write-Output "Downloading extra Python packages (5/7)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c 'pip2 install empy'"

Write-Output "Downloading and installing JSBSim (6/7)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c ../jsbsim_install.sh"

Write-Output "Installing MAVProxy (7/7)"
& $PSScriptRoot\MAVProxySetup-latest.exe /SILENT | Out-Null

Write-Host "Finished. Press any key to continue ..."
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")