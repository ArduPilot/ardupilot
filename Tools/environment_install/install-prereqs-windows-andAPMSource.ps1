#Powershell script to download and configure the APM SITL environment

Import-Module BitsTransfer

Write-Output "Starting Downloads"

Write-Output "Downloading MAVProxy (1/6)"
Start-BitsTransfer -Source "https://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe" -Destination "$PSScriptRoot\MAVProxySetup-latest.exe"

Write-Output "Downloading Cygwin x64 (2/6)"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "Installing Cygwin x64 (3/6)"
Start-Process -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin64 --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel,libxml2-devel,python36,python36-future,python36-lxml,python36-pip,libxslt-devel,python36-devel,procps-ng,zip,gdb,ddd --quiet-mode"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c 'ln -s /usr/bin/python3.6 /usr/bin/python'"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c 'ln -s /usr/bin/pip3.6 /usr/bin/pip'"

Write-Output "Downloading extra Python packages (4/6)"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c 'pip install empy pyserial pymavlink'"

Write-Output "Downloading APM source (5/6)"
Copy-Item "APM_install.sh" -Destination "C:\cygwin64\home"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c ../APM_install.sh"

Write-Output "Installing MAVProxy (6/6)"
& $PSScriptRoot\MAVProxySetup-latest.exe /SILENT | Out-Null

Write-Host "Finished. Press any key to continue ..."
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
