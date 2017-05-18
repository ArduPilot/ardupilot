#Powershell script to donwload and configure the APM SITL environment

Import-Module BitsTransfer

Write-Output "Starting Downloads"

Write-Output "Downloading MAVProxy (1/6)"
Start-BitsTransfer -Source "http://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe" -Destination "$PSScriptRoot\MAVProxySetup-latest.exe"

Write-Output "Downloading Cygwin x64 (2/6)"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "Installing Cygwin x64 (3/6)"
& $PSScriptRoot\setup-x86_64.exe --root="C:\cygwin" --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site="http://cygwin.mirror.constant.com" --packages autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel, libxml2-devel,python-libxml2,libxslt-devel,python-devel,procps-ng --quiet-mode | Out-Null
Start-Process -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel, libxml2-devel,python-libxml2,libxslt-devel,python-devel,procps-ng --quiet-mode"

Write-Output "Configuring Cygwin (4/6)"
Copy-Item apm_install.sh C:\cygwin\home

Write-Output "Downloading APM Source Code Cygwin (5/6)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c ../apm_install.sh"

Write-Output "Installing MAVProxy (6/6)"
& $PSScriptRoot\MAVProxySetup-latest.exe /SILENT | Out-Null

Write-Host "Finished. Press any key to continue ..."
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")