#Powershell script to download and configure the APM SITL environment

Import-Module BitsTransfer

Write-Output "Starting Downloads"

Write-Output "Downloading MAVProxy (1/8)"
Start-BitsTransfer -Source "https://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe" -Destination "$PSScriptRoot\MAVProxySetup-latest.exe"

Write-Output "Downloading Cygwin x64 (2/8)"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "Downloading ARM GCC Compiler 10-2020-Q4-Major (3/8)"
Start-BitsTransfer -Source "https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-win32.exe" -Destination "$PSScriptRoot\gcc-arm-none-eabi-10-2020-q4-major-win32.exe"

Write-Output "Installing Cygwin x64 (4/8)"
Start-Process -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin64 --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages autoconf,automake,ccache,cygwin32-gcc-g++,gcc-g++=7.4.0-1,libgcc1=7.4.0.1,gcc-core=7.4.0-1,git,libtool,make,gawk,libexpat-devel,libxml2-devel,python37,python37-future,python37-lxml,python37-pip,libxslt-devel,python37-devel,procps-ng,zip,gdb,ddd --quiet-mode"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c 'ln -sf /usr/bin/python3.7 /usr/bin/python'"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c 'ln -sf /usr/bin/pip3.7 /usr/bin/pip'"

Write-Output "Downloading extra Python packages (5/8)"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c 'pip install empy pyserial pymavlink intelhex dronecan'"

Write-Output "Downloading APM source (6/8)"
Copy-Item "APM_install.sh" -Destination "C:\cygwin64\home"
Start-Process -wait -FilePath "C:\cygwin64\bin\bash" -ArgumentList "--login -i -c ../APM_install.sh"

Write-Output "Installing ARM GCC Compiler 10-2020-Q4-Major (7/8)"
& $PSScriptRoot\gcc-arm-none-eabi-10-2020-q4-major-win32.exe /S /P /R

Write-Output "Installing MAVProxy (8/8)"
& $PSScriptRoot\MAVProxySetup-latest.exe /SILENT | Out-Null

Write-Host "Finished. Press any key to continue ..."
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
