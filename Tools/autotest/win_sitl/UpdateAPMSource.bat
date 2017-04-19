rem File to update the APM source
rem Assumes a Cgywin install at C:\cygwin
chdir C:\cygwin\bin
bash --login -i -c "cd ./ardupilot && git pull && git submodule update --init --recursive"
pause
