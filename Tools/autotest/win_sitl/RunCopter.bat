rem File run APM:Copter SITL
rem Assumes a Cgywin install at C:\cygwin
chdir C:\cygwin\bin
bash --login -i -c "cd ./ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py"
