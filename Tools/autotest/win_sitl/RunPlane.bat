rem File run APM:Plane SITL
rem Assumes a Cgywin install at C:\cygwin
chdir C:\cygwin\bin
bash --login -i -c "cd ./ardupilot/ArduPlane && ../Tools/autotest/sim_vehicle.py"
