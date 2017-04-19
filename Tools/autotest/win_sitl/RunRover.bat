rem File run APM:Rover SITL
rem Assumes a Cgywin install at C:\cygwin
chdir C:\cygwin\bin
bash --login -i -c "cd ./ardupilot/APMrover2 && ../Tools/autotest/sim_vehicle.py"
