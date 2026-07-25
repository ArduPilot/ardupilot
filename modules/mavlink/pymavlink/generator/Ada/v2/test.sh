#!/bin/bash
for p in 1.0 2.0
do
  for w in ASLUAV.xml common.xml cubepilot.xml icarous.xml marsh.xml minimal.xml ualberta.xml ardupilotmega.xml AVSSUAS.xml csAirLink.xml development.xml loweheiser.xml matrixpilot.xml paparazzi.xml standard.xml uAvionix.xml
  do
    rm -rf ./gen
    python3 -m pymavlink.tools.mavgen --lang=Ada --wire-protocol=$p --output=gen mavlink/message_definitions/v1.0/$w

    if ! gprbuild ./gen/tests/test.gpr; then
      exit 1;
    fi
    
    if ! ./gen/tests/obj/test; then
      exit 1;
    fi
  done
done
echo "all is ok"
