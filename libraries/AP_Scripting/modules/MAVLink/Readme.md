These .lua definition for messages can be generated using pymavlink with the following command:
(change the last path acoordingly to place the files where you need them)
```
cd ardupilot/modules/mavlink
python ./pymavlink/tools/mavgen.py --lang Lua ./message_definitions/v1.0/all.xml --out ./modules/MAVLink
```