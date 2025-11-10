These .lua definition for messages can be generated using pymavlink with the following command:
(change the last path accordingly to place the files where you need them)
```
cd ardupilot/modules/mavlink
python ./pymavlink/tools/mavgen.py --lang Lua ./message_definitions/v1.0/all.xml --out ./modules/MAVLink --wire-protocol 2.0
```

Add `--wire-protocol 2.0` to include extension fields in the generated code
