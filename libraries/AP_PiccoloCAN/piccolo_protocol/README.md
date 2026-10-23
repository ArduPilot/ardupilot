# Piccolo Protocol Generation

The Piccolo CAN protocol messages are generated using the [ProtoGen](https://github.com/billvaglienti/ProtoGen) protocol generation tool.

The raw protocol definition for each device type is provided in a `.xml` file (e.g. `protocol_esc_velocity.xml`).

To regenerate the protocol, run the following command:

`protogen.exe protocol_esc_velocity.xml --license license.txt`
