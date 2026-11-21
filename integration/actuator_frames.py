"""Encode complete actuator frames for software tests and a future serial sender.

No serial connection or motor output is performed here.
"""

from binascii import crc_hqx


def encode_frame(command: str, session: int, sequence: int, pwms=None) -> str:
    if command not in {"ARM", "PWM", "DISARM", "RESET"}:
        raise ValueError("unknown actuator command")
    if not 0 <= session <= 0xFFFFFFFF or not 1 <= sequence <= 0xFFFFFFFF:
        raise ValueError("session or sequence outside uint32 range")
    fields = ["A1", command, f"{session:08X}", str(sequence)]
    if command == "PWM":
        if pwms is None or len(pwms) != 8 or any(
            not isinstance(v, int) or isinstance(v, bool) or not 1300 <= v <= 1700
            for v in pwms
        ):
            raise ValueError("PWM needs eight values from 1300 to 1700 us")
        fields.extend(str(v) for v in pwms)
    elif pwms is not None:
        raise ValueError("PWM values only belong to PWM commands")
    body = ",".join(fields)
    return f"{body},{crc_hqx(body.encode('ascii'), 0xFFFF):04X}"
