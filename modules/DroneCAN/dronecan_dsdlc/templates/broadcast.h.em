@{
if msg.kind == msg.KIND_MESSAGE:
    from dronecan_dsdlc_helpers import *
    empy.include('templates/msg.h.em', get_empy_env_broadcast(msg))
}@
