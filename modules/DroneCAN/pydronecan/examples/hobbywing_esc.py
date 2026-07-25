#!/usr/bin/env python3
'''
 hobbywing ESC control program, for testing hobbywing ESC messages
'''

import dronecan, time, math, binascii, sys
from multiprocessing import freeze_support

# get command line arguments
from argparse import ArgumentParser

def handle_msg(msg):
    if msg is not None:
        print('REPLY: ', dronecan.to_yaml(msg))
    else:
        print("No reply")
    sys.exit(0)

def wait_online():
    '''wait for some nodes to come online'''
    print('Waiting for other nodes to become online...')
    while len(node_monitor.get_all_node_id()) < 1:
        try:
            node.spin(timeout=1)
        except Exception as ex:
            print(ex)
    print("done")


def command_SetBaud(arguments):
    '''set CAN baudrate'''
    req = dronecan.com.hobbywing.esc.SetBaud.Request()
    bmap = {
        1000000 : req.BAUD_1MBPS,
        500000 : req.BAUD_500KBPS,
        250000 : req.BAUD_250KBPS,
        200000 : req.BAUD_200KBPS,
        100000 : req.BAUD_100KBPS,
        50000 : req.BAUD_50KBPS,
    }
    if len(arguments) < 1:
        print("Usage: SetBaud BAUDRATE")
        return
    baudrate = int(arguments[0])
    if not baudrate in bmap:
        print("Valid baudrates: ", ','.join(bmap.keys()))
        return
    req.baud = bmap[baudrate]
    node.request(req, args.target_node_id, handle_msg)

def handle_get_info(msg):
    '''handle GetInformation reply'''
    if msg is None:
        print("No response")
        sys.exit(1)
    r = msg.response
    if r.option == 0:
        print("comms_sw_ver %s" % bytes(r.v0).decode("utf-8"))
        print("comms_hw_ver %s" % bytes(r.v1).decode("utf-8"))
        print("comms_dev_type %s" % bytes(r.v2).decode("utf-8"))
        print("comms_unique_code %s" % bytes(r.v3).decode("utf-8"))
    elif r.option == 1:
        print("driver_sw_ver %s" % bytes(r.v0).decode("utf-8"))
        print("driver_hw_ver %s" % bytes(r.v1).decode("utf-8"))
        print("driver_dev_type %s" % bytes(r.v2).decode("utf-8"))
        print("driver_unique_code %s" % bytes(r.v3).decode("utf-8"))
    elif r.option == 2:
        print("hw_unique_code %s" % bytes(r.v0)[:12].hex())
        print("protocol_version %s" % bytes(r.v1)[:8].decode("utf-8"))
    sys.exit(0)

def command_GetInfo(arguments):
    '''get ESC information'''
    req = dronecan.com.hobbywing.esc.GetInformation.Request()
    if len(arguments) < 1:
        print("Usage: GetInformation INFOTYPE")
        return
    req.info_type = int(arguments[0])
    node.request(req, args.target_node_id, handle_get_info)

def command_GetMaintenance(arguments):
    '''get ESC maintenance information'''
    req = dronecan.com.hobbywing.esc.GetMaintenanceInformation.Request()
    if len(arguments) < 1:
        print("Usage: GetMaintenanceInformation OPTION")
        return
    req.option = int(arguments[0])
    node.request(req, args.target_node_id, handle_msg)

def handle_GetESCId(msg):
    '''handle GetEscID reply'''
    r = msg.message
    if len(r.payload) >= 2:
        print("ESC[%u] NodeID=%d ThrottleNum=%d" % (msg.transfer.source_node_id, r.payload[0], r.payload[1]))

def command_GetESCId(arguments):
    '''get ESC IDs from all ESCs'''
    req = dronecan.com.hobbywing.esc.GetEscID()
    if len(arguments) < 1:
        print("Usage: GetESCId OPTION")
        return
    req.payload = [int(arguments[0])]
    node.add_handler(dronecan.com.hobbywing.esc.GetEscID, handle_GetESCId)
    node.broadcast(req)
    
def command_GetMajorConfig(arguments):
    '''get ESC config information'''
    req = dronecan.com.hobbywing.esc.GetMajorConfig.Request()
    if len(arguments) < 1:
        print("Usage: GetMajorConfig OPTION")
        return
    req.option = int(arguments[0])
    node.request(req, args.target_node_id, handle_msg)
    
def command_SetID(arguments):
    '''set ESC ID'''
    req = dronecan.com.hobbywing.esc.SetID.Request()
    if len(arguments) < 2:
        print("Usage: SetID NodeID ThrID")
        return
    req.node_id = int(arguments[0])
    req.throttle_channel = int(arguments[1])
    node.request(req, args.target_node_id, handle_msg)

def command_SetLED(arguments):
    '''set ESC LED'''
    req = dronecan.com.hobbywing.esc.SetLED.Request()
    if len(arguments) < 3:
        print("Usage: SetLED OPTION COLOR BLINK")
        return
    req.option = int(arguments[0])
    req.color = int(arguments[1])
    req.blink = int(arguments[2])
    node.request(req, args.target_node_id, handle_msg)

def command_SetAngle(arguments):
    '''set angle'''
    req = dronecan.com.hobbywing.esc.SetAngle.Request()
    if len(arguments) < 2:
        print("Usage: SetAngle OPTION ANGLE")
        return
    req.option = int(arguments[0])
    req.angle = int(arguments[1])
    node.request(req, args.target_node_id, handle_msg)
    
def command_SelfTest(arguments):
    '''self test'''
    req = dronecan.com.hobbywing.esc.SelfTest.Request()
    node.request(req, args.target_node_id, handle_msg)
    
def command_SetDirection(arguments):
    '''set ESC direction'''
    req = dronecan.com.hobbywing.esc.SetDirection.Request()
    if len(arguments) < 1:
        print("Usage: SetDirection 0/1")
        return
    req.direction = int(arguments[0])
    node.request(req, args.target_node_id, handle_msg)

def command_RawCommand(arguments):
    '''send RawCommand'''
    req = dronecan.com.hobbywing.esc.RawCommand()
    if len(arguments) < 1:
        print("Usage: RawCommand THROTTLES..")
        return
    req.command = [ int(a) for a in arguments ]

    # callback for printing ESC status
    node.add_handler(dronecan.com.hobbywing.esc.StatusMsg1, lambda msg: print(dronecan.to_yaml(msg)))
    node.add_handler(dronecan.com.hobbywing.esc.StatusMsg2, lambda msg: print(dronecan.to_yaml(msg)))
    node.add_handler(dronecan.com.hobbywing.esc.StatusMsg3, lambda msg: print(dronecan.to_yaml(msg)))

    while True:
        node.broadcast(req)
        node.spin(0.02)
    
def command_SetReportingFrequency(arguments):
    '''set reporting frequency'''
    req = dronecan.com.hobbywing.esc.SetReportingFrequency.Request()
    if len(arguments) < 2:
        print("Usage: SetID MSG_ID RATE")
        return
    req.option = req.OPTION_WRITE
    req.MSG_ID = int(arguments[0])
    rmap = {
        500 : req.RATE_500HZ,
        250 : req.RATE_250HZ,
        200 : req.RATE_200HZ,
        100 : req.RATE_100HZ,
        50 : req.RATE_50HZ,
        20 : req.RATE_20HZ,
        10 : req.RATE_10HZ,
        1 : req.RATE_1HZ,
    }
    rate = int(arguments[1])
    if not rate in rmap:
        print("Invalid rate %d - must be one of %s" % (rate, ','.join(rmap.keys())))
        sys.exit(1)
    req.rate = rmap[rate]
    node.request(req, args.target_node_id, handle_msg)
    
commands = {
    "SetBaud" : command_SetBaud,
    "GetESCId" : command_GetESCId,
    "GetInfo" : command_GetInfo,
    "GetMaintenance" : command_GetMaintenance,
    "GetMajorConfig" : command_GetMajorConfig,
    "SetID" : command_SetID,
    "SetLED" : command_SetLED,
    "SetAngle" : command_SetAngle,
    "SelfTest" : command_SelfTest,
    "SetReportingFrequency" : command_SetReportingFrequency,
    "SetDirection" : command_SetDirection,
    "RawCommand" : command_RawCommand,
}

if __name__ == "__main__":
    freeze_support()
    parser = ArgumentParser(description='Hobbywing ESC control example')
    parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
    parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
    parser.add_argument("--target-node-id", default=1, type=int, help="CAN node ID")
    parser.add_argument("--signing-key", default=None, help="MAVLink2 signing key for mavcan")
    parser.add_argument("port", default=None, type=str, help="serial port")
    parser.add_argument("command", help="command")
    parser.add_argument("args", nargs='*', help="command arguments")
    args = parser.parse_args()

    
    if not args.command in commands:
        clist = ','.join(commands.keys())
        print(f"Invalid command '{args.command}' - must be one of {clist}")
        sys.exit(1)

        # Initializing a DroneCAN node instance.
    node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

    if args.signing_key is not None:
        node.can_driver.set_signing_passphrase(args.signing_key)

    # Initializing a node monitor, so we can see what nodes are online
    node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

        
    wait_online()

    commands[args.command](args.args)

    # run for 2 seconds
    startt = time.time()
    while time.time() - startt < 2:
        try:
            node.spin(0.1)
        except Exception as ex:
            pass
