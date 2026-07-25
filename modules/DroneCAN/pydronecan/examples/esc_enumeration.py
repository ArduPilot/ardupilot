#!/usr/bin/env python3

import dronecan, time
from dronecan import uavcan

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='ESC enumeration example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--dna-server", action='store_true', default=False, help="run DNA server")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()

# Determining how many ESC nodes are present.
# In real use cases though the number of ESC should be obtained from elsewhere, e.g. from control mixer settings.
# There is a helper class in PyDroneCAN that allows one to automate what we're doing here,
# but we're not using it for the purposes of greater clarity of what's going on on the protocol level.
def detect_esc_nodes():
    esc_nodes = set()
    handle = node.add_handler(uavcan.equipment.esc.Status, lambda event: esc_nodes.add(event.transfer.source_node_id))
    try:
        node.spin(timeout=3)            # Collecting ESC status messages, thus determining which nodes are ESC
    finally:
        handle.remove()

    return esc_nodes


# Enumerating ESC.
# In this example we're using blocking code for simplicity reasons,
# but real applications will most likely resort either to asynchronous code (callback-based),
# or implement the logic in a dedicated thread.
# Conversion of the code from synchronous to asynchronous/multithreaded pertains to the domain of general
# programming issues, so these questions are not covered in this demo.
def enumerate_all_esc(esc_nodes, timeout=60):
    begin_responses_succeeded = 0

    def begin_response_checker(event):
        nonlocal begin_responses_succeeded
        if not event:
            raise Exception('Request timed out')

        if event.response.error != event.response.ERROR_OK:
            raise Exception('Enumeration rejected\n' + dronecan.to_yaml(event))

        begin_responses_succeeded += 1

    overall_deadline = time.monotonic() + timeout

    print('Starting enumeration on all nodes...')
    begin_request = uavcan.protocol.enumeration.Begin.Request(timeout_sec=timeout)
    for node_id in esc_nodes:
        print('Sending enumeration begin request to', node_id)
        node.request(begin_request, node_id, begin_response_checker)

    while begin_responses_succeeded < len(esc_nodes):
        node.spin(0.1)

    print('Listening for indications...')
    enumerated_nodes = []
    next_index = 0
    while set(enumerated_nodes) != esc_nodes:
        received_indication = None

        def indication_callback(event):
            nonlocal received_indication
            if event.transfer.source_node_id in enumerated_nodes:
                print('Indication callback from node %d ignored - already enumerated' % event.transfer.source_node_id)
            else:
                print(dronecan.to_yaml(event))
                received_indication = event

        indication_handler = node.add_handler(uavcan.protocol.enumeration.Indication, indication_callback)
        print('=== PROVIDE ENUMERATION FEEDBACK ON ESC INDEX %d NOW ===' % next_index)
        print('=== e.g. turn the motor, press the button, etc, depending on your equipment ===')
        try:
            while received_indication is None:
                node.spin(0.1)
                if time.monotonic() > overall_deadline:
                    raise Exception('Process timed out')
        finally:
            indication_handler.remove()

        target_node_id = received_indication.transfer.source_node_id
        print('Indication received from node', target_node_id)

        print('Stopping enumeration on node', target_node_id)
        begin_responses_succeeded = 0
        node.request(uavcan.protocol.enumeration.Begin.Request(), target_node_id, begin_response_checker)
        while begin_responses_succeeded < 1:
            node.spin(0.1)

        print('Setting config param %r to %r...' % (received_indication.message.parameter_name.decode(), next_index))
        configuration_finished = False

        def param_set_response(event):
            if not event:
                raise Exception('Request timed out')

            assert event.response.name == received_indication.message.parameter_name
            assert event.response.value.integer_value == next_index
            print(dronecan.to_yaml(event))
            node.request(uavcan.protocol.param.ExecuteOpcode.Request(
                             opcode=uavcan.protocol.param.ExecuteOpcode.Request().OPCODE_SAVE),
                         target_node_id,
                         param_opcode_response)

        def param_opcode_response(event):
            nonlocal configuration_finished
            if not event:
                raise Exception('Request timed out')

            print(dronecan.to_yaml(event))
            if not event.response.ok:
                raise Exception('Param opcode execution rejected\n' + dronecan.to_yaml(event))
            else:
                configuration_finished = True

        node.request(uavcan.protocol.param.GetSet.Request(value=uavcan.protocol.param.Value(integer_value=next_index),
                                                          name=received_indication.message.parameter_name),
                     target_node_id,
                     param_set_response)

        while not configuration_finished:
            node.spin(0.1)

        print('Node', target_node_id, 'assigned ESC index', next_index)
        next_index += 1
        enumerated_nodes.append(target_node_id)
        print('Enumerated so far:', enumerated_nodes)

    return enumerated_nodes


# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

if args.dna_server:
    # optionally start a DNA server
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

# Waiting for at least one other node to appear online
while len(node_monitor.get_all_node_id()) <= 1:
    print('Waiting for other nodes to become online...')
    node.spin(timeout=1)

print("There are %u nodes online" % len(node_monitor.get_all_node_id()))

print('Detecting ESC nodes...')
esc_nodes = detect_esc_nodes()
print('ESC nodes:', esc_nodes)

enumerated_esc = enumerate_all_esc(esc_nodes)
print('All ESC enumerated successfully; index order is as follows:', enumerated_esc)
