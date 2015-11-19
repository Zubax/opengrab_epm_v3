#!/usr/bin/env python
#
# This is just a testing script for OpenGrab EPM based on Pyuavcan.
# Learn more: http://uavcan.org/Implementations/Pyuavcan
#

from __future__ import division, absolute_import, print_function, unicode_literals

import sys, threading, time, json

try:
    import uavcan
    import uavcan.node
except ImportError:
    print('Please install Pyuavcan:\n\tsudo pip install uavcan', file=sys.stderr)
    exit(1)

uavcan.load_dsdl()

node_status_registry = {}

class NodeStatusMonitor(uavcan.node.Monitor):
    def on_message(self, msg):
        node_status_registry[self.transfer.source_node_id] = repr(msg)

hardpoint_status_registry = {}

class HardpointStatusMonitor(uavcan.node.Monitor):
    def on_message(self, msg):
        hardpoint_status_registry[self.transfer.source_node_id] = repr(msg)

def redrawer():
    while True:
        time.sleep(0.2)
        print(end='\x1b[1J\x1b[H')
        print('ENTER COMMAND AND PRESS ENTER')
        print(json.dumps(node_status_registry, indent=4))
        print(json.dumps(hardpoint_status_registry, indent=4))

command = 0
hardpoint_id = 0

def publisher():
    while True:
        time.sleep(0.1)
        m = uavcan.equipment.hardpoint.Command()
        m.hardpoint_id = hardpoint_id
        m.command = command
        node.send_message(m)

def start_thread(target, *args, **kwargs):
    thd = threading.Thread(target=lambda: target(*args, **kwargs))
    thd.daemon = True
    thd.start()

if __name__ == '__main__':
    if len(sys.argv) <= 2:
        print('Usage: %s <can-iface> <hardpoint-id> [initial-command]' % sys.argv[0], file=sys.stderr)
        exit(1)

    node = uavcan.node.Node([(uavcan.equipment.hardpoint.Status, HardpointStatusMonitor),
                             (uavcan.protocol.NodeStatus, NodeStatusMonitor)])

    device = sys.argv[1]
    hardpoint_id = int(sys.argv[2])
    command = command if len(sys.argv) <= 3 else int(sys.argv[3])

    start_thread(node.listen, device)
    start_thread(redrawer)
    start_thread(publisher)

    while True:
        try:
            command = int(raw_input())
        except Exception:
            print('INVALID INPUT')
        else:
            print('COMMAND', command)
