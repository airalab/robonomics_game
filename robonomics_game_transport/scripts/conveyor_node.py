#!/usr/bin/env python

import rospy
from robonomics_game_transport.srv import Conveyor, ConveyorResponse

from opcua_client import OpcuaClient


class Conveyor:
    def __init__(self, opcua_server_namespace, opcua_node,
                 tr_load_time,  # ms, time to move after sensor find an object
                 tr_unload_time,  # ms, time to move after object gone from sensor
                 tr_timeout,  # ms, time to wait object until generate fault
                 tl_unload_time  # ms, unload for linear conveyour
                 ):
        self.opcua = OpcuaClient(opcua_node)
        self.opcua_ns = opcua_server_namespace

        self.opcua.write(self.opcua_ns + '/Settings/Tr_load_time', 'uint16', tr_load_time)
        self.opcua.write(self.opcua_ns + '/Settings/Tr_unload_time', 'uint16', tr_unload_time)
        self.opcua.write(self.opcua_ns + '/Settings/Tr_timeout', 'uint16', tr_timeout)
        self.opcua.write(self.opcua_ns + '/Settings/Tl_unload_time', 'uint16', tl_unload_time)
        self.opcua.write(self.opcua_ns + '/Destination', 'bool', True) # to warehouse by default
        rospy.Service('~destination', Conveyor, self.set_destination) # set items destination
        rospy.loginfo('Conveyor ready')

    def enable(self):
        response = self.opcua.write(self.opcua_ns + '/Enable', 'bool', True)
        if response.success:
            rospy.loginfo('Converyor enabled')
        else:
            rospy.logwarn('Can not enable conveyor')
        return response.success

    def disable(self):
        response = self.opcua.write(self.opcua_ns + '/Enable', 'bool', True)
        if response.success:
            rospy.loginfo('Converyor disabled')
        else:
            rospy.logwarn('Can not disable conveyor')
        return response.success

    def set_destination(self, request):
        if request.destination.lower() == 'warehouse':
            self.opcua.write(self.opcua_ns + '/Destination', 'bool', True) # True -> to warehouse
        elif request.destination.lower() == 'garbage':
            self.opcua.write(self.opcua_ns + '/Destination', 'bool', False) # False -> to garbage
        else:
            rospy.logwarn('Unknown destination request:' + request.destination)
        return ConveyorResponse()


if __name__ == '__main__':
    rospy.init_node('conveyor')

    if not rospy.has_param('opcua_server_namespace'):
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" must be specified in accordance with OPCU-UA'
            'Model. Example: /Airalab/Conveyor')
    opcua_server_namespace = rospy.get_param('opcua_server_namespace')

    if not rospy.has_param('opcua_client_node'):
        rospy.logwarn('Using default ROS OPC-UA Client node path: /opcua/opcua')
        rospy.logwarn('You can specify it in parameter \'opcua_node\'')
    opcua_client_node = rospy.get_param('opcua_client_node', '/opcua/opcua_client')

    tr_load_time = rospy.get_param('tr_load_time', 2000)
    tr_unload_time = rospy.get_param('tr_unload_time', 2000)
    tr_timeout = rospy.get_param('tr_timeout', 5000)
    tl_unload_time = rospy.get_param('tl_unload_time', 2000)

    conveyor = Conveyor(opcua_server_namespace, opcua_client_node,
                        tr_load_time, tr_unload_time, tr_timeout, tl_unload_time)
    conveyor.enable()
    rospy.on_shutdown(conveyor.disable)
    rospy.spin()
