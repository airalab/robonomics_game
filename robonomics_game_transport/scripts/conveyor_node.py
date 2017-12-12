#!/usr/bin/env python

import threading
import rospy

from opcua_client import OpcuaClient
from robonomics_game_transport.srv import ConveyorDestination, ConveyorDestinationResponse
from robonomics_game_transport.srv import ConveyorLoad, ConveyorLoadResponse


class Conveyor:
    state = [-1, -1, -1, -1]
    def __init__(self, opcua_client_node, opcua_endpoint, opcua_server_namespace,
                 tr_load_time,  # ms, time to move after sensor find an object
                 tr_unload_time,  # ms, time to move after object gone from sensor
                 tr_timeout,  # ms, time to wait object until generate fault
                 tl_unload_time  # ms, unload for linear conveyour
                 ):
        self.opcua = OpcuaClient(opcua_client_node, opcua_endpoint)
        self.opcua_ns = opcua_server_namespace

        #self.opcua.write(self.opcua_ns + '/Settings/Tr_load_time', 'uint16', tr_load_time)
        #self.opcua.write(self.opcua_ns + '/Settings/Tr_unload_time', 'uint16', tr_unload_time)
        #self.opcua.write(self.opcua_ns + '/Settings/Tr_timeout', 'uint16', tr_timeout)
        #self.opcua.write(self.opcua_ns + '/Settings/Tl_unload_time', 'uint16', tl_unload_time)
        self.opcua.write(self.opcua_ns + '/Enable', 'bool', False)
        self.opcua.write(self.opcua_ns + '/Destination', 'bool', False) # to warehouse by default
        for i in range(1, 5): # disable conveyors
            self.opcua.write(self.opcua_ns + '/LoadTR' + str(i), 'bool', False)

        self._state_updater_thread = threading.Thread(target=self._state_updater)
        self._state_updater_thread.daemon = True
        self._state_updater_thread.start()

        rospy.Service('~destination', ConveyorDestination, self.set_destination) # set items destination
        rospy.Service('~load', ConveyorLoad, self.load)

        rospy.logwarn('Disable: ' + str(self.disable()))
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
            self.opcua.write(self.opcua_ns + '/Destination', 'bool', False) # False -> to warehouse
        elif request.destination.lower() == 'garbage':
            self.opcua.write(self.opcua_ns + '/Destination', 'bool', True) # True -> to garbage
        else:
            rospy.logwarn('Unknown destination request:' + request.destination)
        return ConveyorDestinationResponse()
    
    def load(self, request):
        self.opcua.write(self.opcua_ns + '/LoadTR' + str(request.consignor), 'bool', True)
        rospy.sleep(2) # time to let state change
        state_prev = [2, 2, 2, 2] # waiting for job
        while self.state in range(3, 8): # from start until unload to next
            if self.state != state_prev:
                rospy.loginfo('State: %s' % self.state)
            rospy.sleep(1)
        self.opcua.write(self.opcua_ns + '/LoadTR' + str(request.consignor), 'bool', False)
        return ConveyorLoadResponse()

    def _state_updater(self):
        while not rospy.is_shutdown():
            for i in range(0, 4): # from 1st to 4th rotary conveyor
                state = self.opcua.read_data(self.opcua_ns + '/StateTR' + str(i+1))
                if state in range(0, 9):
                    self.state[i] = state
                else:
                    rospy.logwarn('Unexpected state: ' + str(state) + ' of rotary conveyor ' + str(i+1))
                    self.state[i] = -1
                rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('conveyor')

    opcua_endpoint = rospy.get_param('~opcua_endpoint')
    if not rospy.has_param('~opcua_server_namespace'):
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" must be specified in accordance with OPCU-UA'
            'Model. Example: "ns=3;/Airalab/Conveyor"')
    opcua_server_namespace = rospy.get_param('~opcua_server_namespace')

    if not rospy.has_param('opcua_client_node'):
        rospy.logwarn('Using default ROS OPC-UA Client node path: /opcua/opcua')
        rospy.logwarn('You can specify it in parameter \'opcua_node\'')
    opcua_client_node = rospy.get_param('opcua_client_node', '/opcua/opcua_client')

    tr_load_time = rospy.get_param('tr_load_time', 2000)
    tr_unload_time = rospy.get_param('tr_unload_time', 2000)
    tr_timeout = rospy.get_param('tr_timeout', 5000)
    tl_unload_time = rospy.get_param('tl_unload_time', 2000)

    conveyor = Conveyor(opcua_client_node, opcua_endpoint, opcua_server_namespace, 
                        tr_load_time, tr_unload_time, tr_timeout, tl_unload_time)
    conveyor.enable()
    rospy.on_shutdown(conveyor.disable)
    rospy.spin()
