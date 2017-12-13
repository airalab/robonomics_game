#!/usr/bin/env python

from sys import version_info
if version_info[0] <= 2:
    from Queue import Queue
else:
    from queue import Queue
import time
import threading
import rospy
import actionlib
import ros_opcua_srvs.srv as ros_opcua
from robonomics_game_plant.msg import OrderAction, OrderFeedback, OrderResult
from robonomics_game_plant.srv import Unload, UnloadResponse
from .opcua_client import OpcuaClient

class Plant:
    name = ''
    state = -1  # -1 means undefined
    orders_queue = Queue()
    _last_enable_time = 0  # ms

    def __init__(self, name, opcua_client_node, opcua_endpoint, opcua_server_namespace, 
                 unload_time,  # ms, time conveyor will move after detail gone from sensor
                 handle_time,  # ms, detail processing time
                 timeout  # ms, proximity sensors and communication response timeout
                 ):
        # connect to OPC-UA Server using ros opcua client
        rospy.logdebug('Connecting to OPC UA endpoint: ' + opcua_endpoint)

        # connect to opcua client
        self.opcua = OpcuaClient(opcua_client_node, opcua_endpoint)  # ros opcua client read/write interface
        self.opcua_ns = opcua_server_namespace

        # prepare plant action server
        self.name = name
        self._server = actionlib.ActionServer(name, OrderAction, self.plan_job, auto_start=False)
        self._server.start()

        # queue goals from action server and proc them in separate thread
        self._orders_proc_thread = threading.Thread(target=self._orders_proc)
        self._orders_proc_thread.daemon = True
        self._orders_proc_thread.start()

        # load plant parameters to opcua server
        self._unload_time = unload_time
        self._handle_time = handle_time
        self._timeout = timeout  # ms, proximity sensors and communication timeout
        
        try:
            self.opcua.write(self.opcua_ns + '/Settings/UnloadTime', 'uint16', unload_time)
            self.opcua.write(self.opcua_ns + '/Settings/HandleTime', 'uint16', handle_time)
            self.opcua.write(self.opcua_ns + '/Settings/Timeout', 'uint16', timeout)
        except rospy.ServiceException as e:
            rospy.logerr('Exception raised while OPC-UA request: %s' % e)
            rospy.logerr('Check OPC-UA client, server connection or server model')
            rospy.signal_shutdown(e)

        # update plant state in separate thread
        self._state_updater_thread = threading.Thread(target=self._state_updater)
        self._state_updater_thread.daemon = True
        self._state_updater_thread.start()

        # unload signal service
        rospy.Service('~unload', Unload, self.unload)

    def plan_job(self, order):
        rospy.logdebug('Got a new order')
        if self.state == -1:
            rospy.logwarn(
                'Plant state undefined. Check ROS node to PLC connection.'
                'Queueing orders until connection establish')
        elif self.state == 0:  # disabled
            pass
        elif self.state in range(1, 11):
            rospy.loginfo('Plant busy. Queuing new order')
        elif self.state == 11:
            rospy.logwarn('Plant FAULT. Queueing orders until fault reset')
        else:
            raise ValueError('Plant state value: %s deprecated' % str(self.state))
        order.set_accepted()
        self.orders_queue.put(order)

    def _orders_proc(self):
        while not rospy.is_shutdown():
            if not self.orders_queue.empty() and self.state == 0:  # order in plan and plant off
                self.start_job(self.orders_queue.get_nowait())
                rospy.logdebug('Orders queue: ' + str(self.orders_queue.queue))
            rospy.logdebug('Orders queue: ' + str(self.orders_queue.queue))
            rospy.sleep(1)

    def start_job(self, order):
        self.enable()  # enable plant to start the job, returns with (state != 0)
        state_prev = 0
        rospy.sleep(2)
        rospy.logdebug('Staring new job')
        while self.state is not 0:  # publish changing feedback while order in proc
            if self.state != state_prev:
                feedback = OrderFeedback()
                feedback.status = str(self.state)
                order.publish_feedback(feedback)
                state_prev = self.state
            if self.state == -1 or self.state == 11:  # abort order if something go wrong
                reslt = OrderResult()
                result.act = 'Order %s %s aborted' % (
                    str(order.get_goal_id()), str(order.get_goal()))
                order.set_aborted(result)
                return
            rospy.sleep(1)
        # Reset for a next rising edge
        self.opcua.write(self.opcua_ns + '/Enable', 'bool', False)
        result = OrderResult()
        result.act = 'Order %s %s complete' % (str(order.get_goal_id()), str(order.get_goal()))
        order.set_succeeded(result)

    def enable(self):
        """
        Minimum call period is (HandleTime + 2*UnloadTime)
        Each call period timeout will be ignored
        """
        now = int(round(time.time() * 1000))  # ms
        last = self._last_enable_time
        period = self._handle_time + 2 * self._unload_time
        if now > last + period:
            self._last_enable_time = now
            try:
                # Rising edge enables plant, check LOW state first
                response = self.opcua.read(self.opcua_ns + '/Enable')
                if not response.success:
                    rospy.logerr('Enable request not successful')
                    return
                if response.data.bool_d is True:
                    rospy.logerr('Plant enabled without node call. Reseting enable signal')
                    self.opcua.write(self.opcua_ns + '/Enable', 'bool', False)
                    rospy.sleep(1)
                response = self.opcua.write(self.opcua_ns + '/Enable', 'bool', True)
                # ensure PLC starts the order checking state is not undefined or off
                t = time.time() * 1000  # ms
                while self.state <= 0:  # undefined or disabled
                    if time.time() * 1000 - t > self._timeout:
                        rospy.logerr('Plant enable timeout.')
                        return
                    time.sleep(1)
            except rospy.ServiceException as e:
                rospy.logerr('Exception raised while OPC-UA request: %s' % e)
        else:
            rospy.logwarn('Order run attempt after %d sec from the last run ingnored.'
                          'Minium run period: %d sec.' % (now - last, period))
    def unload(self, request):
        rospy.loginfo('Unloading...')
        self.opcua.write(self.opcua_ns + '/Unload', 'bool', True)
        rospy.sleep(2)
        self.opcua.write(self.opcua_ns + '/Unload', 'bool', False)
        return UnloadResponse()

    def _state_updater(self):
        while not rospy.is_shutdown():
            response = self.opcua.read(self.opcua_ns + '/State')
            if not response.success:
                rospy.logwarn('State update unsuccessful')
                self.state = -1
                continue
            self.state = getattr(response.data, '%s_d' % response.data.type)
            rospy.logdebug('State: ' + str(self.state))
            if self.state not in range(0, 12):  # PLC state codes can be fromm 0 to 11
                rospy.logwarn(
                    'Deprecated state code: %d, set state to undefined (-1)' % self.state)
                self.state = -1
            time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('plant')
    node_name = rospy.get_name()

    opcua_endpoint= rospy.get_param('~opcua_endpoint')
    if not rospy.has_param('~opcua_server_namespace'):
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" must be specified in accordance with OPCU-UA'
            'Model. Example: /Airalab/Plant1')
    opcua_server_namespace = rospy.get_param('~opcua_server_namespace')
    if 'ns=' not in opcua_server_namespace: # use only string type nodeId
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" template: "ns=<int>;s=/<VendorName>/<PlantName>"')

    if not rospy.has_param('opcua_client_node'):
        rospy.logwarn('Using default ROS OPC-UA Client node path: /opcua/opcua_client')
        rospy.logwarn('You can specify it in parameter \'opcua_client_node\'')
    opcua_client_node = rospy.get_param('opcua_client_node', '/opcua/opcua_client')

    unload_time = rospy.get_param('~unload_time', 2000)
    handle_time = rospy.get_param('~handle_time', 2000)
    timeout = rospy.get_param('~timeout', 5000)

    plant = Plant(node_name, opcua_client_node, opcua_endpoint, opcua_server_namespace,
                    unload_time, handle_time, timeout)

    rospy.spin()
