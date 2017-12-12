#!/usr/bin/env python

import rospy
import ros_opcua_srvs.srv as ros_opcua

class OpcuaClient:
    connected = False
    def __init__(self, client_node, endpoint):

        self.client_node = client_node
        rospy.logdebug('Connecting to client: ' + client_node)
        rospy.wait_for_service(client_node + '/connect')

        rospy.logdebug('Endpoint: ' + endpoint)
        client_connect = rospy.ServiceProxy(client_node + '/connect', ros_opcua.Connect)
        request = ros_opcua.ConnectRequest()
        request.endpoint = endpoint
        response = ros_opcua.ConnectResponse()
        while not self.connected: # wait for OPC UA Server connection
            response = client_connect(request)
            self.connected = response.success
            rospy.logdebug('OPC UA connection: ' + str(self.connected))
            rospy.sleep(1)

        self.client_read = rospy.ServiceProxy(
            client_node + '/read', ros_opcua.Read, persistent=True)
        self.client_write = rospy.ServiceProxy(
            client_node + '/write', ros_opcua.Write, persistent=True)

    def write(self, nodeId, data_type, value):
        """
        Usage: write(ns + '/Settings/UnloadTime', 'uint16', unload_time)
        where ns='ns=<int>;s=/<VendorName>/<ObjectName>'
        """
        request = ros_opcua.WriteRequest()
        request.node.nodeId = nodeId
        request.data.type = data_type 
        setattr(request.data, '%s_d' % data_type, value)
        response = ros_opcua.WriteResponse()
        try:
            response = self.client_write(request)
        except rospy.ServiceException as e:
            rospy.logerr('Check OPC-UA client, server connection or server model')
        finally:
            if not response.success:
                rospy.logwarn('No response to write request: %s' % request)
            return response

    def read(self, nodeId):
        """
        Usage: read(ns + '/Enable')
        where ns='ns=<int>;s=/<VendorName>/<ObjectName>'
        """
        request = ros_opcua.ReadRequest()
        request.node.nodeId = nodeId
        response = ros_opcua.ReadResponse()
        try:
            response = self.client_read(request)
        except rospy.ServiceException as e:
            rospy.logerr('Check OPC-UA client, server connection or server model')
        finally:
            if not response.success:
                rospy.logwarn('No response to read request: %s' % request)
            return response # take data: getattr(response.data, '%s_d'%response.data.type)

    def get_data(self, response):
        if response.success:
            return getattr(response.data, response.data.type + '_d')
        else:
            return None

    def read_data(self, nodeId):
        resp = self.read(nodeId)
        return self.get_data(resp)
