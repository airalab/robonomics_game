# -*- coding: utf-8 -*-

import rospy
import ros_opcua_srvs.srv as ros_opcua

class OpcuaClient:
    def __init__(self, client_node, endpoint):

        self.client_node = client_node
        rospy.logdebug('Connecting to client: ' + client_node)
        rospy.logdebug('Endpoint: ' + endpoint)

        rospy.wait_for_service(client_node + '/connect')
        client_connect = rospy.ServiceProxy(client_node + '/connect', ros_opcua.Connect)

        request = ros_opcua.ConnectRequest()
        request.endpoint = endpoint
        try:
            response = ros_opcua.ConnectResponse()
            response.success = False
            response = client_connect(request)
            rospy.logdebug('OPC UA connection: ' + str(response))
            rospy.sleep(1)
        except rospy.ServiceException as e:
            rospy.logerr('OPC UA connection failure!')

        rospy.wait_for_service(client_node + '/read')
        self.client_read = rospy.ServiceProxy(client_node + '/read', ros_opcua.Read)

        rospy.wait_for_service(client_node + '/write')
        self.client_write = rospy.ServiceProxy(client_node + '/write', ros_opcua.Write)

    def write(self, nodeId, data_type, value):
        """
        Usage: write(ns + '/Settings/UnloadTime', 'uint16', unload_time)
        where ns='ns=<int>;s=/<VendorName>/<ObjectName>'
        """
        request = ros_opcua.WriteRequest()
        request.node.nodeId = nodeId
        request.data.type = data_type 
        setattr(request.data, '%s_d' % data_type, value)
        try:
            response = ros_opcua.WriteResponse()
            response.success = False
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
        try:
            response = ros_opcua.ReadResponse()
            response.success = False
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
