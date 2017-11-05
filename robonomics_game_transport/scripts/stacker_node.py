#!/usr/bin/env python

import rospy
import actionlib

from stacker import Stacker

if __name__ == '__main__':
    rospy.init_node('stacker')
    node_name = rospy.get_name()

    if not rospy.has_param('opcua_server_namespace'):
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" must be specified in accordance with OPCU-UA'
            'Model. Example: /Airalab/Stacker_goods')
    opcua_server_namespace = rospy.get_param('opcua_server_namespace')
    if 'ns=' not in opcua_server_namespace:  # use only string type nodeId
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" template: "ns=<int>;s=/<VendorName>/<ObjectName>"')

    if not rospy.has_param('opcua_client_node'):
        rospy.logwarn('Using default ROS OPC-UA Client node path: /opcua/opcua_client')
        rospy.logwarn('You can specify it in parameter \'opcua_client_node\'')
    opcua_client_node = rospy.get_param('opcua_client_node', '/opcua/opcua_client')

    if not rospy.has_param('~direction'):
        raise rospy.ROSInitException(
            'Parameter "direction" must be specified "forward", "fw" or "backward", "bw"')
    direction = rospy.get_param('~direction')
    timeout = rospy.get_param('~timeout', 5000)

    stacker = Stacker(node_name, opcua_server_namespace, opcua_client_node, timeout, direction)
    stacker.enable()

    rospy.spin()
