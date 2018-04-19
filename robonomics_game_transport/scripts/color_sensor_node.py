#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from robonomics_game_common.opcua_client import OpcuaClient


def code_to_color(code):  # eastern rainbow order color code
    color = ''
    if 1000 < code < 1600:
        color = 'yellow'
    elif 3500 < code < 3800:
        color = 'purple'
    elif 5200 < code < 5500:
        color = 'green'
    elif 5600 < code < 5800:
        color = 'blue'
    return color


if __name__ == '__main__':
    rospy.init_node('sensor')

    opcua_endpoint = rospy.get_param('~opcua_endpoint')
    if not rospy.has_param('~opcua_server_namespace'):
        raise rospy.ROSInitException(
            'Parameter "opcua_server_namespace" must be specified in accordance with OPCU-UA'
            'Model. Example: /Airalab/Conveyor')
    opcua_server_namespace = rospy.get_param('~opcua_server_namespace')

    if not rospy.has_param('opcua_client_node'):
        rospy.logwarn('Using default ROS OPC-UA Client node path: /opcua/opcua')
        rospy.logwarn('You can specify it in parameter \'opcua_node\'')
    opcua_client_node = rospy.get_param('opcua_client_node', '/opcua/opcua_client')

    # connect to opcua client
    opcua = OpcuaClient(opcua_client_node, opcua_endpoint)
    opcua_ns = opcua_server_namespace

    rospy.logdebug('Color senspr node started')

    # advertise color
    pub = rospy.Publisher('color', String, queue_size=2)
    while not rospy.is_shutdown():
        color_code = opcua.read_data(opcua_ns + '/DetectedColor')
        color = code_to_color(color_code)
        rospy.logdebug( 'New color: ' + str(color) )
        pub.publish(color)
        rospy.sleep(1)
