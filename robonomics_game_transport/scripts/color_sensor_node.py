#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from opcua_client import OpcuaClient


def code_to_color(code):  # eastern rainbow order color code
    if code == 0:
        color = ''
    elif code == 3:
        color = 'yellow'
    elif code == 4:
        color = 'green'
    elif code == 6:
        color = 'blue'
    elif code == 7:
        color = 'purple'
    return color


if __name__ == '__main__':
    rospy.init_node('sensor')

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
    opcua = OpcuaClient(opcua_client_node)
    opcua_ns = opcua_server_namespace

    # advertise color
    pub = rospy.Publisher('color', String, queue_size=1)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        response = opcua.read(opcua_ns + '/Color')
        if response.success:
            color_code = getattr(response.data, response.data.type + '_d')
            color = code_to_color(color_code)
            pub.publish(color)
        else:
            rospy.logwarn('Color update unsuccessful')
            continue
        rate.sleep()
