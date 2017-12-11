# -*- coding: utf-8 -*-

from robonomics_market.srv import AsksGenerator
from std_msgs.msg import String
import rospy

class SupplyChain:
    def __init__(self):
        rospy.init_node('supply_chain', anonymous=True)
        rospy.wait_for_service('market/gen_ask')
        self.ask = rospy.ServiceProxy('market/gen_ask', AsksGenerator)

        def run(msg):
            self.prepare(msg.data)
            self.task(msg.data)
            self.finalize(msg.data)
        rospy.Subscribe('run', String, run)
