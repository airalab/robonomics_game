# -*- coding: utf-8 -*-
#
# Robonomics game supply chain.
#

from std_msgs.msg import String
import rospy, json
from .supply_chain import SupplyChain

class Supply(SupplyChain):
    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def prepare(self, objective):
        pass

    def task(self, objective):
        pass

    def finalize(self, objective);
        pass
