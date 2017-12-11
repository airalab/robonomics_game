# -*- coding: utf-8 -*-

from robonomics_market.srv import AsksGenerator, BidsGenerator
from std_msgs.msg import String
import rospy

class SupplyChain:
    current_market = ""

    def __init__(self):
        rospy.init_node('supply_chain', anonymous=True)

        rospy.wait_for_service('market/gen_ask')
        rospy.wait_for_service('market/gen_bid')
        self.ask = rospy.ServiceProxy('market/gen_ask', AsksGenerator)
        self.bid = rospy.ServiceProxy('market/gen_bid', BidsGenerator)

        rospy.Subscribe('/market/current', String, lambda msg: self.current_market = msg.data)

        def run(msg):
            self.prepare(msg.data)
            self.task(msg.data)
            self.finalize(msg.data)
            self.make_bids()
        rospy.Subscribe('run', String, run)

    def make_bids(self):
        self.bid(0, 1, self.current_market, 1, 50)

