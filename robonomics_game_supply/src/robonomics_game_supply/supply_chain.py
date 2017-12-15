# -*- coding: utf-8 -*-

from robonomics_market.srv import AsksGenerator, BidsGenerator
from std_msgs.msg import String
import rospy

class SupplyChain:
    current_market = ""

    def __init__(self):
        rospy.init_node('supply_chain', anonymous=True, log_level=rospy.DEBUG)
        rospy.logdebug('Supply chain node starting...')

        rospy.wait_for_service('/market/gen_asks')
        rospy.wait_for_service('/market/gen_bids')
        self.ask = rospy.ServiceProxy('/market/gen_asks', AsksGenerator)
        self.bid = rospy.ServiceProxy('/market/gen_bids', BidsGenerator)

        def set_current(msg):
            self.current_market = msg.data
        rospy.Subscriber('/market/current', String, set_current)

        def run(msg):
            rospy.logdebug('SupplyChain.run, msg.data: ' + msg.data)
            self.prepare(msg.data)
            self.task(msg.data)
            self.finalize(msg.data)
            self.make_bids()
        rospy.Subscriber('/run', String, run)

        rospy.logdebug('Supply chain node started')

    def make_bids(self):
        rospy.logdebug('Making bids...')
        self.bid(0, 1, self.current_market, 1, 50)

