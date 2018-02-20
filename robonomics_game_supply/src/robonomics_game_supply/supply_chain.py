# -*- coding: utf-8 -*-

import datetime
import rospy
from std_msgs.msg import String
from robonomics_game_common.marketdata import bids_params 
from robonomics_market.srv import AsksGenerator, BidsGenerator


markets = {
        'blue'  : 'QmbnPpVAA2c5Fsfuo4D4VhwY5YfNe2o5P6KZnYjw47ebQT', # metallurgy
        'green' : 'QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW', # medicine
        'purple': 'QmfM63vD3hpDHFSikoRFxucyEgKb7FERcXvLDKpGXnWWh8', # electronics
        'yellow': 'QmZky14ya2BbSNmjkrybxGdADTg7w7r4rKrVcotsz16HvP'  # automotive
        }


class DataSpreadsheet:
    uid = ''
    creds = ''


class SupplyChain:
    current_market = ""

    def __init__(self):
        rospy.init_node('supply_chain', anonymous=True, log_level=rospy.DEBUG)
        rospy.logdebug('Supply chain node starting...')

        self.plant_type = rospy.get_param('~plant_type')

        rospy.wait_for_service('/market/gen_asks')
        rospy.wait_for_service('/market/gen_bids')
        self.ask = rospy.ServiceProxy('/market/gen_asks', AsksGenerator)
        self.bid = rospy.ServiceProxy('/market/gen_bids', BidsGenerator)

        self.data_spreadsheet = DataSpreadsheet()
        self.data_spreadsheet.uid = rospy.get_param('~spreadsheet_id',
                        '16OYBRS4Nt1V_Fkte0oy0rm4pfBrEu37_QeOoqjjAFmI')
        self.data_spreadsheet.creds = rospy.get_param('~account_secret', 'owner_secret.json')

        def set_current(msg):
            old_market = self.current_market
            self.current_market = msg.data
            rospy.loginfo('Current market updated: ' + self.current_market)

            if len(old_market) == 0:
                rospy.loginfo('First market catched, making bids...')
                self.make_bids()

        rospy.Subscriber('/control/current', String, set_current)

        def run(msg):
            rospy.logdebug('SupplyChain.run, msg.data: ' + msg.data)
            self.prepare(msg.data)
            self.task(msg.data)
            self.finalize(msg.data)
            self.make_bids()
        rospy.Subscriber('/run', String, run)

        rospy.logdebug('Supply chain node started')

    def make_bids(self):
        rospy.loginfo('Making bids...')
        rangeName = 'Day %s!A1:W500' % self.get_current_period()
        params = bids_params(self.data_spreadsheet.creds,
                             self.data_spreadsheet.uid, rangeName)[self.plant_type]
        for c, p in params.items():
            if markets[c] is self.current_market:
                self.bid(p['a'],
                         p['k'],
                         markets[c],
                         p['fee'],
                         p['price_range'])

    def get_current_period(self):
        first_day = datetime.datetime.strptime(rospy.get_param('~first_day_game'),
                                               '%d-%m-%Y').date()
        today = datetime.date.today()
        return (today - first_day).days
