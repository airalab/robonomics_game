# -*- coding: utf-8 -*-

import datetime
import json
import socket
import requests
import rospy
from std_msgs.msg import String
from robonomics_game_common.marketdata import bids_params 
from robonomics_game_common.marketdata import download_matched_bids
from robonomics_game_common.srv import Step, StepResponse
from robonomics_market.srv import AsksGenerator, BidsGenerator
from robonomics_market.msg import Ask, Bid


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
        self.signing_ask = rospy.Publisher('/market/signing/ask', Ask, queue_size=128)
        self.bid = rospy.ServiceProxy('/market/gen_bids', BidsGenerator)
        self.signing_bid = rospy.Publisher('/market/signing/bid', Bid, queue_size=128)

        self.data_spreadsheet = DataSpreadsheet()
        self.data_spreadsheet.uid = rospy.get_param('~spreadsheet_id',
                        '16OYBRS4Nt1V_Fkte0oy0rm4pfBrEu37_QeOoqjjAFmI')
        self.data_spreadsheet.creds = rospy.get_param('~account_secret', 'owner_secret.json')

        def step(request):
            rospy.loginfo('Factory %s running step %d...', self.plant_type, request.step)
            self.make_bids(request.step)
            return StepResponse()
        rospy.Service('~step', Step, step)

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
        rospy.Subscriber('/run', String, run)

    def make_bids(self, step=None):
        rospy.loginfo('Current market %s, making bids...', self.current_market)
        range_name = 'Day %s!A1:W500' % (step or self.get_current_launch_num())

        bids = download_matched_bids(self.data_spreadsheet.creds,
                                     self.data_spreadsheet.uid,
                                     range_name)
        for bid in bids:
            msg = Bid()
            msg.model = self.current_market
            if bid['Factory'] == self.plant_type and markets[bid['Color']] == self.current_market:
                msg.fee = bid['Fee']
                msg.cost = bid['Price']
                msg.count = int(bid['Quantity'])
                self.signing_bid.publish(msg)

    def get_current_launch_num(self):
        host = 'http://' + socket.gethostbyname('officetlt.corp.aira.life') + ':8088'
        return json.loads(requests.get(host + '/last_launch_num').content)['lastLaunchNumber']
