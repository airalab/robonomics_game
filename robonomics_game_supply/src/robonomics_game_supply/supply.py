# -*- coding: utf-8 -*-
#
# Robonomics game supply chain.
#

from sys import version_info
if version_info[0] <= 2:
    from Queue import Queue
else:
    from queue import Queue
import threading
from std_msgs.msg import String
from std_srvs.srv import Empty
import rospy, json
from .supply_chain import SupplyChain
from actionlib import ActionClient
from actionlib_msgs.msg import GoalStatus
from robonomics_market.msg import Ask
from robonomics_game_plant.msg import OrderAction as PlantAction, OrderGoal as PlantGoal 
from robonomics_game_plant.srv import Unload as PlantUnload

supplier_market = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dz4'
storage_market = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dz5'
#supplier_market = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dz6'
#storage_market = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dz7'

supplier = {
    'blue': [ 'Qmda9BiBqyJoqHWis1VV7UbLLnhp1xk9gMMMQV7qcP4poT'
            , 'QmXStJ4WUhjZsJFC2e4Gpzw8Yuiv2RKYSrsHdsR83hXiv4'
            , 'Qmc4Xivnzb8NNeLhMkHVoWyU3iiChsUijdkVPJSp6DoNdJ'
            , 'QmXoL8MPmcKcBPNqtSyb83My8eG266FyTd8sRoVHwskShr' ],
    'green': [ 'QmXBcs72dmEADuWxSvd5EN82g4oq2q95aYwP8oPFZZKnvQ'
             , 'QmdRtgPgApqobbUbK9uErSs1CgXwPW7x5AdPsPPPPRRJ91'
             , 'QmdV1cMf4iJgLkNUxozuk3mkQKRGz3wECovNfy2juqoB4V'
             , 'QmdEjE8givNRo1LxJbcLsxthDkws74Pfxjj15dg5q16qP8' ],
    'purple': [ 'Qmbye3KYdZMCTVXRW2Nm2Z8hSwVMhmkk57vRSEWUEiTscP'
              , 'QmfYTh2vQauXHK3xMjjp8BLuqKqZSfvh1GhkjfqXu4oCRz'
              , 'QmRG8iaQyHvhJQZHVSZ112uLYM3TnEgJXQ4ncxaLhg1cRK'
              , 'QmS1aSht4hqw7zCt6kdXSkyTuASeBB43GDVLUCGD9d8cXY' ],
    'yellow': [ 'QmQ6vxC6gEuaC7Nz1PFvbzg7yQ5vC3tamVCexnjixv5T2V'
              , 'QmSb6HJL68JzpgA9SkQMiHQAfEoxoKPyR5nheWJx7ewixW'
              , 'QmSM19NJFna8h3KX2Xi45wWU8hWL2bdbqELxCXm6ti6uPZ'
              , 'QmRnDNt44xZkjy9AETWCFefo1Sj36UQRKHszbzVbw8juWt' ]
    }

storage = {
    'blue': [ 'QmT3VCADCYtrwgW7PS2TmGhnQiZpHThueDcwcbB1dQpR6s'
            , 'QmQ4HHWTGHQgq2HGStFBfMKS1a7U6FUoxbvmGqXXdLwgL9'
            , 'QmTbih2r4zaE5CCaasmgSiidrVaYduNiSme5dHE4xBfEaN'
            , 'QmUzGy37dzFPyqDFVd66H2Znyh5pfattTusvzq6tXqZKTh' ],
    'green': [ 'QmYfrFmvwue1xAPYyyxSp8Ju9gsu5Zi3fq53uLWz9aV2F7'
             , 'QmejqVsLdf1df5LXGnzuatmVp1tQrKJUPrm9v5Pb9c3Dh7'
             , 'QmaAzvvaspkUUb1QRzxeR3SgKraFQjT3jegd4XUSu6N2ME'
             , 'QmeHdpUmVLf59vq6tJu7NsTtaYpkoDtrGyaQ1V5d9G5TaU' ],
    'purple': [ 'QmWX16FU8aG4sM56BFgPopxH7kQV8VkK6np8i8DXZsM2vs'
              , 'QmURBwZh8tCXLgsgtHBmzs7JNoxMLX4yDRcw9eZPq8q48d'
              , 'QmSnrPMc1gtETwU5kf8x87wQmbFHo8raj65k4CtNoqbGTg'
              , 'QmYGnuQXQ1X1wnsfT2S4FHe3RqYtCmvUqpu8U8HAY5iKEe' ],
    'yellow': [ 'QmZhW1cJ31vm7iDZJcGhW59oeBLkQMETTRv5txarBcKRUC'
              , 'QmVVbc4aooy4VVJRcyW7fvJUzUbTZE9g7qB2S1xSwjuJWi'
              , 'QmbwuMiAExbVkAMbAwaZfkug24uCXV9nmNGRg6K5iRfcEC'
              , 'Qmd8NdbUKB7o99t4ooHTpE1daPGqzTBXeC42odMB536Cdy' ]
    }


class Supply(SupplyChain):

    def __init__(self):
        SupplyChain.__init__(self)

        self.addr = rospy.get_param('~plant_addr')

        self.plant_node = rospy.get_param('~plant_node')
        self.plant = ActionClient(self.plant_node, PlantAction)
        self.plant.wait_for_server()
        rospy.wait_for_service(self.plant_node + '/unload')
        self.unload = rospy.ServiceProxy(self.plant_node + '/unload', PlantUnload)

        rospy.wait_for_service('/liability/finish')
        self.finish = rospy.ServiceProxy('/liability/finish', Empty)

        rospy.logdebug('Supply chain node ready')

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def prepare(self, objective):
        rospy.logdebug('Supply.prepare: ' + objective)
        # ask for raw material
        # self.ask(10, 1, supplier_market, supplier[objective][self.addr-1], 1, 50)
        msg = Ask()
        msg.model     = supplier_market
        msg.objective = supplier[objective][self.addr-1]
        msg.cost      = 42
        msg.count     = 1
        msg.fee       = 3
        self.signing_ask.publish(msg)

    def task(self, objective):
        rospy.logdebug('Supply.task: ' + objective)
        self.start_order(objective) # returns after order complete

    def start_order(self, order):
        rospy.logdebug( 'New order: ' + order + ', to plant: ' + self.plant_node)
        plant_gh = self.plant.send_goal( PlantGoal(specification=order) )
        while not rospy.is_shutdown(): # wait until plant complete its job
            if plant_gh.get_goal_status() < 2:
                rospy.logdebug( 'Goal status: ' + str(plant_gh.get_goal_status()) )
                rospy.sleep(1)
            else:
                break
        rospy.logdebug( 'Goal complete, status: ' + str(plant_gh.get_goal_status()) )
        plant_gh = None

    def finalize(self, objective):
        rospy.logdebug('Supply.finalize: ' + objective)
        # ask for goods storage
        # self.ask(10, 1, storage_market, storage[objective][self.addr-1], 1, 50)
        msg = Ask()
        msg.model     = storage_market
        msg.objective = storage[objective][self.addr-1]
        msg.cost      = 10
        msg.count     = 1
        msg.fee       = 1
        self.signing_ask.publish(msg)
        self.unload()
        rospy.sleep(5)
        rospy.logdebug('Order complete')
        self.finish()
        self.make_bids() # publish offer for next launch
