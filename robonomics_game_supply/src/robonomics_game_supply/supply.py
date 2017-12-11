# -*- coding: utf-8 -*-
#
# Robonomics game supply chain.
#

from std_msgs.msg import String
import rospy, json
from .supply_chain import SupplyChain

supplier_market = ''
storage_market = ''

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
        rospy.init_node('supply_chain', anonymous=True)
        self.addr = 1

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        self.make_bids()
        rospy.spin()

    def prepare(self, objective):
        self.ask(10, 1, supplier_market, supplier[objective][self.addr], 1, 50) 

    def task(self, objective):
        pass

    def finalize(self, objective):
        self.ask(10, 1, storage_market, storage[objective][self.addr], 1, 50) 
