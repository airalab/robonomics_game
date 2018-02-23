# -*- coding: utf-8 -*-

from rospy import SubscribeListener


class SubscribersCounter(SubscribeListener):
    num_peers = 0
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self.num_peers += 1
    def peer_unsubscribe(self, topic_name, num_peers):
        self.num_peers = num_peers
