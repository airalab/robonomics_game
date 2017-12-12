#!/usr/bin/env python

from sys import version_info
if version_info[0] <= 2:
    from Queue import Queue
else:
    from queue import Queue
import threading
import re
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from robonomics_game_warehouse.srv import Place as WarehousePlace
from robonomics_game_transport.srv import ConveyorLoad, ConveyorDestination
from robonomics_game_transport.msg import TransportAction, TransportFeedback, TransportResult
from robonomics_game_transport.msg import StackerAction, StackerGoal

class Storage:
    busy = False
    jobs_queue = Queue()

    def __init__(self, name, catalog, stacker_node):
        self._server = actionlib.ActionServer('storage', TransportAction, self.plan_job, auto_start=False)
        self._server.start()

        self._jobs_proc_thread = threading.Thread(target=self._jobs_proc)
        self._jobs_proc_thread.daemon = True
        self._jobs_proc_thread.start()

        self.warehouse = dict() # warehouses place service proxy
        for content in catalog:
            self.warehouse.update({ content : rospy.ServiceProxy('/warehouse/goods/' + content + '/place', WarehousePlace) })
        self.conveyor_load = rospy.ServiceProxy('/transport_goods/conveyor/load', ConveyorLoad)
        self.conveyor_destination = rospy.ServiceProxy('/transport_goods/conveyor/destination', ConveyorDestination) # 'warehouse' or 'garbage'

        self.stacker = actionlib.ActionClient(stacker_node, StackerAction)
        self.stacker.wait_for_server()

        self.finish = rospy.Publisher('/finish', String, queue_size=10)

        self.current_gh = None

    def plan_job(self, job):
        job.set_accepted()
        self.jobs_queue.put(job)

    def _jobs_proc(self):
        while not rospy.is_shutdown(): # wait condition
            if not self.jobs_queue.empty() and not self.busy:
                self.busy = True
                job = self.jobs_queue.get_nowait()
                self.start_job(job)
            rospy.sleep(1)

    def start_job(self, job):
        result = TransportResult()
        try:
            goal = job.get_goal()
            color = goal.specification
            cell = self.warehouse.get(color)()
            if cell.available:
                self.conveyor_destination('warehouse')
                self.conveyor_load(goal.address) # plant must already wait for low level unload premission
                rospy.logdebug( 'Address: ' + str(goal.address) )
                rospy.sleep(20) # wait until chunk comes to the 5th conveyor, #TODO
                self.current_gh = self.stacker.send_goal( StackerGoal([12, 1,  cell.x, cell.z]) ) # [12, 1] is conveyor
                while not rospy.is_shutdown(): # wait for stacker complete its job
                    if self.current_gh.get_goal_status() < 2: # actionlib_msgs/GoalStatus
                        rospy.sleep(1)
                    else:
                        break
                result.act = 'Job %s %s' % ( str(job.get_goal_id()), GoalStatus.to_string(self.current_gh.get_terminal_state()) )
                job.set_succeeded(result)
            else:
                self.conveyor_destination('garbage')
                self.conveyor_load(goal.address)
                result.act = 'Place for %s is not available. Throwing to garbage.' % color
                rospy.sleep(10) # wait until chunk throwed down, #TODO
                job.set_succeeded(result)
        finally:
            self.finish.publish(result.act)
            self.busy = False

if __name__ == '__main__':
    rospy.init_node('supplier')
    node_name = rospy.get_name()
    stacker_node = rospy.get_param('~stacker_node')
    catalog = rospy.get_param('~catalog', ['yellow', 'green', 'blue', 'purple'])
    supplier = Storage(node_name, catalog, stacker_node)
