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
from robonomics_liability.msg import Liability
from robonomics_game_warehouse.srv import Order as WarehouseOrder, FillAll as WarehouseFillAll, EmptyAll as WarehouseEmptyAll
from robonomics_game_transport.msg import TransportAction, TransportFeedback, TransportResult
from robonomics_game_transport.msg import StackerAction, StackerGoal

class Supplier:
    busy = False
    jobs_queue = Queue()
    liability_address = ''

    def __init__(self, name, catalog, warehouse_init_state, stacker_node, liability_node):
        self._server = actionlib.ActionServer('supplier', TransportAction, self.plan_job, auto_start=False)
        self._server.start()

        self._jobs_proc_thread = threading.Thread(target=self._jobs_proc)
        self._jobs_proc_thread.daemon = True
        self._jobs_proc_thread.start()

        self.warehouse = dict() # warehouses order service proxy
        for content in catalog: # call: resp = warehouse.get(content)() -> {ok, q}
            self.warehouse.update({ content : rospy.ServiceProxy('/warehouse/raws/' + content + '/order', WarehouseOrder) })
            if warehouse_init_state == 'full':
                rospy.wait_for_service('/warehouse/raws/' + content + '/fill_all')
                fill_srv = rospy.ServiceProxy('/warehouse/raws/' + content + '/fill_all', WarehouseFillAll)
                fill_srv()
            elif warehouse_init_state == 'empty':
                rospy.wait_for_service('/warehouse/raws/' + content + '/empty_all')
                empty_srv = rospy.ServiceProxy('/warehouse/raws/' + content + '/empty_all', WarehouseEmptyAll)
                empty_srv()

        self.stacker = actionlib.ActionClient(stacker_node, StackerAction)
        self.stacker.wait_for_server()

        def update_liability(msg):
            self.liability_address = msg.address
        rospy.Subscriber(liability_node + '/current', Liability, update_liability)

        self.finish = rospy.Publisher(liability_node + '/finish', String, queue_size=10)

        self.current_gh = None

    def spin(self):
        rospy.spin()

    def plan_job(self, job):
        job.set_accepted()
        self.jobs_queue.put(job)

    def _jobs_proc(self):
        while not rospy.is_shutdown(): # wait condition
            if not self.jobs_queue.empty() and not self.busy:
                self.busy = True
                job = self.jobs_queue.get_nowait()
                self.start_job(job)

    def start_job(self, job):
        result = TransportResult()
        try:
            goal = job.get_goal()
            color = goal.specification
            item = self.warehouse.get(color)()
            if item.available:
                rospy.logwarn( 'Call stacker for job: ' + str([item.x, item.z, goal.address, 1]) )
                self.current_gh = self.stacker.send_goal( StackerGoal([item.x, item.z, goal.address, 1]) )
                while not rospy.is_shutdown(): # wait for stacker complete its job
                    if self.current_gh.get_goal_status() < 2: # actionlib_msgs/GoalStatus
                        rospy.sleep(1)
                    else:
                        break
                result.act = 'Job %s %s' % ( str(job.get_goal_id()), GoalStatus.to_string(self.current_gh.get_terminal_state()) )
                job.set_succeeded(result)
            else:
                result.act = 'Spec %s is not available' % color
                job.set_aborted(result)
                rospy.logdebug('Job aborted')
        finally:
            msg = String()
            msg.data = self.liability_address
            self.finish.publish(msg)
            self.busy = False

if __name__ == '__main__':
    rospy.init_node('supplier')
    node_name = rospy.get_name()
    catalog = rospy.get_param('~catalog', ['yellow', 'green', 'blue', 'purple'])
    warehouse_init_state = rospy.get_param('~warehouse_init_state', '') # leave state undefined if not given
    stacker_node = rospy.get_param('~stacker_node')
    liability_node = rospy.get_param('~liability_node')
    Supplier(node_name, catalog, warehouse_init_state, stacker_node, liability_node).spin()
