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
from std_srvs.srv import Empty
from robonomics_liability.msg import Liability
from robonomics_game_warehouse.srv import Place as WarehousePlace, FillAll as WarehouseFillAll, EmptyAll as WarehouseEmptyAll
from robonomics_game_transport.srv import ConveyorLoad, ConveyorDestination, ConveyorProductReady
from robonomics_game_transport.msg import TransportAction, TransportFeedback, TransportResult
from robonomics_game_transport.msg import StackerAction, StackerGoal

class Storage:
    busy = False
    jobs_queue = Queue()

    def __init__(self, name, catalog, warehouse_init_state, stacker_node, liability_node):
        self._server = actionlib.ActionServer('storage', TransportAction, self.plan_job, auto_start=False)
        self._server.start()
        rospy.logdebug('Storage.ActionServer ready')

        rospy.wait_for_service(liability_node + '/finish')
        self.finish = rospy.ServiceProxy(liability_node + '/finish', Empty)
        rospy.logdebug('Storage.finish ready')

        self.warehouse = dict() # warehouses place service proxy
        for content in catalog:
            rospy.logdebug('Storage.wh.%s starting' % content)
            rospy.wait_for_service('/warehouse/goods/' + content + '/place')
            self.warehouse.update({ content : rospy.ServiceProxy('/warehouse/goods/' + content + '/place', WarehousePlace) })
            rospy.logdebug('Storage.wh.%s place' % content)
            if warehouse_init_state == 'full':
                rospy.wait_for_service('/warehouse/goods/' + content + '/fill_all')
                rospy.logdebug('Storage.wh.%s.full' % content)
                fill_srv = rospy.ServiceProxy('/warehouse/goods/' + content + '/fill_all', WarehouseFillAll)
                fill_srv()
            elif warehouse_init_state == 'empty':
                rospy.wait_for_service('/warehouse/goods/' + content + '/empty_all')
                rospy.logdebug('Storage.wh.%s.empty' % content)
                empty_srv = rospy.ServiceProxy('/warehouse/goods/' + content + '/empty_all', WarehouseEmptyAll)
                empty_srv()
            rospy.logdebug('Storage.wh.%s ready' % content)
        rospy.logdebug('Storage.warehouse ready')

        rospy.wait_for_service('/transport_goods/conveyor/conveyor/load')
        self.conveyor_load = rospy.ServiceProxy('/transport_goods/conveyor/conveyor/load', ConveyorLoad)
        rospy.wait_for_service('/transport_goods/conveyor/conveyor/destination')
        self.conveyor_destination = rospy.ServiceProxy('/transport_goods/conveyor/conveyor/destination', ConveyorDestination) # 'warehouse' or 'garbage'
        rospy.wait_for_service('/transport_goods/conveyor/conveyor/product_ready')
        self.conveyor_product_ready= rospy.ServiceProxy('/transport_goods/conveyor/conveyor/product_ready', ConveyorProductReady) # 'warehouse' or 'garbage'
        rospy.logdebug('Goods conveyor proxy ready')

        self.stacker = actionlib.ActionClient(stacker_node, StackerAction)
        self.stacker.wait_for_server()
        rospy.logdebug('Stacker client ready')

        self.current_gh = None

        rospy.logdebug('Creating job_proc_thread')
        self._jobs_proc_thread = threading.Thread(target=self._jobs_proc)
        self._jobs_proc_thread.daemon = True
        self._jobs_proc_thread.start()

        rospy.logdebug('Storage node initialized')

    def spin(self):
        rospy.spin()

    def plan_job(self, job):
        rospy.logdebug( 'Storage.plan_job: ' + str(job) )
        job.set_accepted()
        self.jobs_queue.put(job)

    def _jobs_proc(self):
        while not rospy.is_shutdown(): # wait condition
            if not self.jobs_queue.empty() and not self.busy:
                self.busy = True
                job = self.jobs_queue.get_nowait()
                rospy.logdebug( 'Starting new job: ' + str(job) )
                self.start_job(job)
            rospy.sleep(1)

    def start_job(self, job):
        rospy.logdebug( 'Storage.start_job: ' + str(job) )
        result = TransportResult()
        try:
            goal = job.get_goal()
            color = goal.specification
            cell = self.warehouse.get(color)()
            if cell.available:
                self.conveyor_destination('warehouse')
                self.conveyor_load(goal.address) # plant must already wait for low level unload premission
                rospy.logdebug( 'Address: ' + str(goal.address) )
                while not rospy.is_shutdown():
                    product = self.conveyor_product_ready()
                    if product.ready:
                        break
                    rospy.sleep(1)
                self.current_gh = self.stacker.send_goal( StackerGoal([12, 1,  cell.x, cell.z]) ) # [12, 1] is conveyor
                while not rospy.is_shutdown(): # wait for stacker complete its job
                    goal_status = self.current_gh.get_goal_status()
                    if goal_status > 2: # actionlib_msgs/GoalStatus
                        break
                    rospy.logdebug('Stacker goal status: ' + str(goal_status))
                    rospy.sleep(1)
                result.act = 'Job %s %s' % ( str(job.get_goal_id()), GoalStatus.to_string(self.current_gh.get_terminal_state()) )
                job.set_succeeded(result)
            else:
                self.conveyor_destination('garbage')
                self.conveyor_load(goal.address)
                result.act = 'Place for %s is not available. Throwing to garbage.' % color
                rospy.sleep(10) # wait until chunk throwed down, #TODO
                job.set_succeeded(result)
        finally:
            self.finish()
            self.busy = False

if __name__ == '__main__':
    rospy.init_node('supplier')
    node_name = rospy.get_name()
    catalog = rospy.get_param('~catalog', ['yellow', 'green', 'blue', 'purple'])
    warehouse_init_state = rospy.get_param('~warehouse_init_state', '') # leave state undefined if not given
    stacker_node = rospy.get_param('~stacker_node')
    liability_node = rospy.get_param('~liability_node')
    Storage(node_name, catalog, warehouse_init_state, stacker_node, liability_node).spin()
