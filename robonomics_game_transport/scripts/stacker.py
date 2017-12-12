#!/usr/bin/env python

from sys import version_info
if version_info[0] <= 2:
    from Queue import Queue
else:
    from queue import Queue
import threading
import rospy
import actionlib

from opcua_client import OpcuaClient
from robonomics_game_transport.msg import StackerAction, StackerFeedback, StackerResult


class Stacker:
    state = -1
    enabled = False
    direction = ''
    jobs_queue = Queue()

    def __init__(self, name, opcua_client_node, opcua_endpoint, opcua_server_namespace,
                 timeout,  # ms
                 direction  # 'fw' for [src, dest], 'bw' for [dest, src]
                 ):
        self.name = name
        self.timeout = timeout
        self.set_direction(direction)

        # connect to opcua client
        self.opcua = OpcuaClient(opcua_client_node, opcua_endpoint)  # ros opcua client read/write interface
        self.opcua_ns = opcua_server_namespace

        # create stacker action server
        self._server = actionlib.ActionServer(name, StackerAction, self.plan_job, auto_start=False)
        self._server.start()

        # queue goals from action server and proc them in separate thread
        self._jobs_proc_thread = threading.Thread(target=self._jobs_proc)
        self._jobs_proc_thread.daemon = True
        self._jobs_proc_thread.start()

        # load parameters to opcua server
        self.opcua.write(self.opcua_ns + '/Reverse', 'bool', self._reverse)
        self.opcua.write(self.opcua_ns + '/Timeout', 'uint16', timeout)

        # update state in separate thread
        self._state_updater_thread = threading.Thread(target=self._state_updater)
        self._state_updater_thread.daemon = True
        self._state_updater_thread.start()

    def set_direction(self, direction):
        if direction in ['forward', 'fw']:
            self.direction = 'forward'
            self._reverse = False
        elif direction in ['backward', 'bw']:
            self.direction = 'backward'
            self._reverse = True
        else:
            raise ValueError('Direction must be "forward", "fw"  or "backward", "bw"')

    def plan_job(self, job):
        rospy.logdebug('New job recieved')
        if self.state == -1:
            rospy.logwarn(
                'Stacker state undefined. Check ROS to PLC connection. Queueing jobs until connection establish')
        elif self.state in range(0, 2):
            rospy.logwarn('Stacker disabled or not ready. Enable stacker on next movement')
        elif self.state == 2:  # enabled and waiting for job
            pass
        elif self.state in range(2, 11):
            rospy.loginfo('Stacker is busy. Queuing new job')
        elif self.state == 11:
            rospy.logwarn('Stacker FAULT. Queueing jobs until fault reset')
        else:
            raise ValueError('Stacker state value: %s deprecated' % str(self.state))
        job.set_accepted()
        self.jobs_queue.put(job)

    def _jobs_proc(self):
        while not rospy.is_shutdown():
            if not self.jobs_queue.empty() and self.state == 2:  # 2 means waiting for a job
                job = self.jobs_queue.get_nowait()
                self.start_job(job)
            rospy.logdebug('Current jobs queue: ' + str(self.jobs_queue.queue))
            rospy.sleep(1)

    def start_job(self, job):
        if not self.enabled or self.state == 0:  # 0 means disabled
            self.enable()
        rospy.logdebug('Starting new job: ' + str(job.get_goal().job))
        rospy.logdebug('GoalHandler: %s, Goal: %s, Job: %s' % ( str(dir(job)), str( dir(job.get_goal()) ), str( dir(job.get_goal().job) ) ) )
        source = job.get_goal().job[0:2]
        destination = job.get_goal().job[2:4]
        result = StackerResult()
        move = self.move(source, destination)
        if not move:
            result.act = 'Job %s %s aborted' % (str(job.get_goal_id()), str(job.get_goal()))
            job.set_aborted(result)
            return
        rospy.sleep(1)  # time to update state
        state_prev = 0
        rate = rospy.Rate(1)  # Hz
        while self.state is not 2:
            if self.state != state_prev:
                feedback = StackerFeedback()
                feedback.status = str(self.state)
                job.publish_feedback(feedback)
                state_prev = self.state
            if self.state not in range(2, 11):
                result.act = 'Job %s %s aborted' % (str(job.get_goal_id()), str(job.get_goal()))
                job.set_aborted(result)
                return
            rate.sleep()
        # reset for next rising edge start
        self.opcua.write(self.opcua_ns + '/Start', 'bool', False)
        result.act = 'Job %s %s done' % (str(job.get_goal_id()), str(job.get_goal()))
        job.set_succeeded(result)

    def move(self, source, destination):
        rospy.logdebug('Move from: ' + str(source) + ' to: ' + str(destination))
        success = True
        response = []
        response.append(self.opcua.write(self.opcua_ns + '/Point1X', 'uint16', source[0]))
        response.append(self.opcua.write(self.opcua_ns + '/Point1Z', 'uint16', source[1]))
        response.append(self.opcua.write(self.opcua_ns + '/Point2X', 'uint16', destination[0]))
        response.append(self.opcua.write(self.opcua_ns + '/Point2Z', 'uint16', destination[1]))
        response.append(self.opcua.write(self.opcua_ns + '/Start', 'bool', True))
        for r in response:
            success = success and r.success
        return success

    def enable(self):
        response = self.opcua.write(self.opcua_ns + '/Enable', 'bool', True)
        if response.success:
            rospy.loginfo('Stacker ' + self.name + ' enabled')
            self.enabled = True
        else:
            rospy.logwarn('Can not enable ' + self.name + ' stacker')
            self.enabled = False
        return response.success

    def _state_updater(self):
        while not rospy.is_shutdown():
            response = self.opcua.read(self.opcua_ns + '/State')
            rospy.logdebug('Got new state: ' + str(response.success))
            if response.success:
                self.state = getattr(response.data, response.data.type + '_d')
            else:
                rospy.logwarn('State update unsuccessful')
                self.state = -1
            if self.state not in range(0, 12):  # in accordance with PLC states that is from 0 to 11
                rospy.logwarn(
                    'Reading deprecated state value: %d, setting state to undefined (-1)' % self.state)
                self.state = -1
            rospy.sleep(1)
