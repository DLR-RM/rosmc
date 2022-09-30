#!/usr/bin/env python
#
# The BSD 3-Clause License
#
# Copyright (c) 2022, DLR-RM All rights reserved.
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, 
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, 
#    this list of conditions and the following disclaimer in the documentation 
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors 
#    may be used to endorse or promote products derived from this software 
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Contributors:
# Ryo Sakagami <ryo.sakagami@dlr.de>

import math

import rospy
import tf

from rosmc_interface_msgs.msg import AgentStatus


class AgentsStatusPublisher(object):
    """
    Class that publishes all turltes' status information.
    The value of the status is dummy.
    """

    def __init__(self, mission_server_namespace='/mission_control/'):
        # Get the private namespace parameters from the parameter server:
        # set from either command line or launch file.
        rate = rospy.get_param('~rate', 10.0)

        # Get the list of agents
        agent_library_dict = rospy.get_param('{}agent_library_dict/'.format(mission_server_namespace))

        self.agent_publisher_dict = {}
        for agent_name in agent_library_dict:
            # TODO: modify the next line so that the real information is publihsed as AgentStatus
            pub = TurtleStatusPublisher(agent_name, rate)
            self.agent_publisher_dict[agent_name] = pub

        rospy.loginfo("ready")


class TurtleStatusPublisher(object):

    def __init__(self, agent_name, rate):
        self.agent_name = agent_name

        # Parameters for dummy data
        self.wifi_max = -50.
        self.wifi_min = -67.
        self.wifi_center = (5, 5)  # 2D pose where wifi is max
        self.wifi_r = 5  # radius where wifi is min
        self.battery_init = 1.0
        self.battery_consumption_basic = 0.0001
        self.battery_consumption_move = 0.0004
        self.battery = self.battery_init
        self.is_moving = False
        self.is_moving_threshold = 0.001
        self.x = None
        self.y = None

        # Publisher
        self.pub = rospy.Publisher(agent_name + '/status', AgentStatus, queue_size=10)

        # TF listener
        self.listener = tf.TransformListener()

        # Timer
        rospy.Timer(rospy.Duration(1 / rate), self.timer_cb)

    def update_battery(self):
        self.battery -= self.battery_consumption_basic
        if self.x is None or self.y is None:
            rospy.loginfo("No (x, y) is subscribed.")
            return
        if self.is_moving:
            rospy.loginfo("Agent {} is moving!".format(self.agent_name))
            self.battery -= self.battery_consumption_move

    def timer_cb(self, _event):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', self.agent_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # update members for battery calculation
        if self.x is not None and self.y is not None:
            self.is_moving = math.sqrt((trans[0] - self.x) ** 2 + (trans[1] - self.y) ** 2) > self.is_moving_threshold
        self.x = trans[0]
        self.y = trans[1]
        self.update_battery()

        time = rospy.Time.now()
        agent_status = AgentStatus()
        agent_status.agent_name = self.agent_name
        agent_status.pose_stamped.header.frame_id = self.agent_name
        agent_status.pose_stamped.header.stamp = time
        agent_status.pose_stamped.pose.position.x = trans[0]
        agent_status.pose_stamped.pose.position.y = trans[1]
        agent_status.pose_stamped.pose.position.z = trans[2]
        agent_status.pose_stamped.pose.orientation.x = rot[0]
        agent_status.pose_stamped.pose.orientation.y = rot[1]
        agent_status.pose_stamped.pose.orientation.z = rot[2]
        agent_status.pose_stamped.pose.orientation.w = rot[3]
        dist = math.sqrt((trans[0] - self.wifi_center[0]) ** 2 + (trans[1] - self.wifi_center[1]) ** 2)
        agent_status.wlan_signal = \
            self.wifi_min + abs(self.wifi_max - self.wifi_min) * (self.wifi_r - dist) / self.wifi_r
        agent_status.battery = self.battery
        self.pub.publish(agent_status)


# Main function.
if __name__ == '__main__':
    rospy.init_node('turtles_status_publisher')
    try:
        AgentsStatusPublisher()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
