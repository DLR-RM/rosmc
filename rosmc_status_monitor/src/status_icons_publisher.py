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

import copy
import math
import numpy as np

import rospy
import tf

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from std_msgs.msg import ColorRGBA, Header, String

from rosmc_interface_msgs.msg import AgentStatus, ActionStatus
from rosmc_msgs.msg import CurrentActions


class StatusIconsPublisher(object):
    """
    Class that subscribes AgentStatus and publishes status icons for all agents
    """

    def __init__(self, mission_server_namespace='/mission_control/'):
        # TODO: use a launch file with parameters for StatusIconPublisher specified in it
        rate = rospy.get_param('~rate', 20.0)
        icon_height = rospy.get_param('~icon_height', 3.0)
        frame_id_suffix = rospy.get_param('~frame_id_suffix', '')

        # Get the list of agents
        agent_library_dict = rospy.get_param('{}agent_library_dict/'.format(mission_server_namespace))

        # Icon publishers
        self.status_icon_publisher_dict = {}
        for agent_name in agent_library_dict:
            agent_frame_id = agent_name + frame_id_suffix
            pub = StatusIconPublisher(agent_name, rate, icon_height, mission_server_namespace=mission_server_namespace,
            agent_frame_id=agent_frame_id)
            self.status_icon_publisher_dict[agent_name] = pub

        rospy.loginfo("ready")


class StatusIconPublisher(object):
    """
    Class that subscribes AgentStatus and publishes status for each agent
    """

    def __init__(self, agent_name, rate, icon_height, icon_size=2.4, font_size=1.8, vertical_interval=0.25,
                 horizontal_icon_offset=0.2, horizontal_interval=0.85, mission_server_namespace='/mission_control/',
                 agent_frame_id=None):
        self.agent_name = agent_name
        self.rate = rate
        self.icon_height = icon_height
        self.icon_size = icon_size
        self.font_size = font_size
        self.vertical_interval = vertical_interval
        self.horizontal_icon_offset = horizontal_icon_offset
        self.horizontal_interval = horizontal_interval
        self.tf_icons = self.agent_name + "_icons"
        self.mission_server_namespace = mission_server_namespace
        if agent_frame_id is None:
            self.agent_frame_id = self.agent_name
        else:
            self.agent_frame_id = agent_frame_id

        # Members that are updated by subscriber
        self.camera_pose_stamped = None
        self.agent_status = None
        self.current_action = None

        # Publisher
        self.pub = rospy.Publisher(agent_name + '/icons', PictogramArray, queue_size=10)

        # Subscriber
        ## status should be published under the namespace of its robot
        self.sub_status = rospy.Subscriber('/' + agent_name + '/status', AgentStatus, callback=self.agent_status_callback)
        self.sub_camera_pose_stamped = rospy.Subscriber('rviz/camera_pose_stamped', PoseStamped,
                                                        callback=self.camera_pose_stamped_callback)
        self.sub_current_actions = rospy.Subscriber('{}current_actions'.format(self.mission_server_namespace), CurrentActions,
                                                    callback=self.current_actions_callback)

        # TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # TF Listener for tf of rviz view
        self.tf_listener = tf.TransformListener()

        # Timer
        rospy.Timer(rospy.Duration(1 / rate), self.timer_callback)

        # PictogramArray that is to be published as icons
        self.pictogram_array = PictogramArray()

        rospy.loginfo("StatusIconPublisher of agent {} is ready".format(self.agent_name))

    def timer_callback(self, _event):
        # If camera pose is not yet obtained, do nothing
        if self.camera_pose_stamped is None or self.agent_status is None:
            return

        ###################################################
        # Calculate position and orientation of the center of icons
        try:
            (agent_position, agent_quaternion) = self.tf_listener.lookupTransform(
                self.camera_pose_stamped.header.frame_id, self.agent_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf error when resolving tf between {} and {}".format(self.camera_pose_stamped.header.frame_id, self.agent_frame_id))
            return
        #self.tf_listener.waitForTransform(self.camera_pose_stamped.header.frame_id, self.agent_frame_id, rospy.Time(), rospy.Duration(4.0))
        #try:
        #    (agent_position, agent_quaternion) = self.tf_listener.lookupTransform(
        #        self.camera_pose_stamped.header.frame_id, self.agent_frame_id, rospy.Time.now())
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    rospy.logerr("tf error when resolving tf between {} and {}".format(self.camera_pose_stamped.header.frame_id, self.agent_name))
        #    return


        # Position of the center of icons
        icon_center_position = Point(agent_position[0], agent_position[1], agent_position[2])
        icon_center_position.z = self.icon_height

        # Orientation is the facing direction to camera
        icon_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi/2.)
        icon_quaternion = Quaternion(icon_quaternion[0], icon_quaternion[1], icon_quaternion[2], icon_quaternion[3])
        rot = copy.deepcopy(self.camera_pose_stamped.pose.orientation)

        icon_quaternion_list = [icon_quaternion.x, icon_quaternion.y, icon_quaternion.z, icon_quaternion.w]
        rot_list = [rot.x, rot.y, rot.z, rot.w]

        icon_quaternion_list = tf.transformations.quaternion_multiply(rot_list, icon_quaternion_list)
        icon_quaternion = Quaternion(icon_quaternion_list[0],
                                     icon_quaternion_list[1],
                                     icon_quaternion_list[2],
                                     icon_quaternion_list[3])

        ###################################################
        # Create icons
        # wlan -> fa-wifi, change only color
        # mission status ->
        #   IDLE: fa-check-circle
        #   RUNNING: fa-play-circle
        #   WARNING: fa-warning
        #   ERROR: fa-times-circle
        # battery -> progress-0, progress-1, progress-2, progress-3
        self.pictogram_array = PictogramArray()

        self.pictogram_array.header.frame_id = self.camera_pose_stamped.header.frame_id
        self.pictogram_array.header.stamp = rospy.Time.now()

        # WLAN icon
        icon_wlan = Pictogram()
        icon_wlan.header.frame_id = self.camera_pose_stamped.header.frame_id
        icon_wlan.header.stamp = rospy.Time.now()
        icon_wlan.action = Pictogram.ADD
        position_diff_list = [self.vertical_interval, self.horizontal_icon_offset, 0, 0]
        position_diff_list = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(icon_quaternion_list, position_diff_list),
            tf.transformations.quaternion_conjugate(icon_quaternion_list))
        icon_wlan.pose.position = copy.deepcopy(icon_center_position)
        icon_wlan.pose.position.x += position_diff_list[0]
        icon_wlan.pose.position.y += position_diff_list[1]
        icon_wlan.pose.position.z += position_diff_list[2]
        icon_wlan.pose.orientation = icon_quaternion
        icon_wlan.size = self.icon_size
        icon_wlan.ttl = 1.0 / self.rate
        if self.agent_status is None:
            icon_wlan.color.r = 1.0
            icon_wlan.color.g = 0.0
            icon_wlan.color.b = 0.0
            icon_wlan.color.a = 1.0
            icon_wlan.character = "fa-spinner"
        elif self.agent_status.wlan_signal < -68.:
            icon_wlan.color.r = 1.0
            icon_wlan.color.g = 0.0
            icon_wlan.color.b = 0.0
            icon_wlan.color.a = 1.0
            icon_wlan.character = "fa-wifi"
        elif self.agent_status.wlan_signal < -60.:
            icon_wlan.color.r = 1.0
            icon_wlan.color.g = 1.0
            icon_wlan.color.b = 0.0
            icon_wlan.color.a = 1.0
            icon_wlan.character = "fa-wifi"
        else:
            icon_wlan.color.r = 0.0
            icon_wlan.color.g = 1.0
            icon_wlan.color.b = 0.0
            icon_wlan.color.a = 1.0
            icon_wlan.character = "fa-wifi"
        self.pictogram_array.pictograms.append(icon_wlan)

        # WLAN text
        icon_wlan_text = Pictogram()
        icon_wlan_text.header.frame_id = self.camera_pose_stamped.header.frame_id
        icon_wlan_text.header.stamp = rospy.Time.now()
        icon_wlan_text.action = Pictogram.ADD
        position_diff_list = [self.vertical_interval, -self.horizontal_interval, 0, 0]
        position_diff_list = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(icon_quaternion_list, position_diff_list),
            tf.transformations.quaternion_conjugate(icon_quaternion_list))
        icon_wlan_text.pose.position = copy.deepcopy(icon_center_position)
        icon_wlan_text.pose.position.x += position_diff_list[0]
        icon_wlan_text.pose.position.y += position_diff_list[1]
        icon_wlan_text.pose.position.z += position_diff_list[2]
        icon_wlan_text.pose.orientation = icon_quaternion
        icon_wlan_text.size = self.font_size
        icon_wlan_text.ttl = 1.0 / self.rate
        icon_wlan_text.mode = Pictogram.STRING_MODE
        icon_wlan_text.color = icon_wlan.color
        if self.agent_status is None:
            icon_wlan_text.character = 'NO WIFI DATA'
        else:
            icon_wlan_text.character = '%.2f dBm' % self.agent_status.wlan_signal
        self.pictogram_array.pictograms.append(icon_wlan_text)

        # mission_status
        icon_mission_status = Pictogram()
        icon_mission_status.header.frame_id = self.camera_pose_stamped.header.frame_id
        icon_mission_status.header.stamp = rospy.Time.now()
        icon_mission_status.action = Pictogram.ADD
        position_diff_list = [0, self.horizontal_icon_offset, 0, 0]
        position_diff_list = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(icon_quaternion_list, position_diff_list),
            tf.transformations.quaternion_conjugate(icon_quaternion_list))
        icon_mission_status.pose.position = copy.deepcopy(icon_center_position)
        icon_mission_status.pose.position.x += position_diff_list[0]
        icon_mission_status.pose.position.y += position_diff_list[1]
        icon_mission_status.pose.position.z += position_diff_list[2]
        icon_mission_status.pose.orientation = icon_quaternion
        icon_mission_status.size = self.icon_size
        icon_mission_status.ttl = 1.0 / self.rate

        if self.current_action is None:
            icon_mission_status.color.r = 0.69
            icon_mission_status.color.g = 0.69
            icon_mission_status.color.b = 0.69
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-spinner"
        elif self.current_action.action_content.status.value == ActionStatus.RUNNING:
            icon_mission_status.color.r = 0.0
            icon_mission_status.color.g = 1.0
            icon_mission_status.color.b = 0.0
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-play"
        elif self.current_action.action_content.status.value == ActionStatus.SYNCWAITING:
            icon_mission_status.color.r = 193 / 255.
            icon_mission_status.color.g = 183 / 255.
            icon_mission_status.color.b = 34 / 255.
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-clock-o"
        elif self.current_action.action_content.status.value == ActionStatus.PAUSED:
            icon_mission_status.color.r = 0.69
            icon_mission_status.color.g = 0.69
            icon_mission_status.color.b = 0.69
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-pause"
        elif self.current_action.action_content.status.value == ActionStatus.FAILURE:
            icon_mission_status.color.r = 1.0
            icon_mission_status.color.g = 0.0
            icon_mission_status.color.b = 0.0
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-times-circle"
        elif self.current_action.action_content.status.value == ActionStatus.SUCCESS:
            icon_mission_status.color.r = 46 / 255.
            icon_mission_status.color.g = 234 / 255.
            icon_mission_status.color.b = 255 / 255.
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-check-circle"
        elif self.current_action.action_content.status.value == ActionStatus.IDLE_SYNCHRONISED:
            icon_mission_status.color.r = 0.9
            icon_mission_status.color.g = 0.9
            icon_mission_status.color.b = 0.9
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-download"
        else:
            icon_mission_status.color.r = 0.69
            icon_mission_status.color.g = 0.69
            icon_mission_status.color.b = 0.69
            icon_mission_status.color.a = 1.0
            icon_mission_status.character = "fa-spinner"

        self.pictogram_array.pictograms.append(icon_mission_status)

        # mission_status text
        icon_mission_status_text = Pictogram()
        icon_mission_status_text.header.frame_id = self.camera_pose_stamped.header.frame_id
        icon_mission_status_text.header.stamp = rospy.Time.now()
        icon_mission_status_text.action = Pictogram.ADD
        position_diff_list = [0, -self.horizontal_interval, 0, 0]
        position_diff_list = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(icon_quaternion_list, position_diff_list),
            tf.transformations.quaternion_conjugate(icon_quaternion_list))
        icon_mission_status_text.pose.position = copy.deepcopy(icon_center_position)
        icon_mission_status_text.pose.position.x += position_diff_list[0]
        icon_mission_status_text.pose.position.y += position_diff_list[1]
        icon_mission_status_text.pose.position.z += position_diff_list[2]
        icon_mission_status_text.pose.orientation = icon_quaternion
        icon_mission_status_text.size = self.font_size
        icon_mission_status_text.ttl = 1.0 / self.rate
        icon_mission_status_text.mode = Pictogram.STRING_MODE
        icon_mission_status_text.color = icon_mission_status.color

        icon_mission_status_text.color = icon_mission_status.color
        if self.current_action is None:
            icon_mission_status_text.character = "NO ACTION DATA"
        elif self.current_action.action_content.status.value != ActionStatus.IDLE:
            icon_mission_status_text.character = self.current_action.action_content.action_label  # use label instead of name
        else:
            icon_mission_status_text.character = "NO ACTION SYNCHRONISED YET"

        self.pictogram_array.pictograms.append(icon_mission_status_text)

        # battery
        icon_battery = Pictogram()
        icon_battery.header.frame_id = self.camera_pose_stamped.header.frame_id
        icon_battery.header.stamp = rospy.Time.now()
        icon_battery.action = Pictogram.ADD
        position_diff_list = [-self.vertical_interval, self.horizontal_icon_offset, 0, 0]
        position_diff_list = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(icon_quaternion_list, position_diff_list),
            tf.transformations.quaternion_conjugate(icon_quaternion_list))
        icon_battery.pose.position = copy.deepcopy(icon_center_position)
        icon_battery.pose.position.x += position_diff_list[0]
        icon_battery.pose.position.y += position_diff_list[1]
        icon_battery.pose.position.z += position_diff_list[2]
        icon_battery.pose.orientation = icon_quaternion
        icon_battery.size = self.icon_size
        icon_battery.ttl = 1.0 / self.rate
        if self.agent_status is None:
            icon_battery.color.r = 1.0
            icon_battery.color.g = 0.0
            icon_battery.color.b = 0.0
            icon_battery.color.a = 1.0
            icon_battery.character = "fa-spinner"
        elif self.agent_status.battery < 0.25:
            icon_battery.color.r = 1.0
            icon_battery.color.g = 0.0
            icon_battery.color.b = 0.0
            icon_battery.color.a = 1.0
            icon_battery.character = "progress-0"
        elif self.agent_status.battery < 0.5:
            icon_battery.color.r = 1.0
            icon_battery.color.g = 1.0
            icon_battery.color.b = 0.0
            icon_battery.color.a = 1.0
            icon_battery.character = "progress-1"
        elif self.agent_status.battery < 0.75:
            icon_battery.color.r = 0.0
            icon_battery.color.g = 1.0
            icon_battery.color.b = 0.0
            icon_battery.color.a = 1.0
            icon_battery.character = "progress-2"
        else:
            icon_battery.color.r = 0.0
            icon_battery.color.g = 1.0
            icon_battery.color.b = 0.0
            icon_battery.color.a = 1.0
            icon_battery.character = "progress-3"
        self.pictogram_array.pictograms.append(icon_battery)

        # battery text
        icon_battery_text = Pictogram()
        icon_battery_text.header.frame_id = self.camera_pose_stamped.header.frame_id
        icon_battery_text.header.stamp = rospy.Time.now()
        icon_battery_text.action = Pictogram.ADD
        position_diff_list = [-self.vertical_interval, -self.horizontal_interval, 0, 0]
        position_diff_list = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(icon_quaternion_list, position_diff_list),
            tf.transformations.quaternion_conjugate(icon_quaternion_list))
        icon_battery_text.pose.position = copy.deepcopy(icon_center_position)
        icon_battery_text.pose.position.x += position_diff_list[0]
        icon_battery_text.pose.position.y += position_diff_list[1]
        icon_battery_text.pose.position.z += position_diff_list[2]
        icon_battery_text.pose.orientation = icon_quaternion
        icon_battery_text.size = self.font_size
        icon_battery_text.ttl = 1.0 / self.rate
        icon_battery_text.mode = Pictogram.STRING_MODE
        icon_battery_text.color = icon_battery.color
        if self.agent_status is None:
            icon_battery_text.character = 'NO BATTERY DATA'
        else:
            icon_battery_text.character = '%.2f' % self.agent_status.battery
        self.pictogram_array.pictograms.append(icon_battery_text)

        # Publish icons
        self.pub.publish(self.pictogram_array)

    def camera_pose_stamped_callback(self, pose_stamped):
        self.camera_pose_stamped = pose_stamped

    def agent_status_callback(self, agent_status):
        self.agent_status = agent_status

    def current_actions_callback(self, current_actions):
        for i, action in enumerate(current_actions.current_actions):
            if self.agent_name == current_actions.agent_names[i]:
                self.current_action = action


# Main function.
if __name__ == '__main__':
    rospy.init_node('status_icons_publisher')
    try:
        StatusIconsPublisher()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
