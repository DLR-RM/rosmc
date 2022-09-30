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

import functools
import yaml

import rospy
import interactive_markers.interactive_marker_server

import geometry_msgs.msg
import interactive_markers.interactive_marker_server
import interactive_markers.menu_handler
import tf
import visualization_msgs.msg
from visualization_msgs.msg import InteractiveMarker

from rosmc_msgs.srv import AddAction, UpdateMarkerStatus, \
    ExtendAddActionServices, ExtendAddActionServicesRequest, GetIntMarker, GetIntMarkerResponse
import rosmc_msgs.srv


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


class AgentIntMarkerPublisher(object):
    """
    Base class of interactive marker publisher for all types of agents.
    """

    def __init__(self, topic_name, agent_names, mission_server_namespace='/mission_control/'):
        self.topic_name = topic_name
        agent_names.sort()  # make it sure that agent_names are reordered alphabetically
        self.agent_names = agent_names
        self.mission_server_namespace = mission_server_namespace

        # Create interactive marker server
        self.server = interactive_markers.interactive_marker_server.InteractiveMarkerServer(self.topic_name)

        # Create service client for adding action
        self.add_action_client = rospy.ServiceProxy('{}add_action'.format(self.mission_server_namespace), AddAction)

        # Create service server
        self.update_marker_status_service = rospy.Service('{}/update_marker_status'.format(self.topic_name),
                                                          UpdateMarkerStatus, self.handle_update_marker_status)
        self.get_int_marker_service = rospy.Service('{}/get_int_marker'.format(self.topic_name),
                                                    GetIntMarker, self.handle_get_int_marker)

        # Create menu for each agent
        self.menu_handlers = {}
        self.menu_num_parameters_dict = self.setup_menu()

        # Members for interactive markers
        self.INT_MARKER = InteractiveMarker()
        self.ROLL = 0.0
        self.PITCH = 0.0
        self.YAW = 0.0

        # If one of the markers is associated with frame_id which is not available,
        # all the markers would not appear (which seems to be a bug of interactive marker in general)
        # Therefore, self.init_markers append the markers into self.int_markers_waiting_for_tf
        # by calling self.add_agent_marker.
        # and self.timer adds it self.server and apply changes via self.insert_marker_to_server
        self.transform_listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.mission_control_frame_id = rospy.get_param('{}/frame_id'.format(self.mission_server_namespace), 'map')
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)
        self.int_markers_waiting_for_tf = []

        # Initialize agent interactive markers
        self.init_markers()

        # Apply changes to interactive marker server
        self.server.applyChanges()

    def timer_callback(self, event):
        for i, int_marker in enumerate(self.int_markers_waiting_for_tf):
            frame_id = int_marker.header.frame_id
            if not self.transform_listener.canTransform(frame_id, self.mission_control_frame_id, rospy.Time(0)):
                rospy.logwarn('Transformation between {} and {} is not available; skip adding marker {} to the server'.format(self.mission_control_frame_id, frame_id, int_marker.name))
                continue
            self.insert_marker_to_server(int_marker)
            del self.int_markers_waiting_for_tf[i]
        if len(self.int_markers_waiting_for_tf) == 0:
            rospy.loginfo('All markers are now added to the server; shutting down timer and transform listener...')
            self.timer.shutdown()
            del self.transform_listener

    def handle_get_int_marker(self, req):
        res = GetIntMarkerResponse()
        try:
            int_marker = self.server.get(req.marker_name)
            res.int_marker = int_marker
        except:
            rospy.logerr("Requested marker name '%s' is not found in server." % req.marker_name)
        return res

    def handle_update_marker_status(self, req):
        """
        Implement this function for logic of updates of interactive marker status
        :param req:
        :return:
        """
        raise NotImplementedError

    def setup_menu(self):
        """
        Implement this function for setting up appropriate menu
        :return:
        """
        raise NotImplementedError

    def init_markers(self):
        """
        Implement this function to achieve automatic generation of interactive markers.
        NOTE: for adding interactive marker to server,
        use self.add_agent_marker(position, calib_pose, frame_id, agent_name, mesh_path, scale, color)
        """
        raise NotImplementedError

    def add_agent_marker(self, position, calib_pose, frame_id, agent_name, mesh_path, scale, color):
        """
        Function for adding interactive marker to self.int_markers_waiting_for_tf
        :param position:
        :param calib_pose:
        :param frame_id:
        :param agent_name:
        :param mesh_path:
        :param scale:
        :param color:
        :return:
        """
        int_marker = visualization_msgs.msg.InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.header.stamp = rospy.Time(0)
        int_marker.pose.position = position
        int_marker.scale = 1
        int_marker.name = agent_name
        int_marker.description = agent_name

        control_visual = visualization_msgs.msg.InteractiveMarkerControl()
        control_visual.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MENU
        control_visual.name = "visualization_with_menu"

        marker = visualization_msgs.msg.Marker()
        #marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time(0)
        marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
        marker.mesh_resource = mesh_path
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        marker.pose = calib_pose

        control_visual.markers.append(marker)
        control_visual.always_visible = True
        int_marker.controls.append(control_visual)

        self.int_markers_waiting_for_tf.append(int_marker)


    def insert_marker_to_server(self, int_marker):
        """
        Function to insert interactive marker to self.server
        """
        self.server.insert(int_marker, self.process_feedback)
        self.menu_handlers[int_marker.name].apply(self.server, int_marker.name)
        self.server.applyChanges()
        rospy.loginfo("Added agent marker named {}".format(int_marker.name))


    def process_feedback(self, feedback):
        """
        Basic feedback function called by any interaction from agent interactive marker.
        Override this if necessary.
        :param feedback:
        :return:
        """
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(s + ": button click" + mp + ".")
        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(s + ": pose changed")
        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()
