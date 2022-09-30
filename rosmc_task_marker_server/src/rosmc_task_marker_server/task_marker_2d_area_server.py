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
import yaml
import numpy as np

import rospy

import geometry_msgs.msg
import visualization_msgs.msg
import tf

from rosmc_msgs.srv import AddTaskMarkerArea, AddTaskMarkerAreaRequest, AddTaskMarkerAreaResponse,\
    UpdateMarkerStatusResponse, AddActionRequest, GetIntMarkerVertices, GetIntMarkerVerticesResponse,\
    RenameMarkerResponse

from rosmc_task_marker_server.task_marker_server import *


class TaskMarker2DAreaServer(TaskMarkerServer):

    def __init__(self):
        super(TaskMarker2DAreaServer, self).__init__()
        self.marker_type = '2d_area'
        self.add_marker_service_class = AddTaskMarkerArea
        self.onInitialize()
        self.offset_controller = self.offset_ground + 0.1  # offset of controller from ground surface
        # Create another interactive marker server for vertices
        self.server_vertices = interactive_markers.interactive_marker_server.InteractiveMarkerServer(self.topic_name + '_vertices')
        self.feedback_publisher_to_center = rospy.Publisher(self.topic_name + '/feedback',
                                                            visualization_msgs.msg.InteractiveMarkerFeedback,
                                                            queue_size=10)
        # Create additional service for getting vertex marker from external node
        self.get_int_marker_vertex_service = rospy.Service('{}/get_int_marker'.format(self.topic_name + '_vertices'),
                                                           GetIntMarker, self.handle_get_int_marker_vertex)
        # Create another service for getting all vertices marker from external node
        self.get_int_marker_vertices_service = rospy.Service('{}/get_int_marker_list'.format(self.topic_name + '_vertices'),
                                                             GetIntMarkerVertices, self.handle_get_int_marker_vertices)
        # parameter for area
        self.minimum_area_size = 1.0
        rospy.loginfo("Ready")

    def handle_get_int_marker_vertex(self, req):
        try:
            int_marker = self.server_vertices.get(req.marker_name)
        except:
            rospy.logerr("Requested marker name '%s' is not found in server." % req.marker_name)
        res = GetIntMarkerResponse()
        res.int_marker = int_marker
        return res

    def handle_get_int_marker_vertices(self, req):
        int_marker_name_center = req.marker_name_center
        res = GetIntMarkerVerticesResponse()
        vertex_num = 0
        is_vertex_exist = True
        while is_vertex_exist:
            int_marker_name_vertex = int_marker_name_center + " (vertex " + str(vertex_num) + ")"
            int_marker_vertex = self.server_vertices.get(int_marker_name_vertex)
            if int_marker_vertex is None:
                is_vertex_exist = False
            else:
                res.int_marker_vertices.append(int_marker_vertex)
                vertex_num += 1
        return res

    def add_task_marker(self, poses, status_value, name=None, description="\n(No agent is assigned)"):
        # TODO: make it possible to create arbitrary polygon
        position = geometry_msgs.msg.Point()
        position.x = (poses[0].position.x + poses[2].position.x) / 2.0
        position.y = (poses[0].position.y + poses[2].position.y) / 2.0
        x_scale = abs(poses[0].position.x - poses[2].position.x)
        y_scale = abs(poses[0].position.y - poses[2].position.y)

        if name is None:
            while self.server.get("2D Area " + str(self.marker_counter)) is not None:
                self.marker_counter += 1
            name = "2D Area " + str(self.marker_counter)
            # Increment marker_counter if name is not given
            self.marker_counter += 1

        #####################################################
        # Center Marker
        #####################################################
        int_marker_center = self.task_marker_center(position, x_scale, y_scale, status_value, name, description)
        self.server.insert(int_marker_center, self.process_feedback_center)
        if status_to_is_deletable(status_value):
            self.menu_handler_with_delete.apply(self.server, int_marker_center.name)
        elif status_to_is_movable(status_value):
            self.menu_handler_with_rename.apply(self.server, int_marker_center.name)
        else:
            self.menu_handler.apply(self.server, int_marker_center.name)

        #####################################################
        # Vertices
        #####################################################
        for i in range(4):
            int_marker_vertex = self.task_marker_vertex(position, x_scale, y_scale, status_value, name, i)
            self.server_vertices.insert(int_marker_vertex, self.process_feedback_vertex)
            # NOTE: do not apply menu to vertex markers

        #####################################################
        self.server.applyChanges()
        self.server_vertices.applyChanges()
        return name

    def task_marker_center(self, position, x_scale, y_scale, status_value, name, description):
        # Initialization
        int_marker_center = visualization_msgs.msg.InteractiveMarker()
        int_marker_center.header.frame_id = self.frame_id
        int_marker_center.pose.position.x = position.x
        int_marker_center.pose.position.y = position.y
        int_marker_center.pose.position.z = self.offset_controller  # offset
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        int_marker_center.pose.orientation.x = quaternion[0]
        int_marker_center.pose.orientation.y = quaternion[1]
        int_marker_center.pose.orientation.z = quaternion[2]
        int_marker_center.pose.orientation.w = quaternion[3]
        int_marker_center.scale = 3 * self.marker_scale
        int_marker_center.name = name
        int_marker_center.description = name + description

        # Context menu with visualization
        # NOTE: All markers to be visualized must attached to this control with name "visualization_with_menu"
        control_visual = visualization_msgs.msg.InteractiveMarkerControl()
        control_visual.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MENU
        control_visual.name = "visualization_with_menu"

        # Cube
        cube_pose = geometry_msgs.msg.Pose()
        cube_pose.position.x = 0.0
        cube_pose.position.y = 0.0
        cube_pose.position.z = -self.offset_controller + self.offset_ground + 0.05
        cube_pose.orientation.w = 1.0
        cube_pose.orientation.x = 0.0
        cube_pose.orientation.y = 0.0
        cube_pose.orientation.z = 0.0
        cube_marker = visualization_msgs.msg.Marker()
        cube_marker.type = visualization_msgs.msg.Marker.CUBE
        cube_marker.scale.x = x_scale
        cube_marker.scale.y = y_scale
        cube_marker.scale.z = 0.1 * self.marker_scale
        cube_marker.color = status_to_color(status_value)
        cube_marker.pose = cube_pose
        control_visual.markers.append(cube_marker)

        control_visual.always_visible = True
        int_marker_center.controls.append(control_visual)

        # Depending on status_value, add control method for movements in xy plane
        if status_to_is_movable(status_value):
            control_pose = visualization_msgs.msg.InteractiveMarkerControl()
            control_pose.orientation.w = 1.0
            control_pose.orientation.x = 0.0
            control_pose.orientation.y = 1.0
            control_pose.orientation.z = 0.0
            control_pose.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MOVE_PLANE
            int_marker_center.controls.append(control_pose)

        return int_marker_center

    def task_marker_vertex(self, position, x_scale, y_scale, status_value, name, vertex_num):
        # Initialization
        int_marker_vertex = visualization_msgs.msg.InteractiveMarker()
        int_marker_vertex.header.frame_id = self.frame_id
        if vertex_num == 0:  # (pos_x, pos_y)
            int_marker_vertex.pose.position.x = position.x + x_scale / 2.
            int_marker_vertex.pose.position.y = position.y + y_scale / 2.
        elif vertex_num == 1:  # (neg_x, pos_y)
            int_marker_vertex.pose.position.x = position.x - x_scale / 2.
            int_marker_vertex.pose.position.y = position.y + y_scale / 2.
        elif vertex_num == 2:  # (neg_x, neg_y)
            int_marker_vertex.pose.position.x = position.x - x_scale / 2.
            int_marker_vertex.pose.position.y = position.y - y_scale / 2.
        elif vertex_num == 3:  # (pos_x, neg_y)
            int_marker_vertex.pose.position.x = position.x + x_scale / 2.
            int_marker_vertex.pose.position.y = position.y - y_scale / 2.
        int_marker_vertex.pose.position.z = self.offset_controller  # offset
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        int_marker_vertex.pose.orientation.x = quaternion[0]
        int_marker_vertex.pose.orientation.y = quaternion[1]
        int_marker_vertex.pose.orientation.z = quaternion[2]
        int_marker_vertex.pose.orientation.w = quaternion[3]
        int_marker_vertex.scale = 1.5 * self.marker_scale
        int_marker_vertex.name = name + " (vertex " + str(vertex_num) + ")"
        int_marker_vertex.description = int_marker_vertex.name

        # Context menu with visualization
        # NOTE: All markers to be visualized must attached to this control with name "visualization_with_menu"
        control_visual = visualization_msgs.msg.InteractiveMarkerControl()
        control_visual.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MENU
        control_visual.name = "visualization_with_menu"

        # Cylinder
        cylinder_pose = geometry_msgs.msg.Pose()
        cylinder_pose.position.x = 0.0
        cylinder_pose.position.y = 0.0
        cylinder_pose.position.z = -self.offset_controller + self.offset_ground + 0.15
        cylinder_pose.orientation.w = 1.0
        cylinder_pose.orientation.x = 0.0
        cylinder_pose.orientation.y = 0.0
        cylinder_pose.orientation.z = 0.0
        cylinder_marker = visualization_msgs.msg.Marker()
        cylinder_marker.type = visualization_msgs.msg.Marker.CYLINDER
        cylinder_marker.scale.x = 0.1 * self.marker_scale
        cylinder_marker.scale.y = 0.1 * self.marker_scale
        cylinder_marker.scale.z = 0.3 * self.marker_scale
        cylinder_marker.color = status_to_color(status_value)
        cylinder_marker.pose = cylinder_pose
        control_visual.markers.append(cylinder_marker)

        control_visual.always_visible = True
        int_marker_vertex.controls.append(control_visual)

        # Depending on status_value, add control method for movements in xy plane
        if status_to_is_movable(status_value):
            control_pose = visualization_msgs.msg.InteractiveMarkerControl()
            control_pose.orientation.w = 1.0
            control_pose.orientation.x = 0.0
            control_pose.orientation.y = 1.0
            control_pose.orientation.z = 0.0
            control_pose.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MOVE_PLANE
            int_marker_vertex.controls.append(control_pose)

        return int_marker_vertex

    def handle_add_task_marker(self, req):
        name = self.add_task_marker(req.poses, req.status.value, name=None, description="")
        res = AddTaskMarkerAreaResponse()
        res.name = name
        return res

    def handle_update_marker_status(self, req):
        # Retrieve int_marker information
        poses = []
        for i in range(4):
            int_marker_vertex = self.server_vertices.get(req.marker_name + " (vertex " + str(i) + ")")
            if int_marker_vertex is None:
                rospy.logwarn("Requested marker is not found. Skip status updates.")
                return UpdateMarkerStatusResponse()
            poses.append(int_marker_vertex.pose)

        # Delete int_markers
        self.server.erase(req.marker_name)
        for i in range(4):
            self.server_vertices.erase(req.marker_name + " (vertex " + str(i) + ")")

        # Add int_markers with retrieved information
        self.add_task_marker(poses, req.updated_status.value, name=req.marker_name, description="")
        rospy.loginfo("Updated status of marker %s to %d" % (req.marker_name, req.updated_status.value))
        return UpdateMarkerStatusResponse()

    def handle_update_int_marker(self, req):
        try:
            self.server.erase(req.marker_name)
        except:
            rospy.logerr("Requested marker name '%s' is not found in server" % req.marker_name)
            return UpdateIntMarkerResponse()

        # Retrieve status
        control_name_list = [control.name for control in req.int_marker.controls]
        control_visual = req.int_marker.controls[control_name_list.index('visualization_with_menu')]
        status = color_to_status(control_visual.markers[0].color)

        if req.marker_name.endswith(' (vertex 0)') or req.marker_name.endswith(' (vertex 1)') \
                or req.marker_name.endswith(' (vertex 2)') or req.marker_name.endswith(' (vertex 3)'):
            self.server_vertices.insert(req.int_marker, self.process_feedback_vertex)
        else:
            self.server.insert(req.int_marker, self.process_feedback_center)
            if status_to_is_deletable(status):
                self.menu_handler_with_delete.apply(self.server, req.int_marker.name)
            elif status_to_is_movable(status):
                self.menu_handler_with_rename.apply(self.server, req.int_marker.name)
            else:
                self.menu_handler.apply(self.server, req.int_marker.name)
        self.server.applyChanges()
        self.server_vertices.applyChanges()
        return UpdateIntMarkerResponse()

    def rename_marker(self, name_old, name_new):
        # Retrieve int_marker information
        poses = []
        for i in range(4):
            int_marker_vertex = self.server_vertices.get(name_old + " (vertex " + str(i) + ")")
            if int_marker_vertex is None:
                error_msg = "Requested marker is not found. Skip status updates."
                rospy.logwarn(error_msg)
                return error_msg
            poses.append(int_marker_vertex.pose)

        # Get int_marker with old name
        int_marker = self.server.get(name_old)
        if int_marker is None:
            error_msg = "Requested marker is not found. Skip status updates."
            rospy.logwarn(error_msg)
            return error_msg

        # Delete int_markers
        self.server.erase(name_old)
        for i in range(4):
            self.server_vertices.erase(name_old + " (vertex " + str(i) + ")")

        # Retrieve status
        control_name_list = [control.name for control in int_marker.controls]
        control_visual = int_marker.controls[control_name_list.index('visualization_with_menu')]
        status = color_to_status(control_visual.markers[0].color)

        # Add marker with new name
        self.add_task_marker(poses, status, name=name_new, description="")
        rospy.loginfo("Renamed marker from %s to %s" % (name_old, name_new))
        return ""

    def add_action_callback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
            library, action_name, agent_name = self.menu_num_parameters_dict[feedback.menu_entry_id]

            add_action_req = AddActionRequest()
            add_action_req.agent_name = agent_name

            # Fill in the initial contents
            add_action_req.action_content.action_name = action_name
            add_action_req.action_content.action_label = action_name
            add_action_req.action_content.battery_flag = True
            add_action_req.action_content.wlan_flag = True

            # parameter specification
            # First, fill in default parameters
            parameters_dict = rospy.get_param('{}agent_action_dict/{}/{}'.format(self.mission_server_namespace, agent_name, action_name)).copy()
            for key in parameters_dict:
                parameters_dict[key]['value'] = parameters_dict[key]['default_value']
                del parameters_dict[key]['default_value']

            # Second, fill in interactive marker value
            self.INT_MARKER = self.server.get(feedback.marker_name)
            self.INT_MARKER.pose.position.z = 0.0  # override
            euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                              self.INT_MARKER.pose.orientation.y,
                                                              self.INT_MARKER.pose.orientation.z,
                                                              self.INT_MARKER.pose.orientation.w,))
            self.ROLL = euler[0]
            self.PITCH = euler[1]
            self.YAW = euler[2]

            marker_for_parametrization_list = self.topic_lib_action_int_marker_dict[self.topic_name][library][
                action_name]
            is_valid = False
            for marker_for_parametrization in marker_for_parametrization_list:
                # Check if the parametrization uses this marker topic or not
                if self.topic_name not in marker_for_parametrization["topics"]:
                    # If this marker topic is not used, fill in default value in parameters_dict[key]['marker']
                    parameters_dict[param_name]['marker'] = ""
                    parameters_dict[param_name]['marker_topic'] = marker_for_parametrization["topics"][0]
                    continue
                # Currently, this marker is supported only for break_down mode
                int_marker_type = marker_for_parametrization['type']
                if int_marker_type == 'TASK_LIST_POSEDICT':
                    param_name = marker_for_parametrization['name']
                    # Calculate new_value
                    new_value = []
                    for i in range(4):
                        int_marker_vertex = self.server_vertices.get(feedback.marker_name + ' (vertex {})'.format(i))
                        pose_dict = {
                            'x': float(int_marker_vertex.pose.position.x),
                            'y': float(int_marker_vertex.pose.position.y),
                            'z': 0.0,  # override
                            'roll': 0.0,
                            'pitch': 0.0,
                            'yaw': 0.0
                        }
                        new_value.append(pose_dict)
                    parameters_dict[param_name]['value'] = new_value
                    parameters_dict[param_name]['marker'] = feedback.marker_name
                    parameters_dict[param_name]['marker_topic'] = self.topic_name
                    is_valid = True
            if not is_valid:
                rospy.logerr('ERROR: 2d_area marker is currently supported only for type TASK_LIST_POSEDICT')
                return

            add_action_req.action_content.parameters_yaml = yaml.dump(parameters_dict)
            resp = self.add_action_service_proxy(add_action_req)
            rospy.loginfo("Add new action %s with id %s" % (add_action_req.action_content.action_name, resp.action_id))

            # Update color only when its marker status is NOT_ASSIGNED
            int_marker = self.server.get(feedback.marker_name)
            control_name_list = [control.name for control in int_marker.controls]
            control_visual = int_marker.controls[control_name_list.index('visualization_with_menu')]
            is_not_assigned = True
            for marker in control_visual.markers:
                is_not_assigned = is_not_assigned and marker.color == COLOR_NOT_ASSIGNED
            if is_not_assigned:
                self.server.erase(feedback.marker_name)
                poses = []
                for i in range(4):
                    int_marker_vertex = self.server_vertices.get(feedback.marker_name + " (vertex " + str(i) + ")")
                    poses.append(int_marker_vertex.pose)
                    self.server_vertices.erase(feedback.marker_name + " (vertex " + str(i) + ")")
                self.add_task_marker(poses, MarkerStatus.IDLE, name=feedback.marker_name,
                                     description=int_marker.description[len(int_marker.name):])
        self.server.applyChanges()
        self.server_vertices.applyChanges()

    def handle_save_int_markers(self, req):
        """
        Override function
        :param req:
        :return:
        """
        name_int_marker_dict_center = {}
        for name, marker_context in self.server.marker_contexts.iteritems():
            name_int_marker_dict_center[name] = marker_context.int_marker
        name_int_marker_dict_vertices = {}
        for name, marker_context in self.server_vertices.marker_contexts.iteritems():
            name_int_marker_dict_vertices[name] = marker_context.int_marker
        with open(req.file_path + '.pkl', 'wb') as outfile:
            pickle.dump(name_int_marker_dict_center, outfile, pickle.HIGHEST_PROTOCOL)
        rospy.loginfo("Saved self.marker_contexts to %s" % req.file_path)
        file_path_vertices = req.file_path + '_vertices'
        with open(file_path_vertices + '.pkl', 'wb') as outfile:
            pickle.dump(name_int_marker_dict_vertices, outfile, pickle.HIGHEST_PROTOCOL)
        rospy.loginfo("Saved self.marker_contexts to %s" % file_path_vertices)
        return SaveIntMarkersResponse()

    def handle_load_int_markers(self, req):
        """
        Override function
        :param req:
        :return:
        """
        # Load the interactive marker information
        with open(req.file_path + '.pkl', 'rb') as stream:
            new_name_int_marker_dict_center = pickle.load(stream)
            self.server.clear()
            for name, int_marker in new_name_int_marker_dict_center.iteritems():
                self.server.insert(int_marker, self.process_feedback)
                control_list = int_marker.controls
                control_name_list = [control.name for control in control_list]
                control_visual = control_list[control_name_list.index("visualization_with_menu")]
                status_color = control_visual.markers[0].color
                status_value = color_to_status(status_color)
                if status_to_is_deletable(status_value):
                    self.menu_handler_with_delete.apply(self.server, int_marker.name)
                elif status_to_is_movable(status_value):
                    self.menu_handler_with_rename.apply(self.server, int_marker.name)
                else:
                    self.menu_handler.apply(self.server, int_marker.name)
                rospy.loginfo("Loaded interactive marker %s" % name)
            self.server.applyChanges()

        file_path_vertices = req.file_path + '_vertices'
        with open(file_path_vertices + '.pkl', 'rb') as stream:
            new_name_int_marker_dict_vertices = pickle.load(stream)
            self.server_vertices.clear()
            for name, int_marker in new_name_int_marker_dict_vertices.iteritems():
                self.server_vertices.insert(int_marker, self.process_feedback)
                rospy.loginfo("Loaded interactive marker %s" % name)
            self.server_vertices.applyChanges()

        # Reset the global counter of markers. This will be properly incremented in self.add_task_marker()
        self.marker_counter = 0

        return LoadIntMarkersResponse()

    def delete_callback(self, feedback):
        """
        Override function
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

        if feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
            self.server.erase(feedback.marker_name)
            for i in range(4):
                self.server_vertices.erase(feedback.marker_name + " (vertex " + str(i) + ")")
            rospy.loginfo("Deleted Marker %s" % feedback.marker_name)
        self.server.applyChanges()
        self.server_vertices.applyChanges()

    def process_feedback_center(self, feedback):
        """
        Process feedback function only for the central interactive marker
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
            # Update position of vertex markers
            int_marker = self.server.get(feedback.marker_name)
            control_name_list = [control.name for control in int_marker.controls]
            control_visual = int_marker.controls[control_name_list.index('visualization_with_menu')]
            status = color_to_status(control_visual.markers[0].color)
            for i in range(4):
                self.server_vertices.erase(feedback.marker_name + " (vertex " + str(i) + ")")
                int_marker_vertex = self.task_marker_vertex(feedback.pose.position, control_visual.markers[0].scale.x,
                                                            control_visual.markers[0].scale.y, status,
                                                            feedback.marker_name, i)
                self.server_vertices.insert(int_marker_vertex, self.process_feedback_vertex)
        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()
        self.server_vertices.applyChanges()

    def process_feedback_vertex(self, feedback):
        """
        Process feedback function only for the vertex interactive markers
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
            # Update the rest of markers
            # Case of vertex 0
            if feedback.marker_name.endswith(' (vertex 0)'):
                marker_center_name = feedback.marker_name[:-len(' (vertex 0)')]

                # Calculate updated parameters
                int_marker_opposite_vertex = self.server_vertices.get(marker_center_name + ' (vertex 2)')
                # Do not allow to go through other vertex
                if feedback.pose.position.x < int_marker_opposite_vertex.pose.position.x + self.minimum_area_size:
                    feedback.pose.position.x = int_marker_opposite_vertex.pose.position.x + self.minimum_area_size
                if feedback.pose.position.y < int_marker_opposite_vertex.pose.position.y + self.minimum_area_size:
                    feedback.pose.position.y = int_marker_opposite_vertex.pose.position.y + self.minimum_area_size
                position = geometry_msgs.msg.Point()
                position.x = (feedback.pose.position.x + int_marker_opposite_vertex.pose.position.x) / 2.
                position.y = (feedback.pose.position.y + int_marker_opposite_vertex.pose.position.y) / 2.
                position.z = 0.0
                x_scale = np.abs(feedback.pose.position.x - int_marker_opposite_vertex.pose.position.x)
                y_scale = np.abs(feedback.pose.position.y - int_marker_opposite_vertex.pose.position.y)

                int_marker_center = self.server.get(marker_center_name)
                control_name_list = [control.name for control in int_marker_center.controls]
                control_visual = int_marker_center.controls[control_name_list.index('visualization_with_menu')]
                status = color_to_status(control_visual.markers[0].color)
                # omit marker name to avoid recursive addition
                description = int_marker_center.description[len(int_marker_center.name):]

                # Create new markers
                int_marker_center = self.task_marker_center(position, x_scale, y_scale, status, marker_center_name, description)
                int_marker_vertex_1 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 1)
                int_marker_vertex_3 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 3)

                # Delete outdated markers
                self.server.erase(marker_center_name)
                self.server_vertices.erase(marker_center_name + ' (vertex 1)')
                self.server_vertices.erase(marker_center_name + ' (vertex 3)')

                # Add new markers
                self.server.insert(int_marker_center, self.process_feedback_center)
                self.server_vertices.insert(int_marker_vertex_1, self.process_feedback_vertex)
                self.server_vertices.insert(int_marker_vertex_3, self.process_feedback_vertex)

                # Add  menu for center marker
                if status_to_is_deletable(status):
                    self.menu_handler_with_delete.apply(self.server, int_marker_center.name)
                elif status_to_is_movable(status):
                    self.menu_handler_with_rename.apply(self.server, int_marker_center.name)
                else:
                    self.menu_handler.apply(self.server, int_marker_center.name)

            # Case of vertex 1
            elif feedback.marker_name.endswith(' (vertex 1)'):
                marker_center_name = feedback.marker_name[:-len(' (vertex 1)')]

                # Calculate updated parameters
                int_marker_opposite_vertex = self.server_vertices.get(marker_center_name + ' (vertex 3)')
                # Do not allow to go through other vertex
                if feedback.pose.position.x > int_marker_opposite_vertex.pose.position.x - self.minimum_area_size:
                    feedback.pose.position.x = int_marker_opposite_vertex.pose.position.x - self.minimum_area_size
                if feedback.pose.position.y < int_marker_opposite_vertex.pose.position.y + self.minimum_area_size:
                    feedback.pose.position.y = int_marker_opposite_vertex.pose.position.y + self.minimum_area_size
                position = geometry_msgs.msg.Point()
                position.x = (feedback.pose.position.x + int_marker_opposite_vertex.pose.position.x) / 2.
                position.y = (feedback.pose.position.y + int_marker_opposite_vertex.pose.position.y) / 2.
                position.z = 0.0
                x_scale = np.abs(feedback.pose.position.x - int_marker_opposite_vertex.pose.position.x)
                y_scale = np.abs(feedback.pose.position.y - int_marker_opposite_vertex.pose.position.y)

                int_marker_center = self.server.get(marker_center_name)
                control_name_list = [control.name for control in int_marker_center.controls]
                control_visual = int_marker_center.controls[control_name_list.index('visualization_with_menu')]
                status = color_to_status(control_visual.markers[0].color)
                # omit marker name to avoid recursive addition
                description = int_marker_center.description[len(int_marker_center.name):]

                # Create new markers
                int_marker_center = self.task_marker_center(position, x_scale, y_scale, status, marker_center_name,
                                                            description)
                int_marker_vertex_0 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 0)
                int_marker_vertex_2 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 2)

                # Delete outdated markers
                self.server.erase(marker_center_name)
                self.server_vertices.erase(marker_center_name + ' (vertex 0)')
                self.server_vertices.erase(marker_center_name + ' (vertex 2)')

                # Add new markers
                self.server.insert(int_marker_center, self.process_feedback_center)
                self.server_vertices.insert(int_marker_vertex_0, self.process_feedback_vertex)
                self.server_vertices.insert(int_marker_vertex_2, self.process_feedback_vertex)

                # Add  menu for center marker
                if status_to_is_deletable(status):
                    self.menu_handler_with_delete.apply(self.server, int_marker_center.name)
                elif status_to_is_movable(status):
                    self.menu_handler_with_rename.apply(self.server, int_marker_center.name)
                else:
                    self.menu_handler.apply(self.server, int_marker_center.name)

            # Case of vertex 2
            elif feedback.marker_name.endswith(' (vertex 2)'):
                marker_center_name = feedback.marker_name[:-len(' (vertex 2)')]

                # Calculate updated parameters
                int_marker_opposite_vertex = self.server_vertices.get(marker_center_name + ' (vertex 0)')
                # Do not allow to go through other vertex
                if feedback.pose.position.x > int_marker_opposite_vertex.pose.position.x - self.minimum_area_size:
                    feedback.pose.position.x = int_marker_opposite_vertex.pose.position.x - self.minimum_area_size
                if feedback.pose.position.y > int_marker_opposite_vertex.pose.position.y - self.minimum_area_size:
                    feedback.pose.position.y = int_marker_opposite_vertex.pose.position.y - self.minimum_area_size
                position = geometry_msgs.msg.Point()
                position.x = (feedback.pose.position.x + int_marker_opposite_vertex.pose.position.x) / 2.
                position.y = (feedback.pose.position.y + int_marker_opposite_vertex.pose.position.y) / 2.
                position.z = 0.0
                x_scale = np.abs(feedback.pose.position.x - int_marker_opposite_vertex.pose.position.x)
                y_scale = np.abs(feedback.pose.position.y - int_marker_opposite_vertex.pose.position.y)

                int_marker_center = self.server.get(marker_center_name)
                control_name_list = [control.name for control in int_marker_center.controls]
                control_visual = int_marker_center.controls[control_name_list.index('visualization_with_menu')]
                status = color_to_status(control_visual.markers[0].color)
                # omit marker name to avoid recursive addition
                description = int_marker_center.description[len(int_marker_center.name):]

                # Create new markers
                int_marker_center = self.task_marker_center(position, x_scale, y_scale, status, marker_center_name,
                                                            description)
                int_marker_vertex_1 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 1)
                int_marker_vertex_3 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 3)

                # Delete outdated markers
                self.server.erase(marker_center_name)
                self.server_vertices.erase(marker_center_name + ' (vertex 1)')
                self.server_vertices.erase(marker_center_name + ' (vertex 3)')

                # Add new markers
                self.server.insert(int_marker_center, self.process_feedback_center)
                self.server_vertices.insert(int_marker_vertex_1, self.process_feedback_vertex)
                self.server_vertices.insert(int_marker_vertex_3, self.process_feedback_vertex)

                # Add  menu for center marker
                if status_to_is_deletable(status):
                    self.menu_handler_with_delete.apply(self.server, int_marker_center.name)
                elif status_to_is_movable(status):
                    self.menu_handler_with_rename.apply(self.server, int_marker_center.name)
                else:
                    self.menu_handler.apply(self.server, int_marker_center.name)

            # Case of vertex 3
            elif feedback.marker_name.endswith(' (vertex 3)'):
                marker_center_name = feedback.marker_name[:-len(' (vertex 3)')]

                # Calculate updated parameters
                int_marker_opposite_vertex = self.server_vertices.get(marker_center_name + ' (vertex 1)')
                # Do not allow to go through other vertex
                if feedback.pose.position.x < int_marker_opposite_vertex.pose.position.x + self.minimum_area_size:
                    feedback.pose.position.x = int_marker_opposite_vertex.pose.position.x + self.minimum_area_size
                if feedback.pose.position.y > int_marker_opposite_vertex.pose.position.y - self.minimum_area_size:
                    feedback.pose.position.y = int_marker_opposite_vertex.pose.position.y - self.minimum_area_size
                position = geometry_msgs.msg.Point()
                position.x = (feedback.pose.position.x + int_marker_opposite_vertex.pose.position.x) / 2.
                position.y = (feedback.pose.position.y + int_marker_opposite_vertex.pose.position.y) / 2.
                position.z = 0.0
                x_scale = np.abs(feedback.pose.position.x - int_marker_opposite_vertex.pose.position.x)
                y_scale = np.abs(feedback.pose.position.y - int_marker_opposite_vertex.pose.position.y)

                int_marker_center = self.server.get(marker_center_name)
                control_name_list = [control.name for control in int_marker_center.controls]
                control_visual = int_marker_center.controls[control_name_list.index('visualization_with_menu')]
                status = color_to_status(control_visual.markers[0].color)
                # omit marker name to avoid recursive addition
                description = int_marker_center.description[len(int_marker_center.name):]

                # Create new markers
                int_marker_center = self.task_marker_center(position, x_scale, y_scale, status, marker_center_name,
                                                            description)
                int_marker_vertex_0 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 0)
                int_marker_vertex_2 = self.task_marker_vertex(position, x_scale, y_scale, status, marker_center_name, 2)

                # Delete outdated markers
                self.server.erase(marker_center_name)
                self.server_vertices.erase(marker_center_name + ' (vertex 0)')
                self.server_vertices.erase(marker_center_name + ' (vertex 2)')

                # Add new markers
                self.server.insert(int_marker_center, self.process_feedback_center)
                self.server_vertices.insert(int_marker_vertex_0, self.process_feedback_vertex)
                self.server_vertices.insert(int_marker_vertex_2, self.process_feedback_vertex)

                # Add  menu for center marker
                if status_to_is_deletable(status):
                    self.menu_handler_with_delete.apply(self.server, int_marker_center.name)
                elif status_to_is_movable(status):
                    self.menu_handler_with_rename.apply(self.server, int_marker_center.name)
                else:
                    self.menu_handler.apply(self.server, int_marker_center.name)

            # Otherwise
            else:
                rospy.logerr("Vertex number is not implemented. Marker name is %s" % feedback.marker_name)

        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
            # Trigger fake MOUSE_UP feedback on self.server so that Mission Control GUI can update is parametrization
            # NOTE: This is required because of having two separate interactive marker servers (center and vertices)
            marker_center_name = feedback.marker_name[:-len(' (vertex n)')]
            int_marker_center = self.server.get(marker_center_name)
            feedback_to_center = visualization_msgs.msg.InteractiveMarkerFeedback()
            feedback_to_center.header.frame_id = self.frame_id
            feedback_to_center.client_id = self.topic_name
            feedback_to_center.marker_name = marker_center_name
            feedback_to_center.control_name = ''
            feedback_to_center.event_type = visualization_msgs.msg.InteractiveMarkerFeedback.MOUSE_UP
            feedback_to_center.pose = int_marker_center.pose
            feedback_to_center.mouse_point = feedback.mouse_point
            feedback_to_center.mouse_point_valid = feedback.mouse_point_valid
            self.feedback_publisher_to_center.publish(feedback_to_center)

        self.server.applyChanges()
        self.server_vertices.applyChanges()

    def process_feedback(self, feedback):
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
        self.server_vertices.applyChanges()


# if __name__ == '__main__':
#     rospy.init_node('task_marker_server_2d_area')
#     try:
#         server = TaskMarker2DAreaServer()
#     except rospy.ROSInterruptException:
#         pass
#     rospy.spin()
