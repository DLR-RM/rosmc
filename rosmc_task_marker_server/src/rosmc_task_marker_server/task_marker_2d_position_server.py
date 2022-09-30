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

import numpy as np
import yaml

import rospy

import geometry_msgs.msg
import visualization_msgs.msg
import tf

from rosmc_msgs.srv import AddTaskMarker, AddTaskMarkerResponse, AddActionRequest,\
    UpdateMarkerStatusResponse, RenameMarkerResponse

from rosmc_task_marker_server.task_marker_server import *


class TaskMarker2DPositionServer(TaskMarkerServer):

    def __init__(self):
        super(TaskMarker2DPositionServer, self).__init__()
        self.marker_type = '2d_position'
        self.add_marker_service_class = AddTaskMarker
        self.onInitialize()
        self.offset_ground += 0.1  # This avoids overlap on 2D area marker
        self.offset_controller = self.offset_ground  # offset of controller from ground surface
        rospy.loginfo("Ready")

    def add_task_marker(self, pose, status_value, name=None, description="\n(No agent is assigned)"):
        # Initialize interactive marker
        position = pose.position
        int_marker = visualization_msgs.msg.InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.pose.position.x = position.x
        int_marker.pose.position.y = position.y
        int_marker.pose.position.z = self.offset_controller  # offset
        # int_marker.pose.orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        int_marker.pose.orientation.x = quaternion[0]
        int_marker.pose.orientation.y = quaternion[1]
        int_marker.pose.orientation.z = quaternion[2]
        int_marker.pose.orientation.w = quaternion[3]
        int_marker.scale = 2 * self.marker_scale
        if name is None:
            while self.server.get("2D Position " + str(self.marker_counter)) is not None:
                self.marker_counter += 1
            name = "2D Position " + str(self.marker_counter)
            # Increment marker_counter if name is not given
            self.marker_counter += 1
        int_marker.name = name
        int_marker.description = name + description

        # Context menu with visualization
        # NOTE: All markers to be visualized must attached to this control with name "visualization_with_menu"
        control_visual = visualization_msgs.msg.InteractiveMarkerControl()
        control_visual.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MENU
        control_visual.name = "visualization_with_menu"

        # Arrow
        # arrow_pose = geometry_msgs.msg.Pose()
        # arrow_pose.position.x = 0.0
        # arrow_pose.position.y = 0.0
        # arrow_pose.position.z = -0.2  # offset
        # arrow_pose.orientation.w = 1.0
        # arrow_pose.orientation.x = 0.0
        # arrow_pose.orientation.y = 0.0
        # arrow_pose.orientation.z = 0.0
        # arrow_marker = visualization_msgs.msg.Marker
        # arrow_marker.type = visualization_msgs.msg.Marker.ARROW
        # arrow_marker.scale.x = 2.0
        # arrow_marker.scale.y = 0.2
        # arrow_marker.scale.z = 0.2
        # arrow_marker.color.a = 1.0
        # arrow_marker.color.r = 0.5
        # arrow_marker.color.g = 0.5
        # arrow_marker.color.b = 0.5
        # arrow_marker.pose = arrow_pose
        # control_visual.markers.append(arrow_marker)

        # Flag
        flag_pose = geometry_msgs.msg.Pose()
        flag_pose.position.x = 0.0
        flag_pose.position.y = 0.0
        flag_pose.position.z = -self.offset_controller + self.offset_ground
        flag_pose.orientation.w = 1.0
        flag_pose.orientation.x = 0.0
        flag_pose.orientation.y = 0.0
        flag_pose.orientation.z = 0.0
        flag_marker = visualization_msgs.msg.Marker()
        flag_marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
        flag_marker.mesh_resource = "package://rosmc_task_marker_server/meshes/flag.dae"
        flag_marker.scale.x = 1.0 * self.marker_scale
        flag_marker.scale.y = 1.0 * self.marker_scale
        flag_marker.scale.z = 1.0 * self.marker_scale
        flag_marker.color = status_to_color(status_value)
        flag_marker.pose = flag_pose
        control_visual.markers.append(flag_marker)

        control_visual.always_visible = True
        int_marker.controls.append(control_visual)

        # Depending on status_value, add control method for movements in xy plane
        if status_to_is_movable(status_value):
            control_pose = visualization_msgs.msg.InteractiveMarkerControl()
            control_pose.orientation.w = 1.0
            control_pose.orientation.x = 0.0
            control_pose.orientation.y = 1.0
            control_pose.orientation.z = 0.0
            control_pose.interaction_mode = visualization_msgs.msg.InteractiveMarkerControl.MOVE_PLANE
            int_marker.controls.append(control_pose)

        self.server.insert(int_marker, self.process_feedback)
        if status_to_is_deletable(status_value):
            self.menu_handler_with_delete.apply(self.server, int_marker.name)
        elif status_to_is_movable(status_value):
            self.menu_handler_with_rename.apply(self.server, int_marker.name)
        else:
            self.menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()

        return name

    def handle_add_task_marker(self, req):
        name = self.add_task_marker(req.pose, req.status.value, description="")
        res = AddTaskMarkerResponse()
        res.name = name
        return res

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

            marker_for_parametrization_list = self.topic_lib_action_int_marker_dict[self.topic_name][library][action_name]
            for marker_for_parametrization in marker_for_parametrization_list:
                # If this marker is assigned to multiple parameters (group),
                if 'members' in marker_for_parametrization:
                    members = marker_for_parametrization['members']
                    for member in members:
                        param_name = member['name']
                        value = member['value']
                        # Check if the parametrization uses this marker topic or not
                        if self.topic_name not in marker_for_parametrization["topics"]:
                            # If this marker topic is not used, fill in default value in parameters_dict[key]['marker']
                            parameters_dict[param_name]['marker'] = ""
                            parameters_dict[param_name]['marker_topic'] = marker_for_parametrization["topics"][0]
                        else:
                            new_value = rgetattr(self, value)
                            parameters_dict[param_name]['value'] = new_value
                            parameters_dict[param_name]['marker'] = feedback.marker_name
                            parameters_dict[param_name]['marker_topic'] = self.topic_name
                # Else if this marker is assigned to single parameter
                elif 'value' in marker_for_parametrization:
                    param_name = marker_for_parametrization['name']
                    value = marker_for_parametrization['value']
                    # Check if the parametrization uses this marker topic or not
                    if self.topic_name not in marker_for_parametrization["topics"]:
                        # If this marker topic is not used, fill in default value in parameters_dict[key]['marker']
                        parameters_dict[param_name]['marker'] = ""
                        parameters_dict[param_name]['marker_topic'] = marker_for_parametrization["topics"][0]
                    else:
                        new_value = rgetattr(self, value)
                        parameters_dict[param_name]['value'] = new_value
                        parameters_dict[param_name]['marker'] = feedback.marker_name
                        parameters_dict[param_name]['marker_topic'] = self.topic_name
                    new_value = rgetattr(self, value)
                    parameters_dict[param_name]['value'] = new_value
                    parameters_dict[param_name]['marker'] = feedback.marker_name
                    parameters_dict[param_name]['marker_topic'] = self.topic_name
                else:
                    rospy.logerr("ERROR: either 'members' or 'value' key is required for int_marker assignment!")
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
                self.add_task_marker(feedback.pose, MarkerStatus.IDLE, name=feedback.marker_name, description="")
        self.server.applyChanges()

    def handle_update_marker_status(self, req):
        int_marker = self.server.get(req.marker_name)
        if int_marker is None:
            rospy.logwarn("Requested marker is not found. Skip status updates.")
            return UpdateMarkerStatusResponse()
        self.server.erase(req.marker_name)
        self.add_task_marker(int_marker.pose, req.updated_status.value, name=req.marker_name, description="")
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

        self.server.insert(req.int_marker, self.process_feedback)
        if status_to_is_deletable(status):
            self.menu_handler_with_delete.apply(self.server, req.int_marker.name)
        elif status_to_is_movable(status_value):
            self.menu_handler_with_rename.apply(self.server, int_marker.name)
        else:
            self.menu_handler.apply(self.server, req.int_marker.name)
        self.server.applyChanges()
        return UpdateIntMarkerResponse()

    def rename_marker(self, name_old, name_new):
        # Get int_marker with old name
        int_marker = self.server.get(name_old)
        if int_marker is None:
            error_msg = "Requested marker is not found. Skip status updates."
            rospy.logwarn(error_msg)
            return error_msg
        # Delete the marker
        self.server.erase(name_old)
        # Retrieve status
        control_name_list = [control.name for control in int_marker.controls]
        control_visual = int_marker.controls[control_name_list.index('visualization_with_menu')]
        status = color_to_status(control_visual.markers[0].color)
        # Add marker with new name
        self.add_task_marker(int_marker.pose, status, name=name_new, description="")
        rospy.loginfo("Renamed marker from %s to %s" % (name_old, name_new))
        return ""

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


# if __name__ == '__main__':
#     rospy.init_node('task_marker_server_2d_position')
#     try:
#         server = TaskMarker2DPositionServer()
#     except rospy.ROSInterruptException:
#         pass
#     rospy.spin()
