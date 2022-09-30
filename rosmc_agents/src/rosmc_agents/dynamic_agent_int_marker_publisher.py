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

import geometry_msgs.msg
import interactive_markers.interactive_marker_server
import interactive_markers.menu_handler
import tf
import visualization_msgs.msg
from visualization_msgs.msg import InteractiveMarker

from rosmc_msgs.msg import MarkerStatus
from rosmc_msgs.srv import AddAction, AddActionRequest, UpdateMarkerStatusResponse
import rosmc_msgs.srv

from rosmc_agents.agent_int_marker_publisher import rsetattr, rgetattr, AgentIntMarkerPublisher


class DynamicAgentIntMarkerPublisher(AgentIntMarkerPublisher):
    """
    Basic class of interactive marker publisher for 'dynamic' agents.
    'Dynamic' agents means that they can be operated by Mission Control GUI
    """

    def __init__(self, topic_name, agent_names, mission_server_namespace='/mission_control/'):
        super(DynamicAgentIntMarkerPublisher, self).__init__(topic_name, agent_names, mission_server_namespace)

    def handle_update_marker_status(self, req):
        rospy.loginfo("Do nothing for marker updates in dynamic agent markers")
        return UpdateMarkerStatusResponse()

    def setup_menu(self):
        for agent_name in self.agent_names:
            menu_handler = interactive_markers.menu_handler.MenuHandler()
            add_action_submenu_handler = menu_handler.insert("Add Action")
            # NOTE: this node assumes that Mission Server is already running
            # and rosparams /agent_action_dict is already set
            action_list = rospy.get_param('{}agent_action_dict/{}'.format(self.mission_server_namespace, agent_name))
            for action_name in sorted(action_list):
                menu_handler.insert(action_name, parent=add_action_submenu_handler, callback=self.add_action_callback)
            self.menu_handlers[agent_name] = menu_handler
        return None

    def init_markers(self):
        """
        Implement this function to achieve automatic generation of interactive markers.
        NOTE: for adding interactive marker to server,
        use self.add_agent_marker(position, calib_pose, frame_id, agent_name, mesh_path, scale, color)
        """
        raise NotImplementedError

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
            agent_name = feedback.marker_name
            action_name = self.menu_handlers[agent_name].getTitle(feedback.menu_entry_id)

            add_action_req = AddActionRequest()
            add_action_req.agent_name = agent_name

            # Fill in the initial contents
            add_action_req.action_content.action_name = action_name
            add_action_req.action_content.action_label = action_name
            add_action_req.action_content.battery_flag = True
            add_action_req.action_content.wlan_flag = True

            # Parameter specification
            parameters_dict = {}
            # In case this action requires no parameter, this block is skipped.
            if rospy.get_param('{}agent_action_dict/{}/{}'.format(self.mission_server_namespace, agent_name, action_name)) != 'None':
                parameters_dict = rospy.get_param('{}agent_action_dict/{}/{}'.format(self.mission_server_namespace, agent_name, action_name)).copy()
            for parameter_name in parameters_dict:
                # First, fill in default parameters
                parameters_dict[parameter_name]['value'] = parameters_dict[parameter_name]['default_value']
                del parameters_dict[parameter_name]['default_value']

            # If we use interactive markers, fill in interactive marker value
            if rospy.get_param('{}use_interactive_markers'.format(self.mission_server_namespace)):
                # Check if the action requires parametrization by interactive marker
                # If required, create a new interactive marker for each kind of type
                library = rospy.get_param('{}agent_library_dict/{}'.format(self.mission_server_namespace, agent_name))
                action_int_marker_list = rospy.get_param('{}lib_action_int_marker_dict/{}'.format(self.mission_server_namespace, library))

                # TODO: From here, codes are exactly the same as the one in function add_action(self, action_name) in ScheduleWidget.py
                try:
                    marker_for_parametrization_list = action_int_marker_list[action_name]
                except KeyError:
                    marker_for_parametrization_list = []

                # Add TASK interactive markers in int_marker_list
                for i, marker_for_parametrization in enumerate(marker_for_parametrization_list):
                    if marker_for_parametrization['type'] == 'TASK':
                        # Retrieve information from marker_for_parametrization
                        service_type_for_addition = marker_for_parametrization['service_type_for_addition']
                        service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                        service_req_attr = marker_for_parametrization['service_req_attr']
                        service_req_value = marker_for_parametrization['service_req_value']

                        # Obtain the corresponding service classes for adding markers
                        service_type = getattr(rosmc_msgs.srv, service_type_for_addition)
                        req = getattr(rosmc_msgs.srv, service_type_for_addition + 'Request')()
                        req.status.value = MarkerStatus.IDLE

                        # Create int_marker from the parameters in this group
                        self.INT_MARKER = InteractiveMarker()
                        self.ROLL = 0.0
                        self.PITCH = 0.0
                        self.YAW = 0.0
                        # If this marker is assigned to multiple parameters (group),
                        if 'members' in marker_for_parametrization:
                            members = marker_for_parametrization['members']
                            for member in members:
                                param_name = member['name']
                                value = member['value']
                                rsetattr(self, value, parameters_dict[param_name]['value'])
                        # Else if this marker is assigned to single parameter
                        elif 'value' in marker_for_parametrization:
                            param_name = marker_for_parametrization['name']
                            value = marker_for_parametrization['value']
                            rsetattr(self, value, parameters_dict[param_name]['value'])
                        else:
                            rospy.logerr(
                                "ERROR: either 'members' or 'value' key is required for int_marker assignment!")
                            return
                        quaternion = tf.transformations.quaternion_from_euler(self.ROLL, self.PITCH, self.YAW)
                        self.INT_MARKER.pose.orientation.x = quaternion[0]
                        self.INT_MARKER.pose.orientation.y = quaternion[1]
                        self.INT_MARKER.pose.orientation.z = quaternion[2]
                        self.INT_MARKER.pose.orientation.w = quaternion[3]

                        # Enter the values for service req
                        value = rgetattr(self, service_req_value)
                        rsetattr(req, service_req_attr, value)

                        # Add marker
                        add_task_marker_service_proxy = \
                            rospy.ServiceProxy('{}/add_task_marker'.format(service_topic_for_addition), service_type)
                        res = add_task_marker_service_proxy(req)
                        # Store the information about the name of the added marker
                        # NOTE: Every request for adding task markers should return the name of the marker added.
                        marker_for_parametrization_list[i]['marker_name'] = res.name
                    elif marker_for_parametrization['type'] == 'TASK_POSEDICT':
                        # Hard-coded mode for pose dict
                        param_name = marker_for_parametrization['name']
                        service_type_for_addition = marker_for_parametrization['service_type_for_addition']
                        service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                        service_type = getattr(rosmc_msgs.srv, service_type_for_addition)
                        req = getattr(rosmc_msgs.srv, service_type_for_addition + 'Request')()
                        req.status.value = MarkerStatus.IDLE
                        # Convert pose into geometry_msgs/Pose
                        pose = parameters_dict[param_name]['value']
                        geometry_pose = geometry_msgs.msg.Pose()
                        geometry_pose.position.x = pose['x']
                        geometry_pose.position.y = pose['y']
                        geometry_pose.position.z = pose['z']
                        quaternion = tf.transformations.quaternion_from_euler(pose['roll'], pose['pitch'],
                                                                              pose['yaw'])
                        geometry_pose.orientation.x = quaternion[0]
                        geometry_pose.orientation.y = quaternion[1]
                        geometry_pose.orientation.z = quaternion[2]
                        geometry_pose.orientation.w = quaternion[3]
                        req.pose = geometry_pose
                        add_task_marker_service_proxy = \
                            rospy.ServiceProxy('{}/add_task_marker'.format(service_topic_for_addition), service_type)
                        res = add_task_marker_service_proxy(req)
                        # Store the information about the name of the added marker
                        # NOTE: Every request for adding task markers should return the name of the marker added.
                        marker_for_parametrization_list[i]['marker_name'] = res.name

                    elif marker_for_parametrization['type'] == 'TASK_LIST_POSEDICT':
                        # Hard-coded mode for area marker
                        param_name = marker_for_parametrization['name']
                        service_type_for_addition = marker_for_parametrization['service_type_for_addition']
                        service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                        service_type = getattr(rosmc_msgs.srv, service_type_for_addition)
                        req = getattr(rosmc_msgs.srv, service_type_for_addition + 'Request')()
                        req.status.value = MarkerStatus.IDLE

                        # Convert list_pose into list of geometry_msgs/Pose
                        list_pose = parameters_dict[param_name]['value']
                        list_geometry_pose = []
                        for pose in list_pose:
                            geometry_pose = geometry_msgs.msg.Pose()
                            geometry_pose.position.x = pose['x']
                            geometry_pose.position.y = pose['y']
                            geometry_pose.position.z = pose['z']
                            quaternion = tf.transformations.quaternion_from_euler(pose['roll'], pose['pitch'],
                                                                                  pose['yaw'])
                            geometry_pose.orientation.x = quaternion[0]
                            geometry_pose.orientation.y = quaternion[1]
                            geometry_pose.orientation.z = quaternion[2]
                            geometry_pose.orientation.w = quaternion[3]
                            list_geometry_pose.append(geometry_pose)

                        # NOTE: implicitly assumes that req has key 'poses', which is list of geometry_msgs/Pose
                        req.poses = list_geometry_pose
                        add_task_marker_service_proxy = \
                            rospy.ServiceProxy('{}/add_task_marker'.format(service_topic_for_addition), service_type)
                        res = add_task_marker_service_proxy(req)
                        # Store the information about the name of the added marker
                        # NOTE: Every request for adding task markers should return the name of the marker added.
                        marker_for_parametrization_list[i]['marker_name'] = res.name

                # Add the name and the topic_name of interactive markers to the parameters
                for param_name in parameters_dict:
                    for marker_for_parametrization in marker_for_parametrization_list:
                        if marker_for_parametrization['type'] == 'TASK':
                            service_topic_for_addition = marker_for_parametrization[
                                'service_topic_for_addition']
                            # If this marker is assigned to multiple parameters (group),
                            if 'members' in marker_for_parametrization:
                                members = marker_for_parametrization['members']
                                for member in members:
                                    if param_name == member['name']:
                                        marker_name = marker_for_parametrization['marker_name']
                                        parameters_dict[param_name]['marker'] = marker_name
                                        parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                        break
                            # Else if this marker is assigned to single parameter
                            elif 'value' in marker_for_parametrization:
                                if param_name == marker_for_parametrization['name']:
                                    marker_name = marker_for_parametrization['marker_name']
                                    parameters_dict[param_name]['marker'] = marker_name
                                    parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                    break
                        elif marker_for_parametrization['type'] == 'TASK_LIST_POSEDICT' or \
                                marker_for_parametrization['type'] == 'TASK_POSEDICT':
                            # hard-coded behavior for adding markers
                            service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                            if param_name == marker_for_parametrization['name']:
                                marker_name = marker_for_parametrization['marker_name']
                                parameters_dict[param_name]['marker'] = marker_name
                                parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                break
                        elif marker_for_parametrization['type'] == 'TASK_OBJECT':
                            service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                            # If this marker is assigned to multiple parameters (group),
                            if 'members' in marker_for_parametrization:
                                members = marker_for_parametrization['members']
                                for member in members:
                                    if param_name == member['name']:
                                        marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                        parameters_dict[param_name]['marker'] = marker_name
                                        parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                        break
                            # Else if this marker is assigned to single parameter
                            elif 'value' in marker_for_parametrization:
                                if param_name == marker_for_parametrization['name']:
                                    marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                    parameters_dict[param_name]['marker'] = marker_name
                                    parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                    break
                        elif marker_for_parametrization['type'] == 'AGENT':
                            # If this marker is assigned to multiple parameters (group),
                            if 'members' in marker_for_parametrization:
                                members = marker_for_parametrization['members']
                                for member in members:
                                    if param_name == member['name']:
                                        marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                        parameters_dict[param_name]['marker'] = marker_name
                                        # TODO: do we need to fill in here? - Maybe not.
                                        #  This is because key 'marker_topic' is only used for update_marker_status() in mission_server.py
                                        #  and update_marker_status() takes no effect for AGENT markers.
                                        parameters_dict[param_name]['marker_topic'] = None
                                        break
                            # Else if this marker is assigned to single parameter
                            elif 'value' in marker_for_parametrization:
                                if param_name == marker_for_parametrization['name']:
                                    marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                    parameters_dict[param_name]['marker'] = marker_name
                                    # TODO: do we need to fill in here? - Maybe not.
                                    #  This is because key 'marker_topic' is only used for update_marker_status() in mission_server.py
                                    #  and update_marker_status() takes no effect for AGENT markers.
                                    parameters_dict[param_name]['marker_topic'] = None
                                    break

            add_action_req.action_content.parameters_yaml = yaml.dump(parameters_dict)
            # resp = self.add_action_clients[agent_name](add_action_req)
            resp = self.add_action_client(add_action_req)
            rospy.loginfo("Add new action %s with id %s" % (add_action_req.action_content.action_name, resp.action_id))
