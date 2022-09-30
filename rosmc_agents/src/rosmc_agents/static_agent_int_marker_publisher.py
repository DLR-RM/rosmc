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

import yaml

import rospy

import geometry_msgs.msg
import interactive_markers.interactive_marker_server
import interactive_markers.menu_handler
import tf
import visualization_msgs.msg
from visualization_msgs.msg import InteractiveMarker

from rosmc_msgs.srv import AddActionRequest, UpdateMarkerStatusResponse

from rosmc_agents.agent_int_marker_publisher import AgentIntMarkerPublisher, rsetattr, rgetattr


class StaticAgentIntMarkerPublisher(AgentIntMarkerPublisher):
    """
    Basic class of interactive marker publisher for 'static' agents.
    'Static' agents mean that they cannot be operated by Mission Control GUI
    """

    def __init__(self, topic_name, agent_names, mission_server_namespace='/mission_control/'):
        # Interactive markers information
        # TODO: Currently, this class works only when we set the flag use_interactive_markers as true
        #       because the following rosparam are set only if use_interactive_markers == true
        self.topic_lib_action_int_marker_dict = rospy.get_param('{}topic_lib_action_int_marker_dict'.format(mission_server_namespace))
        self.agent_library_dict = rospy.get_param('{}agent_library_dict'.format(mission_server_namespace))

        # __init__ of parent is called after above because function setup_menu,
        # which is called in __init__, is using information above.
        super(StaticAgentIntMarkerPublisher, self).__init__(topic_name, agent_names, mission_server_namespace)

    def handle_update_marker_status(self, req):
        # TODO: update color?
        rospy.loginfo("Do nothing for marker updates in dynamic agent markers")
        return UpdateMarkerStatusResponse()

    def setup_menu(self):
        """
        Overridden function because static agents are not used as subjects but rather as objects of actions
        :return:
        """
        # Menu is specific to topic name in case of 'static' agents. It is not dependent on agent_name
        menu_handler = interactive_markers.menu_handler.MenuHandler()

        # TODO: from here, we are using identical code blocks in TaskMarkerServer.
        add_action_submenu_handler = menu_handler.insert("Add Action")
        # Dictionary for parameters corresponding to each menu
        # Parameters: library_name, action_name, agent_name
        menu_num_parameters_dict = {}
        for library, action_parameter_int_marker_dict in sorted(self.topic_lib_action_int_marker_dict[self.topic_name].items()):
            library_submenu_handler = menu_handler.insert(library, parent=add_action_submenu_handler)
            agent_list = []
            for agent_name, library_name in self.agent_library_dict.iteritems():  # NOTE: agent_name in here refers to 'dynamic' agents
                if library_name == library:
                    agent_list.append(agent_name)
            agent_list.sort()
            for action_name, int_marker_info_list in sorted(action_parameter_int_marker_dict.items()):
                if len(int_marker_info_list) > 0:
                    action_submenu_handler = menu_handler.insert(action_name, parent=library_submenu_handler)
                    for agent_name in agent_list:
                        menu_num = menu_handler.insert(agent_name, parent=action_submenu_handler,
                                                       callback=self.add_action_callback)
                        menu_num_parameters_dict[menu_num] = [library, action_name, agent_name]
        # TODO: until here, we are using identical code blocks in TaskMarkerServer.

        for agent_name in self.agent_names:  # NOTE: agent_name in here refers to 'static' agents
            self.menu_handlers[agent_name] = menu_handler
        return menu_num_parameters_dict

    def init_markers(self):
        """
        Overridden function for automatic generation of interactive markers
        :return:
        """
        raise NotImplementedError

    def add_action_callback(self, feedback):
        """
        Overridden function because static agents are not used as subjects but rather as objects of actions
        :param feedback:
        :return:
        """
        # TODO: from here, we are using almost identical code blocks in TaskMarker2dPoseServer.
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
            library, action_name, agent_name = self.menu_num_parameters_dict[feedback.menu_entry_id]  # NOTE: agent_name here is 'dynamic' agents

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
            euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                              self.INT_MARKER.pose.orientation.y,
                                                              self.INT_MARKER.pose.orientation.z,
                                                              self.INT_MARKER.pose.orientation.w,))
            self.ROLL = euler[0]
            self.PITCH = euler[1]
            self.YAW = euler[2]

            marker_for_parametrization_list = self.topic_lib_action_int_marker_dict[self.topic_name][library][
                action_name]
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
                else:
                    rospy.logerr("ERROR: either 'members' or 'value' key is required for int_marker assignment!")
                    return

            add_action_req.action_content.parameters_yaml = yaml.dump(parameters_dict)
            resp = self.add_action_client(add_action_req)  # FIXME: Need to fix service names
            rospy.loginfo("Add new action %s with id %s" % (add_action_req.action_content.action_name, resp.action_id))

        self.server.applyChanges()
