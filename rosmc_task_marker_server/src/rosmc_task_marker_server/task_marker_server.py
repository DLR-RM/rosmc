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
import pickle
import rospy
import Tkinter
import tkSimpleDialog

from rosmc_msgs.msg import MarkerStatus
from rosmc_msgs.srv import AddAction, UpdateMarkerStatus, SaveIntMarkers, SaveIntMarkersResponse, \
    LoadIntMarkers, LoadIntMarkersResponse, UpdateIntMarker, UpdateIntMarkerResponse, GetIntMarker, GetIntMarkerResponse, \
    RenameMarker, RenameMarkerRequest, RenameMarkerResponse

import interactive_markers.interactive_marker_server
import interactive_markers.menu_handler
import visualization_msgs.msg

from std_msgs.msg import ColorRGBA

COLOR_NOT_ASSIGNED = ColorRGBA(round(124 / 255.0, 2), round(124 / 255.0, 2), round(124 / 255.0, 2), 0.35)
COLOR_IDLE = ColorRGBA(round(124 / 255.0, 2), round(124 / 255.0, 2), round(124 / 255.0, 2), 0.5)
COLOR_IDLE_SYNCHRONISED = ColorRGBA(round(233 / 255.0, 2), round(233 / 255.0, 2), round(233 / 255.0, 2), 0.5)
COLOR_RUNNING = ColorRGBA(round(133 / 255.0, 2), round(255 / 255.0, 2), round(167 / 255.0, 2), 0.5)
COLOR_SUCCESS = ColorRGBA(round(61 / 255.0, 2), round(121 / 255.0, 2), round(128 / 255.0, 2), 0.5)
COLOR_FAILURE = ColorRGBA(round(255 / 255.0, 2), round(47 / 255.0, 2), 0.0, 0.5)
COLOR_PAUSED = ColorRGBA(round(128 / 255.0, 2), round(102 / 255.0, 2), round(83 / 255.0, 2), 0.5)
COLOR_SYNCWAITING = ColorRGBA(round(120 / 255.0), round(117 / 255.0, 2), round(71 / 255.0), 0.5)


def status_to_color(status_value):
    if status_value == MarkerStatus.NOT_ASSIGNED:
        return COLOR_NOT_ASSIGNED
    elif status_value == MarkerStatus.IDLE:
        return COLOR_IDLE
    elif status_value == MarkerStatus.IDLE_SYNCHRONISED:
        return COLOR_IDLE_SYNCHRONISED
    elif status_value == MarkerStatus.RUNNING:
        return COLOR_RUNNING
    elif status_value == MarkerStatus.SUCCESS:
        return COLOR_SUCCESS
    elif status_value == MarkerStatus.FAILURE:
        return COLOR_FAILURE
    elif status_value == MarkerStatus.PAUSED:
        return COLOR_PAUSED
    elif status_value == MarkerStatus.SYNCWAITING:
        return COLOR_SYNCWAITING
    else:
        rospy.logerr("Unimplemented marker status {} is requested in status_to_color()".format(status_value))
        return None


def is_color_equal(color1, color2, threshold=0.01):
    if abs(color1.r - color2.r) >= threshold  or \
        abs(color1.g - color2.g) >= threshold or \
        abs(color1.b - color2.b) >= threshold or \
        abs(color1.a - color2.a) >= threshold:
        return False
    else:
        return True


def color_to_status(color):
    if is_color_equal(color, COLOR_NOT_ASSIGNED):
        return MarkerStatus.NOT_ASSIGNED
    elif is_color_equal(color, COLOR_IDLE):
        return MarkerStatus.IDLE
    elif is_color_equal(color, COLOR_IDLE_SYNCHRONISED):
        return MarkerStatus.IDLE_SYNCHRONISED
    elif is_color_equal(color, COLOR_RUNNING):
        return MarkerStatus.RUNNING
    elif is_color_equal(color, COLOR_SUCCESS):
        return MarkerStatus.SUCCESS
    elif is_color_equal(color, COLOR_FAILURE):
        return MarkerStatus.FAILURE
    elif is_color_equal(color, COLOR_PAUSED):
        return MarkerStatus.PAUSED
    elif is_color_equal(color, COLOR_SYNCWAITING):
        return MarkerStatus.SYNCWAITING
    else:
        rospy.logerr("No corresponding status defined in MarkerStatus with color {}".format(color))
        return None


def status_to_is_movable(status_value):
    if status_value == MarkerStatus.NOT_ASSIGNED:
        return True
    elif status_value == MarkerStatus.IDLE:
        return True
    elif status_value == MarkerStatus.IDLE_SYNCHRONISED:
        return False
    elif status_value == MarkerStatus.RUNNING:
        return False
    elif status_value == MarkerStatus.SUCCESS:
        return False
    elif status_value == MarkerStatus.FAILURE:
        return False
    elif status_value == MarkerStatus.PAUSED:
        return False
    elif status_value == MarkerStatus.SYNCWAITING:
        return False
    else:
        rospy.logerr("Unimplemented marker status is requested in status_to_color()")
        return False


def status_to_is_deletable(status_value):
    if status_value == MarkerStatus.NOT_ASSIGNED:
        return True
    elif status_value == MarkerStatus.IDLE:
        return False
    elif status_value == MarkerStatus.IDLE_SYNCHRONISED:
        return False
    elif status_value == MarkerStatus.RUNNING:
        return False
    elif status_value == MarkerStatus.SUCCESS:
        return False
    elif status_value == MarkerStatus.FAILURE:
        return False
    elif status_value == MarkerStatus.PAUSED:
        return False
    elif status_value == MarkerStatus.SYNCWAITING:
        return False
    else:
        rospy.logerr("Unimplemented marker status is requested in status_to_color()")
        return False


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


class TaskMarkerServer(object):

    def __init__(self):
        """
        Abstract class for task marker server
        :param
        """
        self.topic_prefix = 'interactive_markers_'
        self.mission_server_namespace = '/mission_control/'
        self.frame_id = rospy.get_param('frame_id', 'map')  # use relative namespace for defining frame_id used in ROSMC
        self.menu_handler = interactive_markers.menu_handler.MenuHandler()
        self.menu_handler_with_rename = interactive_markers.menu_handler.MenuHandler()
        self.menu_handler_with_delete = interactive_markers.menu_handler.MenuHandler()

        # The following members for interactive marekrs
        self.INT_MARKER = None
        self.ROLL = None
        self.PITCH = None
        self.YAW = None

        # The following members should be manually assigned in __init__() of child class
        self.marker_type = None
        self.add_marker_service_class = None

        # The following members should be automatically assigned by self.onInitialize() in child class
        self.topic_name = None
        self.server = None
        self.add_task_marker_service = None
        self.update_marker_status_service = None
        self.save_int_markers_service = None
        self.load_int_markers_service = None
        self.add_action_service_proxy = None
        self.update_int_marker_service = None
        self.get_int_marker_service = None
        self.rename_marker_service_proxy = rospy.ServiceProxy('{}/rename_marker'.format(self.mission_server_namespace), RenameMarker)  # to Mission Server
        self.marker_counter = None
        self.topic_lib_action_int_marker_dict = None
        self.agent_library_dict = None
        self.menu_num_parameters_dict = None

    def handle_add_task_marker(self, req):
        raise NotImplementedError()

    def handle_update_marker_status(self, req):
        raise NotImplementedError()

    def add_action_callback(self, feedback):
        raise NotImplementedError()

    def process_feedback(self, feedback):
        raise NotImplementedError()

    def handle_update_int_marker(self, req):
        raise NotImplementedError()

    def rename_marker_menu_callback(self, feedback):
        if feedback.event_type == visualization_msgs.msg.InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo("Rename marker menu item is clicked")
            root = Tkinter.Tk()
            root.withdraw()
            name_new = tkSimpleDialog.askstring("Rename Marker", "Enter a new name")
            if name_new is None:
                return
            req = RenameMarkerRequest()
            req.name_old = feedback.marker_name
            req.name_new = name_new
            req.marker_topic = self.topic_name
            self.handle_rename_marker(req)

    def rename_marker(self, name_old, name_new):
        """
        Function to rename marker in this server.
        input:
          - name_old (str): a name of the marker to be renamed
          - name_new (str): a new name
        output:
          - error_msg (str): an error message. if no error occurred, empty string.
        """
        raise NotImplementedError()

    def handle_rename_marker(self, req):
        error_msg = self.rename_marker(req.name_old, req.name_new)
        res = self.rename_marker_service_proxy(req)
        error_msg += res.error_msg
        return RenameMarkerResponse(error_msg)

    def onInitialize(self):
        self.topic_name = self.topic_prefix + self.marker_type
        self.server = interactive_markers.interactive_marker_server.InteractiveMarkerServer(self.topic_name)
        self.add_task_marker_service = rospy.Service('{}/add_task_marker'.format(self.topic_name),
                                                     self.add_marker_service_class, self.handle_add_task_marker)
        self.update_marker_status_service = rospy.Service('{}/update_marker_status'.format(self.topic_name),
                                                          UpdateMarkerStatus, self.handle_update_marker_status)
        self.save_int_markers_service = rospy.Service('{}/save_int_markers'.format(self.topic_name),
                                                      SaveIntMarkers, self.handle_save_int_markers)
        self.load_int_markers_service = rospy.Service('{}/load_int_markers'.format(self.topic_name),
                                                      LoadIntMarkers, self.handle_load_int_markers)
        self.add_action_service_proxy = rospy.ServiceProxy('{}add_action'.format(self.mission_server_namespace),
                                                           AddAction)
        self.update_int_marker_service = rospy.Service('{}/update_int_marker'.format(self.topic_name),
                                                       UpdateIntMarker, self.handle_update_int_marker)
        self.get_int_marker_service = rospy.Service('{}/get_int_marker'.format(self.topic_name),
                                                    GetIntMarker, self.handle_get_int_marker)
        self.rename_marker_service = rospy.Service('{}/rename_marker'.format(self.topic_name),
                                                   RenameMarker, self.handle_rename_marker)
        self.marker_counter = 0  # used for naming marker

        # Interactive markers information
        self.topic_lib_action_int_marker_dict = rospy.get_param('{}topic_lib_action_int_marker_dict'.format(self.mission_server_namespace))
        self.agent_library_dict = rospy.get_param('{}agent_library_dict'.format(self.mission_server_namespace))

        # Marker visualization parameters
        self.offset_ground = rospy.get_param('{}task_marker_offset_from_ground'.format(self.mission_server_namespace), 0.0)
        self.offset_controller = self.offset_ground
        self.marker_scale = rospy.get_param('{}task_marker_scale'.format(self.mission_server_namespace), 1.0)

        # Menu Setup
        self.menu_num_parameters_dict = self.setup_menu(self.menu_handler)

        # Menu with rename option
        self.setup_menu(self.menu_handler_with_rename)
        self.menu_handler_with_rename.insert("Rename Marker", callback=self.rename_marker_menu_callback)

        # Menu with rename and delete option
        self.setup_menu(self.menu_handler_with_delete)
        self.menu_handler_with_delete.insert("Rename Marker", callback=self.rename_marker_menu_callback)
        self.menu_handler_with_delete.insert("Delete", callback=self.delete_callback)

    def setup_menu(self, menu_handler):
        # Dictionary for parameters corresponding to each menu
        # Parameters: library_name, action_name, agent_name
        add_action_submenu_handler = menu_handler.insert("Add Action")
        menu_num_parameters_dict = {}
        for library, action_parameter_int_marker_dict in sorted(self.topic_lib_action_int_marker_dict[self.topic_name].items()):
            library_submenu_handler = menu_handler.insert(library, parent=add_action_submenu_handler)
            agent_list = []
            for agent_name, library_name in self.agent_library_dict.iteritems():
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
        return menu_num_parameters_dict

    def handle_save_int_markers(self, req):
        name_int_marker_dict = {}
        for name, marker_context in self.server.marker_contexts.iteritems():
            name_int_marker_dict[name] = marker_context.int_marker
        with open(req.file_path + '.pkl', 'wb') as outfile:
            pickle.dump(name_int_marker_dict, outfile, pickle.HIGHEST_PROTOCOL)
        rospy.loginfo("Saved self.marker_contexts to %s" % req.file_path)
        return SaveIntMarkersResponse()

    def handle_load_int_markers(self, req):
        # Load the interactive marker information
        with open(req.file_path + '.pkl', 'rb') as stream:
            new_name_int_marker_dict = pickle.load(stream)
            self.server.clear()
            for name, int_marker in new_name_int_marker_dict.iteritems():
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

        # Reset the global counter of markers. This will be properly incremented in self.add_task_marker()
        self.marker_counter = 0

        return LoadIntMarkersResponse()

    def handle_get_int_marker(self, req):
        try:
            int_marker = self.server.get(req.marker_name)
        except:
            rospy.logerr("Requested marker name '%s' is not found in server." % req.marker_name)
        res = GetIntMarkerResponse()
        res.int_marker = int_marker
        return res

    def delete_callback(self, feedback):
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
            rospy.loginfo("Deleted Marker %s" % feedback.marker_name)
        self.server.applyChanges()
