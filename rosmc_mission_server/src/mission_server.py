#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

import time

from rosmc_interface_msgs.msg import Action, ActionStatus, AgentActions, SyncIDs
from rosmc_interface_msgs.srv import Mission, MissionRequest, RegisterToServer, RegisterToServerResponse, \
    RegisterToServerRequest, UpdateActionStatus, UpdateActionStatusResponse
from rosmc_msgs.msg import MarkerStatus, CurrentActions
from rosmc_msgs.srv import AddAction, AddActionResponse, EditAction, EditActionResponse, \
    SaveMission, SaveMissionResponse, LoadMission, LoadMissionResponse, NewMission, NewMissionResponse, \
    DeleteAction, DeleteActionResponse, ReorderActions, ReorderActionsResponse, \
    AddSync, AddSyncResponse, DeleteSync, DeleteSyncResponse, UpdateMarkerStatus, UpdateMarkerStatusRequest, \
    SaveIntMarkers, SaveIntMarkersRequest, LoadIntMarkers, LoadIntMarkersRequest, ExtendAddActionServices, \
    ExtendAddActionServicesResponse, RenameMarker, RenameMarkerResponse
import copy
import os
import pickle
import rospkg
import rospy
import uuid
from threading import RLock
from visualization_msgs.msg import InteractiveMarkerInit
import yaml
from agent_action_dict import parse_agent_actionlib_config, agent_action_dict, yaml2dict


class MissionServer(object):

    def __init__(self):
        ############################################################
        # Get the private namespace parameters from the parameter server:
        # set from either command line or launch file.
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rosmc_mission_server')
        agent_actionlib_config_path = rospy.get_param(
            'agent_actionlib_config_path',
            pkg_path + "/configs/agent_actionlib/turtlesim_agent_actionlib_config.yaml")
        parameter_group_config_folder = rospy.get_param(
            'parameter_group_config_folder', pkg_path + "/configs/parameter_group")  # without slash
        int_marker_config_folder = rospy.get_param(
            'int_marker_config_folder', pkg_path + "/configs/int_markers")  # without slash

        # Resolve environment variables in paths
        agent_actionlib_config_path = os.path.expandvars(agent_actionlib_config_path)
        parameter_group_config_folder = os.path.expandvars(parameter_group_config_folder)
        int_marker_config_folder = os.path.expandvars(int_marker_config_folder)

        ############################################################
        # Mutexes
        # self.update_action_status_mutex = False
        # rospy.loginfo("Set update_action_status_mutex")

        self.update_mission_lock = RLock()

        ############################################################
        # create core parameter dictionaries and set it as rosparam

        # TODO: merge self.agent_action_dict and self.agent_library_dict to self.agent_lib_action_dict
        # agent action dictionary
        self.agent_actionlib_config = parse_agent_actionlib_config(agent_actionlib_config_path)
        self.agent_action_dict = agent_action_dict(self.agent_actionlib_config)
        rospy.set_param('agent_action_dict', self.agent_action_dict)
        rospy.loginfo("Set rosparam with key: agent_action_dict")

        # agent library dict
        self.agent_library_dict = {}
        for agent_name in self.agent_actionlib_config:
            self.agent_library_dict[agent_name] = self.agent_actionlib_config[agent_name]['name']
        libraries = []
        for key, value in self.agent_library_dict.iteritems():
            if value not in libraries:
                libraries.append(value)
        rospy.set_param('agent_library_dict', self.agent_library_dict)
        rospy.loginfo("Set rosparam with key: agent_library_dict")

        # parameter groups
        self.lib_action_parameter_groups_dict = {}
        for lib in libraries:
            self.lib_action_parameter_groups_dict[lib] = yaml2dict(parameter_group_config_folder + '/' + lib + '.yaml')
        rospy.loginfo("Loaded configuration files for parameter groups of actions")
        rospy.set_param('lib_action_parameter_groups_dict', self.lib_action_parameter_groups_dict)
        rospy.loginfo("Set rosparam with key: lib_action_parameter_groups_dict")

        ############################################################
        # parameter dictionaries related to interactive markers
        use_interactive_markers = rospy.get_param('use_interactive_markers', False)
        self.use_interactive_markers = use_interactive_markers
        rospy.loginfo("Set rosparam use_interactive_markers as %s" % use_interactive_markers)

        if self.use_interactive_markers:
            # Create core parameter dictionaries and set it as rosparam

            # common information about int_markers
            self.common_int_marker = yaml2dict(int_marker_config_folder + '/common.yaml')
            rospy.set_param('common_int_marker', self.common_int_marker)

            # library action int_marker dict
            self.lib_action_int_marker_dict = {}
            for lib in libraries:
                action_int_marker_dict = yaml2dict(int_marker_config_folder + '/' + lib + '.yaml')
                # Assert yaml file is valid or not
                # Check if topic name in 'topics' is 'type'
                for action_name, int_marker_info_list in action_int_marker_dict.iteritems():
                    for int_marker_info in int_marker_info_list:
                        int_marker_type = int_marker_info['type']
                        possible_topic_names = [x['name'] for x in self.common_int_marker['topics'] if
                                                x['type'] == int_marker_type]
                        topic_names = int_marker_info['topics']
                        assert all([topic_name in possible_topic_names for topic_name in topic_names])
                # Add to dict
                self.lib_action_int_marker_dict[lib] = action_int_marker_dict
            rospy.loginfo("Loaded configuration files for parametrization of actions by interactive markers")
            # set it as rosparam
            rospy.set_param('lib_action_int_marker_dict', self.lib_action_int_marker_dict)
            rospy.loginfo("Set rosparam with key: lib_action_int_marker_dict")

            # topic library action int_marker dict (ONLY FOR CONVENIENCE)
            self.topic_lib_action_int_marker_dict = {}
            for topic in self.common_int_marker['topics']:
                topic_name = topic['name']
                topic_type = topic['type']
                lib_action_int_marker_dict = {}
                for library, action_int_marker_dict in self.lib_action_int_marker_dict.iteritems():
                    action_int_marker_dict_ = {}
                    for action, int_marker_list in action_int_marker_dict.iteritems():
                        is_topic_found = False
                        for int_marker_info in int_marker_list:
                            if is_topic_found:
                                break
                            for t in int_marker_info['topics']:
                                if is_topic_found:
                                    break
                                if t == topic_name:
                                    action_int_marker_dict_[action] = int_marker_list
                                    is_topic_found = True
                    lib_action_int_marker_dict[library] = action_int_marker_dict_
                self.topic_lib_action_int_marker_dict[topic_name] = lib_action_int_marker_dict
            # set it as rosparam
            rospy.set_param('topic_lib_action_int_marker_dict', self.topic_lib_action_int_marker_dict)
            rospy.loginfo("Set rosparam with key: topic_lib_action_int_marker_dict")

        ############################################################
        # initialize mission data
        # WARNING: DO NOT DIRECTLY CHANGE THE VALUE OF self.mission_req!
        #          USE THE FUNCTION self.set_mission_req(new_mission_req)
        self.mission_req = MissionRequest()
        rospy.loginfo("Initialize Mission Data")
        for key in self.agent_action_dict:
            agent_actions = AgentActions()
            agent_actions.agent_name = key
            self.mission_req.agents_actions.append(agent_actions)
            rospy.loginfo("... AgentActions for %s created" % key)
        rospy.loginfo("Initialize Mission Data finished")

        ############################################################
        # specify service servers for mission update
        # proxies are appended after checking the existence of external processes by calling register_to_mission_server service
        self.send_mission_service_proxies_full = []
        self.send_mission_service_proxies_sync = []
        self.send_mission_service_name = 'send_mission'

        ############################################################
        # advertise services to register external processes
        # so that they can receive mission update
        self.register_to_server_service = rospy.Service('register_to_mission_server', RegisterToServer, self.handle_register_to_server)

        ############################################################
        # advertise services to add a new action to mission_req
        self.add_action_service = rospy.Service('add_action', AddAction, self.handle_add_action)
        rospy.loginfo("Ready to add actions by calling services externally")

        ############################################################
        # advertise services to edit existing action
        self.edit_action_service = rospy.Service('edit_action', EditAction, self.handle_edit_action)
        rospy.loginfo("Ready to edit actions by calling service externally")

        ############################################################
        # advertise services to update action status
        # The process that updates action status is rafcon (or other software)
        self.update_action_status_service = rospy.Service('update_action_status', UpdateActionStatus, self.handle_update_action_status)
        rospy.loginfo("Ready to update action status by calling service externally")

        ############################################################
        # advertise services to delete an action from mission_req
        self.delete_action_service = rospy.Service('delete_action', DeleteAction, self.handle_delete_action)
        rospy.loginfo("Ready to delete actions by calling services externally")

        ############################################################
        # advertise services to reorder actions in mission_req
        # The process that reorders actions is mission control gui
        self.reorder_actions_service = rospy.Service('reorder_actions', ReorderActions, self.handle_reorder_actions)
        rospy.loginfo("Ready to reorder actions by calling services externally")

        ############################################################
        # advertise services to save, load, create new mission data
        self.save_mission_service = rospy.Service('save_mission', SaveMission, self.handle_save_mission)
        self.load_mission_service = rospy.Service('load_mission', LoadMission, self.handle_load_mission)
        self.new_mission_service = rospy.Service('new_mission', NewMission, self.handle_new_mission)

        ############################################################
        # advertise services to add/delete synchronization points for actions
        self.add_sync_service = rospy.Service('add_sync', AddSync, self.handle_add_sync)
        self.delete_sync_service = rospy.Service('delete_sync', DeleteSync, self.handle_delete_sync)

        ############################################################
        # create service proxy to update marker status
        if self.use_interactive_markers:
            self.update_marker_status_service_proxy_dict = {}
            for topic in self.common_int_marker['topics']:
                self.update_marker_status_service_proxy = rospy.ServiceProxy('{}/update_marker_status'.format(topic['name']),
                                                                             UpdateMarkerStatus)
                self.update_marker_status_service_proxy_dict[topic['name']] = self.update_marker_status_service_proxy

        ############################################################
        # create service proxy to save/load interactive markers
        if self.use_interactive_markers:
            self.save_int_markers_service_proxy_dict = {}
            self.load_int_markers_service_proxy_dict = {}
            for topic in self.common_int_marker['topics']:
                if str(topic['type']).startswith('TASK'):
                    self.save_int_markers_service_proxy_dict[topic['name']] = \
                        rospy.ServiceProxy('{}/save_int_markers'.format(topic['name']), SaveIntMarkers)
                    self.load_int_markers_service_proxy_dict[topic['name']] = \
                        rospy.ServiceProxy('{}/load_int_markers'.format(topic['name']), LoadIntMarkers)

        ############################################################
        # advertise service to rename marker in mission_req
        if self.use_interactive_markers:
            self.rename_marker_service = rospy.Service('rename_marker', RenameMarker, self.handle_rename_marker)

        ############################################################
        # create subscribers of marker update
        if self.use_interactive_markers:
            self.int_marker_update_full_subscriber_dict = {}
            for topic in self.common_int_marker['topics']:
                int_marker_update_full_subscriber = self.IntMarkerUpdateFullSubscriber(topic['name'])
                self.int_marker_update_full_subscriber_dict[topic['name']] = int_marker_update_full_subscriber

        ############################################################
        # create publisher of current actions for status visualization in rviz
        self.current_actions_publisher = rospy.Publisher('current_actions', CurrentActions, queue_size=10)
        rospy.Timer(rospy.Duration(1), self.current_actions_timer_callback)

        ############################################################
        rospy.loginfo("MissionServer is ready")

    class IntMarkerUpdateFullSubscriber:

        def __init__(self, topic):
            rospy.logdebug("Function IntMarkerUpdateFullSubscriber.__init__ is called")
            self.topic = topic
            self.int_marker_update_full_subscriber = rospy.Subscriber('{}/update_full'.format(topic), InteractiveMarkerInit,
                                                                      self.int_marker_update_full_callback)
            self.int_marker_name_list = None
            self.update_full_mutex = False

        def int_marker_update_full_callback(self, msg):
            rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback is called")

            rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback: Wait for mutex")
            while self.update_full_mutex:
                pass
            rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback: Mutex acquired")
            self.update_full_mutex = True

            int_marker_name_list = [marker.name for marker in msg.markers]
            int_marker_name_list.sort()
            # If marker is added or deleted,
            if self.int_marker_name_list != int_marker_name_list:
                # Update member self.int_marker_name_list
                self.int_marker_name_list = int_marker_name_list

            rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback: Mutex unlocked")
            self.update_full_mutex = False

    def handle_register_to_server(self, req):
        """
        Handle service call from external processes that request Mission Server to send mission updates.
        Service type: RegisterToServer.srv
        This function appends ServiceProxy to self.send_mission_service_proxies so that
        Mission Control can send mission updates to external processes.
        :param req: RegisterToServerRequest().
        :return: RegisterToServerResponse().
        """
        rospy.logdebug("Function handle_register_to_server is called")
        res = RegisterToServerResponse()
        res.mission_server_namespace = rospy.get_namespace()
        service_name = '{}{}'.format(req.namespace, self.send_mission_service_name)

        # Adjust service proxy list and mission to send depending on mode
        if req.mission_receive_mode == RegisterToServerRequest.FULL:
            send_mission_service_proxies = self.send_mission_service_proxies_full
            mission_req = self.mission_req
        elif req.mission_receive_mode == RegisterToServerRequest.SYNC:
            send_mission_service_proxies = self.send_mission_service_proxies_sync
            mission_req = self.synchronised_mission_req()
        else:
            rospy.logerr("Unexpected mission_receive_mode: {}".format(req))
            res.success = False
            return res

        # Process service request
        if service_name not in [x.resolved_name for x in send_mission_service_proxies]:
            # if the service does not exist, append the service proxy.
            service_proxy = rospy.ServiceProxy(service_name, Mission)
            send_mission_service_proxies.append(service_proxy)
            rospy.loginfo("Setting up service proxy %s" % service_name)
        else:
            # if the service already exists, skip appending and get the corresponding service proxy.
            index = [x.resolved_name for x in send_mission_service_proxies].index(service_name)
            service_proxy = send_mission_service_proxies[index]
            rospy.logwarn("Service proxy %s already exists. Skip appending." % service_name)
        # Send mission for initialization
        service_proxy(mission_req)
        rospy.loginfo("Initialization: Send mission via service %s" % service_proxy.resolved_name)
        res.success = True
        return res

    def handle_add_action(self, req):
        """
        Handle service call from external processes to add actions to self.mission_req.
        Service type: AddAction.srv
        :param req: AddActionRequest().
        :return: AddActionResponse().
        """
        rospy.logdebug("Function handle_add_action is called")
        new_action = Action()
        new_action_id = str(uuid.uuid4())
        new_action.action_id = new_action_id
        new_action.action_content = req.action_content

        res = AddActionResponse()

        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            is_agent_found = False
            for agent_actions in new_mission_req.agents_actions:
                if agent_actions.agent_name == req.agent_name:
                    agent_actions.actions.append(new_action)
                    rospy.loginfo("New action %s (id: %s, label: %s) will be added to agent %s"
                                  % (new_action.action_content.action_name, new_action_id, new_action.action_content.action_label, req.agent_name))
                    is_agent_found = True
                    break
            if is_agent_found:
                error_msg = self.set_mission_req(new_mission_req)
                res.action_id = new_action_id
                res.error_msg = error_msg
                return res
            else:
                error_msg = "Requested agent name does not exist in self.mission_req"
                rospy.logerr(error_msg)
                res.action_id = "None"
                res.error_msg = error_msg
                return res

    def handle_extend_add_action_services(self, req):
        """
        Handle service call from external processes (especially agent interactive marker publishers) to extend
        self.add_action_services.
        Constructor of AgentIntMarkerPublisher calls services and trigger this function.
        :param req: ExtendAddActionServicesRequest()
        :return: ExtendAddActionServicesResponse()
        """
        rospy.logdebug("Function handle_extend_add_action_services is called")
        for service_name in req.service_names:
            try:
                add_action = rospy.Service(service_name, AddAction, self.handle_add_action)
            except rospy.ServiceException:
                rospy.logwarn("Requested service " + service_name + " has already been advertised. Skip extension.")
                continue
            self.add_action_services.append(add_action)
            rospy.loginfo("Service %s is advertised" % service_name)
        return ExtendAddActionServicesResponse()

    def handle_edit_action(self, req):
        """
        Handle service call from external processes to edit existing actions in self.mission_req.
        Service type: EditAction.srv
        :param req: EditActionRequest().
        :return: EditActionResponse()
        """
        rospy.logdebug("Function handle_edit_action is called")
        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            is_id_found = False
            for i, agent_actions in enumerate(new_mission_req.agents_actions):
                for j, action in enumerate(agent_actions.actions):
                    if action.action_id == req.edited_action.action_id:
                        # Assure that the action_name of req.edited_action is available for agent_actions.agent_name
                        available_action_names = list(rospy.get_param('agent_action_dict/' + agent_actions.agent_name))
                        assert req.edited_action.action_content.action_name in available_action_names, \
                            "The requested action is not available for the requested agent."

                        new_mission_req.agents_actions[i].actions[j] = req.edited_action
                        rospy.loginfo("Existing action %s (id: %s, label: %s) of agent %s will be edited"
                                      % (action.action_content.action_name, action.action_id, action.action_content.action_label, agent_actions.agent_name))
                        is_id_found = True
                        break
            if is_id_found:
                error_msg = self.set_mission_req(new_mission_req)
            else:
                error_msg = "Requested action id does not exist in self.mission_req"
                rospy.logerr(error_msg)
            res = EditActionResponse()
            res.error_msg = error_msg
            return res

    def handle_update_action_status(self, req):
        """
        Handle service call from external processes to update action status in self.mission_req.
        Service type: UpdateActionStatus.srv
        :param req: UpdateActionStatusRequest()
        :return: UpdateActionStatusResponse()
        """
        rospy.loginfo("Function handle_update_action_status is called")

        # Wait for mutex to be False
        # while self.update_action_status_mutex:  # TODO: suspicious implementation. Sleep at least for several sec or msec.
        #     rospy.loginfo("Function handle_update_action_status: mutex is taken.")
        #     time.sleep(0.1)
        #     pass

        rospy.loginfo("acquiring lock")
        with self.update_mission_lock:
            rospy.loginfo("lock acquired")

            # Set mutex True
            # self.update_action_status_mutex = True

            with self.update_mission_lock:
                new_mission_req = copy.deepcopy(self.mission_req)
                is_id_found = False
                is_req_valid = False
                for i, agent_actions in enumerate(new_mission_req.agents_actions):
                    for j, action in enumerate(agent_actions.actions):
                        if action.action_id == req.action_id:
                            is_id_found = True
                            if action.action_content.status.value != req.updated_status.value:
                                is_req_valid = True
                                rospy.loginfo(
                                    "Status of existing action %s (id: %s) of agent %s will be updated \n %s -> %s"
                                    % (action.action_content.action_name, action.action_id, agent_actions.agent_name,
                                       str(action.action_content.status), str(req.updated_status)))
                                new_mission_req.agents_actions[i].actions[j].action_content.status = req.updated_status
                            break
                if is_id_found and is_req_valid:
                    rospy.loginfo("call self.set_mission_req")
                    error_msg = self.set_mission_req(new_mission_req)
                elif not is_id_found:
                    error_msg = "Requested action id does not exist in self.mission_req"
                    rospy.logerr(error_msg)
                elif not is_req_valid:
                    error_msg = "Requested status is the same as in self.mission_req. Update ignored."
                    rospy.logwarn(error_msg)
                else:
                    pass

                # Set mutex False
                rospy.loginfo("Function handle_update_action_status: release mutex")
                # self.update_action_status_mutex = False
                res = UpdateActionStatusResponse()
                res.error_msg = error_msg

                return res

    def handle_delete_action(self, req):
        """
        Handle service call from external processes to delete action from self.mission_req.
        Service type: DeleteAction.srv
        :param req: DeleteActionRequest()
        :return: DeleteActionResponse()
        """
        rospy.logdebug("Function handle_delete_action is called")

        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            is_id_found = False
            for i, agent_actions in enumerate(new_mission_req.agents_actions):
                for j, action in enumerate(agent_actions.actions):
                    if action.action_id == req.action_id:
                        is_id_found = True
                        rospy.loginfo("Action %s (id: %s) of agent %s will be deleted"
                                      % (action.action_content.action_name, action.action_id, agent_actions.agent_name))
                        del new_mission_req.agents_actions[i].actions[j]
                        break
            # Check if the action id to be deleted is included in sync list or not
            # If it is included, delete the synchronization point as well
            for i, sync_ids in enumerate(new_mission_req.sync_ids_list):
                if req.action_id in sync_ids.sync_ids:
                    if len(sync_ids.sync_ids) > 2:
                        # Delete only req.action_id from the sync_ids
                        del new_mission_req.sync_ids_list[i].sync_ids[
                            new_mission_req.sync_ids_list[i].sync_ids.index(req.action_id)]
                    else:
                        # Delete the entire sync_ids
                        del new_mission_req.sync_ids_list[i]
            if is_id_found:
                error_msg = self.set_mission_req(new_mission_req)
            else:
                error_msg = "Requested action id does not exist in self.mission_req"
                rospy.logerr(error_msg)
            res = DeleteActionResponse()
            res.error_msg = error_msg
            return res

    def handle_reorder_actions(self, req):
        rospy.logdebug("Function handle_reorder_actions is called")
        is_agent_found = False
        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            for i, agent_actions in enumerate(new_mission_req.agents_actions):
                if agent_actions.agent_name == req.agent_name:
                    is_agent_found = True
                    new_actions = [agent_actions.actions[j] for j in req.new_order]
                    new_mission_req.agents_actions[i].actions = new_actions
                    break
            if is_agent_found:
                error_msg = self.set_mission_req(new_mission_req)
            else:
                error_msg = "Requested agent does not exist in self.mission_req"
                rospy.logerr(error_msg)
            res = ReorderActionsResponse()
            res.error_msg = error_msg
            return res

    def handle_save_mission(self, req):
        """
        Handle service call from MissionControlGUI to save current mission data
        :param req: SaveMissionRequest()
        :return:
        """
        rospy.logdebug("Function handle_save_mission is called")
        if self.use_interactive_markers:
            self.save_task_interactive_markers(req.folder_path)
        self.save_mission_req(req.folder_path + '/mission.pkl')
        return SaveMissionResponse()

    def handle_load_mission(self, req):
        """
        Handle service call from MissionControlGUI to load mission data
        :param req: LoadMissionRequest()
        :return:
        """
        rospy.logdebug("Function handle_load_mission is called")
        if self.use_interactive_markers:
            self.load_task_interactive_markers(req.folder_path)
        error_msg = self.load_mission_req(req.folder_path + '/mission.pkl')
        res = LoadMissionResponse()
        res.error_msg = error_msg
        return res

    def save_task_interactive_markers(self, folder_path):
        rospy.logdebug("Function save_task_interactive_markers is called")
        for topic, service_proxy in self.save_int_markers_service_proxy_dict.iteritems():
            file_path = folder_path + '/' + topic
            req = SaveIntMarkersRequest()
            req.file_path = file_path
            service_proxy(req)

    def load_task_interactive_markers(self, folder_path):
        rospy.logdebug("Function load_task_interactive_markers is called")
        for topic, service_proxy in self.load_int_markers_service_proxy_dict.iteritems():
            file_path = folder_path + '/' + topic
            req = LoadIntMarkersRequest()
            req.file_path = file_path
            service_proxy(req)

    def handle_new_mission(self, req):
        """
        Handle service call from MissionControlGUI to create new mission data.
        :param req:
        :return:
        """
        rospy.logdebug("Function handle_new_mission is called")
        error_msg = self.new_mission_req()
        res = NewMissionResponse()
        res.error_msg = error_msg
        return res

    def current_actions_timer_callback(self, _event):
        """
        Callback function triggered by timer.
        This function is for publishing current actions by all agents.
        :param _event:
        :return:
        """
        current_actions = CurrentActions()
        for agent_actions in self.mission_req.agents_actions:
            agent_name = agent_actions.agent_name
            actions = agent_actions.actions
            is_active = False
            for action in reversed(actions):
                if action.action_content.status.value == ActionStatus.RUNNING \
                        or action.action_content.status.value == ActionStatus.SYNCWAITING \
                        or action.action_content.status.value == ActionStatus.PAUSED \
                        or action.action_content.status.value == ActionStatus.FAILURE \
                        or action.action_content.status.value == ActionStatus.SUCCESS:
                    current_actions.agent_names.append(agent_name)
                    current_actions.current_actions.append(action)
                    is_active = True
                    break
            if not is_active:
                for action in actions:
                    if action.action_content.status.value == ActionStatus.IDLE_SYNCHRONISED \
                            or action.action_content.status.value == ActionStatus.IDLE:
                        current_actions.agent_names.append(agent_name)
                        current_actions.current_actions.append(action)
                        break
        self.current_actions_publisher.publish(current_actions)

    def sanity_check(self, mission_req):
        mission_req_checked = copy.deepcopy(mission_req)
        for i, agent_actions in enumerate(mission_req_checked.agents_actions):
            for j, action in enumerate(agent_actions.actions):
                # Sanity check for specific action
                if action.action_content.action_name == 'setup_infrastructure':
                    mission_req_checked.agents_actions[i].actions[j] = self.sanity_check_setup_infrastructure(action)
        return mission_req_checked

    def sanity_check_setup_infrastructure(self, action):
        try:
            assert action.action_content.action_name == 'setup_infrastructure'
        except AssertionError:
            rospy.logerr("Unexpected action is passed to the function sanity_check_setup_infrastructure: {}".format(action))
        parameters_dict = yaml.safe_load(action.action_content.parameters_yaml)
        if 'line' in parameters_dict['pose']['marker'].lower():
            parameters_dict['formation']['value'] = 'line'
        elif 'rectangle' in parameters_dict['pose']['marker'].lower():
            parameters_dict['formation']['value'] = 'rectangle'
        else:
            rospy.logerr("Neither line of rectangle is included in name of LOFAR marker. Sanity check failed.")
        action.action_content.parameters_yaml = yaml.dump(parameters_dict)
        return action

    def set_mission_req(self, new_mission_req):
        """
        Set new MissionRequest() value to the member of this class (self.mission_req).
        :param new_mission_req: updated version of mission data
        :return: None
        """
        rospy.loginfo("acquiring lock")

        with self.update_mission_lock:
            rospy.loginfo("lock acquired")

            rospy.logdebug("Function set_mission_req is called")

            error_msg = ""

            # Check if mission is changed or not
            # NOTE: do not use != operator for rosmsg because results are always True in python2!
            is_mission_req_changed = not self.mission_req == new_mission_req
            is_sync_mission_req_changed = \
                not self.synchronised_mission_req() == self.synchronised_mission_req(new_mission_req)
            rospy.loginfo("is_sync_mission_req_changed:" + str(is_sync_mission_req_changed))
            rospy.loginfo("is_mission_req_changed: " + str(is_mission_req_changed))

            # Sanity check for the mission consistency
            new_mission_req = self.sanity_check(new_mission_req)

            # Send mission to external processes
            # NOTE: first send synchronized part because this fails if communication fails.
            #       The process which receives full mission (e.g. Mission Control GUI) visualizes updated results
            #       only if mission synchronization is successful.
            if is_sync_mission_req_changed:
                process_sent_dict = self.send_mission_req(new_mission_req, mode='sync')

                # print(process_sent_dict)  # Debug purpose only
                # Revert mission modification in agent if mission is failed to be synchronized
                for service_name in process_sent_dict:
                    if process_sent_dict[service_name]:
                        continue
                    # NOTE: Assume mission services are advertised under the namespace of agent_name
                    agent_name = service_name.split('/')[1]
                    msg = "Unable to synchronize mission to agent " + agent_name + ".\n"\
                        + "Revert mission modification in agent " + agent_name + ".\n"
                    rospy.logerr(msg)
                    error_msg += msg
                    for i in range(len(new_mission_req.agents_actions)):
                        if new_mission_req.agents_actions[i].agent_name != agent_name:
                            continue
                        for agent_actions in self.mission_req.agents_actions:
                            if agent_actions.agent_name != agent_name:
                                continue
                            new_mission_req.agents_actions[i] = agent_actions
                            break
                        break

            if is_mission_req_changed:
                process_sent_dict = self.send_mission_req(new_mission_req, mode='full')
                if not all(process_sent_dict.values()):
                    msg = "Abort setting new mission in Mission Server.\nRevert all mission modification requested.\n"
                    rospy.logerr(msg)
                    error_msg += msg
                    self.send_mission_req(self.mission_req, mode='sync')
                    self.send_mission_req(self.mission_req, mode='full')
                    return error_msg

            # update the value in Mission Server
            self.mission_req = new_mission_req
            rospy.loginfo("Set new value for self.mission_req.")
            rospy.loginfo(self.mission_req)

            # functions to be called just after the value is changed
            if self.use_interactive_markers:
                self.update_marker_status()

            return error_msg

    def send_mission_req(self, mission_req, mode='full'):
        """
        Call service in rafcon, mcg, etc. to send the updated mission_req.
        This function should only be called inside of set_mission_req().
        If mode == 'full', the entire mission_req is sent.
        Else if mode == 'sync', the synchronised part of the mission_req is sent.
        :return: process_sent_dict: key: process_name, value: boolean.
                value means if mission is successfully sent to the process_name.
        """
        rospy.logdebug("Function send_mission_req is called")

        # Prepare mission to be sent and services to be used depending on mode
        if mode == 'full':
            mission_ = mission_req
            send_mission_service_proxies = self.send_mission_service_proxies_full
        elif mode == 'sync':
            mission_ = self.synchronised_mission_req(mission_req)
            send_mission_service_proxies = self.send_mission_service_proxies_sync
        else:
            rospy.logerr("Unimplemented send_mission_req mode: " + str(mode))
            rospy.logerr("Abort sending mission.")
            return None

        process_sent_dict = {}
        for service_proxy in send_mission_service_proxies:
            service_name = service_proxy.resolved_name
            try:
                resp = service_proxy(mission_)
                rospy.loginfo("Send %s mission via service %s" % (mode, service_name))
                process_sent_dict[service_name] = True
            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
                process_sent_dict[service_name] = False
        return process_sent_dict

    def update_marker_status(self, mission_req=None):
        """
        Call service to send updates of the status of action in mission_req.
        This function should only be called inside of set_mission_req()
        If mission_req is None, self.mission_req is used.
        :return:
        """
        rospy.logdebug("Function update_marker_status is called")

        if mission_req is None:
            mission_req = copy.deepcopy(self.mission_req)

        topic_used_marker_names_dict = {}
        for topic in self.common_int_marker['topics']:
            topic_used_marker_names_dict[topic['name']] = []

        for agent_actions in mission_req.agents_actions:
            agent_name = agent_actions.agent_name
            actions = agent_actions.actions
            sorted_actions = self.sort_actions_by_marker_visualization_priority(actions)

            library = self.agent_library_dict[agent_name]
            action_int_marker_dict = self.lib_action_int_marker_dict[library]

            # update marker status from highest priority
            # if the marker is already used in higher priority, continue to next loop
            for action in sorted_actions:
                action_parameters = yaml.safe_load(action.action_content.parameters_yaml)
                for action_parameter_name, action_parameter_content in action_parameters.iteritems():
                    # Do the following procedure only if this action is using interactive markers
                    try:
                        action_parameter_name_marker = action_parameter_content['marker']
                    except KeyError:
                        continue

                    # Get which topic does this marker use
                    try:
                        topic_ = action_parameter_content['marker_topic']
                        assert topic_ is not None
                    except (KeyError, AssertionError):
                        continue

                    # If it is not updated yet,
                    if action_parameter_name_marker not in topic_used_marker_names_dict[topic_]:
                        # Update marker status
                        req = UpdateMarkerStatusRequest()
                        req.marker_name = action_parameter_name_marker
                        # NOTE: ActionStatus.msg and MarkerStatus.msg should use the same const for the same status
                        req.updated_status.value = action.action_content.status.value
                        self.update_marker_status_service_proxy_dict[topic_](req)

                        # Append it to topic_used_marker_names_dict
                        topic_used_marker_names_dict[topic_].append(action_parameter_name_marker)

        # Update status of marker that is not used in mission to NOT_ASSIGNED
        for topic, int_marker_update_full_subscriber in self.int_marker_update_full_subscriber_dict.iteritems():
            try:
                assert int_marker_update_full_subscriber.int_marker_name_list is not None
            except AssertionError:
                continue
            for marker_name in int_marker_update_full_subscriber.int_marker_name_list:
                if marker_name not in topic_used_marker_names_dict[topic]:
                    req = UpdateMarkerStatusRequest()
                    req.marker_name = marker_name
                    req.updated_status.value = MarkerStatus.NOT_ASSIGNED
                    self.update_marker_status_service_proxy_dict[topic](req)

    def sort_actions_by_marker_visualization_priority(self, actions):
        """
        Sort actions by the priority of visualization of markers.
        Priority:
            1. RUNNING
            2. SYNCWAITING
            3. PAUSED
            4. FAILURE
            5. SUCCESS
            6. IDLE_SYNCHRONISED
            7. IDLE
            8. anything else
            (9. NOT_ASSIGNED)
        :param actions:
        :return: sorted_actions
        """
        running_actions = []
        syncwaiting_actions = []
        paused_actions = []
        failure_actions = []
        success_actions = []
        idle_synchronised_actions = []
        idle_actions = []
        other_actions = []
        for action in actions:
            if action.action_content.status.value == ActionStatus.RUNNING:
                running_actions.append(action)
            elif action.action_content.status.value == ActionStatus.SYNCWAITING:
                syncwaiting_actions.append(action)
            elif action.action_content.status.value == ActionStatus.PAUSED:
                paused_actions.append(action)
            elif action.action_content.status.value == ActionStatus.FAILURE:
                failure_actions.append(action)
            elif action.action_content.status.value == ActionStatus.SUCCESS:
                success_actions.append(action)
            elif action.action_content.status.value == ActionStatus.IDLE_SYNCHRONISED:
                idle_synchronised_actions.append(action)
            elif action.action_content.status.value == ActionStatus.IDLE:
                idle_actions.append(action)
            else:
                other_actions.append(action)
        sorted_actions = running_actions + \
                         syncwaiting_actions + \
                         paused_actions + \
                         failure_actions + \
                         success_actions + \
                         idle_synchronised_actions + \
                         idle_actions + \
                         other_actions
        return sorted_actions

    def synchronised_mission_req(self, mission_req=None):
        """
        Returns synchronised part of mission_req.
        In concrete, this function appends actions until it reaches one whose status is IDLE.
        sync_ids_list is the same as in mission_req.
        If argument mission_req is not given, self.mission_req is used.
        :return:
        """
        rospy.logdebug("Function synchronised_mission_req is called")

        # Use self.mission_req as default
        if mission_req is None:
            mission_req = copy.deepcopy(self.mission_req)

        sync_mission_req = MissionRequest()
        sync_mission_req.sync_ids_list = mission_req.sync_ids_list
        for agent_actions in mission_req.agents_actions:
            sync_agent_actions = AgentActions()
            sync_agent_actions.agent_name = agent_actions.agent_name
            for action in agent_actions.actions:
                if action.action_content.status.value == ActionStatus.IDLE:
                    break
                else:
                    sync_agent_actions.actions.append(action)
            sync_mission_req.agents_actions.append(sync_agent_actions)
        return sync_mission_req

    def save_mission_req(self, file_path):
        """
        Save self.mission_req as a yaml file
        :param: file_path - full path for saving self.mission_req
        :return:
        """
        rospy.logdebug("Function save_mission_req is called")
        with open(file_path, 'wb') as outfile:
            pickle.dump(self.mission_req, outfile, pickle.HIGHEST_PROTOCOL)
        rospy.loginfo("Saved current mission data to %s" % file_path)

    def load_mission_req(self, file_path):
        """
        Load mission file, assign new ids to each action, and store it to self.mission_req
        :param file_path: - full path for loading mission data
        :return:
        """
        rospy.logdebug("Function load_mission_req is called")
        with open(file_path, 'rb') as stream:
            new_mission_req = pickle.load(stream)
            # Assign new id to each action
            for i, agent_actions in enumerate(new_mission_req.agents_actions):
                for j, action in enumerate(agent_actions.actions):
                    new_action_id = str(uuid.uuid4())
                    old_action_id = action.action_id
                    # Replace id in sync_ids_list if it has it
                    for k, sync_ids in enumerate(new_mission_req.sync_ids_list):
                        if old_action_id in sync_ids.sync_ids:
                            new_mission_req.sync_ids_list[k].sync_ids[sync_ids.sync_ids.index(old_action_id)] \
                                = new_action_id
                    new_mission_req.agents_actions[i].actions[j].action_id = new_action_id
            error_msg = self.set_mission_req(new_mission_req)
        rospy.loginfo("Loaded mission data from %s" % file_path)
        return error_msg

    def new_mission_req(self):
        """
        Clear self.mission_req
        :return:
        """
        rospy.logdebug("Function new_mission_req is called")
        new_mission_req = MissionRequest()
        for key in self.agent_action_dict:
            agent_actions = AgentActions()
            agent_actions.agent_name = key
            new_mission_req.agents_actions.append(agent_actions)
        error_msg = self.set_mission_req(new_mission_req)
        rospy.loginfo("Cleared mission data")
        return error_msg

    def handle_add_sync(self, req):
        rospy.logdebug("Function handle_add_sync is called")
        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            # If any action in new request is already included in self,mission_req, merge the request
            is_already_exist = False
            for i, sync_ids in enumerate(new_mission_req.sync_ids_list):
                for j, sync_id in enumerate(sync_ids.sync_ids):
                    if sync_id in req.ids:
                        is_already_exist = True
                        new_mission_req.sync_ids_list[i].sync_ids.extend(req.ids)
                        new_mission_req.sync_ids_list[i].sync_ids = list(set(new_mission_req.sync_ids_list[i].sync_ids))
                        rospy.loginfo("Extended synchronization for actions")
                        error_msg = self.set_mission_req(new_mission_req)
                        break
                else:
                    continue
                break
            # Otherwise, simply append new synchronization point
            if not is_already_exist:
                new_sync = SyncIDs()
                new_sync.sync_ids.extend(req.ids)
                new_mission_req.sync_ids_list.append(new_sync)
                rospy.loginfo("Added new synchronization for actions")
                error_msg = self.set_mission_req(new_mission_req)
            res = AddSyncResponse()
            res.error_msg = error_msg
            return res

    def handle_delete_sync(self, req):
        rospy.logdebug("Function handle_delete_sync is called")
        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            for id in req.ids:
                for i, sync_ids in enumerate(new_mission_req.sync_ids_list):
                    if id in sync_ids.sync_ids:
                        print(new_mission_req.sync_ids_list[i])
                        del new_mission_req.sync_ids_list[i]
            rospy.loginfo("Deleted synchronization for actions")
            error_msg = self.set_mission_req(new_mission_req)
            res = DeleteSyncResponse()
            res.error_msg = error_msg
            return res

    def handle_rename_marker(self, req):
        rospy.logdebug("Function handle_rename_marker is called")
        with self.update_mission_lock:
            new_mission_req = copy.deepcopy(self.mission_req)
            for i, agent_actions in enumerate(new_mission_req.agents_actions):
                for j, action in enumerate(agent_actions.actions):
                    parameters = yaml.safe_load(action.action_content.parameters_yaml)
                    edited_parameters = copy.deepcopy(parameters)
                    for parameter_name, parameter_content in parameters.iteritems():
                        # Do the following procedure only if this action is using interactive markers
                        try:
                            parameter_name_marker = parameter_content['marker']
                            parameter_name_marker_topic = parameter_content['marker_topic']
                        except KeyError:
                            continue
                        if req.name_old == parameter_name_marker and req.marker_topic == parameter_name_marker_topic:
                            rospy.loginfo("Rename marker from {} to {} used in parameter {} of action {}".format(req.name_old, req.name_new, parameter_name, action.action_content.action_name))
                            edited_parameters[parameter_name]['marker'] = req.name_new
                    new_mission_req.agents_actions[i].actions[j].action_content.parameters_yaml = yaml.dump(edited_parameters)
            error_msg = self.set_mission_req(new_mission_req)
            res = RenameMarkerResponse()
            res.error_msg = error_msg
            return res


if __name__ == '__main__':
    rospy.init_node('mission_server')
    try:
        server = MissionServer()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
