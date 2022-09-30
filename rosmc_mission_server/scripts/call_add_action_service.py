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

import rospy
import yaml
from rosmc_msgs.srv import AddAction, AddActionRequest
from rosmc_interface_msgs.msg import SyncIDs, AgentActions, Action, ActionContent, ActionStatus


def execute(agent_name):
    rospy.init_node(agent_name)
    service_name = '/mission_control/add_action'
    print "wait for service: " + str(service_name)
    rospy.wait_for_service(service_name)

    try:
        add_action_request = rospy.ServiceProxy(service_name, AddAction)

        action_status1 = ActionStatus(1, "")
        action_status2 = ActionStatus(2, "")
        params = {'turtle_name': {'data_type': 'str', 'value': 'turtle1'},
                #   'pose': {'data_type': 'list', 'value': [1.0, 8.0, 45.0]},
                  'x_pos': {'data_type': 'float', 'value': 1.0},
                  'y_pos': {'data_type': 'float', 'value': 8.0},
                  'yaw': {'data_type': 'float', 'value': 0.785}}
        params = yaml.dump(params)
        action_content0 = ActionContent("move_to_pose", "", False, True, False, action_status2, params)
        params = {'turtle_name': {'data_type': 'str', 'value': 'turtle1'},
                  'x_pos': {'data_type': 'float', 'value': 5.0},
                  'x_scale': {'data_type': 'float', 'value': 4.0},
                  'y_pos': {'data_type': 'float', 'value': 5.0},
                  'y_scale': {'data_type': 'float', 'value': 3.0}}
        params = yaml.dump(params)
        action_content1 = ActionContent("explore", "", True, False, False, action_status1, params)
        params = {'turtle_name': {'data_type': 'str', 'value': 'turtle1'},
                  'x_pos': {'data_type': 'float', 'value': 1.0},
                  'y_pos': {'data_type': 'float', 'value': 8.0}}
        params = yaml.dump(params)
        action_content2 = ActionContent("move_to_position", "", False, True, False, action_status2, params)
        params = {'turtle_name': {'data_type': 'str', 'value': 'turtle1'},
                  'x_pos': {'data_type': 'float', 'value': 4.0},
                  'y_pos': {'data_type': 'float', 'value': 2.0}}
        params = yaml.dump(params)
        action_content3 = ActionContent("move_to_position_backward", "", False, False, True, action_status1, params)
        params = {'turtle_name': {'data_type': 'str', 'value': 'turtle1'},
                  'x_pos': {'data_type': 'float', 'value': 7.0},
                  'y_pos': {'data_type': 'float', 'value': 3.0}}
        params = yaml.dump(params)
        action_content4 = ActionContent("take_measurements", "", True, True, False, action_status1, params)
        params = {'turtle_name': {'data_type': 'str', 'value': 'turtle1'},
                  'turtle_to_follow': {'data_type': 'str', 'value': 'turtle2'}}
        params = yaml.dump(params)
        action_content5 = ActionContent("follow_turtle", "", False, True, True, action_status2, params)
        params = {'list_pose': {'data_type': 'list', 'value': [
                {
                    "pitch": 0.0,
                    "roll": 0.0,
                    "x": 5.0,
                    "y": 7.0,
                    "yaw": 0.0,
                    "z": 0.0
                },
                {
                    "pitch": 0.0,
                    "roll": 0.0,
                    "x": 2.0,
                    "y": 7.0,
                    "yaw": 0.0,
                    "z": 0.0
                },
                {
                    "pitch": 0.0,
                    "roll": 0.0,
                    "x": 2.0,
                    "y": 3.0,
                    "yaw": 0.0,
                    "z": 0.0
                },
                {
                    "pitch": 0.0,
                    "roll": 0.0,
                    "x": 5.0,
                    "y": 3.0,
                    "yaw": 0.0,
                    "z": 0.0
                }
            ]}}
        params = yaml.dump(params)
        action_content6 = ActionContent("mapping", "", False, True, True, action_status2, params)
        req0 = AddActionRequest("turtle1", action_content0)
        req1 = AddActionRequest("turtle1", action_content1)
        req2 = AddActionRequest("turtle1", action_content2)
        req3 = AddActionRequest("turtle1", action_content3)
        req4 = AddActionRequest("turtle1", action_content4)
        req5 = AddActionRequest("turtle1", action_content5)
        req6 = AddActionRequest("turtle1", action_content6)

        for req in [req0, req1, req2, req3, req4, req5, req6]:
            resp = add_action_request(req)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e


if __name__ == "__main__":
    execute('mission_caller')
