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

import geometry_msgs.msg
import std_msgs.msg

from rosmc_agents.dynamic_agent_int_marker_publisher import DynamicAgentIntMarkerPublisher


class TurtlesIntMarkerPublisher(DynamicAgentIntMarkerPublisher):

    def __init__(self, topic_name, agent_names):
        super(TurtlesIntMarkerPublisher, self).__init__(topic_name, agent_names)

    def init_markers(self):
        """
        TODO: modify this loop to achieve automatic generation of interactive markers.
              It may require to use external config file, which describes parameters and paths to meshes.
              Current version is only for turtles.
        """
        # Create interactive markers for each agent
        for i, agent_name in enumerate(self.agent_names):
            position = geometry_msgs.msg.Point()
            position.x = 0.0
            position.y = 0.0
            position.z = 0.0
            calib_pose = geometry_msgs.msg.Pose()
            calib_pose.position.x = 0.0
            calib_pose.position.y = 0.0
            calib_pose.position.z = 0.0
            frame_id = agent_name

            # For turtles located in decimal number in agent_names, use cube as Interactive Marker
            if i % 2 == 0:
                mesh_path = "package://rosmc_agents/meshes/cube.stl"
                calib_pose.orientation.w = 0.707
                calib_pose.orientation.x = 0.707
                calib_pose.orientation.y = 0.0
                calib_pose.orientation.z = 0.0
                scale = 0.45

            else:
                # For turtles located in odd number in agent_names, use sphere as Interactive Marker
                mesh_path = "package://rosmc_agents/meshes/sphere.stl"
                calib_pose.orientation.w = 0.707
                calib_pose.orientation.x = 0.707
                calib_pose.orientation.y = 0.0
                calib_pose.orientation.z = 0.0
                scale = 0.65

            self.add_agent_marker(position, calib_pose, frame_id, agent_name, mesh_path, scale, std_msgs.msg.ColorRGBA(0.5, 0.5, 0.5, 1.0))


if __name__ == '__main__':
    rospy.init_node('turtles_int_marker_publisher')

    # TODO: use external configuration file for agent names
    topic_name = 'interactive_markers_agents'
    agent_names = ['turtle1', 'turtle2']

    try:
        server = TurtlesIntMarkerPublisher(topic_name, agent_names)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
