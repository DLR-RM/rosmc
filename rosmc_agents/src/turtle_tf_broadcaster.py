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
import tf
import turtlesim.msg


class TurtleTFBroadcaster(object):
    """
    Class that subscribes turtle position and orientation, calculates corresponding Pose, and publish as TF
    """

    def __init__(self):
        # Get the private namespace parameters from the parameter server:
        # set from either command line or launch file.
        rate = rospy.get_param('~rate', 10.0)
        self.z = rospy.get_param('~z', 0.0)

        # Subscriber
        rospy.Subscriber(rospy.get_name() + '/pose', turtlesim.msg.Pose, self.pose_sub_cb)

        # broadcaster of pose to tf of agent relative to "map"
        self.tf_broadcaster = tf.TransformBroadcaster()

    def pose_sub_cb(self, turtle_pose):
        yaw = turtle_pose.theta
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.tf_broadcaster.sendTransform((turtle_pose.x, turtle_pose.y, self.z),
                                          quaternion,
                                          rospy.Time.now(),
                                          rospy.get_name(),
                                          "map")


# Main function.
if __name__ == '__main__':
    rospy.init_node('turtle1')
    try:
        TurtleTFBroadcaster()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()