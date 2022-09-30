//
// The BSD 3-Clause License
//
// Copyright (c) 2022, DLR-RM All rights reserved.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation 
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors 
//    may be used to endorse or promote products derived from this software 
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Contributors:
// Ryo Sakagami <ryo.sakagami@dlr.de>
//

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>
#include <rviz/frame_manager.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "camera_pose_publisher_panel.h"


namespace rosmc_status_monitor
{

CameraPosePublisherPanel::CameraPosePublisherPanel( QWidget *parent )
: rviz::Panel( parent )
{
  // Set up publisher for camera pose in rviz OGRE scene
  pose_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>( "rviz/camera_pose_stamped", 10 );

  // Set up timer for publishing camera pose
  timer_ = nh_.createTimer( ros::Duration(0.01), &CameraPosePublisherPanel::timerCallback, this );
}

CameraPosePublisherPanel::~CameraPosePublisherPanel()
{
  pose_stamped_publisher_.shutdown();
}

void CameraPosePublisherPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void CameraPosePublisherPanel::load( const rviz::Config &config )
{
  rviz::Panel::load( config );
}

void CameraPosePublisherPanel::timerCallback( const ros::TimerEvent &event )
{
  Ogre::Quaternion quaternion = vis_manager_->getViewManager()->getCurrent()->getCamera()->getOrientation();
  Ogre::Vector3 position = vis_manager_->getViewManager()->getCurrent()->getCamera()->getPosition();
  QVariant target_frame = vis_manager_->getViewManager()->getCurrent()->subProp( "Target Frame" )->getValue();
  std::string target_frame_str = target_frame.toString().toStdString();
  if ( target_frame.isValid() )
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.position.x = position.x;
    pose_stamped.pose.position.y = position.y;
    pose_stamped.pose.position.z = position.z;
    pose_stamped.pose.orientation.x = quaternion.x;
    pose_stamped.pose.orientation.y = quaternion.y;
    pose_stamped.pose.orientation.z = quaternion.z;
    pose_stamped.pose.orientation.w = quaternion.w;

    if ( target_frame_str == "<Fixed Frame>" )
    {
      pose_stamped.header.frame_id = vis_manager_->getFrameManager()->getFixedFrame();
    } else
    {
      pose_stamped.header.frame_id = target_frame_str;
    }
    pose_stamped_publisher_.publish( pose_stamped );
  }
}

}  // end namespace rosmc_status_monitor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rosmc_status_monitor::CameraPosePublisherPanel, rviz::Panel )