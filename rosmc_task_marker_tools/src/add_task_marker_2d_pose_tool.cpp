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

#include <tf2/LinearMath/Quaternion.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/tool_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

#include <rosmc_msgs/AddTaskMarker.h>
#include <rosmc_msgs/MarkerStatus.h>

#include "add_task_marker_2d_pose_tool.h"

namespace rosmc_task_marker_tools
{

AddTaskMarker2dPoseTool::AddTaskMarker2dPoseTool()
: Tool()
, moving_node_( NULL )
, topic_name_( "interactive_markers_2d_pose" )
, offset_ground_( 0.0f )
, marker_scale_( 1.0f )
{

}

AddTaskMarker2dPoseTool::~AddTaskMarker2dPoseTool()
{
  // Delete moving nodes
  scene_manager_->destroySceneNode( moving_node_ );
  delete moving_arrow_;
}

void AddTaskMarker2dPoseTool::onInitialize()
{
  // Initialize service client
  add_task_marker_2d_pose_service_client_ = nh_.serviceClient<rosmc_msgs::AddTaskMarker>( topic_name_ + "/add_task_marker" );

  // Get offset_ground rosparam
  nh_.getParam("task_marker_offset_from_ground", offset_ground_);
  nh_.getParam("task_marker_scale", marker_scale_);

  // Set up moving_node_ that follows mouse position during processMouseEvent
  mesh_resource_ = "package://rosmc_task_marker_tools/media/flag.dae";
  if ( rviz::loadMeshFromResource( mesh_resource_ ).isNull() )
  {
    ROS_ERROR( "AddTaskMarker2dPoseTool: failed to load model resource from '%s'", mesh_resource_.c_str() );
    return;
  }
  moving_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( mesh_resource_ );
  moving_node_->attachObject( entity );
  moving_node_->setPosition( Ogre::Vector3( 0.0f, 0.0f, offset_ground_ ) );
  moving_node_->setScale( Ogre::Vector3( marker_scale_, marker_scale_, marker_scale_) );
  moving_node_->setVisible( false );

  // Set up moving_arrow_ that follows mouse position during processMouseEvent
  moving_arrow_ = new rviz::Arrow( scene_manager_, scene_manager_->getRootSceneNode(), 1.5f, 0.2f, 0.5f, 0.35f );
  moving_arrow_->setPosition( Ogre::Vector3( 0.0f, 0.0f, offset_ground_ ) );
  moving_arrow_->setOrientation( Ogre::Quaternion( 0.707f, 0.0f, -0.707f, 0.0f ) );
  moving_arrow_->setScale( Ogre::Vector3( marker_scale_, marker_scale_, marker_scale_) );
  moving_arrow_->setColor( 0.8f, 0.8f, 0.8f, 1.0f );
  moving_arrow_->getSceneNode()->setVisible( false );
}

void AddTaskMarker2dPoseTool::activate()
{
  if ( moving_node_ && moving_arrow_ )
  {
    // Make moving scene nodes visible
    moving_node_->setVisible( true );
    moving_arrow_->getSceneNode()->setVisible( true );
  }
}

void AddTaskMarker2dPoseTool::deactivate()
{
  if ( moving_node_ && moving_arrow_ )
  {
    // Make moving scene nodes invisible
    moving_node_->setVisible( false );
    moving_arrow_->getSceneNode()->setVisible( false );
  }
}

int AddTaskMarker2dPoseTool::processMouseEvent( rviz::ViewportMouseEvent &event )
{
  if ( !moving_node_ || !moving_arrow_ )
  {
    return Render;
  }

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

  // If mouse is pointing at XY plane,
  if ( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection) )
  {
    // Make moving scene node visible
    moving_node_->setVisible( true );
    moving_arrow_->getSceneNode()->setVisible( true );

    // Raise intersection altitude a little for visual purpose
    intersection.z = intersection.z + offset_ground_;

    // Set the position of moving scene node at intersection
    moving_node_->setPosition( intersection );
    moving_arrow_->setPosition( intersection );

    // If left-clicked,
    if ( event.leftDown() )
    {
      // Change the color of moving_arrow_ so that the user can understand that they can change its direction.
      moving_arrow_->setColor( 0.0f, 0.8f, 0.0f, 1.0f );

      // Remember the left-clicked position so that the flag does not move anymore.
      pos_leftdown_ = intersection;
    }

    // If left-dragged,
    else if ( event.type == QEvent::MouseMove && event.left() )
    {
      Ogre::Vector3 cur_pos;
      if ( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                              ground_plane,
                                              event.x, event.y, cur_pos ))
      {
        // Remember the current mouse position during left-dragging.
        pos_leftduringdrag_ = cur_pos;

        // Lock the position of moving nodes at the leftDown position.
        moving_node_->setPosition( pos_leftdown_ );
        moving_arrow_->setPosition( pos_leftdown_ );

        // Calculate the target directions of arrow
        angle_leftduringdrag_ = atan2( pos_leftduringdrag_.y - pos_leftdown_.y, pos_leftduringdrag_.x - pos_leftdown_.x );

        // We need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );
        moving_arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian( angle_leftduringdrag_ ), Ogre::Vector3::UNIT_Z ) * orient_x );
        moving_node_->setOrientation( 1.0f, 0.0f, 0.0f, angle_leftduringdrag_ );
      }
    }

    // If left-click-up
    else if ( event.leftUp() )
    {
      rosmc_msgs::AddTaskMarker srv;

      geometry_msgs::Pose pose;
      pose.position.x = pos_leftdown_.x;
      pose.position.y = pos_leftdown_.y;

      tf2::Quaternion quaternion;
      quaternion.setRPY( 0, 0, angle_leftduringdrag_ );
      pose.orientation.x = quaternion.x();
      pose.orientation.y = quaternion.y();
      pose.orientation.z = quaternion.z();
      pose.orientation.w = quaternion.w();

      srv.request.pose = pose;

      srv.request.status.value = rosmc_msgs::MarkerStatus::NOT_ASSIGNED;
      if ( add_task_marker_2d_pose_service_client_.call( srv ) )
      {
        ROS_INFO( "New 2D Pose Marker '%s' is added to task marker server", srv.response.name.c_str() );
        return Render | Finished;
      } else {
        ROS_ERROR( "Failed to call service add_task_marker_2d_pose_service_client_" );
        return Render;
      }
    }
  }
  // If mouse is not pointing at XY plane,
  else{
    // Make moving scene node invisible
    moving_node_->setVisible( false );
    moving_arrow_->getSceneNode()->setVisible( false );
  }
  return Render;
}

void AddTaskMarker2dPoseTool::load( const rviz::Config &config )
{

}

void AddTaskMarker2dPoseTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
}

}  // end namespace rosmc_task_marker_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rosmc_task_marker_tools::AddTaskMarker2dPoseTool, rviz::Tool )
