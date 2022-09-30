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
#include <rviz/tool_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

#include <rosmc_msgs/AddTaskMarker.h>
#include <rosmc_msgs/MarkerStatus.h>

#include "add_task_marker_2d_position_tool.h"

namespace rosmc_task_marker_tools
{

AddTaskMarker2dPositionTool::AddTaskMarker2dPositionTool()
: Tool()
, moving_node_( NULL )
, topic_name_( "interactive_markers_2d_position" )
, offset_ground_( 0.0f )
, marker_scale_( 1.0f )
{

}

AddTaskMarker2dPositionTool::~AddTaskMarker2dPositionTool()
{
  // Delete moving nodes
  scene_manager_->destroySceneNode( moving_node_ );
}

void AddTaskMarker2dPositionTool::onInitialize()
{
  // Initialize service client
  add_task_marker_2d_position_service_client_ = nh_.serviceClient<rosmc_msgs::AddTaskMarker>( topic_name_ + "/add_task_marker" );

  // Get offset_ground rosparam
  nh_.getParam("task_marker_offset_from_ground", offset_ground_);
  nh_.getParam("task_marker_scale", marker_scale_);

  // Set up moving_node_ that follows mouse position during proessMouseEvent
  mesh_resource_ = "package://rosmc_task_marker_tools/media/flag.dae";
  if ( rviz::loadMeshFromResource( mesh_resource_ ).isNull() )
  {
    ROS_ERROR( "AddTaskMarker2dPositionTool: failed to load model resource from '%s'", mesh_resource_.c_str() );
    return;
  }
  moving_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( mesh_resource_ );
  moving_node_->attachObject( entity );
  moving_node_->setPosition( Ogre::Vector3( 0.0f, 0.0f, offset_ground_ ) );
  moving_node_->setScale( Ogre::Vector3( marker_scale_, marker_scale_, marker_scale_) );
  moving_node_->setVisible( false );
}

void AddTaskMarker2dPositionTool::activate()
{
  if ( moving_node_ )
  {
    // Make moving scene node visible
    moving_node_->setVisible( true );
  }
}

void AddTaskMarker2dPositionTool::deactivate()
{
  if ( moving_node_ )
  {
    // Make moving scene node invisible
    moving_node_->setVisible( false );
  }
}

int AddTaskMarker2dPositionTool::processMouseEvent( rviz::ViewportMouseEvent &event )
{
  if ( !moving_node_ )
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

    // Raise intersection altitude a little for visual purpose
    intersection.z = intersection.z + offset_ground_;

    // Set the position of moving scene node at intersection
    moving_node_->setPosition( intersection );

    // If left-clicked,
    if ( event.leftDown() )
    {
      // Remember the left-clicked position so that the flag does not move anymore.
      pos_leftdown_ = intersection;
    }

    // Call service to add interactive marker
    else if ( event.leftUp() )
    {
      rosmc_msgs::AddTaskMarker srv;

      geometry_msgs::Pose pose;
      pose.position.x = pos_leftdown_.x;
      pose.position.y = pos_leftdown_.y;

      srv.request.pose = pose;

      srv.request.status.value = rosmc_msgs::MarkerStatus::NOT_ASSIGNED;
      if ( add_task_marker_2d_position_service_client_.call(srv) )
      {
        ROS_INFO( "New 2D Position Marker '%s' is added to task marker server", srv.response.name.c_str() );
        return Render | Finished;
      } else {
        ROS_ERROR( "Failed to call service add_task_marker_2d_position_service_client_" );
        return Render;
      }
    }
  }
  // If mouse is not pointing at XY plane,
  else
  {
    // Make moving scene node invisible
    moving_node_->setVisible( false );
  }
  return Render;
}

void AddTaskMarker2dPositionTool::load( const rviz::Config &config )
{

}

void AddTaskMarker2dPositionTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
}

}  // end namespace rosmc_task_marker_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rosmc_task_marker_tools::AddTaskMarker2dPositionTool, rviz::Tool )