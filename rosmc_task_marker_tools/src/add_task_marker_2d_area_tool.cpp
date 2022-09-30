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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/geometry.h>
#include <rviz/tool_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

#include <rosmc_msgs/AddTaskMarkerArea.h>
#include <rosmc_msgs/MarkerStatus.h>

#include "add_task_marker_2d_area_tool.h"
#include "quad_area.h"

namespace rosmc_task_marker_tools
{

AddTaskMarker2dAreaTool::AddTaskMarker2dAreaTool()
: Tool()
, moving_area_( NULL )
, topic_name_( "interactive_markers_2d_area" )
, offset_ground_( 0.0f )
{

}

AddTaskMarker2dAreaTool::~AddTaskMarker2dAreaTool()
{
  // Delete moving nodes
  delete moving_area_;
}

void AddTaskMarker2dAreaTool::onInitialize()
{
  // Initialize service client
  add_task_marker_2d_area_service_client_ = nh_.serviceClient<rosmc_msgs::AddTaskMarkerArea>( topic_name_ + "/add_task_marker" );

  // Get offset_ground rosparam
  nh_.getParam("task_marker_offset_from_ground", offset_ground_);

  // Set up moving_node_ that follows mouse position during proessMouseEvent
  moving_area_ = new QuadArea( scene_manager_, NULL, 0.0f, 0.0f, 0.05f, 0.05f, offset_ground_ );
  moving_area_->setColor( 0.0f, 0.5f, 0.0f, 1.0f );
  moving_area_->getSceneNode()->setVisible( false );
}

void AddTaskMarker2dAreaTool::activate()
{
  if ( moving_area_ )
  {
    // Make moving scene node visible
    moving_area_->getSceneNode()->setVisible( true );
  }
}

void AddTaskMarker2dAreaTool::deactivate()
{
  if ( moving_area_ )
  {
    // Make moving scene node invisible
    moving_area_->getSceneNode()->setVisible( false );
  }
}

int AddTaskMarker2dAreaTool::processMouseEvent( rviz::ViewportMouseEvent &event )
{
  if (!moving_area_)
  {
    return Render;
  }

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

  // If mouse is pointing at XY plane,
  if (rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
  {
    // Make moving scene node visible
    moving_area_->getSceneNode()->setVisible( true );

    // Set the position of moving scene node at intersection
    moving_area_->set( intersection.x,
                       intersection.y,
                       intersection.x + 0.05f,
                       intersection.y + 0.05f,
                       offset_ground_ );

    if (event.leftDown())
    {
      // Store the position on xy plane when clicked
      pos_leftdown_ = intersection;
    } else if (event.type == QEvent::MouseMove && event.left())
    {
      // update visualization while dragging
      Ogre::Vector3 cur_pos;
      if (rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                             ground_plane,
                                             event.x, event.y, cur_pos ))
      {
        pos_leftduringdrag_ = cur_pos;
        moving_area_->set( pos_leftdown_.x,
                           pos_leftdown_.y,
                           pos_leftduringdrag_.x,
                           pos_leftduringdrag_.y,
                           offset_ground_ );
      }
    } else if (event.leftUp())
    {
      float assigned_x_size = std::abs( pos_leftdown_.x - pos_leftduringdrag_.x );
      float assigned_y_size = std::abs( pos_leftdown_.y - pos_leftduringdrag_.y );
      float assigned_x_center = ( pos_leftdown_.x + pos_leftduringdrag_.x ) / 2.0f;
      float assigned_y_center = ( pos_leftdown_.y + pos_leftduringdrag_.y ) / 2.0f;
      // Create a new area only if there is decent amount of size
      if ((assigned_x_size > 0.05f) && (assigned_y_size > 0.05f))
      {
        rosmc_msgs::AddTaskMarkerArea srv;

        geometry_msgs::Pose pose;
        pose.position.z = 0.0f;
        // vertex 1
        pose.position.x = pos_leftdown_.x;
        pose.position.y = pos_leftdown_.y;
        srv.request.poses.push_back(pose);
        // vertex 2
        pose.position.x = pos_leftduringdrag_.x;
        pose.position.y = pos_leftdown_.y;
        srv.request.poses.push_back(pose);
        // vertex 3
        pose.position.x = pos_leftduringdrag_.x;
        pose.position.y = pos_leftduringdrag_.y;
        srv.request.poses.push_back(pose);
        // vertex 4
        pose.position.x = pos_leftdown_.x;
        pose.position.y = pos_leftduringdrag_.y;
        srv.request.poses.push_back(pose);

        srv.request.status.value = rosmc_msgs::MarkerStatus::NOT_ASSIGNED;
        if ( add_task_marker_2d_area_service_client_.call(srv) )
        {
          ROS_INFO( "New 2D Area Marker '%s' is added to task marker server", srv.response.name.c_str() );
          return Render | Finished;
        } else
        {
          ROS_ERROR( "Failed to call service add_task_marker_2d_area_service_client_" );
          return Render;
        }
      } else
      {
        ROS_WARN_STREAM( "Area is not created. Area size is too small." );
        ROS_WARN_STREAM( "Selected x size: " << assigned_x_size << ", y size: " << assigned_y_size );
      }

      return Render | Finished;
    }
  }
    // If mouse is not pointing at XY plane,
  else
  {
    // Make moving scene node invisible
    moving_area_->getSceneNode()->setVisible( false );
  }
  return Render;
}

void AddTaskMarker2dAreaTool::load( const rviz::Config &config )
{

}

void AddTaskMarker2dAreaTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
}

}  // end namespace rosmc_task_marker_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rosmc_task_marker_tools::AddTaskMarker2dAreaTool, rviz::Tool )