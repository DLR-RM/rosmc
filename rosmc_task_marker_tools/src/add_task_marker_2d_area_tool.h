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


#ifndef ROSMC_TASK_MARKER_TOOLS_ADD_TASK_MARKER_2D_AREA_TOOL_H
#define ROSMC_TASK_MARKER_TOOLS_ADD_TASK_MARKER_2D_AREA_TOOL_H

#include <OGRE/OgreVector3.h>

#include <ros/ros.h>
#include <rviz/tool.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rosmc_task_marker_tools
{

class QuadArea;

class AddTaskMarker2dAreaTool: public rviz::Tool
{
Q_OBJECT
public:
  AddTaskMarker2dAreaTool();
  ~AddTaskMarker2dAreaTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:
  // Members related to visualization during mouse interaction when this tool is activated
  QuadArea* moving_area_;
  Ogre::Vector3 pos_leftdown_;
  Ogre::Vector3 pos_leftduringdrag_;
  float offset_ground_;

  // Members related to publisher/subscriber and service server/client
  ros::NodeHandle nh_;
  ros::ServiceClient add_task_marker_2d_area_service_client_;
  std::string topic_name_;
};
}

#endif //ROSMC_TASK_MARKER_TOOLS_ADD_TASK_MARKER_2D_AREA_TOOL_H
