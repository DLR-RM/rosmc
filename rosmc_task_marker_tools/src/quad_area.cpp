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


#include <ros/console.h>
#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <sstream>

#include "quad_area.h"

namespace rosmc_task_marker_tools
{

QuadArea::QuadArea( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                    float start_x, float start_y,
                    float end_x, float end_y, float offset_ground)
    : Object( scene_manager )
{
  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  cube_ = new rviz::Shape( rviz::Shape::Cube, scene_manager_, scene_node_ );

  set( start_x, start_y, end_x, end_y, offset_ground );
  setOrientation( Ogre::Quaternion::IDENTITY );
}

QuadArea::~QuadArea()
{
  delete cube_;
  scene_manager_->destroySceneNode( scene_node_->getName() );
}

void QuadArea::set( float start_x, float start_y,
                    float end_x, float end_y, float offset_ground )
{
  float size_x = end_x - start_x;
  float size_y = end_y - start_y;
  scene_node_->setScale( Ogre::Vector3( size_x, size_y, 0.1f ) );
  scene_node_->setPosition( Ogre::Vector3( (end_x + start_x) / 2.0f,
                                           (end_y + start_y) / 2.0f,
                                           offset_ground ) );
}

void QuadArea::setColor( float r, float g, float b, float a )
{
  cube_->setColor( Ogre::ColourValue(r, g, b, a) );
}

void QuadArea::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void QuadArea::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void QuadArea::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( scale );
}

const Ogre::Vector3& QuadArea::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Vector3& QuadArea::getStartPosition()
{
  const Ogre::Vector3& center_pos = scene_node_->getPosition();
  const Ogre::Vector3& scale = scene_node_->getScale();
  const float start_pos_x = center_pos.x - scale.x / 2.0f;
  const float start_pos_y = center_pos.y - scale.y / 2.0f;
  const float start_pos_z = 0.0f;
  const Ogre::Vector3& start_pos = Ogre::Vector3( start_pos_x, start_pos_y, start_pos_z );
  // FIXME: For some reason, the following stream is necessary to obtain correct value.
  // ROS_INFO_STREAM("");
  // ROS_INFO_STREAM( "QuadArea  Start Pos: " << start_pos.x << ", " << start_pos.y );
  return start_pos;
}

const Ogre::Vector3& QuadArea::getEndPosition()
{
  const Ogre::Vector3& center_pos = scene_node_->getPosition();
  const Ogre::Vector3& scale = scene_node_->getScale();
  const float end_pos_x = center_pos.x + scale.x / 2.0f;
  const float end_pos_y = center_pos.y + scale.y / 2.0f;
  const float end_pos_z = 0.0f;
  const Ogre::Vector3& end_pos = Ogre::Vector3( end_pos_x, end_pos_y, end_pos_z );
  // FIXME: For some reason, the following stream is necessary to obtain correct value.
  // ROS_INFO_STREAM("");
  // ROS_INFO_STREAM( "QuadArea  End Pos: " << end_pos.x << ", " << end_pos.y );
  return end_pos;
}

const Ogre::Quaternion& QuadArea::getOrientation()
{
  return scene_node_->getOrientation();
}

Ogre::SceneNode* QuadArea::getSceneNode()
{
  return scene_node_;
}

rviz::Shape* QuadArea::getCube()
{
  return cube_;
}

void QuadArea::setUserData( const Ogre::Any& data )
{
  cube_->setUserData( data );
}

}  // end namespace rosmc_task_marker_tools