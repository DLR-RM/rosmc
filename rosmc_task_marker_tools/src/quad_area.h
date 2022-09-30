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


#ifndef ROSMC_TASK_MARKER_TOOLS_QUAD_AREA_H
#define ROSMC_TASK_MARKER_TOOLS_QUAD_AREA_H

#include <rviz/ogre_helpers/object.h>

namespace Ogre
{
class Any;
class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
}

namespace rosmc_task_marker_tools
{
class QuadArea: public rviz::Object
{
public:
  QuadArea( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = 0,
            float start_x = 0.0f, float start_y = 0.0f,
            float end_x = 0.3f, float end_y =  0.2f, float offset_ground = 0.0f );
  virtual ~QuadArea();

  void set( float start_x = 0.0f, float start_y = 0.0f,
            float end_x = 0.3f, float end_y =  0.2f, float offset_ground = 0.0f );

  virtual void setColor( float r, float g, float b, float a );

  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setScale( const Ogre::Vector3& scale );

  virtual const Ogre::Vector3& getPosition();
  const Ogre::Vector3& getStartPosition();
  const Ogre::Vector3& getEndPosition();
  virtual const Ogre::Quaternion& getOrientation();

  Ogre::SceneNode* getSceneNode();

  rviz::Shape* getCube();

  void setUserData( const Ogre::Any& data );

private:
  Ogre::SceneNode* scene_node_;
  rviz::Shape* cube_;

};

}  // end namespace rosmc_task_marker_tools

#endif //ROSMC_TASK_MARKER_TOOLS_QUAD_AREA_H
