
/*
 * Copyright TODO
 */

#ifndef RVIZ_COLORED_MESH_DISPLAY_H
#define RVIZ_COLORED_MESH_DISPLAY_H

#include "rviz/display.h"
#include "rviz/frame_manager.h" // TODO ?

#include <gamesh_bridge/GameshMesh.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/ColorRGBA.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>
#include <OGRE/OgreLight.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class Axes;
class RenderPanel;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class VectorProperty;
class StringProperty;
}

namespace rviz
{

class ColoredMeshDisplayCustom: public rviz::Display, public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  ColoredMeshDisplayCustom();
  virtual ~ColoredMeshDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
//  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();
//  virtual void fixedFrameChanged();

private Q_SLOTS:
  void updateMeshProperties();
  void updateLightProperties();
  void updateTopic();
  void updateName();
  virtual void updateQueueSize();

protected:
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

private:
  void clear();
//  void updateStatus(); // TODO ?

//  void updateMesh( const shape_msgs::Mesh::ConstPtr& mesh );
  void updateMesh( const gamesh_bridge::GameshMesh::ConstPtr& mesh );

  RosTopicProperty* mesh_topic_property_;
  FloatProperty* mesh_alpha_property_;
  ColorProperty* mesh_color_property_;

  VectorProperty* light_position_property_;
  ColorProperty* light_diffuse_color_property_;
  ColorProperty* light_specular_color_property_;

//  shape_msgs::Mesh last_mesh_;
  gamesh_bridge::GameshMesh last_mesh_;

  ros::NodeHandle nh_;

  Ogre::SceneNode* mesh_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::Light* lightGV_;
  Ogre::MaterialPtr mesh_material_;

  ros::Subscriber mesh_sub_;

  bool initialized_;

  boost::mutex mesh_mutex_;
};

} // namespace rviz

#endif
