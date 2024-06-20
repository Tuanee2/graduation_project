#ifndef MY_CUSTOM_DISPLAY_HPP_
#define MY_CUSTOM_DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_common/display.hpp>
#include "rviz_common/display_context.hpp"
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <vector>
#include <iostream>
#include <memory> // For smart pointers
#include <math.h>

class MyCustomDisplay : public rviz_common::Display
{
Q_OBJECT
public:
  MyCustomDisplay();
  virtual ~MyCustomDisplay();

protected:
  virtual void onInitialize() override;
  virtual void onEnable() override;
  virtual void onDisable() override;
  virtual void reset() override;

private:
  void cmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void subscribe();
  void unsubscribe();

  void createPoint(const geometry_msgs::msg::Point& point,const std::string& type);
  void connectPoints(const std::string& type);
  void clearPath();
  void clearWall();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;

  Ogre::MaterialPtr createOrGetPointMaterial(const std::string& type);
  Ogre::MaterialPtr createOrGetLineMaterial(const std::string& type);
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
  std::unique_ptr<rviz_common::properties::IntProperty> size_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> line_type_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> workspace_status_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> line_typeOfShape_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> line_Orientation_type_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> radius_property_;
  Ogre::MaterialPtr line_material_;
  std::vector<Ogre::Vector3> point_positions_path;
  std::vector<Ogre::Vector3> point_positions_wall;
  std::map<Ogre::SceneNode*, Ogre::ManualObject*> points_path_;
  std::map<Ogre::SceneNode*, Ogre::ManualObject*> points_wall_;
  std::vector<Ogre::ManualObject*> lines_path;
  std::vector<Ogre::ManualObject*> lines_wall;
  bool path_loop_;
};

#endif // MY_CUSTOM_DISPLAY_HPP_
