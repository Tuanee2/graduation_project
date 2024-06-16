#include "../include/myCustomDisplay.hpp"
#include <pluginlib/class_list_macros.hpp>

using namespace rviz_common;

MyCustomDisplay::MyCustomDisplay():
    color_property_(new properties::ColorProperty("Color", QColor(255, 0, 0),
                                                  "Color of the point.", this)),
    size_property_(new properties::IntProperty("Size", 10,
                                               "Size of the point.", this)),
    line_type_property_(new properties::EnumProperty("Line Type","Path","Choose between wall or path.",this)),

    workspace_status_property_(new properties::EnumProperty("Workspace status","Incomplete","Choose between Incomplete or Completed.",this)),
    
    line_typeOfShape_property_(new properties::EnumProperty("Line Type of shape", "Straight",
                                                     "Choose between a straight line or a curved line.", this)),
    line_Orientation_type_property_(new properties::EnumProperty("Orientation Type", "UpsideDown",
                                                                 "Choose between a UpsideDown type or UpRight type.",this)),
    radius_property_(new properties::FloatProperty("Radius", 1.0,
                                                   "Radius of the curve (only used if 'Curved' is selected).", this))
{
    line_type_property_->addOption("Path",0);
    line_type_property_->addOption("Wall",1);
    workspace_status_property_->addOption("Incomplete",0);
    workspace_status_property_->addOption("Completed",1);
    line_typeOfShape_property_->addOption("Straight", 0);
    line_typeOfShape_property_->addOption("Curved", 1);
    line_Orientation_type_property_->addOption("UpsideDown", 0);
    line_Orientation_type_property_->addOption("UpRight", 1);
}

MyCustomDisplay::~MyCustomDisplay()
{
    unsubscribe();
    clearPath();
    clearWall();
}

void MyCustomDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (ros_node_abstraction) {
    node_ = ros_node_abstraction->get_raw_node();
  }
}

void MyCustomDisplay::onEnable()
{
  subscribe();
}

void MyCustomDisplay::onDisable()
{
  unsubscribe();
}

void MyCustomDisplay::reset()
{
  rviz_common::Display::reset();
  unsubscribe();
  subscribe();
}

void MyCustomDisplay::subscribe()
{
  if (!node_) {
    return; // Early return if the node is not initialized
  }
  cmd_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "cmdToControl", 10,
    std::bind(&MyCustomDisplay::cmdCallback, this, std::placeholders::_1));
  
  point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 10,
    std::bind(&MyCustomDisplay::pointCallback, this, std::placeholders::_1));
}

void MyCustomDisplay::unsubscribe()
{
  if (cmd_sub_) {
    cmd_sub_.reset();
  }
  if (point_sub_) {
    point_sub_.reset();
  }
}

void MyCustomDisplay::cmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    switch (msg->data) {
        case 2:
            if (!point_positions_path.empty()) {
                clearPath();
                std::cout << "clear path" << std::endl;
            }
            break;
        case 4:
            if (!point_positions_wall.empty()) {
                clearWall();
                std::cout << "clear wall" << std::endl;
            }
            break;
        default:
            std::cout << "Received unhandled cmd: " << msg->data << std::endl;
            break;
    }
}

void MyCustomDisplay::clearPath(){
    for (auto& point : points_path_) {
        scene_manager_->destroyManualObject(point.second);
        scene_node_->removeAndDestroyChild(point.first);
    }

    point_positions_path.clear();

    for (auto& line : lines_path) {
        scene_manager_->destroyManualObject(line);
    }
    lines_path.clear();
}

void MyCustomDisplay::clearWall(){
    for (auto& point : points_wall_) {
        scene_manager_->destroyManualObject(point.second);
        scene_node_->removeAndDestroyChild(point.first);
    }
    points_wall_.clear();

    for (auto& line : lines_wall) {
        scene_manager_->destroyManualObject(line);
    }
    lines_wall.clear();
}

void MyCustomDisplay::pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    
    if (!msg) {
        std::cerr << "Received invalid message." << std::endl;
        return;
    }

    
    std::string type;
    if(line_type_property_->getOptionInt() == 0){
        type = "Path";
        point_positions_path.push_back(Ogre::Vector3(msg->point.x, msg->point.y, 0));
        if(workspace_status_property_->getOptionInt() == 0){
            createPoint(msg->point,type);
            connectPoints(type);
        }else{
            createPoint(msg->point,type);
            connectPoints(type);
        }
    }else{
        type = "Wall";

        if(workspace_status_property_->getOptionInt() == 0){
            point_positions_wall.push_back(Ogre::Vector3(msg->point.x, msg->point.y, 0));
            createPoint(msg->point,type);
        }else{
            if (!point_positions_wall.empty()) {
                point_positions_wall.push_back(Ogre::Vector3(point_positions_wall.front().x, point_positions_wall.front().y, 0));
            }
        }
        connectPoints(type);
    }
}

void MyCustomDisplay::createPoint(const geometry_msgs::msg::Point& point, const std::string& type) {
    Ogre::SceneNode* node = scene_node_->createChildSceneNode();
    Ogre::ManualObject* pointObj = scene_manager_->createManualObject();
    
    pointObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    pointObj->position(point.x, point.y, 0);
    pointObj->colour(color_property_->getOgreColor());
    pointObj->end();
    
    pointObj->setMaterial(0, createOrGetPointMaterial(type));
    node->attachObject(pointObj);
    if(type == "Path"){
        points_path_[node] = pointObj;
    }else{
        points_wall_[node] = pointObj;
    }
    
}

Ogre::MaterialPtr MyCustomDisplay::createOrGetPointMaterial(const std::string& type) {
    std::string materialName = "PointMaterial_" + type;
    if (Ogre::MaterialManager::getSingleton().resourceExists(materialName)) {
        return Ogre::MaterialManager::getSingleton().getByName(materialName);
    } else {
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
            materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setReceiveShadows(false);
        material->getTechnique(0)->getPass(0)->setDiffuse(color_property_->getOgreColor());
        material->getTechnique(0)->getPass(0)->setAmbient(color_property_->getOgreColor());
        material->getTechnique(0)->getPass(0)->setPointSize(size_property_->getInt());
        return material;
    }
}

void MyCustomDisplay::connectPoints(const std::string& type) {

    std::vector<Ogre::Vector3>& points = (type == "Path") ? point_positions_path : point_positions_wall;
    std::vector<Ogre::ManualObject*>& lines = (type == "Path") ? lines_path : lines_wall;

    if (points.size() < 2) {
        return;
    }

    for (const auto& line : lines) {
        scene_manager_->destroyManualObject(line);
    }
    lines.clear();

    for (size_t i = 0; i < points.size() - 1; ++i) {
        const Ogre::Vector3& p1 = points[i];
        const Ogre::Vector3& p2 = points[i + 1];
        
        Ogre::ManualObject* line = scene_manager_->createManualObject();
        line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

        if (line_typeOfShape_property_->getOptionInt() == 0) { // Straight line
            line->position(p1);
            line->position(p2);
            
        }

        line->colour(color_property_->getOgreColor());
        line->end();
        line->setMaterial(0, createOrGetLineMaterial(type));
        scene_node_->attachObject(line);
        lines.push_back(line);
    }
}

Ogre::MaterialPtr MyCustomDisplay::createOrGetLineMaterial(const std::string& type) {
    std::string materialName = "LineMaterial_" + type;

    if (Ogre::MaterialManager::getSingleton().resourceExists(materialName)) {
        return Ogre::MaterialManager::getSingleton().getByName(materialName);
    }

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setReceiveShadows(false);
    if(type == "Wall"){
        material->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    }else{
        material->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 0.0f));
    }
    material->getTechnique(0)->getPass(0)->setLineWidth(20.0f);
    return material;
}

PLUGINLIB_EXPORT_CLASS(MyCustomDisplay, rviz_common::Display)