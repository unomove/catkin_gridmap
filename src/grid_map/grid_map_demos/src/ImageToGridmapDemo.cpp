/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/ImageToGridmapDemo.hpp"


namespace grid_map_demos {

ImageToGridmapDemo::ImageToGridmapDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(grid_map::GridMap({"elevation"})),
      mapInitialized_(false)
{
  readParameters();
  map_.setBasicLayers({"elevation"});
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmapDemo::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  occupancyPublisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  // get_map_service_ = nodeHandle_.advertiseService("static_map", &ImageToGridmapDemo::mapCallback, this);
}

ImageToGridmapDemo::~ImageToGridmapDemo()
{
}

bool ImageToGridmapDemo::readParameters()
{
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image"));
  nodeHandle_.param("resolution", resolution_, 0.03);
  nodeHandle_.param("min_height", minHeight_, 0.0);
  nodeHandle_.param("max_height", maxHeight_, 1.0);
  return true;
}

void ImageToGridmapDemo::imageCallback(const sensor_msgs::Image& msg)
{
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
             map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    // mapInitialized_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
  // grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);

  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
  gridMapPublisher_.publish(mapMessage);

  // convert to occupany grid
  grid_map::GridMapRosConverter::toOccupancyGrid(map_, "elevation", minHeight_, maxHeight_, occupancyGrid);
  occupancyPublisher_.publish(occupancyGrid);

  // update map_resp_, no need in real use.
  // grid_map::GridMapRosConverter::toGetMapResponse(map_, "elevation", minHeight_, maxHeight_, map_resp_);
}

} /* namespace */
