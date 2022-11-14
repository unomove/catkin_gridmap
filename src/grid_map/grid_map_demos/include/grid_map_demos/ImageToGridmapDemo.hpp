/*
 * ImageToGridmapDemo.hpp
 *
 *  Created on: May 4, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <string>

namespace grid_map_demos {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class ImageToGridmapDemo
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ImageToGridmapDemo(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ImageToGridmapDemo();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void imageCallback(const sensor_msgs::Image& msg);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_, occupancyPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Occupancy map lached data;
  nav_msgs::OccupancyGrid occupancyGrid;
  ros::ServiceServer get_map_service_;
  nav_msgs::GetMap::Response map_resp_;

  /** Callback invoked when someone requests our service */
  bool mapCallback(nav_msgs::GetMap::Request  &req,
                    nav_msgs::GetMap::Response &res )
  {
    // request is empty; we ignore it

    // = operator is overloaded to make deep copy (tricky!)
    res = map_resp_;
    ROS_INFO("Sending map");

    return true;
  }

  //! Image subscriber
  ros::Subscriber imageSubscriber_;

  //! Name of the grid map topic.
  std::string imageTopic_;

  //! Length of the grid map in x direction.
  double mapLengthX_;

  //! Resolution of the grid map.
  double resolution_;

  //! Range of the height values.
  double minHeight_;
  double maxHeight_;

  //! Frame id of the grid map.
  std::string mapFrameId_;

  bool mapInitialized_;
};

} /* namespace */
