/*************************************************************************
        > File Name: robot_config.h
        > Author: ma6174
        > Mail: ma6174@163.com
        > Created Time: 2018年08月21日 星期二 10时58分57秒
 ************************************************************************/
#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <memory>

#include <iostream>
#include <fstream>
#include <ctime>
#include <pwd.h>
#include <vector>
#include "websocket.h"

#include "pcl_conversions/pcl_conversions.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// class Websocket;

class RosTalk : public rclcpp::Node
{

public:
  explicit RosTalk(std::shared_ptr<websocket::Websocket> websocket_)
      : websocket(websocket_), Node("webtalk_node")
  {
    this->init();
  }

  RosTalk();
  ~RosTalk() override;
  void init();
  void read();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  void lidarDatasCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // 订阅
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      lidar_datas_sub_;

  std::shared_ptr<websocket::Websocket> websocket;
};
#endif
