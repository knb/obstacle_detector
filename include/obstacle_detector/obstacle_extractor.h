/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "obstacle_detector_interfaces/msg/obstacles.hpp"
#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/segment.h"
#include "obstacle_detector/utilities/circle.h"
#include "obstacle_detector/utilities/point_set.h"
#include "visibility_control.h"

#include <memory>

namespace obstacle_detector
{

class ObstacleExtractor : public rclcpp::Node
{
public:
  OBSTACLE_DETECTOR_PUBLIC
  ObstacleExtractor(const std::string & node_name = "",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~ObstacleExtractor();

private:
  rcl_interfaces::msg::SetParametersResult updateParamsCallback(
      const std::vector<rclcpp::Parameter> &parameters);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg);

  void updateSubscriber();

  void processPoints();
  void groupPoints();
  void publishObstacles();

  void detectSegments(const PointSet& point_set);
  void mergeSegments();
  bool compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment);
  bool checkSegmentsProximity(const Segment& s1, const Segment& s2);
  bool checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2);

  void detectCircles();
  void mergeCircles();
  bool compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<obstacle_detector_interfaces::msg::Obstacles>::SharedPtr  obstacles_pub_;

  rclcpp::Clock::SharedPtr clock_;

  builtin_interfaces::msg::Time stamp_;

  std::string base_frame_id_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::list<Point> input_points_;
  std::list<Segment> segments_;
  std::list<Circle> circles_;

  // Parameters
  bool p_active_;
  bool prev_active_;
  bool p_use_scan_;
  std::string scan_topic_;
  bool p_use_pcl_;
  std::string pcl_topic_;

  bool p_use_split_and_merge_;
  bool p_circles_from_visibles_;
  bool p_discard_converted_segments_;
  bool p_transform_coordinates_;

  int p_min_group_points_;

  double p_distance_proportion_;
  double p_max_group_distance_;
  double p_max_split_distance_;
  double p_max_merge_separation_;
  double p_max_merge_spread_;
  double p_max_circle_radius_;
  double p_radius_enlargement_;

  double p_min_x_limit_;
  double p_max_x_limit_;
  double p_min_y_limit_;
  double p_max_y_limit_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
