// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <math.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define FOLLOW_WALL        0
#define OBSTICLE_TURN_LEFT 1
#define GAP_TURN_RIGHT     2
#define STOP               3

struct Pose {
  double x;
  double y;
  double yaw;
};

struct Wall {
  double angle; // degrees
  double distance; // m
  double certainty;
  double wallStart; // m
  double wallEnd; // m
  double zeroDegDistance; // m
};

class WallFollower : public rclcpp::Node
{
public:
  WallFollower();
  ~WallFollower();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  struct Pose robot_pose_;
  struct Pose prev_robot_pose_;
  double scan_data_[360];
  uint8_t mState;
  int mDebug = 2;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double min_angle(int angle, int range = 5);
  double average_angle(int angle, int range = 5);
  double wall_angle(int angle, int range = 5);
  std::vector<std::pair<double, double>> distances(int angle, int range = 5);
  struct Wall calculateWall(std::vector<std::pair<double, double>> distances);
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
