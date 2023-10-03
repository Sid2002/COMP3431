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
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "ass_1/wall_follower.hpp"

#include <memory>

using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 360; i++) {
		scan_data_[i] = 0.0;
	}

	robot_pose_ = {0.0, 0.0, 0.0};

	mState = FOLLOW_WALL;


	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	update_cmd_vel(0.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	struct Pose pose = {0, 0, yaw};
	robot_pose_ = pose;
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	for (int num = 0; num < 360; num++)
	{
		if (std::isinf(msg->ranges.at(num)))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(num);
		}
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

double WallFollower::min_angle(int angle, int range) {
	double min = -1;
	for (int i = -range; i < range; i++) {
		int check_angle = i + angle;
		if (check_angle >= 360) check_angle = check_angle - 360;
		if (check_angle < 0) check_angle = check_angle + 360;
		if (min == -1.0 || scan_data_[check_angle] < min) {
			min = scan_data_[check_angle];
		}
	}
	return min;
}

double WallFollower::average_angle(int angle, int range) {
	double sum = 0;
	for (int i = -range; i < range; i++) {
		int check_angle = i + angle;
		if (check_angle >= 360) check_angle -= 360;
		if (check_angle < 360) check_angle += 360;
		sum += scan_data_[check_angle];
	}
	return sum;
}


/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{


	switch (mState)
	{
		case FOLLOW_WALL:
			if (mDebug > 1) std::cout << "WALL";
			// Check if state needs changing


			// Execute Follow Wall


			break;

		case OBSTICLE_TURN_LEFT:
			if (mDebug > 1) std::cout << "OBST";
			// Check if state needs changing


			// Execute Left Turn


			break;

		case GAP_TURN_RIGHT:
			if (mDebug > 1) std::cout << "GAPP";
			// Check if state needs changing


			// Execute Right Turn


			break;

		case STOP:
		default:
			update_cmd_vel(0.0, 0.0);
			if (mDebug > 1) std::cout << "STOPPED";
			break;
	}
	if (mDebug > 1) std::cout << std::endl;
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
