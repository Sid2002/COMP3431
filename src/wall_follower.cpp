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
	// pose.pose.position.x, pose.pose.position.y
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	// for the first call
	struct Pose pose = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw};
	robot_pose_ = pose;

	struct Location curr = {msg->pose.pose.position.x, msg->pose.pose.position.y};
	curr_loc_ = curr;

	if (odomcalled == false) {
		start_loc_ = curr;
		start_pose_ = pose;
	}

	if (curr_loc_.x - start_loc_.x > 0.5 || curr_loc_.y - start_loc_.y > 0.5 || curr_loc_.x - start_loc_.x < -0.5 || curr_loc_.y - start_loc_.y < -0.5) {
		left_start_ = true;
	}

	odomcalled = true;


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
		if (check_angle < 0) check_angle += 360;
		sum += scan_data_[check_angle];
	}
	return sum;
}

std::vector<std::pair<double, double>> WallFollower::distances(int angle, int range) {
	std::vector<std::pair<double, double>> distances;
	for (int i = -range; i < range; i++) {
		int check_angle = i + angle;
		if (check_angle >= 360) check_angle -= 360;
		if (check_angle < 0) check_angle += 360;
		double x = scan_data_[check_angle] * std::sin(i * DEG2RAD);
		double y = scan_data_[check_angle] * std::cos(i * DEG2RAD);
		distances.push_back(std::make_pair(x, y));
	}
	return distances;
}

bool comparePointY(std::pair<double, double> i,std::pair<double, double> j) { return (i.second<j.second); }

struct Wall WallFollower::calculateWall(std::vector<std::pair<double, double>> distances) {

	constexpr double max_wall_distance = 1.0;
	struct Wall wall{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	std::vector<std::pair<double, double>> sorted_distances{};

	for (auto&& point : distances) {
		if (point.second < max_wall_distance){
			sorted_distances.push_back(point);
		}
	}

	std::sort(sorted_distances.begin(), sorted_distances.end(), comparePointY);

	int size = sorted_distances.size();
	if (size == 0) {
		return wall;
	}
	std::pair<double, double> median;
	if (size % 2 == 0) {
      median = sorted_distances[size / 2 - 1];
    } else  {
      median =  sorted_distances[size / 2];
    }

	// std::cout << "distances sorted (" << median.first << ", " << median.second << ")";

	int wallStartIndex = distances.size() + 1;
	int wallEndIndex = distances.size() - 1;
	const int numInputs = distances.size();
	for (int i = 0; i <= numInputs; i++) {
		std::pair<double, double> checkPoint = distances[i];
		double range = std::abs((median.first - checkPoint.first) * 1);
		// std::cout << "\t (" << checkPoint.first << ", " << checkPoint.second << ") range:" << range << ", " << std::abs(median.second - checkPoint.second) << "\n";
		if (std::abs(median.second - checkPoint.second) <= range) {
			if (wallStartIndex > i) {
				wallStartIndex = i;
				wallEndIndex = i;
			}
			wallEndIndex = i;
		}
	}
	// std::cout << " start: " << wallStartIndex << ", end: " << wallEndIndex;
	if (wallStartIndex == numInputs + 1) return wall;

	wall.wallStart = distances[wallStartIndex].first;
	wall.wallEnd = distances[wallEndIndex].first;

	double sumY = 0.0;
	double sumX = 0.0;
	for (int i = wallStartIndex; i <= wallEndIndex; i++) {
		auto point = distances[i];
		sumY += point.second;
		sumX += point.first;
	}
	double averageY = sumY / (wallEndIndex-wallStartIndex+1);
	double averageX = sumX / (wallEndIndex-wallStartIndex+1);
	double numeratorSum = 0.0;
	double denominatorSum = 0.0;
	for (int i = wallStartIndex; i <= wallEndIndex; i++) {
		auto point = distances[i];
		numeratorSum += ((point.first - averageX) * (point.second - averageY));
		denominatorSum += std::pow(point.first - averageX, 2);
	}
	double gradient = 0.0;
	if (denominatorSum != 0.0) {
		gradient = numeratorSum / denominatorSum;
	}

	wall.angle = std::atan(gradient) * RAD2DEG;
	double intercept = averageY - gradient * averageX;
	wall.zeroDegDistance = intercept;
	wall.distance = intercept / ( 1.0 + std::pow(gradient, 2) );

	// std::cout << ": " << distances.size() << ", " << averageY << ", angle" << wall.angle << ", distance: " << wall.distance;
	return wall;
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{

	constexpr double wallDistanceTarget = 0.3;
	constexpr double frontDistanceTarget = 0.4;

	struct Wall sideWall = calculateWall(distances(270, 30));
	struct Wall frontWall = calculateWall(distances(0, 7));

	// get current pos 
	
	

	switch (mState)
	{
		case FOLLOW_WALL: {
			if (mDebug > 1) std::cout << "WALL";

			// Check if state needs changing
			if (mDebug > 1) std::cout << ", front: " << frontWall.zeroDegDistance << "side: " << sideWall.distance << " m @ " << sideWall.angle << " degrees (" << sideWall.wallStart << "<x<=" << sideWall.wallEnd << ") ";
			if (frontWall.zeroDegDistance < frontDistanceTarget && frontWall.distance != 0.0) {
				mState = OBSTICLE_TURN_LEFT;
			}
			if (sideWall.wallEnd < 0.09 && sideWall.distance != 0.0) {
				mState = GAP_TURN_RIGHT;
			}
			if (start_loc_.x - curr_loc_.x < 0.5 && start_loc_.x - curr_loc_.x > -0.5) {
				if (start_loc_.y - curr_loc_.y < 0.5 && start_loc_.y - curr_loc_.y > -0.5) {
					if (start_pose_.yaw - robot_pose_.yaw < 0.3 &&  start_pose_.yaw - robot_pose_.yaw > -0.3) {
						if (left_start_ == true) {
							mState = STOP;
						}
					}
				}
			}

			// Execute Follow Wall
			double wallDistanceError = wallDistanceTarget - sideWall.distance;
			if (std::abs(wallDistanceError) < 0.05 && std::abs(sideWall.angle) < 2) {
				std::cout << "STRAIGHT";
				update_cmd_vel(0.3, 0.0);
			} else if (wallDistanceError < 0.0) { // Move Right
				double targetAngle = wallDistanceError * 10 / 0.2;
				if (targetAngle < -13) targetAngle = -13;
				double angleError = targetAngle - sideWall.angle;
				if (std::abs(angleError) < 2) {
					std::cout << "MV(R)ANGLE(" << targetAngle << "): 0.0";
					update_cmd_vel(0.3, 0.0);
				} else {
					double rotation = angleError * 0.3 / 10;
					std::cout << "MV(R)ANGLE(" << targetAngle << "): " << rotation;
					update_cmd_vel(0.3, rotation);
				} 
				
			} else { // wallDistanceError < 0 // Move Left
				double targetAngle = wallDistanceError * 10 / 0.2;
				if (targetAngle > 13) targetAngle = 13;
				double angleError = targetAngle - sideWall.angle;
				if (std::abs(angleError) < 2) {
					std::cout << "MV(L)ANGLE(" << targetAngle << "): 0.0";
					update_cmd_vel(0.3, 0.0);
				} else {
					double rotation = angleError * 0.3 / 10;
					std::cout << "MV(L)ANGLE(" << targetAngle << "): " << rotation;
					update_cmd_vel(0.3, rotation);
				} 
			}

			break;
		}

		case OBSTICLE_TURN_LEFT: {
			if (mDebug > 1) std::cout << "OBST";
			// Check if state needs changing
			if (mDebug > 1) std::cout << ", front: " << frontWall.zeroDegDistance << ", sideAngle: " << sideWall.angle;
			if ((frontWall.zeroDegDistance > 0.6 || frontWall.zeroDegDistance == 0.0) && sideWall.angle > -15) {
				mState = FOLLOW_WALL;
			}

			// Execute Left Turn
			update_cmd_vel(0.0, 1);

			break;
		}
		case GAP_TURN_RIGHT: {
			if (mDebug > 1) std::cout << "GAPP";
			// Check if state needs changing
			// if (mDebug > 1) std::cout << ", side Angle: " << sideWall.angle << ", length: " << (sideWall.wallEnd - sideWall.wallStart) << ", End: " << sideWall.wallEnd;
			if (mDebug > 1) std::cout << ", side Angle: " << sideWall.angle << ", length: " << (sideWall.wallEnd - sideWall.wallStart) << ", End: " << sideWall.wallEnd << ", @310: " << scan_data_[310];
			// if (sideWall.angle < 7 && sideWall.angle > -10 && std::abs(sideWall.wallEnd - sideWall.wallStart) > 0.1 && sideWall.wallEnd > 0.05) {
			if (scan_data_[310] < 1.5 * wallDistanceTarget) {
				mState = FOLLOW_WALL;
			}

			// Execute Right Turn
			update_cmd_vel(0.2, -0.7);

			break;
		}

		case STOP: {
			if (mDebug > 1) std::cout << "RETURNED TO START";
			update_cmd_vel(0.0, 0.0);
		}

		default: {
			update_cmd_vel(0.0, 0.0);
			if (mDebug > 1) std::cout << "STOPPED";
			break;
		}
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