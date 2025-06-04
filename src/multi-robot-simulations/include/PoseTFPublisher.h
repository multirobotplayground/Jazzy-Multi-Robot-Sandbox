/*
 * Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2024 Alysson Ribeiro da Silva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PoseTFPublisher : public rclcpp::Node {
    public:
        PoseTFPublisher();
        ~PoseTFPublisher();
        void PoseSubscriber(geometry_msgs::msg::PoseStamped::UniquePtr msg);
        void Update();
        
    private:
        int aFrequency;
        rclcpp::Time aStartingTimeStamp;
        bool aHasPose;
        geometry_msgs::msg::PoseStamped aPose;
        std::string aOdomFrame;
        std::string aChildFrame;
        std::string aPoseTopic;
        std::string aNamespace;
        std::string aAbsoluteTopicPath;
        rclcpp::TimerBase::SharedPtr apTimer;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apPoseSubscriber;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> aTFBroadcaster;
};