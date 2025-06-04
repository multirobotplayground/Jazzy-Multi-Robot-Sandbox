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

#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "PoseTFPublisher.h"

using namespace std::chrono_literals;

PoseTFPublisher::PoseTFPublisher() : Node("pose_tf_publisher") {
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("child_frame", "base_link");
    declare_parameter<int>("hz", 1000);
    
    aPoseTopic = "/pose";

    get_parameter("odom_frame", aOdomFrame);
    get_parameter("child_frame", aChildFrame);
    get_parameter("hz", aFrequency);

    aNamespace = std::string(get_namespace());
    if(aNamespace.compare("/") == 0) {
        aAbsoluteTopicPath = aPoseTopic;
    } else {
        aAbsoluteTopicPath = aNamespace + aPoseTopic;
        
        // remove the '/' for the correct transform
        aNamespace.erase(aNamespace.begin());
        aOdomFrame = aNamespace + "/" + aOdomFrame;
        aChildFrame = aNamespace + "/" + aChildFrame;
    }

    apPoseSubscriber = create_subscription<geometry_msgs::msg::PoseStamped>(aAbsoluteTopicPath, 5, std::bind(&PoseTFPublisher::PoseSubscriber, this, std::placeholders::_1));

    int ms = (int)(1.0/((double)aFrequency/1000.0));
    RCLCPP_INFO(get_logger(), "odom frame: %s", aOdomFrame.c_str());
    RCLCPP_INFO(get_logger(), "child frame: %s", aChildFrame.c_str());
    RCLCPP_INFO(get_logger(), "update hz and period in ms: %d %d", aFrequency, ms);
    
    apTimer = create_wall_timer(std::chrono::milliseconds(ms), std::bind(&PoseTFPublisher::Update, this));
    aTFBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    aHasPose = false;
    aStartingTimeStamp = get_clock()->now();

    aPose.pose.position.x = 0.0;
    aPose.pose.position.y = 0.0;
    aPose.pose.position.z = 0.0;
    aPose.pose.orientation.x = 0.0;
    aPose.pose.orientation.y = 0.0;
    aPose.pose.orientation.z = 0.0;
    aPose.pose.orientation.w = 1.0;
}

PoseTFPublisher::~PoseTFPublisher() {

}

void PoseTFPublisher::Update() {
    if(!aHasPose) return;

    geometry_msgs::msg::TransformStamped transform;

    // rclcpp::Duration dur = get_clock()->now() - aStartingTimeStamp;
    transform.header.stamp = get_clock()->now();
    
    transform.header.frame_id = aOdomFrame;
    transform.child_frame_id = aChildFrame;
    transform.transform.translation.x = aPose.pose.position.x;
    transform.transform.translation.y = aPose.pose.position.y;
    transform.transform.translation.z = aPose.pose.position.z;
    transform.transform.rotation.x = aPose.pose.orientation.x;
    transform.transform.rotation.y = aPose.pose.orientation.y;
    transform.transform.rotation.z = aPose.pose.orientation.z;
    transform.transform.rotation.w = aPose.pose.orientation.w;
    
    aTFBroadcaster->sendTransform(transform);
}

void PoseTFPublisher::PoseSubscriber(geometry_msgs::msg::PoseStamped::UniquePtr msg) {
    if(aHasPose == false) {
        aHasPose = true;
        RCLCPP_INFO(get_logger(), "Pose received.");
    }
    
    aPose.pose.position = msg->pose.position;
    aPose.pose.orientation = msg->pose.orientation;
}