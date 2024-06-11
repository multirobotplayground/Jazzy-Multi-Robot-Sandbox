#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "PoseTFPublisher.h"

using namespace std::chrono_literals;

PoseTFPublisher::PoseTFPublisher() : Node("pose_tf_publisher") {
    declare_parameter<std::string>("pose_topic", "/pose");
    declare_parameter<std::string>("tf_subtree", "/odom/base_link");
    declare_parameter<std::string>("namespace", "");
    declare_parameter<int>("sleep", 1000);

    get_parameter("pose_topic", aPoseTopic);
    get_parameter("tf_subtree", aTfSubTree);
    get_parameter("sleep", aSleep);

    aNamespace = std::string(get_namespace());
    aAbsoluteTopicPath = "/" + aNamespace + "/" + aPoseTopic;

    apPoseSubscriber = create_subscription<geometry_msgs::msg::Pose>(aAbsoluteTopicPath, 1, std::bind(&PoseTFPublisher::PoseSubscriber, this));
    apTimer = create_wall_timer(std::chrono::milliseconds(aSleep), std::bind(&PoseTFPublisher::Update, this));
    aHasPose = false;
}

PoseTFPublisher::~PoseTFPublisher() {

}

void PoseTFPublisher::Update() {
    if(!aHasPose) {
        RCLCPP_INFO(get_logger(), "Waiting for pose in topic %s.", aAbsoluteTopicPath.c_str());
        return;
    }


}

void PoseTFPublisher::PoseSubscriber(geometry_msgs::msg::Pose::ConstPtr msg) {
    aHasPose = true;
    aPose.position = msg->position;
    aPose.orientation = msg->orientation;
}