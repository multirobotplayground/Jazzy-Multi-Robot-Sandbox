#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"

class PoseTFPublisher : public rclcpp::Node {
    public:
        PoseTFPublisher();
        ~PoseTFPublisher();
        void PoseSubscriber(geometry_msgs::msg::Pose::UniquePtr msg);
        void Update();
        
    private:
        int aFrequency;
        bool aHasPose;
        geometry_msgs::msg::Pose aPose;
        std::string aOdomFrame;
        std::string aChildFrame;
        std::string aPoseTopic;
        std::string aNamespace;
        std::string aAbsoluteTopicPath;
        rclcpp::TimerBase::SharedPtr apTimer;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr apPoseSubscriber;
        std::unique_ptr<tf2_ros::TransformBroadcaster> aTFBroadcaster;
};