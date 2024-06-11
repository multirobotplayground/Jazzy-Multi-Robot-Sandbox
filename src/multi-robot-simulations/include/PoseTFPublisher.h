#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

class PoseTFPublisher : public rclcpp::Node {
    public:
        PoseTFPublisher();
        ~PoseTFPublisher();
        void PoseSubscriber(geometry_msgs::msg::Pose::ConstPtr msg);
        void Update();
        
    private:
        int aSleep;
        bool aHasPose;
        geometry_msgs::msg::Pose aPose;
        std::string aTfSubTree;
        std::string aPoseTopic;
        std::string aNamespace;
        std::string aAbsoluteTopicPath;
        rclcpp::TimerBase::SharedPtr apTimer;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr apPoseSubscriber;
};