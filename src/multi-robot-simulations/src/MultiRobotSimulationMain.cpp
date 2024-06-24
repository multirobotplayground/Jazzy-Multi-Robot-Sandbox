#include "rclcpp/rclcpp.hpp"
#include "PoseTFPublisher.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseTFPublisher>());
    rclcpp::shutdown();
    return 0;
}