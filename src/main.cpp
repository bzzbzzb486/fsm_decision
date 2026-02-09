#include "decision_executor.hpp"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto des = std::make_shared<aw_decision::DemoDecision>(options);

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(1000));
    executor.add_node(des);
    executor.spin();
    executor.remove_node(des);

    rclcpp::shutdown();
}