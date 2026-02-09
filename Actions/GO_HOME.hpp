#pragma once

#include "SimpleState.hpp"
#include "ResourceHub.hpp"
#include "Position2D.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace aw_decision
{
    /***
     * @brief NavigateToPose action to navigate to the goal point
     */
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    /***
     * @brief nav goal handler typedef
     */
    using NavGoalHandler = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /***
     * @brief nav goal client typedef
     */
    using NavClient = rclcpp_action::Client<NavigateToPose>;

    /**
     * @brief 创建回家状态
     */
    class GO_HOME_State : public SimpleState
    {
    public:
        GO_HOME_State(Position2D pos,
                      NavClient::SharedPtr nav_client,
                      std::shared_ptr<rclcpp::Node> node,
                      double timeout,
                      std::shared_ptr<ResourceHub> resource_hub,
                      int hp_threshold = 1)
            : SimpleState("GO_HOME", Actions::GO_HOME),
              position_(pos),
              nav_client_(nav_client),
              node_(node),
              timeout_(timeout),
              resource_hub_(resource_hub),
              hp_threshold_(hp_threshold)
        {
            this->setCondition([this]()
                               { if (resource_hub_->robot.current_hp < hp_threshold_)
                                    {
                                        RCLCPP_INFO(node_->get_logger(), "GO_HOME: 当前血量为%d 低于阈值，准备回家", resource_hub_->robot.current_hp);
                                        return true;
                                    }
                                else
                                    return false; });

            this->setInterrupt([]()
                               { return false; });

            this->setEnter([this]()
                           {
                // 检查 client
                if (this->nav_client_ == nullptr)
                {
                    RCLCPP_ERROR(node_->get_logger(), "nav_client_ is nullptr!");
                    return;
                }

                if (!nav_client_->wait_for_action_server(std::chrono::seconds(3)))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Action server is not available after waiting 3s...");
                    return;
                }

                // 重置状态标志
                is_goal_sent_ = false;
                is_arrived_ = false;
                is_failed_ = false;

                // 记录开始时间
                start_time_ = std::chrono::steady_clock::now();

                // 设置目标点
                auto goal_msg = NavigateToPose::Goal();
                tf2::Quaternion tf_quat;
                tf_quat.setRPY(0, 0, position_.yaw);
                geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf_quat);

                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = node_->now();
                goal_msg.pose.pose.position.x = position_.x;
                goal_msg.pose.pose.position.y = position_.y;
                goal_msg.pose.pose.position.z = 0.0;
                goal_msg.pose.pose.orientation = quat;

                RCLCPP_INFO(node_->get_logger(), 
                            "GO_HOME: 目标点 [%.2f, %.2f, yaw=%.2f]",
                            goal_msg.pose.pose.position.x,
                            goal_msg.pose.pose.position.y,
                            position_.yaw);

                // 配置回调
                auto nav_goal_options = NavClient::SendGoalOptions();

                // goal response callback
                nav_goal_options.goal_response_callback = [this](NavGoalHandler::SharedPtr goal_handle)
                {
                    goal_handle_ = goal_handle;
                    if (!goal_handle_)
                    {
                        RCLCPP_ERROR(node_->get_logger(), "GO_HOME: 目标点被拒绝");
                        is_failed_ = true;
                        return;
                    }
                    if (is_goal_sent_ == false)
                    {
                        is_goal_sent_ = true;
                        RCLCPP_INFO(node_->get_logger(), "GO_HOME: 目标点已发送");
                    }
                };

                // result callback
                nav_goal_options.result_callback = [this](const NavGoalHandler::WrappedResult &result)
                {
                    switch (result.code)
                    {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node_->get_logger(), "GO_HOME: 导航成功");
                        is_arrived_ = true;
                        break;

                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(node_->get_logger(), "GO_HOME: 导航失败");
                        is_failed_ = true;
                        break;

                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(node_->get_logger(), "GO_HOME: 导航被取消");
                        is_failed_ = true;
                        break;

                    default:
                        RCLCPP_ERROR(node_->get_logger(), "GO_HOME: 未知状态");
                        is_failed_ = true;
                        break;
                    }
                };
                RCLCPP_INFO(node_->get_logger(), "GO_HOME: 导航开始");

                nav_client_->async_send_goal(goal_msg, nav_goal_options); });

            this->setExec([this]() -> Actions
                          {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration<double>(now - start_time_).count();
                
                // 1. 给 1 秒缓冲时间等待异步回调确认
                if (!is_goal_sent_ && elapsed < 1.0)
                {
                    RCLCPP_INFO(node_->get_logger(), "GO_HOME: 等待目标点发送确认...");
                    return Actions::GO_HOME;  // 继续保持在当前状态
                }
                
                // 2. 超过缓冲时间仍未发送，说明发送失败
                if (!is_goal_sent_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "GO_HOME: 目标点发送失败");
                    return Actions::DEFAULT;
                }
                
                // 3. 检查超时
                if (elapsed > timeout_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "GO_HOME: 超时！已耗时 %.1f 秒", elapsed);
                    return Actions::DEFAULT;
                }
                
                // 4. 检查是否失败
                if (is_failed_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "GO_HOME: 导航失败，准备退出状态");
                    return Actions::DEFAULT;
                }
                
                // 5. 检查是否到达
                if (is_arrived_)
                {
                    RCLCPP_INFO(node_->get_logger(), "GO_HOME: 成功到达目标点");
                    return Actions::DEFAULT;
                }
                
                // 6. 还在导航中
                RCLCPP_INFO(node_->get_logger(), "GO_HOME: 导航中... 已耗时 %.1f 秒", elapsed);
                return Actions::GO_HOME; });

            this->setExit([this]()
                          {
            if (goal_handle_ && !is_arrived_ && !is_failed_)
            {
                nav_client_->async_cancel_goal(goal_handle_);
                RCLCPP_INFO(node_->get_logger(), "GO_HOME: 退出状态，取消导航");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "GO_HOME: 退出状态");
            } });

            this->setReset([this]()
                           {
                if (goal_handle_ && !is_arrived_ && !is_failed_)
                {
                    nav_client_->async_cancel_goal(goal_handle_);
                    RCLCPP_INFO(node_->get_logger(), "GO_HOME: 重置状态，取消导航");
                }
                else if (goal_handle_)
                {
                    RCLCPP_INFO(node_->get_logger(), "GO_HOME: 重置状态（导航已结束）");
                }
                
                goal_handle_.reset(); });
        }

        ~GO_HOME_State() = default;

    private:
        /***
         * @brief 目标位置
         */
        Position2D position_;

        /***
         * @brief nav发点动作服务器
         */
        NavClient::SharedPtr nav_client_;

        /***
         * @brief ROS2 节点指针
         */
        std::shared_ptr<rclcpp::Node> node_;

        /***
         * @brief 是否已发送目标点
         */
        bool is_goal_sent_ = false;

        /***
         * @brief 是否已到达目标点
         */
        bool is_arrived_ = false;

        /***
         * @brief 是否导航失败
         */
        bool is_failed_ = false;

        /***
         * @brief 目标点处理句柄
         */
        NavGoalHandler::SharedPtr goal_handle_;

        /***
         * @brief 超时时间（秒）
         */
        double timeout_;

        /***
         * @brief 开始时间
         */
        std::chrono::time_point<std::chrono::steady_clock> start_time_;

        /***
         * @brief 资源站
         */
        std::shared_ptr<ResourceHub> resource_hub_;

        /***
         * @brief 血量阈值
         */
        int hp_threshold_ = 1;
    };

} // namespace aw_decision