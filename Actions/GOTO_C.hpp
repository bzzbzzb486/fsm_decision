#pragma once

#include "OrderState.hpp"
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
     * @brief 创建前往C点状态
     */
    class GOTO_C_State : public OrderState
    {
    public:
        GOTO_C_State(Position2D pos,
                     NavClient::SharedPtr nav_client,
                     std::shared_ptr<rclcpp::Node> node,
                     double timeout,
                     std::shared_ptr<ResourceHub> resource_hub,
                     int stay_time,
                     std::shared_ptr<Sequence> sequence)
            : OrderState("GOTO_C", Actions::GOTO_C),
              position_(pos),
              nav_client_(nav_client),
              node_(node),
              timeout_(timeout),
              resource_hub_(resource_hub),
              stay_time_(stay_time)
        {
            this->setSequence(sequence);

            this->setCondition([this]()
                               { return true; });

            this->setInterrupt([]()
                               { return true; });

            this->setEnter([this]()
                           {
                // 检查 client
                if (this->nav_client_ == nullptr)
                {
                    RCLCPP_ERROR(node_->get_logger(), "GOTO_C: nav_client_ is nullptr!");
                    return;
                }

                if (!nav_client_->wait_for_action_server(std::chrono::seconds(3)))
                {
                    RCLCPP_ERROR(node_->get_logger(), "GOTO_C: Action server is not available after waiting 3s...");
                    return;
                }

                // 重置状态标志
                is_goal_sent_ = false;
                is_arrived_ = false;
                is_failed_ = false;
                start_stay_time_ = 0;

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
                            "GOTO_C: 目标点 [%.2f, %.2f, yaw=%.2f]",
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
                        RCLCPP_ERROR(node_->get_logger(), "GOTO_C: 目标点被拒绝");
                        is_failed_ = true;
                        return;
                    }
                    if (is_goal_sent_ == false)
                    {
                        is_goal_sent_ = true;
                        RCLCPP_INFO(node_->get_logger(), "GOTO_C: 目标点已发送");
                    }
                };

                // result callback
                nav_goal_options.result_callback = [this](const NavGoalHandler::WrappedResult &result)
                {
                    switch (result.code)
                    {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node_->get_logger(), "GOTO_C: 导航成功");
                        start_stay_time_ = resource_hub_->common.stage_remain_time;
                        is_arrived_ = true;
                        break;

                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(node_->get_logger(), "GOTO_C: 导航失败");
                        is_failed_ = true;
                        break;

                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(node_->get_logger(), "GOTO_C: 导航被取消");
                        is_failed_ = true;
                        break;

                    default:
                        RCLCPP_ERROR(node_->get_logger(), "GOTO_C: 未知状态");
                        is_failed_ = true;
                        break;
                    }
                };
                RCLCPP_INFO(node_->get_logger(), "GOTO_C: 导航开始");

                nav_client_->async_send_goal(goal_msg, nav_goal_options); });

            this->setExec([this]() -> Actions
                          {
                // 1. 检查 goal 是否发送
                if (!is_goal_sent_)
                {
                    RCLCPP_WARN(node_->get_logger(), "GOTO_C: 目标点未发送");
                    return Actions::DEFAULT;
                }
                
                // 2. 检查超时
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration<double>(now - start_time_).count();
                
                if (elapsed > timeout_ && !is_arrived_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "GOTO_C: 超时！已耗时 %.1f 秒", elapsed);
                    return Actions::DEFAULT;
                }
                
                // 3. 检查是否失败
                if (is_failed_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "GOTO_C: 导航失败，准备退出状态");
                    return Actions::DEFAULT;
                }
                
                // 4. 检查是否到达
                if (is_arrived_)
                {
                    if(start_stay_time_ > 0 || (start_stay_time_ - resource_hub_->common.stage_remain_time < stay_time_))
                    {
                        RCLCPP_INFO(node_->get_logger(), "GOTO_C: 已停留%.1d秒",start_stay_time_ - resource_hub_->common.stage_remain_time);
                        return Actions::GOTO_B;
                    }
                    RCLCPP_INFO(node_->get_logger(), "GOTO_C: 成功到达目标点");
                    return Actions::DEFAULT;
                }
                
                // 5. 还在导航中
                RCLCPP_INFO(node_->get_logger(), "GOTO_C: 导航中... 已耗时 %.1f 秒", elapsed);
                return Actions::GOTO_C; });

            this->setExit([this]()
                          {
            if (goal_handle_ && !is_arrived_ && !is_failed_)
            {
                nav_client_->async_cancel_goal(goal_handle_);
                RCLCPP_INFO(node_->get_logger(), "GOTO_C: 退出状态，取消导航");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "GOTO_C: 退出状态");
            } });

            this->setReset([this]()
                           {
                if (goal_handle_ && !is_arrived_ && !is_failed_)
                {
                    nav_client_->async_cancel_goal(goal_handle_);
                    RCLCPP_INFO(node_->get_logger(), "GOTO_C: 重置状态，取消导航");
                }
                else if (goal_handle_)
                {
                    RCLCPP_INFO(node_->get_logger(), "GOTO_C: 重置状态（导航已结束）");
                }
                
                goal_handle_.reset(); });
        }

        ~GOTO_C_State() = default;

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
         * @brief 停留时间
         */
        int stay_time_ = 5.0;

        /***
         * @brief 开始停留时间
         */
        int start_stay_time_;
    };

} // namespace aw_decision