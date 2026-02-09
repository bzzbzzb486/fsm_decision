#pragma once

#include "OneTimeState.hpp"
#include "ResourceHub.hpp"
#include <functional>

namespace aw_decision
{
    /**
     * @brief 一次性状态类
     */
    class WaitFor_GameStart_State : public OneTimeState
    {
    public:
        WaitFor_GameStart_State(std::shared_ptr<rclcpp::Node> node,
                                std::shared_ptr<ResourceHub> resource_hub)
            : OneTimeState("WaitFor_GameStart", Actions::WAIT_FOR_GAMESTART),
              _node(node),
              _resource_hub(resource_hub)
        {
            setCondition([this]()
                         { return !_resource_hub->common.gamestart; });
            setEnter([this]()
                     { RCLCPP_INFO(this->_node->get_logger(), "WaitFor_GameStart: 正在等待比赛开始..."); });

            setExec([this]()
                    { if(!_resource_hub->common.gamestart)
                        {   RCLCPP_INFO(this->_node->get_logger(), "WaitFor_GameStart: 等待比赛开始...");
                            return Actions::WAIT_FOR_GAMESTART;}
                            return Actions::DEFAULT; });
            setExit([this]()
                    { RCLCPP_INFO(this->_node->get_logger(), "WaitFor_GameStart: 比赛开始！"); });

            setReset([this]()
                     { RCLCPP_ERROR(this->_node->get_logger(), "WaitFor_GameStart: 未定义的行为"); });

            setInterrupt([this]()
                         { return false; });
        }

        ~WaitFor_GameStart_State() = default;

    private:
        /***
         * @brief ROS2 节点指针
         */
        std::shared_ptr<rclcpp::Node> _node;
        /***
         * @brief 资源站
         */
        std::shared_ptr<ResourceHub> _resource_hub;
    };
}