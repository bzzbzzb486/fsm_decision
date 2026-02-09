#pragma once

#include "SimpleState.hpp"
#include "ResourceHub.hpp"
#include "Pattern.hpp"
#include <functional>
#include <memory>

namespace aw_decision
{
    /**
     * @brief 一次性状态类
     */
    class PATTERN_ATTACK : public SimpleState
    {
    public:
        PATTERN_ATTACK(std::shared_ptr<rclcpp::Node> node,
                       std::shared_ptr<ResourceHub> resource_hub,
                       std::shared_ptr<PatternType> pattern)
            : SimpleState("Pattern_Attack", Actions::ATTACK_PATTERN),
              _node(node),
              _resource_hub(resource_hub),
              _pattern(Pattern(PatternType::ATTACK)),
              _current_pattern(pattern)
        {
            setCondition([this]()
                         { if (*_current_pattern == _pattern.getPattern())
                                 return false; 
                            return false; });

            setEnter([this]()
                     { *_current_pattern = _pattern.getPattern(); });

            setExec([this]()
                    {   RCLCPP_INFO(this->_node->get_logger(), "PATTERN_ATTACK: 已切换为攻击姿态");
                        return Actions::DEFAULT; });

            setExit([this]() {});

            setReset([this]()
                     { RCLCPP_ERROR(this->_node->get_logger(), "PATTERN_ATTACK: 未定义的行为"); });

            setInterrupt([this]()
                         { return false; });
        }

        ~PATTERN_ATTACK() = default;

    private:
        /***
         * @brief ROS2 节点指针
         */
        std::shared_ptr<rclcpp::Node> _node;
        /***
         * @brief 资源站
         */
        std::shared_ptr<ResourceHub> _resource_hub;
        /***
         * @brief 形态
         */
        Pattern _pattern;
        /***
         * @brief 当前形态
         */
        std::shared_ptr<PatternType> _current_pattern;
    };
}