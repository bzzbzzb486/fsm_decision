#ifndef DEMO_DECISION_HPP
#define DEMO_DECISION_HPP

#include "StateMachine.hpp"
#include "ResourceHub.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_interfaces/msg/common.hpp"
#include "bt_interfaces/msg/robot_status.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "GO_HOME.hpp"
#include "GOTO_A.hpp"
#include "GOTO_B.hpp"
#include "GOTO_C.hpp"
#include "Pattern.hpp"

namespace aw_decision
{
    /***
     * @brief behavior executor
     */
    class DemoDecision : public rclcpp::Node
    {
    public:
        explicit DemoDecision(const rclcpp::NodeOptions &options);
        ~DemoDecision() = default;

        /***
         * @brief 获取参数
         */
        void getparam();
        /***
         * @brief 初始化资源站
         */
        void init_resource_hub();
        /***
         * @brief 初始化状态机
         */
        void init_state_machine();
        /***
         * @brief 添加状态到状态机
         */
        void add_state_to_state_machine();

    private:
        /***
         * @brief 状态机
         */
        std::shared_ptr<StateMachine> _state_machine;
        /***
         * @brief 资源站
         */
        std::shared_ptr<ResourceHub> _resource_hub;
        /***
         * @brief 定时器定时触发状态机
         */
        rclcpp::TimerBase::SharedPtr _fsm_timer_;
        /***
         * @brief CommonData订阅者
         */
        rclcpp::Subscription<bt_interfaces::msg::Common>::SharedPtr _common_data_sub;
        /***
         * @brief RobotStatus订阅者
         */
        rclcpp::Subscription<bt_interfaces::msg::RobotStatus>::SharedPtr _robot_status_sub;
        /***
         * @brief 是否首次运行
         */
        bool _is_first_run = true;

    private:
        /***
         * @brief nav发点动作服务器
         */
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

        /***
         * @brief 目标点序列
         */
        std::shared_ptr<Sequence> sequence_;

        /***
         * @brief 导航序列
         */
        std::vector<Actions> _sequence = {
            Actions::GOTO_B,
            Actions::GOTO_C};

        /***
         * @brief 当前姿态
         */
        std::shared_ptr<PatternType> _current_pattern;

    private:
        /***
         * @brief 决策参数
         */
        int hp_threshold_;
        int ammo_;
        int time_on_B_;
        int time_on_C_;
        Position2D goto_A_pos_;
        Position2D goto_B_pos_;
        Position2D goto_C_pos_;
        Position2D home_pos_;
    };
}

#endif // DEMO_DECISION_HPP