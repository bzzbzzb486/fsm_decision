#include "decision_executor.hpp"

// Standard library
#include <iostream>
#include <iomanip>
#include <chrono>

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Local headers
#include "ResourceHub.hpp"
#include "Action.hpp"
#include "State.hpp"
#include "SimpleState.hpp"
#include "WaitFor_GameStart.hpp"
#include "GO_HOME.hpp"
#include "GOTO_A.hpp"
#include "GOTO_B.hpp"
#include "GOTO_C.hpp"
#include "PATTERN_MOVE.hpp"
#include "PATTERN_ATTACK.hpp"
#include "PATTERN_DEFENSE.hpp"

namespace aw_decision
{
    DemoDecision::DemoDecision(const rclcpp::NodeOptions &options)
        : Node("demo_decision", options)
    {
        _resource_hub = std::make_shared<ResourceHub>();
        _state_machine = std::make_shared<StateMachine>();
        getparam();
        init_resource_hub();
        _fsm_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]()
            {
                if (_state_machine && _is_first_run)
                {
                    init_state_machine();
                    _is_first_run = false;
                }
                _state_machine->update();
            });
    }

    void DemoDecision::init_resource_hub()
    {
        // 初始化资源站内容
        _common_data_sub = this->create_subscription<bt_interfaces::msg::Common>(
            "/common_msg", 10,
            [this](const bt_interfaces::msg::Common::SharedPtr msg)
            {
                ResourceHub::CommonData common;
                common.gamestart = msg->gamestart;
                common.is_fire = msg->is_fire;
                common.is_birth = msg->is_birth;
                common.stage_remain_time = msg->stage_remain_time;
                _resource_hub->update(common);
            });
        _robot_status_sub = this->create_subscription<bt_interfaces::msg::RobotStatus>(
            "/robotstatus_msg", 10, [this](const bt_interfaces::msg::RobotStatus::SharedPtr msg)
            {
            // RCLCPP_INFO(this->get_logger(), "[DEBUG] Received robot status - HP: %d, Ammo: %d, Our Outpost HP: %d, Enemy Outpost HP: %d",
            //            msg->current_hp, msg->current_ammo, msg->we_outpost_hp, msg->enemy_outpost_hp);
            ResourceHub::RobotStatus robot;
            robot.current_ammo = msg->current_ammo;
            robot.current_hp = msg->current_hp;
            robot.we_outpost_hp = msg->we_outpost_hp;
            robot.enemy_outpost_hp = msg->enemy_outpost_hp;

            // RCLCPP_INFO(this->get_logger(), "[DEBUG] Updated robot status in ResourceHub - HP: %d, Ammo: %d, Our Outpost HP: %d, Enemy Outpost HP: %d",
            //            robot.current_hp, robot.current_ammo, robot.we_outpost_hp, robot.enemy_outpost_hp);
            _resource_hub->update(robot); });
    }

    void DemoDecision::getparam()
    {
        this->declare_parameter<int>("hp_threshold", 300);
        this->declare_parameter<int>("ammo", 50);
        this->declare_parameter<int>("time_on_B", 10);
        this->declare_parameter<int>("time_on_C", 15);

        this->declare_parameter<std::vector<double>>("goto_A_pos", {1.0, 2.0, 0.0});
        this->declare_parameter<std::vector<double>>("goto_B_pos", {5.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("goto_C_pos", {5.0, -5.0, 0.0});
        this->declare_parameter<std::vector<double>>("home_pos", {0.0, 0.0, 0.0});

        this->get_parameter("hp_threshold", hp_threshold_);
        this->get_parameter("ammo", ammo_);
        this->get_parameter("time_on_B", time_on_B_);
        this->get_parameter("time_on_C", time_on_C_);

        auto goto_A_vec = this->get_parameter("goto_A_pos").as_double_array();
        goto_A_pos_ = {goto_A_vec[0], goto_A_vec[1], goto_A_vec[2]};

        auto goto_B_vec = this->get_parameter("goto_B_pos").as_double_array();
        goto_B_pos_ = {goto_B_vec[0], goto_B_vec[1], goto_B_vec[2]};

        auto goto_C_vec = this->get_parameter("goto_C_pos").as_double_array();
        goto_C_pos_ = {goto_C_vec[0], goto_C_vec[1], goto_C_vec[2]};

        auto home_vec = this->get_parameter("home_pos").as_double_array();
        home_pos_ = {home_vec[0], home_vec[1], home_vec[2]};

        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "  hp_threshold: %d", hp_threshold_);
        RCLCPP_INFO(this->get_logger(), "  ammo: %d", ammo_);
        RCLCPP_INFO(this->get_logger(), "  goto_B_pos: [%.2f, %.2f, %.2f]",
                    goto_B_pos_.x, goto_B_pos_.y, goto_B_pos_.yaw);
        RCLCPP_INFO(this->get_logger(), "  goto_C_pos: [%.2f, %.2f, %.2f]",
                    goto_C_pos_.x, goto_C_pos_.y, goto_C_pos_.yaw);
    }

    void DemoDecision::init_state_machine()
    {
        add_state_to_state_machine();
        _state_machine->initState(0);
    }

    void DemoDecision::add_state_to_state_machine()
    {
        _state_machine->addState(std::make_shared<aw_decision::WaitFor_GameStart_State>(
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            _resource_hub));

        _current_pattern = std::make_shared<PatternType>(PatternType::MOVE);

        _state_machine->addState(std::make_shared<aw_decision::PATTERN_MOVE>(
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            _resource_hub,
            _current_pattern));

        _state_machine->addState(std::make_shared<aw_decision::PATTERN_ATTACK>(
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            _resource_hub,
            _current_pattern));

        _state_machine->addState(std::make_shared<aw_decision::PATTERN_DEFENSE>(
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            _resource_hub,
            _current_pattern));

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose");

        _state_machine->addState(std::make_shared<aw_decision::GO_HOME_State>(
            home_pos_,
            nav_client_,              // NavClient::SharedPtr
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            30.0,                     // 超时 30 秒
            _resource_hub,
            hp_threshold_));

        // _state_machine->addState(std::make_shared<aw_decision::GOTO_A_State>(
        //     goto_A_pos_,
        //     nav_client_,              // NavClient::SharedPtr
        //     this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
        //     30.0                      // 超时 30 秒
        //     ));

        sequence_ = std::make_shared<Sequence>(_sequence, 0);

        _state_machine->addState(std::make_shared<aw_decision::GOTO_B_State>(
            goto_B_pos_,
            nav_client_,              // NavClient::SharedPtr
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            30.0,                     // 超时 30 秒
            _resource_hub,
            time_on_B_,
            sequence_));

        _state_machine->addState(std::make_shared<aw_decision::GOTO_C_State>(
            goto_C_pos_,
            nav_client_,              // NavClient::SharedPtr
            this->shared_from_this(), // std::shared_ptr<rclcpp::Node>
            30.0,                     // 超时 30 秒
            _resource_hub,
            time_on_C_,
            sequence_));
    }
}