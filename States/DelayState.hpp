#pragma once

#include "State.hpp"
#include "Action.hpp"
#include "ResourceHub.hpp"
#include <functional>

namespace aw_decision
{
    /**
     * @brief 持续固定时间状态类
     */
    class DelayState : public State
    {
    public:
        DelayState(std::string name, Actions type, std::shared_ptr<ResourceHub> resource_hub, int time, bool is_remember)
            : State(name, type), _resource_hub(resource_hub), _duration(time), _is_remember(is_remember)
        {
            _remaining_time = time;
        }
        ~DelayState() = default;

    private:
        std::function<void()> _onEnter;
        std::function<Actions()> _onExec;
        std::function<void()> _onExit;
        std::function<void()> _onReset;
        std::function<bool()> _onInterrupt;
        std::function<bool()> _onCondition;
        std::shared_ptr<ResourceHub> _resource_hub;
        int _start_time = -1;
        int _duration;
        int _remaining_time;
        bool _is_remember = false;

    public:
        /**
         * @brief 设置状态进入时的回调函数
         */
        void setEnter(std::function<void()> enter)
        {
            this->_onEnter = enter;
        }

        /**
         * @brief 设置状态执行时的回调函数
         */
        void setExec(std::function<Actions()> exec)
        {
            this->_onExec = exec;
        }

        /**
         * @brief 设置状态退出时的回调函数
         */
        void setExit(std::function<void()> exit)
        {
            this->_onExit = exit;
        }

        /**
         * @brief 状态重置时的回调函数
         */
        void setReset(std::function<void()> reset)
        {
            this->_onReset = reset;
        }

        /**
         * @brief 设置判断状态能否被中断的回调函数
         */
        void setInterrupt(std::function<bool()> interrupt)
        {
            this->_onInterrupt = interrupt;
        }

        /**
         * @brief 设置判断当前条件能否进入状态的回调函数
         */
        void setCondition(std::function<bool()> condition)
        {
            this->_onCondition = condition;
        }

    public:
        /**
         * @brief 状态进入时调用
         */
        void onEnter()
        {
            if (this->_onEnter)
            {
                this->_start_time = this->_resource_hub->common.stage_remain_time;
                this->_onEnter();
            }
            else
            {
                std::cerr << "No onEnter function defined for state: " << this->getName() << std::endl;
            }
        }

        /**
         * @brief 每次状态执行时调用
         * @return true 继续执行，false 强制退出状态
         */
        Actions onExec()
        {
            if (_start_time >= 0)
            {
                int current_time = _resource_hub->common.stage_remain_time;
                int elapsed = _start_time - current_time;
                _remaining_time = _duration - elapsed;

                if (_remaining_time <= 0)
                {
                    return Actions::DEFAULT;
                }
            }
            else
            {
                std::cerr << "Start time is invalid" << std::endl;
                return Actions::DEFAULT;
            }

            if (this->_onExec)
            {
                return this->_onExec();
            }
            return Actions::DEFAULT;
        }

        /**
         * @brief 状态退出时调用
         */
        void onExit()
        {
            if (this->_onExit)
            {
                _remaining_time = _duration;
                this->_onExit();
            }
            else
            {
                std::cerr << "No onExit function defined for state: " << this->getName() << std::endl;
            }
        }

        /**
         * @brief 检查是否满足进入条件
         */
        bool checkCondition()
        {
            if (!_onCondition)
                return false;
            return this->_onCondition();
        }

        /**
         * @brief 检查是否可以被中断
         */
        bool checkInterrupt()
        {
            if (!_onInterrupt)
                return true;
            return this->_onInterrupt();
        }

        /**
         * @brief 重置属性
         */
        void reset()
        {
            if (_onReset)
            {
                // 如果不是记住状态或者剩余时间小于等于0，则重置剩余时间
                if (!_is_remember || _remaining_time <= 0)
                    _remaining_time = _duration;

                _onReset();
            }
        }
    };
}