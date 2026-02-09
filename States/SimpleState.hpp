#pragma once

#include "State.hpp"
#include "Action.hpp"
#include "ResourceHub.hpp"
#include <functional>

namespace aw_decision
{
    /**
     * @brief 一般状态类
     */
    class SimpleState : public State
    {
    public:
        SimpleState(std::string name, Actions type) : State(name, type)
        {
        }
        ~SimpleState() = default;

    private:
        std::function<void()> _onEnter;
        std::function<Actions()> _onExec;
        std::function<void()> _onExit;
        std::function<void()> _onReset;
        std::function<bool()> _onInterrupt;
        std::function<bool()> _onCondition;

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
            if (this->_onExec)
            {
                return this->_onExec();
            }
            else
            {
                std::cerr << "No onExec function defined for state: " << this->getName() << std::endl;
                return DEFAULT;
            }
        }

        /**
         * @brief 状态退出时调用
         */
        void onExit()
        {
            if (this->_onExit)
            {
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
            if (this->_onReset)
            {
                this->_onReset();
            }
        }
    };
}