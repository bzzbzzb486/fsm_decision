#pragma once

#include "State.hpp"
#include "Action.hpp"
#include "ResourceHub.hpp"
#include <functional>
#include <vector>
#include <iostream>
#include <string>

namespace aw_decision
{
    /**
     * @brief 顺序序列类
     */
    class Sequence
    {
    private:
        std::vector<Actions> _sequence;
        size_t _currentIndex;

    public:
        Sequence(std::vector<Actions> sequence, size_t currentIndex)
            : _sequence(sequence), _currentIndex(currentIndex)
        {
        }
        ~Sequence() = default;

        void StitchToNext()
        {
            if (_currentIndex < _sequence.size() - 1)
                _currentIndex++;
            else
                _currentIndex = 0;
        }

        bool NeedContinue(Actions action)
        {
            if (_sequence.empty() || _sequence.size() < 2)
                return false;
            return action != _sequence[_currentIndex];
        }

        /**
         * @brief 获取当前索引
         */
        size_t getCurrentIndex() const { return _currentIndex; }

        /**
         * @brief 获取当前索引指向的动作
         */
        Actions getCurrentAction() const
        {
            if (_currentIndex < _sequence.size())
                return _sequence[_currentIndex];
            return Actions::DEFAULT;
        }
    };

    /**
     * @brief 顺序状态类
     */
    class OrderState : public State
    {
    public:
        OrderState(std::string name, Actions type) : State(name, type)
        {
        }
        ~OrderState() = default;

    private:
        std::function<void()> _onEnter;
        std::function<Actions()> _onExec;
        std::function<void()> _onExit;
        std::function<void()> _onReset;
        std::function<bool()> _onInterrupt;
        std::function<bool()> _onCondition;
        std::shared_ptr<Sequence> _sequence;

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

        void setSequence(std::shared_ptr<Sequence> sequence)
        {
            this->_sequence = sequence;
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
                _sequence->StitchToNext();
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
            if (!_onCondition || !_sequence)
                return false;

            bool continue_ = _sequence->NeedContinue(this->getType());
            bool condition_result = this->_onCondition();
            bool final_result = condition_result && !continue_;

            // 打印调试信息
            // std::cout << "[OrderState] 检查 " << this->getName()
            //           << " (type=" << this->getType() << ")"
            //           << " | 序列当前索引: " << _sequence->getCurrentIndex()
            //           << " | 序列指向状态: " << _sequence->getCurrentAction()
            //           << " | NeedContinue: " << (continue_ ? "true" : "false")
            //           << " | _onCondition: " << (condition_result ? "true" : "false")
            //           << " | 最终结果: " << (final_result ? "true" : "false")
            //           << std::endl;

            return final_result;
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