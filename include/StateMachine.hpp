#pragma once

#include <vector>
#include <iostream>
#include "State.hpp"

namespace aw_decision
{
    /**
     * @brief 状态机类
     * @brief 这里定义外部可调用的函数，不建议外部直接调用state的函数
     *
     */
    class StateMachine
    {
    private:
        /**
         * @brief 存储状态的动态数组
         *
         */
        std::vector<std::shared_ptr<State>> _states;
        /**
         * @brief 状态机当前状态
         *
         */
        std::shared_ptr<State> _currentState = nullptr;
        /**
         * @brief 状态机下次状态
         *
         */
        std::shared_ptr<State> _nextState = nullptr;
        /**
         * @brief 状态是否被中断
         *
         */
        bool _interrupt = false;
        /**
         * @brief 切换状态时传递的信号，由状态自行控制，默认为无动作(-1)
         *
         */
        int _action = Actions::DEFAULT;

    public:
        StateMachine() = default;
        ~StateMachine() = default;

    public:
        /**
         * @brief 状态机添加状态(共享指针形式)
         *
         * @param state 单一状态
         */
        void addState(std::shared_ptr<State> state)
        {
            if (state)
            {
                _states.push_back(state);
            }
        }
        /**
         * @brief 初始化状态机
         *
         * @param state 初始状态
         */
        void initState(std::shared_ptr<State> state)
        {
            if (state == nullptr)
            {
                return;
            }
            _currentState = state; // 初始状态
            _nextState = state;
            _action = state->getType();
            state->onEnter();
        }
        /**
         * @brief 初始化状态机
         *
         * @param pos 指针向量中对应下标的状态
         */
        void initState(int pos = 0)
        {
            if (pos < 0 || pos >= static_cast<int>(_states.size()))
            {
                return;
            }
            _currentState = _states[pos];
            _nextState = _states[pos];
            _action = _currentState->getType();
            _currentState->onEnter();
        }

    public:
        /**
         * @brief 更新一次状态机
         *
         */
        void update()
        {
            // 打印当前状态和次态
            std::cout << "[StateMachine] 现态: "
                      << (_currentState ? _currentState->getName() : "nullptr")
                      << " | 次态: "
                      << (_nextState ? _nextState->getName() : "nullptr")
                      << " | 中断标志: " << (_interrupt ? "true" : "false")
                      << std::endl;
            // 检查状态机是否正常
            checkEveryThingsOk();
            // 状态执行且切换次态
            execute();
            // 状态轮询
            std::shared_ptr<State> tmp_state = poll();
            // 检查是否要打断当前状态
            _interrupt = checkIfInterrupt(tmp_state);
            // 状态切换
            stateUpdate(_interrupt);
        }

        /**
         * @brief 检查状态机是否正常
         *
         */
        bool checkEveryThingsOk()
        {
            if (_states.empty())
            {
                return false;
            }
            return true;
        }

        /**
         * @brief 状态执行
         *
         */
        void execute()
        {
            if (_currentState == nullptr)
            {
                return;
            }
            else
            {
                _action = _currentState->onExec();
                if (_action == Actions::DEFAULT)
                {
                    _nextState = nullptr;
                }
                else
                {
                    _nextState = _currentState;
                }
            }
        }

        /**
         * @brief 状态轮询
         *
         */
        std::shared_ptr<State> poll()
        {
            for (auto state : _states)
            {
                if (state->checkCondition())
                {
                    return state;
                }
            }
            return nullptr;
        }

        /**
         * @brief 检查是否要打断当前状态
         *
         ** @param next_state 轮询到的下一个状态
         */
        bool checkIfInterrupt(std::shared_ptr<State> next_state)
        {
            if (next_state == nullptr)
            {
                return false;
            }

            if (_currentState == nullptr)
            {
                _nextState = next_state;
                return false;
            }
            else if (_currentState == next_state)
            {
                return false;
            }
            else if (_currentState != _nextState)
            {
                _nextState = next_state;
                return false;
            }
            else
            {
                if (_currentState->checkInterrupt())
                {
                    _nextState = next_state;
                    return true;
                }
                return false;
            }

            std::cerr << "异常" << std::endl;
            return false;
        }

        /**
         * @brief 状态切换
         *
         ** @param interrupt 是否要打断当前状态
         */
        void stateUpdate(bool interrupt)
        {
            if (interrupt)
            {
                _currentState->reset();
                _currentState = _nextState;
                _currentState->onEnter();
                _action = _currentState->getType();
            }
            else
            {
                if (_currentState == nullptr)
                {
                    if (_nextState != nullptr)
                    {
                        _currentState = _nextState;
                        _currentState->onEnter();
                        _action = _currentState->getType();
                        return;
                    }
                    else
                    {
                        return;
                    }
                }
                else if (_currentState == _nextState)
                {
                    return;
                }
                else
                {
                    _currentState->onExit();
                    _currentState = _nextState;
                    if (_currentState != nullptr)
                    {
                        _currentState->onEnter();
                        _action = _currentState->getType();
                    }
                    else
                    {
                        _action = Actions::DEFAULT;
                    }
                }
            }
        }
    };
} // namespace aw_decision
