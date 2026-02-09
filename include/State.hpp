#pragma once

#include "Action.hpp"

#include <string>
#include <memory>
#include <chrono>

namespace aw_decision
{
    /**
     * @brief 状态属性基类（插件接口）
     */
    class State
    {
    private:
        /**
         * @brief 状态名称(调试用)
         */
        std::string name_;
        /**
         * @brief 状态类型
         */
        Actions type_ = Actions::DEFAULT;

    public:
        State(std::string name, Actions type)
        {
            name_ = name;
            type_ = type;
        }

        std::string getName() const
        {
            return name_;
        }

        Actions getType() const
        {
            return type_;
        }

        ~State() = default;

        /**
         * @brief 状态进入时调用
         */
        virtual void onEnter() {}

        /**
         * @brief 每次状态执行时调用
         * @return true 继续执行，false 强制退出状态
         */
        virtual Actions onExec() { return Actions::DEFAULT; }

        /**
         * @brief 状态退出时调用
         */
        virtual void onExit() {}

        /**
         * @brief 检查是否满足进入条件
         */
        virtual bool checkCondition() { return false; }

        /**
         * @brief 检查是否可以被中断
         */
        virtual bool checkInterrupt() { return true; }

        /**
         * @brief 重置属性
         */
        virtual void reset() {}
    };
}