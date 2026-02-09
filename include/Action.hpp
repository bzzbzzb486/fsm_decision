#pragma once

namespace aw_decision
{
    /**
     * @brief 定义信号(有几个自定义状态就有几个信号，按优先级排列)
     *
     */
    enum Actions
    {
        /**
         * @brief 提供给状态与状态机，表示无默认信号
         *
         */
        DEFAULT = -1,
        /**
         * @brief 状态在下面添加
         *
         */
        WAIT_FOR_GAMESTART,
        MOVE_PATTERN,
        ATTACK_PATTERN,
        DEFENSE_PATTERN,
        GO_HOME,
        GOTO_A,
        GOTO_B,
        GOTO_C,
    };
}