#pragma once

namespace aw_decision
{
    /**
     * @brief 形态类
     */
    enum PatternType
    {
        MOVE,
        ATTACK,
        DEFENSE
    };

    /**
     * @brief 形态类
     */
    class Pattern
    {
    private:
        PatternType _pattern;

    public:
        Pattern(PatternType pattern) : _pattern(pattern)
        {
        }

        ~Pattern() = default;

        PatternType getPattern() const
        {
            return _pattern;
        }

        bool operator==(PatternType pattern)
        {
            return _pattern == pattern;
        }
    };
}