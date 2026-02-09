#pragma once

namespace aw_decision
{
    /***
     * @brief Position2D as a simple type to store 2D point
     * @param x position x
     * @param y position y
     * @param yaw yaw angle [radian]
     */
    struct Position2D
    {
        double x;   // position x
        double y;   // position y
        double yaw; // yaw angle [radian]
    };
}