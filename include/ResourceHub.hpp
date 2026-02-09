#pragma once

#include <functional>
#include <iostream>

namespace aw_decision
{
    /**
     * @brief 共享资源站，存储所有状态需要访问的数据
     */
    class ResourceHub
    {
    public:
        struct CommonData
        {
            bool gamestart = false;
            bool is_fire = false;
            bool is_birth = false;
            int stage_remain_time = 0;
        };

        struct RobotStatus
        {
            int current_ammo = 0;
            int current_hp = 0;
            int we_outpost_hp = 0;
            int enemy_outpost_hp = 0;
        };

        struct Radar
        {
            int enemy_id = -1;
            double enemy_x = 0.0;
            double enemy_y = 0.0;
            double enemy_distance = 0.0;
            double enemy_angle = 0.0;
            bool lidar_flag = false;
        };

        CommonData common;
        RobotStatus robot;
        Radar radar;

        ResourceHub() = default;
        ~ResourceHub() = default;

        // 提供更新接口
        void update(CommonData new_common)
        {
            common = new_common;
        }
        void update(RobotStatus new_robot)
        {
            robot = new_robot;
        }
        void update(Radar new_radar)
        {
            radar = new_radar;
        }

        // 提供便捷的查询函数
    };
}