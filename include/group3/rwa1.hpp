#pragma once
#include <cstdint>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace group3{

    class Order{
        std::string id;
        bool priority;
        int type;
    };

    class Kitting: private Order{
        int agv_id;
        int tray_id;
        int destination;
        int part[3];    // 0-color, 1-type, 2-quadrant
        std::vector<std::array<int, 3>> parts;
    };

    class Assembly: private Order{
        int agv_id;
        int station_id;
        struct part{
            int type;
            int color;
            geometry_msgs::msg::PoseStamped assembled_pose;
            geometry_msgs::msg::Vector3 assembly_dir;
        };
        std::vector<part> parts;
    };

    class Combined: private Order{
        int station_id;
        struct part{
            int type;
            int color;
            geometry_msgs::msg::PoseStamped assembled_pose;
            geometry_msgs::msg::Vector3 assembly_dir;
        };
        std::vector<part> parts;
    };
    
}