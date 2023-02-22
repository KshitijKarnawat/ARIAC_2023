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
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/condition.hpp>
#include <ariac_msgs/msg/challenge.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/assembly_part.hpp>

namespace group3{

    class Order{
        std::string id;
        bool priority;
        int type;
        Order(ariac_msgs::msg::Order order){
            id = order.id;
            priority = order.priority;
            type = order.type;
            if (type == 0){
                Kitting(order.kitting);
            }
            else if (type == 1){
                Assembly(order.assembly);
            }
            else if (type == 2){
                Combined(order.combined);
            }
        }
        ~Order(){}
    };

    class Kitting: private Order{
        int agv_id;
        int tray_id;
        int destination;
        int part[3];    // 0-color, 1-type, 2-quadrant
        std::vector<std::array<int, 3>> parts;

        Kitting(ariac_msgs::msg::KittingTask kitting){
            agv_id = kitting.agv_number
            tray_id = kitting.tray_id;
            destination = kitting.destination;
            for(int i = 0; i < kitting.parts.size(); i++){
                part[0] = kitting.parts[i].color;
                part[1] = kitting.parts[i].type;
                part[2] = kitting.parts[i].quadrant;
                parts.push_back(part);
            }
        }
        ~Kitting(){}
    };

    class Assembly: private Order{
        std::vector<int> agv_numbers;
        int station;
        struct part{
            int type;
            int color;
            geometry_msgs::msg::PoseStamped assembled_pose;
            geometry_msgs::msg::Vector3 assembly_dir;
        };
        std::vector<part> parts;

        Assembly(ariac_msgs::msg::AssemblyTask assembly){
            agv_id = assembly.agv_numbers;
            station_id = assembly.station;
            for(int i = 0; i < assembly.parts.size(); i++){
                part.type = assembly.parts[i].type;
                part.color = assembly.parts[i].color;
                part.assembled_pose = assembly.parts[i].assembled_pose;
                part.assembly_dir = assembly.parts[i].assembly_dir;
                parts.push_back(part);
            }
        }
        ~Assembly(){}
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

        Combined(ariac_msgs::msg::CombinedTask combined){
            station_id = combined.station_id;
            for(int i = 0; i < combined.parts.size(); i++){
                part.type = combined.parts[i].type;
                part.color = combined.parts[i].color;
                part.assembled_pose = combined.parts[i].assembled_pose;
                part.assembly_dir = combined.parts[i].assembly_dir;
                parts.push_back(part);
            }
        }
        ~Combined(){}
    };
    
}