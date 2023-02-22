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

    class Kitting;
    class Assembly;
    class Combined;

    class Order{
        std::string id;
        bool priority;
        int type;
        Order(ariac_msgs::msg::Order order){
            id = order.id;
            priority = order.priority;
            type = order.type;
            if (type == 0){
                group3::Kitting(order.kitting_task);
            }
            else if (type == 1){
                group3::Assembly(order.assembly_task);
            }
            else if (type == 2){
                group3::Combined(order.combined_task);
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
            agv_id = kitting.agv_number;
            tray_id = kitting.tray_id;
            destination = kitting.destination;
            for(int i = 0; i < kitting.parts.size(); i++){
                part[0] = kitting.parts[i].part.color;
                part[1] = kitting.parts[i].part.type;
                part[2] = kitting.parts[i].part.quadrant;
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
            geometry_msgs::msg::Vector3 install_direction;
        };
        part part_var;
        std::vector<part> parts;

        Assembly(ariac_msgs::msg::AssemblyTask assembly){
            agv_numbers = assembly.agv_numbers;
            station = assembly.station;
            for(int i = 0; i < assembly.parts.size(); i++){
                part_var.type = assembly.parts[i].part.type;
                part_var.color = assembly.parts[i].part.color;
                part_var.assembled_pose = assembly.parts[i].assembled_pose;
                part_var.install_direction = assembly.parts[i].install_direction;
                parts.push_back(part_var);
            }
        }
        ~Assembly(){}
    };

    class Combined: private Order{
        int station;
        struct part{
            int type;
            int color;
            geometry_msgs::msg::PoseStamped assembled_pose;
            geometry_msgs::msg::Vector3 install_direction;
        };
        part part_var;
        std::vector<part> parts;

        Combined(ariac_msgs::msg::CombinedTask combined){
            station = combined.station;
            for(int i = 0; i < combined.parts.size(); i++){
                part_var.type = combined.parts[i].part.type;
                part_var.color = combined.parts[i].part.color;
                part_var.assembled_pose = combined.parts[i].assembled_pose;
                part_var.install_direction = combined.parts[i].install_direction;
                parts.push_back(part_var);
            }
        }
        ~Combined(){}
    };
    
}