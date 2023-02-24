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

class Orders: public rclcpp::Node{

 private:
    std::string id;
    bool priority;
    int type;
    
    struct KittingTask{
        int agv_id;
        int tray_id;
        int destination;
        std::array<int, 3> part;    // 0-color, 1-type, 2-quadrant
        std::vector<std::array<int, 3>> parts_kit;
    };

    KittingTask KittingTask_var;

    struct AssemblyTask{
        std::vector<int> agv_numbers;
        int station;
        struct part{
            int type;
            int color;
            geometry_msgs::msg::PoseStamped assembled_pose;
            geometry_msgs::msg::Vector3 install_direction;
        };
        part part_var;
        std::vector<part> parts_assm;
    };

    AssemblyTask AssemblyTask_var;

    struct CombinedTask{
        int station;
        struct part{
            int type;
            int color;
            geometry_msgs::msg::PoseStamped assembled_pose;
            geometry_msgs::msg::Vector3 install_direction;
        };
        part part_var;
        std::vector<part> parts_comb;
    };

    CombinedTask CombinedTask_var;

    ariac_msgs::msg::Order order_;
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subscriber;

    // methods
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

 public:

    Orders(std::string node_name) : Node(node_name) {
        order_subscriber = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, std::bind(&Orders::order_callback, this, std::placeholders::_1));
    }
};
