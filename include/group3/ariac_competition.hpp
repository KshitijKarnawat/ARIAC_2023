#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <iostream>
#include <typeinfo>
#include <chrono>
#include <unistd.h>
#include <string>
#include <array>
#include <vector>
#include <queue>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/assembly_part.hpp>

#include <ariac_msgs/srv/submit_order.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;

class AriacCompetition : public rclcpp::Node
{
 public:
    unsigned int competition_state_;
    unsigned int submit_orders_ = 0;
    AriacCompetition(std::string node_name) : Node(node_name)
    {
        competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&AriacCompetition::competition_state_cb, this, std::placeholders::_1));

        // this->declare_parameter("publishing_interval", 2);
        // rclcpp::Parameter pub_frequency = this->get_parameter("publishing_interval");

        end_competition_timer_ = this->create_wall_timer(100ms, std::bind(&AriacCompetition::end_competition_timer_callback, this));

        order_subscriber = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, std::bind(&AriacCompetition::order_callback, this, std::placeholders::_1));
    }

 
 private:
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

    rclcpp::TimerBase::SharedPtr end_competition_timer_;
    
    ariac_msgs::msg::Order order_;

    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subscriber;

    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

    void end_competition_timer_callback();

    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    void submit_order(std::string order_id);

};

class Orders {

 public:
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

    Orders();

};

std::vector<Orders> orders;