/**
 * @copyright Copyright (c) 2023
 * @file ariac_competition.hpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Class Definitions of RWA1 for ARIAC 2023 (Group 3)
 * @version 0.1
 * @date 2023-02-25
 * 
 * 
 */

#pragma once

#include <unistd.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <typeinfo>
#include <vector>
#include <chrono>
#include <map>
#include <utility>
#include <algorithm>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ariac_msgs/msg/assembly_part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/msg/bin_parts.hpp>
#include <ariac_msgs/msg/conveyor_parts.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class Orders;


/**
 * @brief Class definition for ARIAC Competition
 * 
 */
class AriacCompetition : public rclcpp::Node {
 public:
  unsigned int competition_state_;
  unsigned int submit_orders_ = 0;
  std::vector<Orders> orders;
  struct BinQuadrant {
    int part_type_clr = -1;
    geometry_msgs::msg::PoseStamped part_pose;
  };

  std::vector<int> conveyor_parts;

  std::map<int, BinQuadrant> bin_map;

  int search_bin(int);
  int search_conveyor(int);

  // void complete_kitting_task(Orders o);
  // void complete_assembly_task(Orders o);
  // void complete_combined_task(Orders o);

  void setup_map(){
    for(unsigned int i = 1; i <= 72; i++){
      bin_map[i];
    }
  }

  /**
   * @brief Construct a new Ariac Competition object
   * 
   * @param node_name Name of the node
   */
  AriacCompetition(std::string node_name) : Node(node_name) {
    competition_state_sub_ =
        this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10,
            std::bind(&AriacCompetition::competition_state_cb, this,
                      std::placeholders::_1));

    end_competition_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AriacCompetition::end_competition_timer_callback, this));

    order_subscriber_ = this->create_subscription<ariac_msgs::msg::Order>(
        "/ariac/orders", 10,
        std::bind(&AriacCompetition::order_callback, this,
                  std::placeholders::_1));

    bin_parts_subscriber_ = this->create_subscription<ariac_msgs::msg::BinParts>(
        "/ariac/bin_parts", 10,
        std::bind(&AriacCompetition::bin_parts_callback, this,
                  std::placeholders::_1));

    conveyor_parts_subscriber_ = this->create_subscription<ariac_msgs::msg::ConveyorParts>(
        "/ariac/conveyor_parts", 10,
        std::bind(&AriacCompetition::conveyor_parts_callback, this,
                  std::placeholders::_1));       
  }

 private:
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
      competition_state_sub_;

  rclcpp::TimerBase::SharedPtr end_competition_timer_;

  ariac_msgs::msg::Order order_;

  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subscriber_;
  rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_parts_subscriber_;
  rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_parts_subscriber_;

  /**
   * @brief Callback function for competition state subscriber and to start competition
   * 
   * @param msg CompetitionState message 
   */
  void competition_state_cb(
      const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

  /**
   * @brief Callback function to end the competition
   * 
   */
  void end_competition_timer_callback();

  /**
   * @brief Callback function to store the orders
   * 
   * @param msg Order
   */
  void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

  /**
   * @brief Callback function to retrieve bin part information
   * 
   * @param msg 
   */
  void bin_parts_callback(const ariac_msgs::msg::BinParts::SharedPtr msg);

  /**
   * @brief  Callback function to retrieve conveyor part information
   * 
   * @param msg 
   */
  void conveyor_parts_callback(const ariac_msgs::msg::ConveyorParts::SharedPtr msg);

  /**
   * @brief Method to submit the orders
   * 
   * @param order_id Order ID
   */
  void submit_order(std::string order_id);
};

/**
 * @brief Class Definition to store Kitting, Assembly and Combined orders
 * 
 */
class Orders {
 public:
  std::string id;
  bool priority;
  int type;

  struct KittingTask {
    int agv_id;
    int tray_id;
    int destination;
    std::array<int, 3> part;  // 0-color, 1-type, 2-quadrant
    std::vector<std::array<int, 3>> parts_kit;
  };

  KittingTask KittingTask_var;

  struct AssemblyTask {
    std::vector<int> agv_numbers;
    int station;
    struct part {
      int type;
      int color;
      geometry_msgs::msg::PoseStamped assembled_pose;
      geometry_msgs::msg::Vector3 install_direction;
    };
    part part_var;
    std::vector<part> parts_assm;
  };

  AssemblyTask AssemblyTask_var;

  struct CombinedTask {
    int station;
    struct part {
      int type;
      int color;
      geometry_msgs::msg::PoseStamped assembled_pose;
      geometry_msgs::msg::Vector3 install_direction;
    };
    part part_var;
    std::vector<part> parts_comb;
  };

  CombinedTask CombinedTask_var;

  /**
   * @brief Construct a new Orders object
   * 
   */
  Orders();
};

