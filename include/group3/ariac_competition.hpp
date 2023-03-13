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
#include <rclcpp_action/rclcpp_action.hpp>
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
  int flag{0};
  unsigned int submit_orders_ = 0;
  int competition_state_ = -1;
  std::vector<Orders> incomplete_orders;
  std::vector<Orders> current_order;
  std::vector<Orders> submitted_orders;

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

    end_competition_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AriacCompetition::end_competition_timer_callback, this));     

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

  void process_order();
};


class Kitting {
 public:
    Kitting(unsigned int agv_number,
                    unsigned int tray_id,
                    unsigned int destination,
                    const std::vector<std::array<int, 3>> _parts_kit) : agv_id_(agv_number),
                                                                tray_id_(tray_id),
                                                                destination_(destination),
                                                                parts_kit_(_parts_kit) {}
    unsigned int GetAgvId() const { return agv_id_; }
    unsigned int GetTrayId() const { return tray_id_; }
    unsigned int GetDestination() const { return destination_; }
    const std::vector<std::array<int, 3>> GetParts() const { return parts_kit_; }
 private:
    unsigned int agv_id_;
    unsigned int tray_id_;
    unsigned int destination_;
    std::vector<std::array<int, 3>> parts_kit_;
};

struct Part {
      int type;
      int color;
      geometry_msgs::msg::PoseStamped assembled_pose;
      geometry_msgs::msg::Vector3 install_direction;
};
class Assembly {
 public:
    Assembly(std::vector<unsigned int> agv_numbers, unsigned int station, std::vector<Part> parts_assm) : agv_numbers_(agv_numbers),
                                                                                        station_(station),
                                                                                        parts_assm_(parts_assm) {}
    // std::vector<int> GetAgvNums() const {return agv_numbers_;}
    // std::vector<Part> GetPartAssm() const {return parts_assm_;}

    const std::vector<unsigned int> GetAgvNumbers() const { return agv_numbers_; }

    unsigned int GetStation() const { return station_; }

    const std::vector<Part> GetParts() const { return parts_assm_; }
 private:
    std::vector<unsigned int> agv_numbers_;
    unsigned int station_;
    std::vector<Part> parts_assm_;
};

class Combined {
 public:
    Combined(unsigned int _station, std::vector<Part> parts_comb) : station_(_station),
                                                            parts_comb_(parts_comb) {}
    unsigned int GetStation() const { return station_; }

    const std::vector<Part> GetParts() const { return parts_comb_; }
 private:
    unsigned int station_;
    std::vector<Part> parts_comb_;
};

class Orders {
 protected:
    std::string id_;
    unsigned int type_;
    bool priority_;
    std::shared_ptr<Kitting> kitting_ = nullptr;
    std::shared_ptr<Assembly> assembly_ = nullptr;
    std::shared_ptr<Combined> combined_ = nullptr;
 public:
    Orders(std::string id,
              unsigned int type,
              bool priority) : id_(id),
                                type_(type),
                                priority_(priority) {}
    ~Orders() = default;
    std::string GetId() const { return id_; }
    unsigned int GetType() const { return type_; }
    bool IsPriority() const { return priority_; }
    std::shared_ptr<Kitting> GetKitting() const { return kitting_; }
    virtual void SetKitting(std::shared_ptr<Kitting> _kitting) { kitting_ = _kitting; }
    std::shared_ptr<Assembly> GetAssembly() const { return assembly_; }
    virtual void SetAssembly(std::shared_ptr<Assembly> _assembly) { assembly_ = _assembly; }
    std::shared_ptr<Combined> GetCombined() const { return combined_; }
    virtual void SetCombined(std::shared_ptr<Combined> _combined) { combined_ = _combined; }

};

std::vector<Orders> orders;

