/**
 * @copyright Copyright (c) 2023
 * @file ariac_competition.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of RWA2 for ARIAC 2023 (Group 3)
 * @version 0.2
 * @date 2023-03-04
 * 
 * 
 */

#include "../include/group3/ariac_competition.hpp"

AriacCompetition::AriacCompetition(std::string node_name): Node(node_name),
  floor_robot_node_(std::make_shared<rclcpp::Node>("floor_robot")),
  ceil_robot_node_(std::make_shared<rclcpp::Node>("ceiling_robot")),
  executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
  planning_scene_() 
{
  
  auto floor_mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
        "floor_robot",
        "robot_description");
  floor_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(floor_robot_node_, floor_mgi_options);
  
  auto ceil_mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
        "ceiling_robot",
        "robot_description");
  ceil_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(ceil_robot_node_, ceil_mgi_options);
  
  if (floor_robot_->startStateMonitor()) {
      RCLCPP_INFO(this->get_logger(), "Floor Robot State Monitor Started");
  } else {
      RCLCPP_ERROR(this->get_logger(), "Floor Robot State Monitor Failed to Start");
  }
  
  floor_robot_->setMaxAccelerationScalingFactor(1.0);
  floor_robot_->setMaxVelocityScalingFactor(1.0);

  if (ceil_robot_->startStateMonitor()) {
      RCLCPP_INFO(this->get_logger(), "Ceiling Robot State Monitor Started");
  } else {
      RCLCPP_ERROR(this->get_logger(), "Ceiling Robot State Monitor Failed to Start");
  }
  
  ceil_robot_->setMaxAccelerationScalingFactor(1.0);
  ceil_robot_->setMaxVelocityScalingFactor(1.0);

  rclcpp::SubscriptionOptions options;
  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = topic_cb_group_;

  rclcpp::SubscriptionOptions options2;
  topic_cb_group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options2.callback_group = topic_cb_group2_;

  rclcpp::SubscriptionOptions options3;
  order_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options3.callback_group = order_cb_group_;
  
  competition_state_sub_ =
      this->create_subscription<ariac_msgs::msg::CompetitionState>(
          "/ariac/competition_state", 10,
          std::bind(&AriacCompetition::competition_state_cb, this,
                  std::placeholders::_1));

  order_subscriber_ = this->create_subscription<ariac_msgs::msg::Order>(
      "/ariac/orders", 10,
      std::bind(&AriacCompetition::order_callback, this,
              std::placeholders::_1), options3);

  conveyor_parts_subscriber_ = this->create_subscription<ariac_msgs::msg::ConveyorParts>(
      "/ariac/conveyor_parts", 10,
      std::bind(&AriacCompetition::conveyor_parts_callback, this,
              std::placeholders::_1)); 
  
  conv_camera_sub_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
      "/ariac/sensors/conv_basic_camera/image", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&AriacCompetition::conv_camera_cb, this, std::placeholders::_1), options);
  
  conv_part_detector_sub_ = this->create_subscription<group3::msg::Part>(
        "/conveyor_part_detector", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::conv_part_detector_cb, this, std::placeholders::_1), options);
  
  kts1_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/kts1_rgb_camera/rgb_image", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&AriacCompetition::kts1_rgb_camera_cb, this, std::placeholders::_1), options);

  kts2_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/kts2_rgb_camera/rgb_image", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&AriacCompetition::kts2_rgb_camera_cb, this, std::placeholders::_1), options);

  left_bins_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/left_bins_rgb_camera/rgb_image", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&AriacCompetition::left_bins_rgb_camera_cb, this, std::placeholders::_1), options);

  right_bins_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/right_bins_rgb_camera/rgb_image", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&AriacCompetition::right_bins_rgb_camera_cb, this, std::placeholders::_1), options);

  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&AriacCompetition::floor_gripper_state_cb, this, std::placeholders::_1), options2);

  ceil_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/ceiling_robot_gripper_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&AriacCompetition::ceil_gripper_state_cb, this, std::placeholders::_1), options2);

  right_part_detector_sub_ = this->create_subscription<group3::msg::Parts>(
        "/right_bin_part_detector", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::right_part_detector_cb, this, std::placeholders::_1), options);

  left_part_detector_sub_ = this->create_subscription<group3::msg::Parts>(
        "/left_bin_part_detector", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::left_part_detector_cb, this, std::placeholders::_1), options);

  breakbeam_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
        "/ariac/sensors/breakbeam_0/status", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::breakbeam_cb, this, std::placeholders::_1), options);

  breakbeam1_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
        "/ariac/sensors/breakbeam_1/status", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::breakbeam1_cb, this, std::placeholders::_1), options);

  breakbeam2_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
        "/ariac/sensors/breakbeam_2/status", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::breakbeam2_cb, this, std::placeholders::_1), options);

  as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&AriacCompetition::as1_state_cb, this, std::placeholders::_1), options2);
  
  as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&AriacCompetition::as2_state_cb, this, std::placeholders::_1), options2);

  as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&AriacCompetition::as3_state_cb, this, std::placeholders::_1), options2);

  as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&AriacCompetition::as4_state_cb, this, std::placeholders::_1), options2);

  AddModelsToPlanningScene();

  end_competition_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AriacCompetition::end_competition_timer_callback, this)); 

  executor_->add_node(floor_robot_node_);
  executor_->add_node(ceil_robot_node_);
  executor_thread_ = std::thread([this]()
                                   { this->executor_->spin(); });   

  RCLCPP_INFO(this->get_logger(), "Initialization successful \033[0m");
  
}

void AriacCompetition::competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) {
  competition_state_ = msg->competition_state;

  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
    if (!competition_started_) {
      std::string srv_name = "/ariac/start_competition";

      std::shared_ptr<rclcpp::Node> node =
          rclcpp::Node::make_shared("start_trigger_client");
      
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
          node->create_client<std_srvs::srv::Trigger>(srv_name);

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

      while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
                        "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO_STREAM(this->get_logger(),
                            "Service not available, waiting again...");
      }

      auto result = client->async_send_request(request);

      if (rclcpp::spin_until_future_complete(node, result) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting Competition");
        competition_started_ = true;
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
      }
    }
  }
}

void AriacCompetition::end_competition_timer_callback() {
  if (competition_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submit_orders_) {
    std::string srv_name = "/ariac/end_competition";

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("end_trigger_client");
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
        node->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
      RCLCPP_INFO_STREAM(this->get_logger(), std::string("\033[92;5m") + std::string("All Orders Submitted and Ending Competition") + std::string("\033[0m"));
      RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
      // Exit spin loop to end competition
      executor_->cancel();
      executor_thread_.join();
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }
  }
  
  else if ((!orders.empty() || !current_order.empty()) && conveyor_parts_flag_) {
    // bool flag;
    // flag = process_order();
    process_order();
  }
  else if(orders.empty() && current_order.empty() && incomplete_order.empty() && conveyor_parts_flag_){
    submit_orders_ = true;
    bool is_pump; // Stores whether the part is a pump or not for conveyor belt

    populate_bin_part();
    // Conveyor belt part detection and picking
    while (conveyor_parts.size()!=0){
      if (conveyor_size == conveyor_parts.size()){
      floor_robot_->setJointValueTarget("linear_actuator_joint", -2.75);
      floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 3.14);
      floor_robot_->setJointValueTarget("floor_shoulder_lift_joint", -0.942478);
      FloorRobotMovetoTarget();
      FloorRobotMoveConveyorHome();
      }
      while(!breakbeam_status){
        if (breakbeam2_status){
          is_pump = true;
          pump_rgb = conv_rgb_parts_;
        }
      }
      if (breakbeam_status){
        std::vector<geometry_msgs::msg::Pose> part_pose;
        group3::msg::Part part_rgb;
        part_pose = conv_parts_;
        part_rgb = conv_rgb_parts_;
        if (is_pump){
          is_pump = false;
          FloorRobotPickConvPart(part_pose, pump_rgb);
        }
        else {
          FloorRobotPickConvPart(part_pose, part_rgb);
        }
      } 
    }
  }
}

void AriacCompetition::order_callback(ariac_msgs::msg::Order::SharedPtr msg) {
  Orders order(msg->id, msg->type, msg->priority);
  if(order.IsPriority() == 1) {
    high_priority_order_ = true;
  }

  // Saving KITTING order information
  if (order.GetType() == ariac_msgs::msg::Order::KITTING) {
    std::array<int, 3> part;
    std::vector<std::array<int, 3>> _parts_kit;
    
    for (unsigned int i = 0; i < msg->kitting_task.parts.size(); i++) {
      part[0] = msg->kitting_task.parts[i].part.color;
      part[1] = msg->kitting_task.parts[i].part.type;
      part[2] = msg->kitting_task.parts[i].quadrant;
      _parts_kit.push_back(part);
    }
    
    Kitting kitting_(msg->kitting_task.agv_number, msg->kitting_task.tray_id, 
                    msg->kitting_task.destination, _parts_kit);
    order.SetKitting(std::make_shared<Kitting> (kitting_));
  }  else if (order.GetType() == ariac_msgs::msg::Order::ASSEMBLY) {
    // Saving ASSEMBLY order information
    std::vector<unsigned int> _agv_numbers;
    
    for (unsigned int i = 0; i < msg->assembly_task.agv_numbers.size(); i++) {
      _agv_numbers.push_back(
      static_cast<int>(msg->assembly_task.agv_numbers.at(i)));
    }
    
    Part part;
    std::vector<Part> _parts_assem;
    for (unsigned int i = 0; i < msg->assembly_task.parts.size(); i++) {
      part.type =
          msg->assembly_task.parts[i].part.type;
      part.color =
          msg->assembly_task.parts[i].part.color;
      part.assembled_pose =
          msg->assembly_task.parts[i].assembled_pose;
      part.install_direction =
          msg->assembly_task.parts[i].install_direction;
      _parts_assem.push_back(part);
    }

    Assembly assembly_(_agv_numbers, msg->assembly_task.station, _parts_assem);
    order.SetAssembly(std::make_shared<Assembly> (assembly_));
  }  else if (order.GetType() == ariac_msgs::msg::Order::COMBINED) {
    // Saving COMBINED order information
    Part part;
    std::vector<Part> _parts_comb;

    for (unsigned int i = 0; i < msg->combined_task.parts.size(); i++) {
      part.type =
          msg->combined_task.parts[i].part.type;
      part.color =
          msg->combined_task.parts[i].part.color;
      part.assembled_pose =
          msg->combined_task.parts[i].assembled_pose;
      part.install_direction =
          msg->combined_task.parts[i].install_direction;
      _parts_comb.push_back(part);
    }

    Combined combined_(msg->combined_task.station, _parts_comb);
    order.SetCombined(std::make_shared<Combined> (combined_));
  }

  submit_orders_ = false;

  // Insert order based on priority value 
  if (order.IsPriority() == 0 || orders.size() == 0) {
    orders.push_back(order);
  } else if (orders[orders.size() - 1].IsPriority() == 1) {
    orders.push_back(order);
  } else {
    for (unsigned int i = 0; i < orders.size(); i++) {
      if (orders[i].IsPriority() == 0) {
        orders.insert(i + orders.begin(), order);
        break;
      }
    }
  }
}

void AriacCompetition::populate_bin_part(){
  AriacCompetition::setup_map();
  bin_quadrant_poses = define_poses();
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin map setup");
  while (!right_bins_rgb_camera_received_data && 
         !left_bins_rgb_camera_received_data) {}

  std::vector<std::vector<int>> right_bin;
  for (unsigned int i = 0; i < right_parts_.size(); i++) {
    std::vector<int> right_bin_part;
    right_bin_part.push_back(right_parts_[i].color);
    right_bin_part.push_back(right_parts_[i].type);
    right_bin_part.push_back(right_parts_[i].quad);
    right_bin.push_back(right_bin_part);
  }
  // std::vector<std::vector<int>> right_bin = rightbin(right_bins_rgb_camera_image_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Vector Information populated");
  // std::vector<std::vector<int>> left_bin = leftbin(left_bins_rgb_camera_image_);
  std::vector<std::vector<int>> left_bin;
  for (unsigned int i = 0; i < left_parts_.size(); i++) {
    std::vector<int> left_bin_part;
    left_bin_part.push_back(left_parts_[i].color);
    left_bin_part.push_back(left_parts_[i].type);
    left_bin_part.push_back(left_parts_[i].quad);
    left_bin.push_back(left_bin_part);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Left Vector Information populated");
  int count_right = 0;
  int count_left = 0;
  for (auto part : right_bin){
    occupied_quadrants.push_back(part[2]);
    bin_map[part[2]].part_type_clr = (part[1]*10 + part[0]);
    bin_map[part[2]].part_pose = bin_quadrant_poses[part[2]];
    count_right++;
    RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Information populated with " << bin_map[part[2]].part_type_clr << " " << bin_map[part[2]].part_pose.position.x << " " << part[2]);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Information populated");
  for (auto part : left_bin){
    occupied_quadrants.push_back(part[2]);
    bin_map[part[2]].part_type_clr = (part[1]*10 + part[0]);
    bin_map[part[2]].part_pose = bin_quadrant_poses[part[2]];
    count_left++;
    RCLCPP_INFO_STREAM(this->get_logger(), "Bin left Information populated with " << bin_map[part[2]].part_type_clr << " " << bin_map[part[2]].part_pose.position.x << " " << part[2]);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Left Information populated");
}

void AriacCompetition::conveyor_parts_callback(ariac_msgs::msg::ConveyorParts::SharedPtr msg) {
  
  for (unsigned int part_idx = 0; part_idx < msg->parts.size(); part_idx++) {
    for (int qty = 0; qty < msg->parts[part_idx].quantity; qty++) {
      conveyor_parts.push_back((msg->parts[part_idx].part.type)*10 + (msg->parts[part_idx].part.color));
    }
    conveyor_size = conveyor_parts.size();
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Conveyor Part Information populated: " << conveyor_parts.size());
  conveyor_parts_flag_ = true;
  conveyor_parts_subscriber_.reset();
}

void AriacCompetition::submit_order(std::string order_id) {
  std::string srv_name = "/ariac/submit_order";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("submit_order_client");
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client =
      node->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(this->get_logger(),"submit_order_client response: " << result.get()->success << " " << result.get()->message);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service submit_order");
  }
}

bool AriacCompetition::process_order() {

  if (high_priority_order_ == false) {
      current_order.push_back(orders.at(0));
      orders.erase(orders.begin());
      RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
      RCLCPP_INFO_STREAM(this->get_logger(), "Doing Task " <<  current_order[0].GetId() << " Priority: "  << std::to_string(current_order[0].IsPriority()));
      RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
      if(current_order[0].GetType() == ariac_msgs::msg::Order::KITTING) {
        if ( do_kitting(current_order) == true) {
          submit_order(current_order[0].GetId());
          current_order.clear();
          return true;
        }
        else {
          return false;
        }
      }
      else if (current_order[0].GetType() == ariac_msgs::msg::Order::ASSEMBLY) {
        if ( do_assembly(current_order) == true) {
          submit_order(current_order[0].GetId());
          current_order.clear();
          return true;
        }
        else {
          return false;
        }
      }
      else if (current_order[0].GetType() == ariac_msgs::msg::Order::COMBINED) {
        if (do_combined(current_order) == true) {
          submit_order(current_order[0].GetId());
          current_order.clear();
          return true;
        }
        else {
          return false;
        }
      }
  } 
  else if (high_priority_order_ == true){
    if (current_order.size() != 0) {
      incomplete_order.push_back(current_order[0]);
      current_order.clear();
      current_order.push_back(orders.at(0));
      orders.erase(orders.begin());
    } else {
      current_order.push_back(orders.at(0));
      orders.erase(orders.begin());
    }
    doing_priority = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
    RCLCPP_INFO_STREAM(this->get_logger(), "Doing Task " <<  current_order[0].GetId() << " Priority: "  << std::to_string(current_order[0].IsPriority()));
    RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
    if(current_order[0].GetType() == ariac_msgs::msg::Order::KITTING) {
      high_priority_order_ = false;
      if (do_kitting(current_order) == true) {
        submit_order(current_order[0].GetId());
        current_order.clear();
        if (incomplete_order.size() != 0) {
          current_order.push_back(incomplete_order[0]);
          incomplete_order.erase(incomplete_order.begin());
        }
        doing_priority = false;
        return true;
      }
      else {
        return false;
      }
    }
    else if (current_order[0].GetType() == ariac_msgs::msg::Order::ASSEMBLY) {
      high_priority_order_ = false;
      if ( do_assembly(current_order) == true) {
        submit_order(current_order[0].GetId());
        current_order.clear();
        if (incomplete_order.size() != 0) {
          current_order.push_back(incomplete_order[0]);
          incomplete_order.erase(incomplete_order.begin());
        }
        doing_priority = false;
        return true;
      }
      else {
        return false;
      }
    }
    else if (current_order[0].GetType() == ariac_msgs::msg::Order::COMBINED) {
      high_priority_order_ = false;
      if ( do_combined(current_order) == true) {
        submit_order(current_order[0].GetId());
        current_order.clear();
        if (incomplete_order.size() != 0) {
          current_order.push_back(incomplete_order[0]);
          incomplete_order.erase(incomplete_order.begin());
        }
        doing_priority = false;
        return true;
      }
      else {
        return false;
      }
    }
  }
}

bool AriacCompetition::do_kitting(std::vector<Orders> current_order) {
  int type_color_key;  // Stores the key of the specific part in the bin map
  int type_color_key_replacement;  // Stores the key of the specific part in the bin map
  int type_color_key_missing;  // Stores the key of the specific part in the bin map
  std::vector<std::array<int, 2>> keys; // Stores the key of the specific part in the bin map and whether it is present or not
  std::vector<std::array<int, 2>> keys_dropped; // Stores the key of the specific part that has been dropped in the bin map and whether it is present or not
  int type_color;   // Stores type and color info: For ex: 101 -> Battery Green
  bool is_pump; // Stores whether the part is a pump or not for conveyor belt

  populate_bin_part();
  // Conveyor belt part detection and picking
  while (conveyor_parts.size()!=0){
    if (conveyor_size == conveyor_parts.size()){
    floor_robot_->setJointValueTarget("linear_actuator_joint", -2.75);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 3.14);
    floor_robot_->setJointValueTarget("floor_shoulder_lift_joint", -0.942478);
    FloorRobotMovetoTarget();
    FloorRobotMoveConveyorHome();
    }
    while(!breakbeam_status){
      if (breakbeam2_status){
        is_pump = true;
        pump_rgb = conv_rgb_parts_;
      }
    }
    if (breakbeam_status){
      std::vector<geometry_msgs::msg::Pose> part_pose;
      group3::msg::Part part_rgb;
      part_pose = conv_parts_;
      part_rgb = conv_rgb_parts_;
      if (is_pump){
        is_pump = false;
        FloorRobotPickConvPart(part_pose, pump_rgb);
      }
      else {
        FloorRobotPickConvPart(part_pose, part_rgb);
      }
    } 
  }

  FloorRobotMoveHome();
  CeilRobotMoveHome();
  if (high_priority_order_){
    process_order();
    populate_bin_part();
  }
  FloorRobotPickandPlaceTray(current_order[0].GetKitting().get()->GetTrayId(),current_order[0].GetKitting().get()->GetAgvId());
  populate_bin_part();

  int count = 0;
  for (unsigned int j =0; j<current_order[0].GetKitting().get()->GetParts().size(); j++){
    if (high_priority_order_){
      process_order();
      populate_bin_part();
    }
    type_color = (current_order[0].GetKitting().get()->GetParts()[j][1]*10 + current_order[0].GetKitting().get()->GetParts()[j][0]);
    type_color_key = search_bin(type_color);
    RCLCPP_INFO_STREAM(this->get_logger(), "Type Color Key: " << std::to_string(type_color_key));
    if(type_color_key != -1){
      // 1 denotes part found in Bin
      keys.push_back({type_color_key, 1});
    } else if(doing_priority = true && type_color_key== -1 && incomplete_order[0].GetType() == ariac_msgs::msg::Order::KITTING){
      if(partsonkittray.find(type_color) == partsonkittray.end()) {
        type_color_key = -1;
        keys.push_back({type_color_key, 0});
      }
      else {
        type_color_key = type_color;
        keys.push_back({type_color_key, 2});
      }
    }
    else {
      RCLCPP_WARN_STREAM(this->get_logger(),"The Missing Part is : " << ConvertPartColorToString(type_color%10) << " " << ConvertPartTypeToString(type_color/10));
      RCLCPP_WARN_STREAM(this->get_logger(),"This Kitting order has insufficient parts : " << current_order[0].GetId());
      // 0 denotes part not found anywhere
      keys.push_back({type_color_key, 0});
    }
    for (auto i : keys){
      if (i[1] == 0) {
        count++;
        continue;
      } else if (i[1] == 1) {
          RCLCPP_INFO_STREAM(this->get_logger(),"Picking Part " << ConvertPartColorToString(bin_map[i[0]].part_type_clr%10) << " " << ConvertPartTypeToString(bin_map[i[0]].part_type_clr/10));
          if (FloorRobotReachableWorkspace(i[0])) {
            CeilRobotMoveHome();
            FloorRobotPickBinPart((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]);
            FloorRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]);
            if(traypartpose.position.x != -1000) {
              partsonkittray[type_color] = traypartpose;
            }
          } else {
            // Implement Ceiling Robot FlipPart() later
            FloorRobotMoveHome();
            CeilRobotPickBinPart((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]); 
            CeilRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]); 
          }
          bin_map[i[0]].part_type_clr = -1;
          // Check if the part is dropped and if yes, then pick the replacement part
          if (dropped_parts_.size() != 0) {
            for (auto part : keys){
              bin_map[part[0]].part_type_clr = -1;
            }
            for (auto i : dropped_parts_) {
              type_color_key_replacement = search_bin(i.type*10 + i.color);
              if (type_color_key_replacement == -1) {
                break;
              }
              if (FloorRobotReachableWorkspace(type_color_key_replacement)) {
                CeilRobotMoveHome();
                RCLCPP_INFO_STREAM(this->get_logger(),"Picking Replacement Part " << ConvertPartColorToString(i.color) << " " << ConvertPartTypeToString(i.type));
                FloorRobotPickBinPart(i.color,i.type, bin_map[type_color_key_replacement].part_pose, type_color_key_replacement);
                FloorRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]);
                if(traypartpose.position.x != -1000) {
                  partsonkittray[type_color] = traypartpose;
                }
              } else {
                FloorRobotMoveHome();
                RCLCPP_INFO_STREAM(this->get_logger(),"Picking Replacement Part " << ConvertPartColorToString(i.color) << " " << ConvertPartTypeToString(i.type));
                CeilRobotPickBinPart(i.color,i.type, bin_map[type_color_key_replacement].part_pose, type_color_key_replacement);
                CeilRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]); 
              }
              bin_map[type_color_key_replacement].part_type_clr = -1;
            }
            dropped_parts_.clear();
            populate_bin_part();
          }
      }
      else if (i[1] == 2) {
        FloorRobotPickTrayPart((i[0])%10,(i[0])/10, partsonkittray[i[0]],incomplete_order[0].GetKitting().get()->GetAgvId());
        FloorRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]);
      }
      count++;
    }
    keys.clear();
  }

  if (high_priority_order_){
    process_order();
    populate_bin_part();
  }
  // Final Quality Check for the Kitting Order to check if any part is missing
  auto QualityCheck = CheckFaultyPart(current_order[0].GetId());
  usleep(2000);
  QualityCheck = CheckFaultyPart(current_order[0].GetId());
  if(!QualityCheck[1]){
    populate_bin_part();
    for (unsigned int j =0; j<current_order[0].GetKitting().get()->GetParts().size(); j++){
      if(QualityCheck[4+(j*6)]){
        type_color_key_missing = search_bin(current_order[0].GetKitting().get()->GetParts()[j][1]*10+current_order[0].GetKitting().get()->GetParts()[j][0]);
        if (type_color_key_missing != -1) {
          RCLCPP_INFO_STREAM(this->get_logger(),"Picking Replacement Missing Part " << ConvertPartColorToString((bin_map[type_color_key_missing].part_type_clr)%10) << " " << ConvertPartTypeToString((bin_map[type_color_key_missing].part_type_clr)/10));
          FloorRobotPickBinPart((bin_map[type_color_key_missing].part_type_clr)%10,(bin_map[type_color_key_missing].part_type_clr)/10, bin_map[type_color_key_missing].part_pose, type_color_key_missing);
          FloorRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[j][2]);
        }
      }
    }
  }


  int used_agv = current_order[0].GetKitting().get()->GetAgvId();
  if (available_agvs.size() > 0) {
    available_agvs.erase(std::remove(available_agvs.begin(), available_agvs.end(), used_agv), available_agvs.end());
  }
  else {
    RCLCPP_WARN_STREAM(this->get_logger(),"No more AGVs available!");
  }

  if (high_priority_order_){
    process_order();
  }
  move_agv(current_order[0].GetKitting().get()->GetAgvId(), current_order[0].GetKitting().get()->GetDestination());
  FloorRobotMoveHome();
  CeilRobotMoveHome();
  partsonkittray.clear();
  RCLCPP_INFO_STREAM(this->get_logger(),"Kitting Order Completed");
  return true;
}

bool AriacCompetition::do_assembly(std::vector<Orders>  current_order) {

  bool is_pump; // Stores whether the part is a pump or not for conveyor belt

  populate_bin_part();
  // Conveyor belt part detection and picking
  while (conveyor_parts.size()!=0){
    if (conveyor_size == conveyor_parts.size()){
    floor_robot_->setJointValueTarget("linear_actuator_joint", -2.75);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 3.14);
    floor_robot_->setJointValueTarget("floor_shoulder_lift_joint", -0.942478);
    FloorRobotMovetoTarget();
    FloorRobotMoveConveyorHome();
    }
    while(!breakbeam_status){
      if (breakbeam2_status){
        is_pump = true;
        pump_rgb = conv_rgb_parts_;
      }
    }
    if (breakbeam_status){
      std::vector<geometry_msgs::msg::Pose> part_pose;
      group3::msg::Part part_rgb;
      part_pose = conv_parts_;
      part_rgb = conv_rgb_parts_;
      if (is_pump){
        is_pump = false;
        FloorRobotPickConvPart(part_pose, pump_rgb);
      }
      else {
        FloorRobotPickConvPart(part_pose, part_rgb);
      }
    } 
  }
  int station_num = current_order[0].GetAssembly().get()->GetStation();

  for (auto agv_num : current_order[0].GetAssembly().get()->GetAgvNumbers()) {
    int destination;
    if (station_num == ariac_msgs::msg::AssemblyTask::AS1 || station_num == ariac_msgs::msg::AssemblyTask::AS3)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
    }
    else if (station_num == ariac_msgs::msg::AssemblyTask::AS2 || station_num == ariac_msgs::msg::AssemblyTask::AS4)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
    }

    lock_agv(agv_num);
    move_agv(agv_num, destination);
    unlock_agv(agv_num);
    int used_agv = agv_num;
    if (available_agvs.size() > 0) {
        available_agvs.erase(std::remove(available_agvs.begin(), available_agvs.end(), used_agv), available_agvs.end());
    }
    else {
        RCLCPP_WARN_STREAM(this->get_logger(),"No more AGVs available!");
    }
  }

  CeilRobotMoveToAssemblyStation(station_num);
  if (high_priority_order_){
    process_order();
    CeilRobotMoveToAssemblyStation(station_num);
  }

  std::string srv_name = "/ariac/get_pre_assembly_poses";

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_pre_assembly_poses");
  rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_ = node->create_client<ariac_msgs::srv::GetPreAssemblyPoses>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order[0].GetId();

  while (!pre_assembly_poses_getter_->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Service not available, waiting again...");
  }

  auto result = pre_assembly_poses_getter_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(this->get_logger(),"Pre Assembly Poses recieved");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service get_pre_assembly_poses");
  }

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
  if (result.get()->valid_id) {
    agv_part_poses = result.get()->parts;

    if (agv_part_poses.size() == 0) {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
    }
  } else {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
  }

  for (auto const &part_to_assemble : current_order[0].GetAssembly().get()->GetParts()) {
    if (high_priority_order_){
      process_order();
      CeilRobotMoveToAssemblyStation(station_num);
    }
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part.type = part_to_assemble.type;
    part_to_pick.part.color = part_to_assemble.color;
    for (auto const &agv_part: agv_part_poses) {
      if (agv_part.part.type == part_to_assemble.type && agv_part.part.color == part_to_assemble.color) {
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    // Pick up part
    CeilRobotPickAGVPart(part_to_pick);

    CeilRobotMoveToAssemblyStation(station_num);
    
    CeilRobotAssemblePart(station_num, part_to_assemble);

    CeilRobotMoveToAssemblyStation(station_num);
  }
  CeilRobotMoveHome();
  RCLCPP_INFO_STREAM(this->get_logger(),"Assembly Order Completed");
  return true;
}

bool AriacCompetition::do_combined(std::vector<Orders>  current_order) {

  bool is_pump; // Stores whether the part is a pump or not for conveyor belt
  int type_color_key_replacement;  // Stores the key of the specific part in the bin map

  populate_bin_part();
  // Conveyor belt part detection and picking
  while (conveyor_parts.size()!=0){
    if (conveyor_size == conveyor_parts.size()){
    floor_robot_->setJointValueTarget("linear_actuator_joint", -2.75);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 3.14);
    floor_robot_->setJointValueTarget("floor_shoulder_lift_joint", -0.942478);
    FloorRobotMovetoTarget();
    FloorRobotMoveConveyorHome();
    }
    while(!breakbeam_status){
      if (breakbeam2_status){
        is_pump = true;
        pump_rgb = conv_rgb_parts_;
      }
    }
    if (breakbeam_status){
      std::vector<geometry_msgs::msg::Pose> part_pose;
      group3::msg::Part part_rgb;
      part_pose = conv_parts_;
      part_rgb = conv_rgb_parts_;
      if (is_pump){
        is_pump = false;
        FloorRobotPickConvPart(part_pose, pump_rgb);
      }
      else {
        FloorRobotPickConvPart(part_pose, part_rgb);
      }
    } 
  }

  int agv_num;
  int station_num = current_order[0].GetCombined().get()->GetStation();

  if (station_num == ariac_msgs::msg::CombinedTask::AS1 or station_num == ariac_msgs::msg::CombinedTask::AS2) {
    if (std::find(available_agvs.begin(), available_agvs.end(), 1) != available_agvs.end()) {
      agv_num = 1;
    } else {
      agv_num = 2;
    }  
  } else {
    if (std::find(available_agvs.begin(), available_agvs.end(), 4) != available_agvs.end()) {
      agv_num = 4;
    } else {
      agv_num = 3;
    } 
  }

  if (doing_priority == false){
    old_agv = agv_num;
  }

  int used_agv = agv_num;
  if (available_agvs.size() > 0) {
    available_agvs.erase(std::remove(available_agvs.begin(), available_agvs.end(), used_agv), available_agvs.end());
  }
  else {
    RCLCPP_WARN_STREAM(this->get_logger(),"No more AGVs available!");
  }

  int tray_num = 0;

  auto kts1_vec = tray_detect(kts1_rgb_camera_image_);
  auto kts2_vec = tray_detect(kts2_rgb_camera_image_);
  std::vector<int> tray_id_vec(kts1_vec);
  tray_id_vec.insert(tray_id_vec.end(), kts2_vec.begin(), kts2_vec.end());
  
  std::string part_info;

  RCLCPP_INFO_STREAM(this->get_logger(),"Use AGV " << agv_num << " and Tray ID " << tray_num);
    
  FloorRobotMoveHome();
  CeilRobotMoveToAssemblyStation(station_num);
  if (high_priority_order_){
    process_order();
    populate_bin_part();
  }
  FloorRobotPickandPlaceTray(tray_num, agv_num);
  
  int type_color_key;
  std::vector<std::array<int, 2>> keys;
  int type_color;
  
  int count = 0;
  std::array<int,4> quadrant = {1,2,3,4};
  for (unsigned int j = 0; j < current_order[0].GetCombined().get()->GetParts().size(); j++){
    if (high_priority_order_){
      process_order();
      populate_bin_part();
    }
    type_color = (current_order[0].GetCombined().get()->GetParts()[j].type*10 + current_order[0].GetCombined().get()->GetParts()[j].color);
    type_color_key = search_bin(type_color);
    if(type_color_key != -1){
      keys.push_back({type_color_key, 1});
    }else if(doing_priority = true && type_color_key == -1)  {
      if(partsonkittray.find(type_color) == partsonkittray.end()) {
        type_color_key = -1;
        keys.push_back({type_color_key, 0});
      }
      else {
        type_color_key = type_color;
        keys.push_back({type_color_key, 2});
      }
    } 
    for (auto i : keys){
      if (i[1] == 0) {
        continue;
      } else if (i[1] == 1) {
        part_info = ConvertPartColorToString((bin_map[i[0]].part_type_clr)%10) + " " + ConvertPartTypeToString((bin_map[i[0]].part_type_clr)/10);
        if (FloorRobotReachableWorkspace(i[0])) {
          // CeilRobotMoveHome();
          FloorRobotPickBinPart((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]);
          FloorRobotPlacePartOnKitTray(agv_num,quadrant[count]);
          if(traypartpose.position.x != -1000) {
              partsonkittray[type_color] = traypartpose;
          }
        } else {
          // FloorRobotMoveHome();
          CeilRobotPickBinPart((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]); 
          CeilRobotPlacePartOnKitTray(agv_num,quadrant[count]);
        }
        bin_map[i[0]].part_type_clr = -1;
        if (dropped_parts_.size() != 0) {
          for (auto part : keys){
            bin_map[part[0]].part_type_clr = -1;
          }
          for (auto i : dropped_parts_) {
            type_color_key_replacement = search_bin(i.type*10 + i.color);
            if (type_color_key_replacement == -1) {
              break;
            }
            if (FloorRobotReachableWorkspace(type_color_key_replacement)) {
              CeilRobotMoveHome();
              RCLCPP_INFO_STREAM(this->get_logger(),"Picking Replacement Part " << ConvertPartColorToString(i.color) << " " << ConvertPartTypeToString(i.type));
              FloorRobotPickBinPart(i.color,i.type, bin_map[type_color_key_replacement].part_pose, type_color_key_replacement);
              FloorRobotPlacePartOnKitTray(agv_num,quadrant[count]);
              if(traypartpose.position.x != -1000) {
                partsonkittray[type_color] = traypartpose;
              }
            } else {
              FloorRobotMoveHome();
              RCLCPP_INFO_STREAM(this->get_logger(),"Picking Replacement Part " << ConvertPartColorToString(i.color) << " " << ConvertPartTypeToString(i.type));
              CeilRobotPickBinPart(i.color,i.type, bin_map[type_color_key_replacement].part_pose, type_color_key_replacement);
              CeilRobotPlacePartOnKitTray(agv_num,quadrant[count]); 
            }
            bin_map[type_color_key_replacement].part_type_clr = -1;
          }
          dropped_parts_.clear();
          populate_bin_part();
        }
      } 
      else if (i[1] == 2) {
        if (incomplete_order[0].GetType() == ariac_msgs::msg::Order::KITTING){
          FloorRobotPickTrayPart((i[0])%10,(i[0])/10, partsonkittray[i[0]],incomplete_order[0].GetKitting().get()->GetAgvId());
        }
        else {
          FloorRobotPickTrayPart((i[0])%10,(i[0])/10, partsonkittray[i[0]],old_agv);
        }
        FloorRobotPlacePartOnKitTray(agv_num,quadrant[count]);
      }
      count++;
    }
    keys.clear();
  }

  int Dest;
  if (station_num == ariac_msgs::msg::CombinedTask::AS1 or station_num == ariac_msgs::msg::CombinedTask::AS3) {
    Dest = ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT;
  } else {
    Dest = ariac_msgs::msg::KittingTask::ASSEMBLY_BACK;
  }
  lock_agv(agv_num);
  move_agv(agv_num, Dest);
  FloorRobotMoveHome();
  CeilRobotMoveToAssemblyStation(station_num);
  unlock_agv(agv_num);

  if (high_priority_order_){
    process_order();
    CeilRobotMoveToAssemblyStation(station_num);
  }

  std::string srv_name = "/ariac/get_pre_assembly_poses";

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_pre_assembly_poses");
  rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_ = node->create_client<ariac_msgs::srv::GetPreAssemblyPoses>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order[0].GetId();

  while (!pre_assembly_poses_getter_->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Service not available, waiting again...");
  }

  auto result = pre_assembly_poses_getter_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(this->get_logger(),"Pre Assembly Poses recieved");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service get_pre_assembly_poses");
  }

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
  if (result.get()->valid_id) {
    agv_part_poses = result.get()->parts;

    if (agv_part_poses.size() == 0) {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
    }
  } else {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
  }

  for (auto const &part_to_assemble : current_order[0].GetCombined().get()->GetParts()) {
    if (high_priority_order_){
      process_order();
      CeilRobotMoveToAssemblyStation(station_num);
    }
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part.type = part_to_assemble.type;
    part_to_pick.part.color = part_to_assemble.color;
    for (auto const &agv_part: agv_part_poses) {
      if (agv_part.part.type == part_to_assemble.type && agv_part.part.color == part_to_assemble.color) {
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    // Pick up part
    CeilRobotPickAGVPart(part_to_pick);

    CeilRobotMoveToAssemblyStation(station_num);
    
    CeilRobotAssemblePart(station_num, part_to_assemble);

    CeilRobotMoveToAssemblyStation(station_num);
  }
  CeilRobotMoveHome();
  RCLCPP_INFO_STREAM(this->get_logger(),"Combined Order Completed");
  return true;
}

int AriacCompetition::search_bin(int part) {
  for (auto& it : bin_map) {
    if (it.second.part_type_clr == part) {
      return it.first;
    }
  }
  return -1;
}

void AriacCompetition::setup_map() {
  for(unsigned int i = 1; i <= 72; i++){
    bin_map[i];
  }
}

std::string AriacCompetition::ConvertPartTypeToString(int part_type) {
  if (part_type == ariac_msgs::msg::Part::BATTERY)
    return std::string("Battery")+"\033[0m";
  else if (part_type == ariac_msgs::msg::Part::PUMP)
    return std::string("Pump")+"\033[0m";
  else if (part_type == ariac_msgs::msg::Part::REGULATOR)
    return std::string("Regulator")+"\033[0m";
  else if (part_type == ariac_msgs::msg::Part::SENSOR)
    return std::string("Sensor")+"\033[0m";
  else
    return "None";
}

std::string AriacCompetition::ConvertPartColorToString(int part_color) {
  if (part_color == ariac_msgs::msg::Part::RED)
    return std::string("\033[0;91m")+"Red";
  else if (part_color == ariac_msgs::msg::Part::GREEN)
    return std::string("\033[92m")+"Green";
  else if (part_color == ariac_msgs::msg::Part::BLUE)
    return std::string("\033[94m")+"Blue";
  else if (part_color == ariac_msgs::msg::Part::PURPLE)
    return std::string("\033[95m")+"Purple";
  else if (part_color == ariac_msgs::msg::Part::ORANGE)
    return std::string("\033[0;33m")+"Orange";
  else
    return "None";
}

std::string AriacCompetition::ConvertDestinationToString(int destination, int agv_num) {
    if (agv_num == 1 || agv_num == 2) {
        if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
          return "Assembly Station 1";
        else if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
          return "Assembly Station 2";
    } else if (agv_num == 3 || agv_num == 4) {
        if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
          return "Assembly Station 3";
        else if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
          return "Assembly Station 4";
    }
    
    if (destination == ariac_msgs::msg::KittingTask::KITTING)
      return "Kitting";
    else if (destination == ariac_msgs::msg::KittingTask::WAREHOUSE)
      return "Warehouse";
    else
      return "None";
}

std::string AriacCompetition::ConvertAssemblyStationToString(int station_id) {
  if (station_id == ariac_msgs::msg::AssemblyTask::AS1)
    return "Assembly Station 1";
  else if (station_id == ariac_msgs::msg::AssemblyTask::AS2)
    return "Assembly Station 2";
  else if (station_id == ariac_msgs::msg::AssemblyTask::AS3)
    return "Assembly Station 3";
  else if (station_id == ariac_msgs::msg::AssemblyTask::AS4)
    return "Assembly Station 4";
  else
    return "None";
}

void AriacCompetition::lock_agv(int agv_num) {
  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("lock_agv_client");
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
        node->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(this->get_logger(),"Locked AGV " << agv_num);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }

  node.reset();
  client.reset();
}

void AriacCompetition::unlock_agv(int agv_num) {
  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_unlock_tray";

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("unlock_agv_client");
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
        node->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(this->get_logger(),"Unlocked AGV " << agv_num);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }
}

void AriacCompetition::move_agv(int agv_num, int dest) {
  // AriacCompetition::lock_agv(agv_num);
  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("move_agv_client");
    
    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client =
        node->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();

    request->location = dest;

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(this->get_logger(),"Moved AGV " << agv_num << " to " << ConvertDestinationToString(agv_num,dest));
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }
}

int AriacCompetition::determine_agv(int station_num) {
  std::set<int> stn_assm;
  std::set<int> stn_comb;
  if (station_num == ariac_msgs::msg::AssemblyTask::AS1 || station_num == ariac_msgs::msg::AssemblyTask::AS2){
    stn_assm = {1, 2};
  } else if (station_num == ariac_msgs::msg::AssemblyTask::AS3 || station_num == ariac_msgs::msg::AssemblyTask::AS4) {
    stn_assm = {3, 4};
  } 
  
  if (station_num == ariac_msgs::msg::CombinedTask::AS1 || station_num == ariac_msgs::msg::CombinedTask::AS3){
    stn_comb = {1, 3};
  } else {
    stn_comb = {2, 4};
  }

  std::set<int> result;
  std::set_intersection(stn_assm.begin(), stn_assm.end(), stn_comb.begin(), stn_comb.end(), std::inserter(result, result.begin()));

  return *(result.begin());
}

void AriacCompetition::floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg){
  floor_gripper_state_ = *msg;
}

void AriacCompetition::ceil_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg){
  ceil_gripper_state_ = *msg;
}

void AriacCompetition::conv_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg){
    if (!conv_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from conveyor camera");
        conv_camera_received_data = true;
    }

    conv_parts_ = msg->part_poses;
    conv_camera_pose_ = msg->sensor_pose;
}

void AriacCompetition::kts1_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!kts1_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
        kts1_rgb_camera_received_data = true;
    }
    kts1_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::kts2_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!kts2_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
        kts2_rgb_camera_received_data = true;
    }
    kts2_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::left_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!left_bins_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from left bins camera");
        left_bins_rgb_camera_received_data = true;
    }
    left_bins_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::right_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!right_bins_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from right bins camera");
        right_bins_rgb_camera_received_data = true;
    }
    right_bins_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::right_part_detector_cb(const group3::msg::Parts::ConstSharedPtr msg){
    if (!right_part_detector_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from Right part detector node");
        right_part_detector_received_data = true;
    }
    right_parts_ = msg->parts;
}

void AriacCompetition::left_part_detector_cb(const group3::msg::Parts::ConstSharedPtr msg){
    if (!left_part_detector_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from Left part detector node");
        left_part_detector_received_data = true;
    }
    left_parts_ = msg->parts;
}

void AriacCompetition::conv_part_detector_cb(
    const group3::msg::Part::ConstSharedPtr msg){
    if (!conv_part_detector_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from Conveyor part detector node");
        conv_part_detector_received_data = true;
    }

    conv_rgb_parts_ = *msg;
}

void AriacCompetition::breakbeam_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if (!breakbeam_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from breakbeam node");
        breakbeam_received_data = true;
    }
    breakbeam_time_sec = msg->header.stamp.sec;
    breakbeam_status = msg->object_detected;

    if (breakbeam_trigger == false && breakbeam_status == true){
      conveyor_parts.pop_back();
      breakbeam_trigger = true;
    }

    if (breakbeam_trigger == true && breakbeam_status == false){
      breakbeam_trigger = false;
    }
}

void AriacCompetition::breakbeam1_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if (!breakbeam1_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from breakbeam1 node");
        breakbeam1_received_data = true;
    }
    breakbeam1_status = msg->object_detected;
}

void AriacCompetition::breakbeam2_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if (!breakbeam2_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from breakbeam2 node");
        breakbeam2_received_data = true;
    }
    breakbeam2_status = msg->object_detected;
}

void AriacCompetition::as1_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void AriacCompetition::as2_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void AriacCompetition::as3_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}
void AriacCompetition::as4_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

geometry_msgs::msg::Pose AriacCompetition::MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void AriacCompetition::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose AriacCompetition::BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose AriacCompetition::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double AriacCompetition::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion AriacCompetition::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

void AriacCompetition::AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

void AriacCompetition::AddModelsToPlanningScene()
{
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
        {"as1_insert", std::pair<double, double>(-7.7, 3)},
        {"as2_insert", std::pair<double, double>(-12.7, 3)},
        {"as3_insert", std::pair<double, double>(-7.7, -3)},
        {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert : assembly_insert_positions)
    {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

geometry_msgs::msg::Quaternion AriacCompetition::SetRobotOrientation(double rotation){
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

bool AriacCompetition::FloorRobotMovetoTarget(){
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_->plan(plan));

    if (success)
    {
        return static_cast<bool>(floor_robot_->execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool AriacCompetition::FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf){
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }
    
    robot_trajectory::RobotTrajectory rt(floor_robot_->getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_->getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_->execute(trajectory));
}

void AriacCompetition::FloorRobotWaitForAttach(double timeout){
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_->getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

bool AriacCompetition::FloorRobotReachableWorkspace(int quadrant) {
  if (quadrant >= 19 && quadrant <= 24) {
    return false;
  } else if (quadrant >= 28 && quadrant <= 33) {
    return false;
  } else if (quadrant >= 55 && quadrant <= 60) {
    return false;
  } else if (quadrant >= 64 && quadrant <= 69) {
    return false;
  } else {
    return true;
  }
}

void AriacCompetition::FloorRobotMoveHome() {
  floor_robot_->setNamedTarget("home");
  FloorRobotMovetoTarget();
}

void AriacCompetition::FloorRobotMoveConveyorHome()
{
  // Move ceiling robot to conveyor home joint state
  floor_robot_->setJointValueTarget(floor_conv_home_js_);
  FloorRobotMovetoTarget();
}

bool AriacCompetition::FloorRobotSetGripperState(bool enable) {
  if (floor_gripper_state_.enabled == enable) {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  std::string srv_name = "/ariac/floor_robot_enable_gripper";
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("client_floor_robot_enable_gripper");
    
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
      node->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name);

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                        "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

}

void AriacCompetition::FloorRobotChangeGripper(std::string gripper_type, std::string station) {
  // Move floor robot to the corresponding kit tray table
  if (station == "kts1")
  {
      floor_robot_->setJointValueTarget(floor_kts1_js_);
  }
  else
  {
      floor_robot_->setJointValueTarget(floor_kts2_js_);
  }
  FloorRobotMovetoTarget();

  // Move gripper into tool changer
  auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z, SetRobotOrientation(0.0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  std::string srv_name = "/ariac/floor_robot_change_gripper";
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("client_floor_robot_change_gripper");
  
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr client =
      node->create_client<ariac_msgs::srv::ChangeGripper>(srv_name);

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
  
  if (gripper_type == "trays") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                    "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to change gripper");
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  node.reset();
  client.reset();
}

void AriacCompetition::FloorRobotPickandPlaceTray(int tray_idx , int agv_num){
  tray_poses = define_tray_poses();
  std::vector<int> kts1_vec;
  std::vector<int> kts2_vec;
  geometry_msgs::msg::Pose tray_camera_pose;
  geometry_msgs::msg::Pose camera_pose_;
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  bool dont_change_gripper = false;
  int tray_id;

  if (floor_gripper_state_.type == "tray_gripper"){
    dont_change_gripper = true;
  }

  kts1_vec = tray_detect(kts1_rgb_camera_image_);
  kts2_vec = tray_detect(kts2_rgb_camera_image_);

  if (std::find(kts1_vec.begin(), kts1_vec.end(), tray_idx) != kts1_vec.end()) {
      auto tray_it = std::find(kts1_vec.begin(), kts1_vec.end(), tray_idx);
      tray_id = tray_it - kts1_vec.begin();
      station = "kts1";
      tray_pose = tray_poses[tray_id];
      if (floor_gripper_state_.type != "tray_gripper") {
        FloorRobotChangeGripper("trays","kts1");
      }
  } else if (std::find(kts2_vec.begin(), kts2_vec.end(), tray_idx) != kts2_vec.end()) {
      auto tray_it = std::find(kts2_vec.begin(), kts2_vec.end(), tray_idx);
      tray_id = tray_it - kts2_vec.begin();
      station = "kts2";
      tray_pose = tray_poses[tray_id+3];
      if (floor_gripper_state_.type != "tray_gripper")
      {
        FloorRobotChangeGripper("trays","kts2");
      }
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),"Tray not found");
  }
  
  if (dont_change_gripper){
    if (station == "kts1")
    {
        floor_robot_->setJointValueTarget(floor_kts1_js_);
    }
    else
    {
        floor_robot_->setJointValueTarget(floor_kts2_js_);
    }
  }
  FloorRobotMovetoTarget();
  
  double tray_rotation = GetYaw(tray_pose);
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z, SetRobotOrientation(tray_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
  
  FloorRobotSetGripperState(true);

  // Add kit tray to planning scene
  std::string tray_name = "kit_tray_" + std::to_string(tray_id);
  AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_->attachObject(tray_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

  FloorRobotMovetoTarget();
  double agv_rotation;

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
  if (station == "kts1"){
    agv_rotation = -GetYaw(agv_tray_pose);
  }
  else {
    agv_rotation = GetYaw(agv_tray_pose);
  }

  waypoints.clear();

  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  FloorRobotSetGripperState(false);

  floor_robot_->detachObject(tray_name);

  lock_agv(agv_num);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

}

bool AriacCompetition::FloorRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad){
  std::string bin_side;

  if (part_quad < 37) {
      bin_side = "right_bins";
      if (floor_gripper_state_.type != "part_gripper")
      {
        FloorRobotChangeGripper("parts","kts2");
      }
  } else {
      bin_side = "left_bins";
      if (floor_gripper_state_.type != "part_gripper")
      {
        FloorRobotChangeGripper("parts","kts1");
      }
  }
  double part_rotation = GetYaw(part_pose);

  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part_type == ariac_msgs::msg::Part::PUMP)
  {
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_type], SetRobotOrientation(part_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
    FloorRobotSetGripperState(true);
  }
  else
  {
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);
    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(3.0);
  }

  // Add part to planning scene
  std::string part_name = part_colors_[part_clr] + "_" + part_types_[part_type];
  AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
  floor_robot_->attachObject(part_name);
  ariac_msgs::msg::Part part_to_pick;
  part_to_pick.color = part_clr;
  part_to_pick.type = part_type;
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
}


bool AriacCompetition::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant) {
  if (!floor_gripper_state_.attached) {
      RCLCPP_ERROR(this->get_logger(), "No part attached");
  }

  // Move to agv
  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (floor_robot_attached_part_.type == ariac_msgs::msg::Part::PUMP)
  {
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_ + 0.1,
                                SetRobotOrientation(0)));
    traypartpose = BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_ + 0.1,
                                SetRobotOrientation(0));
    
  }
  else if (quadrant == 4 && floor_robot_attached_part_.type == ariac_msgs::msg::Part::PUMP){
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                 part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                              part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                              SetRobotOrientation(0)));
    traypartpose = BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                              part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                              SetRobotOrientation(0));
  } else {
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                 part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                              part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_ + 0.1,
                              SetRobotOrientation(0)));
    traypartpose = BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                              part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_ + 0.1,
                              SetRobotOrientation(0));
  }

  FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

  auto QualityCheck = CheckFaultyPart(current_order[0].GetId());
  usleep(4000);
  QualityCheck = CheckFaultyPart(current_order[0].GetId());

  if(QualityCheck[6] || QualityCheck[12] || QualityCheck[18] || QualityCheck[24]){
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();
    floor_robot_->setJointValueTarget(floor_disposal_poses_[agv_num]);
    FloorRobotMovetoTarget();
    FloorRobotSetGripperState(false);
    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type];
    floor_robot_->detachObject(part_name);

    planning_scene_.removeCollisionObjects({part_name});
    dropped_parts_.clear();
    dropped_parts_.push_back(floor_robot_attached_part_);
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();
    traypartpose = BuildPose(-1000,0,0,SetRobotOrientation(0));
  } else {

    // Check if flipped
    if(QualityCheck[5] || QualityCheck[11] || QualityCheck[17] || QualityCheck[23]){
      FlipPart(floor_robot_attached_part_.color, floor_robot_attached_part_.type, agv_num, quadrant);
    } else {

      FloorRobotSetGripperState(false);
      std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                              "_" + part_types_[floor_robot_attached_part_.type];
      floor_robot_->detachObject(part_name);
      planning_scene_.removeCollisionObjects({part_name});

      waypoints.clear();
      waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                    part_drop_pose.position.z + 0.3,
                                    SetRobotOrientation(0)));

      FloorRobotMoveCartesian(waypoints, 0.4, 0.1);
    }
  }
}

bool AriacCompetition::FloorRobotPickTrayPart(int part_clr, int part_type, geometry_msgs::msg::Pose part_pose, int agv_num) {
  
  if (floor_gripper_state_.type != "part_gripper"){
    if (agv_num == 1 or agv_num == 2)
      FloorRobotChangeGripper("parts","kts2");
    else
    FloorRobotChangeGripper("parts","kts1");
  }
  // Move to agv
  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();
  std::vector<geometry_msgs::msg::Pose> waypoints;

  part_pose.position.z -= drop_height_ + 0.08;
  waypoints.push_back(part_pose);
  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);
  FloorRobotSetGripperState(true);
  FloorRobotWaitForAttach(100.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part_clr] + "_" + part_types_[part_type];
  AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
  floor_robot_->attachObject(part_name);
  ariac_msgs::msg::Part part_to_pick;
  part_to_pick.color = part_clr;
  part_to_pick.type = part_type;
  floor_robot_attached_part_ = part_to_pick;
  
  // Move up slightly
  waypoints.clear();

  geometry_msgs::msg::Pose starting_pose = floor_robot_->getCurrentPose().pose;
  waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                starting_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
  return true;
}

bool AriacCompetition::FloorRobotPickConvPart(std::vector<geometry_msgs::msg::Pose> part_pose,group3::msg::Part conv_part){
  int q = search_bin(-1);
  while (!FloorRobotReachableWorkspace(q)){
    q+=1;
    while (std::find(occupied_quadrants.begin(), occupied_quadrants.end(), q) != occupied_quadrants.end()){
      q+=1;
    }
  }
  while (std::find(occupied_quadrants.begin(), occupied_quadrants.end(), q) != occupied_quadrants.end()){
      q+=1;
  }
  occupied_quadrants.push_back(q);

  std::string bin_side;
  if (q < 37) {
      bin_side = "right_bins";
      if (floor_gripper_state_.type != "part_gripper")
      {
        FloorRobotChangeGripper("parts","kts2");
      }
  } else {
      bin_side = "left_bins";
      if (floor_gripper_state_.type != "part_gripper")
      {
        FloorRobotChangeGripper("parts","kts1");
      }
  }
  int part_clr = conv_part.color;
  int part_type = conv_part.type;
  geometry_msgs::msg::Pose camera_pose_ = conv_camera_pose_;
  geometry_msgs::msg::Pose part_camera_pose = part_pose[0];

  geometry_msgs::msg::Pose part_pose_;
  part_pose_ = MultiplyPose(camera_pose_, part_camera_pose);
  RCLCPP_INFO_STREAM(this->get_logger(), "\033[0;91m Conveyor Detetcted \033[0m" << ConvertPartColorToString(part_clr) << " " << ConvertPartTypeToString(part_type));

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_->getCurrentPose().pose;
  

  if (part_type == ariac_msgs::msg::Part::REGULATOR){
    starting_pose.position.x = part_pose_.position.x;
    starting_pose.position.y -= 0.4;
    waypoints.push_back(starting_pose);
    FloorRobotMoveCartesian(waypoints, 1, 1);
    waypoints.clear();

    while (!breakbeam1_status){}
    starting_pose.position.z = part_pose_.position.z + part_heights_[part_type] + 0.0009;
    waypoints.push_back(starting_pose);
    FloorRobotSetGripperState(true);
    FloorRobotMoveCartesian(waypoints, 1, 1);
  } 
  else if (part_type == ariac_msgs::msg::Part::PUMP) {
    starting_pose.position.x = part_pose_.position.x;
    starting_pose.position.y -= 0.08;
    starting_pose.position.z = part_pose_.position.z + part_heights_[part_type] + 0.0013;
    
    tf2::Quaternion tf_q;
    tf_q.setRPY(-0.0349066, 3.14159, 0);

    geometry_msgs::msg::Quaternion q_or;

    q_or.x = tf_q.x();
    q_or.y = tf_q.y();
    q_or.z = tf_q.z();
    q_or.w = tf_q.w();

    waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                              starting_pose.position.z, q_or));

    FloorRobotSetGripperState(true);
    FloorRobotMoveCartesian(waypoints, 0.5, 0.5);
  }
  else if (part_type == ariac_msgs::msg::Part::SENSOR){
    starting_pose.position.x = part_pose_.position.x;
    starting_pose.position.y -= 0.4;
    waypoints.push_back(starting_pose);
    FloorRobotMoveCartesian(waypoints, 1, 1);
    waypoints.clear();

    while (!breakbeam1_status){}
    
    starting_pose.position.z = part_pose_.position.z + part_heights_[part_type] + 0.0009;
    waypoints.push_back(starting_pose);
    FloorRobotSetGripperState(true);
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
  }
  else {
    starting_pose.position.x = part_pose_.position.x;
    starting_pose.position.y -= 0.4;
    waypoints.push_back(starting_pose);
    FloorRobotMoveCartesian(waypoints, 1, 1);
    waypoints.clear();

    while (!breakbeam1_status){}
    
    starting_pose.position.z = part_pose_.position.z + part_heights_[part_type] + 0.0006;
    waypoints.push_back(starting_pose);
    FloorRobotSetGripperState(true);
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
  }

  waypoints.clear();
  geometry_msgs::msg::Pose current_pose = floor_robot_->getCurrentPose().pose;
  std::string part_name = part_colors_[part_clr] + "_" + part_types_[part_type];
  AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", current_pose);
  floor_robot_->attachObject(part_name);

  starting_pose.position.z += 0.4;

  waypoints.push_back(starting_pose);
  FloorRobotMoveCartesian(waypoints, 1, 1);
  waypoints.clear();

  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();
  waypoints.clear();

  
  geometry_msgs::msg::Pose set_pose = bin_quadrant_poses[q];

  if (part_type == ariac_msgs::msg::Part::PUMP){
    tf2::Quaternion tf_q;
    tf_q.setRPY(-0.0349066, 3.14159, 0);

    geometry_msgs::msg::Quaternion q_or;

    q_or.x = tf_q.x();
    q_or.y = tf_q.y();
    q_or.z = tf_q.z();
    q_or.w = tf_q.w();
    waypoints.push_back(BuildPose(set_pose.position.x - 0.005, set_pose.position.y-0.02,
                                set_pose.position.z + part_heights_[part_type] + 0.2 + drop_height_, q_or));
  }
  else if (part_type == ariac_msgs::msg::Part::SENSOR) {
    waypoints.push_back(BuildPose(set_pose.position.x - 0.01, set_pose.position.y,
                                set_pose.position.z + part_heights_[part_type] + 0.1 + drop_height_, SetRobotOrientation(0)));
  }
  else{
    waypoints.push_back(BuildPose(set_pose.position.x - 0.005, set_pose.position.y,
                                set_pose.position.z + part_heights_[part_type] + 0.1 + drop_height_, SetRobotOrientation(0)));
  }

  FloorRobotMoveCartesian(waypoints, 1, 1);
  waypoints.clear();

  FloorRobotSetGripperState(false);
  floor_robot_->detachObject(part_name);
  planning_scene_.removeCollisionObjects({part_name});

  waypoints.clear();
  waypoints.push_back(BuildPose(set_pose.position.x, set_pose.position.y,
                                set_pose.position.z + 0.4, SetRobotOrientation(0)));
  FloorRobotMoveCartesian(waypoints, 1, 1);
  
  FloorRobotMoveConveyorHome();

  if (conveyor_parts.size()==0){
    return true;
  }

}

std::vector<bool> AriacCompetition::CheckFaultyPart(std::string order_id){

  std::string srv_name = "/ariac/perform_quality_check";

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("perform_quality_check");
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr client = node->create_client<ariac_msgs::srv::PerformQualityCheck>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = order_id;

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(this->get_logger(),"PerformQualityCheck Done");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service PerformQualityCheck");
  }

  std::vector<bool> QualityCheck;
  QualityCheck.push_back(result.get()->valid_id);
  QualityCheck.push_back(result.get()->all_passed);
  QualityCheck.push_back(result.get()->incorrect_tray);
  QualityCheck.push_back(result.get()->quadrant1.all_passed);
  QualityCheck.push_back(result.get()->quadrant1.missing_part);
  QualityCheck.push_back(result.get()->quadrant1.flipped_part);
  QualityCheck.push_back(result.get()->quadrant1.faulty_part);
  QualityCheck.push_back(result.get()->quadrant1.incorrect_part_type);
  QualityCheck.push_back(result.get()->quadrant1.incorrect_part_color);
  QualityCheck.push_back(result.get()->quadrant2.all_passed);
  QualityCheck.push_back(result.get()->quadrant2.missing_part);
  QualityCheck.push_back(result.get()->quadrant2.flipped_part);
  QualityCheck.push_back(result.get()->quadrant2.faulty_part);
  QualityCheck.push_back(result.get()->quadrant2.incorrect_part_type);
  QualityCheck.push_back(result.get()->quadrant2.incorrect_part_color);
  QualityCheck.push_back(result.get()->quadrant3.all_passed);
  QualityCheck.push_back(result.get()->quadrant3.missing_part);
  QualityCheck.push_back(result.get()->quadrant3.flipped_part);
  QualityCheck.push_back(result.get()->quadrant3.faulty_part);
  QualityCheck.push_back(result.get()->quadrant3.incorrect_part_type);
  QualityCheck.push_back(result.get()->quadrant3.incorrect_part_color);
  QualityCheck.push_back(result.get()->quadrant4.all_passed);
  QualityCheck.push_back(result.get()->quadrant4.missing_part);
  QualityCheck.push_back(result.get()->quadrant4.flipped_part);
  QualityCheck.push_back(result.get()->quadrant4.faulty_part);
  QualityCheck.push_back(result.get()->quadrant4.incorrect_part_type);
  QualityCheck.push_back(result.get()->quadrant4.incorrect_part_color);

  return QualityCheck;
}

void AriacCompetition::FlipPart(int part_clr, int part_type, int agv_num, int part_quad) {
  std::vector<geometry_msgs::msg::Pose> waypoints;

  std::string part_name = part_colors_[part_clr] +
                            "_" + part_types_[part_type];
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Flip_Part"), "Move Robots to Flip Part pose");
  floor_robot_->setJointValueTarget(floor_flip_part_js_);
  FloorRobotMovetoTarget();

  floor_robot_->detachObject(part_name);
  planning_scene_.removeCollisionObjects({part_name});
  
  ceil_robot_->setJointValueTarget(ceil_flip_part_js_);
  CeilRobotMovetoTarget();

  geometry_msgs::msg::Pose ceil_pose = ceil_robot_->getCurrentPose().pose;
  geometry_msgs::msg::Pose floor_pose = floor_robot_->getCurrentPose().pose;
  double offset = floor_pose.position.x - ceil_pose.position.x;

  waypoints.clear();

  // For pump and battery
  // ceil_pose.position.x += offset - part_heights_[part_type] - 0.0005;
  if (part_type == ariac_msgs::msg::Part::SENSOR || part_type == ariac_msgs::msg::Part::REGULATOR)
  {
    ceil_pose.position.x += offset - part_heights_[part_type] - 0.0007;
  }
  else
  {
    ceil_pose.position.x += offset - part_heights_[part_type] - 0.0005;
  }
  
  waypoints.push_back(ceil_pose);
  CeilRobotMoveCartesian(waypoints, 0.05, 0.05,true);

  AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", ceil_pose);
  CeilRobotSetGripperState(true);
  FloorRobotSetGripperState(false);
  ceil_robot_->attachObject(part_name);

  ariac_msgs::msg::Part part;
  part.color = part_clr;
  part.type = part_type;
  ceil_robot_attached_part_ = part;

  ceil_robot_->setJointValueTarget(ceil_flip_part_js_);
  CeilRobotMovetoTarget();

  ceil_robot_->setJointValueTarget("gantry_y_axis_joint", gantry_positions_["agv" + std::to_string(agv_num)]);
  ceil_robot_->setJointValueTarget("gantry_x_axis_joint", 3.835);
  ceil_robot_->setJointValueTarget("gantry_rotation_joint", -1.57);
  CeilRobotMovetoTarget();

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(quad_offsets_[part_quad].first, quad_offsets_[part_quad].second, 0.0,
                                    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  waypoints.clear();

  if (ceil_robot_attached_part_.type == ariac_msgs::msg::Part::PUMP)
  {
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[ceil_robot_attached_part_.type] + drop_height_ + 0.005,
                                SetRobotOrientation(0)));
  }
  else
  {
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                 part_drop_pose.position.z + 0.2, SetRobotOrientation(0)));
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                              part_drop_pose.position.z + part_heights_[ceil_robot_attached_part_.type] + drop_height_ + 0.005,
                              SetRobotOrientation(0)));
  }

  CeilRobotMoveCartesian(waypoints, 0.1, 0.1,true);

  CeilRobotSetGripperState(false);
  ceil_robot_->detachObject(part_name);
  planning_scene_.removeCollisionObjects({part_name});
  waypoints.clear();
  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + 0.2,
                                SetRobotOrientation(0)));
  CeilRobotMoveCartesian(waypoints, 0.5, 0.5,true);

  CeilRobotMoveHome();

}

void AriacCompetition::CeilRobotMoveHome() {
  ceil_robot_->setNamedTarget("home");
  CeilRobotMovetoTarget();
}

bool AriacCompetition::CeilRobotMovetoTarget(){
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(ceil_robot_->plan(plan));

    if (success)
    {
        return static_cast<bool>(ceil_robot_->execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool AriacCompetition::CeilRobotSetGripperState(bool enable) {
  if (ceil_gripper_state_.enabled == enable) {
    if (ceil_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  std::string srv_name = "/ariac/ceiling_robot_enable_gripper";
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("client_ceil_robot_enable_gripper");
    
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
      node->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name);

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                        "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

}

bool AriacCompetition::CeilRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions){
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = ceil_robot_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }
    
    robot_trajectory::RobotTrajectory rt(ceil_robot_->getCurrentState()->getRobotModel(), "ceiling_robot");
    rt.setRobotTrajectoryMsg(*ceil_robot_->getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(ceil_robot_->execute(trajectory));
}

void AriacCompetition::CeilRobotWaitForAttach(double timeout){
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceil_robot_->getCurrentPose().pose;

  while (!ceil_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilRobotMoveCartesian(waypoints, 0.1, 0.1,true);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

void AriacCompetition::CeilRobotChangeGripper(std::string gripper_type, std::string station) {
  // Move floor robot to the corresponding kit tray table
  if (station == "kts1")
  {
      ceil_robot_->setJointValueTarget(ceil_kts1_js_);
  }
  else
  {
      ceil_robot_->setJointValueTarget(ceil_kts2_js_);
  }
  CeilRobotMovetoTarget();

  // Move gripper into tool changer
  auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z, SetRobotOrientation(0.0)));

  CeilRobotMoveCartesian(waypoints, 0.2, 0.1,true);

  std::string srv_name = "/ariac/ceiling_robot_change_gripper";
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("client_ceil_robot_change_gripper");
  
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr client =
      node->create_client<ariac_msgs::srv::ChangeGripper>(srv_name);

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
  
  if (gripper_type == "trays") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                    "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to change gripper");
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  CeilRobotMoveCartesian(waypoints, 0.2, 0.1,true);

  node.reset();
  client.reset();
}

bool AriacCompetition::CeilRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad){
  std::string bin_side;

  if (part_quad < 37) {
      bin_side = "right_bins";
      if (ceil_gripper_state_.type != "part_gripper")
      {
        CeilRobotChangeGripper("parts","kts2");
      }
  } else {
      bin_side = "left_bins";
      if (ceil_gripper_state_.type != "part_gripper")
      {
        CeilRobotChangeGripper("parts","kts1");
      }
  }
  double part_rotation = GetYaw(part_pose);

  ceil_robot_->setJointValueTarget("gantry_y_axis_joint", gantry_positions_[bin_side]);
  ceil_robot_->setJointValueTarget("gantry_x_axis_joint", 2.6);
  ceil_robot_->setJointValueTarget("gantry_rotation_joint", -1.57);
  CeilRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part_type == ariac_msgs::msg::Part::PUMP)
  {
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_type], SetRobotOrientation(part_rotation)));
    CeilRobotMoveCartesian(waypoints, 0.1, 0.1,true);
    CeilRobotSetGripperState(true);
  }
  else
  {
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));

    CeilRobotMoveCartesian(waypoints, 0.3, 0.1,true);
    CeilRobotSetGripperState(true);
    CeilRobotWaitForAttach(3.0);
  }

  // Add part to planning scene
  std::string part_name = part_colors_[part_clr] + "_" + part_types_[part_type];
  AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
  ceil_robot_->attachObject(part_name);
  ariac_msgs::msg::Part part_to_pick;
  part_to_pick.color = part_clr;
  part_to_pick.type = part_type;
  ceil_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.3, SetRobotOrientation(0)));

  CeilRobotMoveCartesian(waypoints, 0.1, 0.1,true);
}

bool AriacCompetition::CeilRobotPlacePartOnKitTray(int agv_num, int quadrant) {
  if (!ceil_gripper_state_.attached) {
      RCLCPP_ERROR(this->get_logger(), "No part attached");
  }

  // Move to agv
  ceil_robot_->setJointValueTarget("gantry_y_axis_joint", gantry_positions_["agv" + std::to_string(agv_num)]);
  ceil_robot_->setJointValueTarget("gantry_x_axis_joint", 3.835);
  ceil_robot_->setJointValueTarget("gantry_rotation_joint", -1.57);
  CeilRobotMovetoTarget();

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (ceil_robot_attached_part_.type == ariac_msgs::msg::Part::PUMP)
  {
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[ceil_robot_attached_part_.type] + drop_height_ + 0.05,
                                SetRobotOrientation(0)));
  }
  else
  {
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                 part_drop_pose.position.z + 0.2, SetRobotOrientation(0)));
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                              part_drop_pose.position.z + part_heights_[ceil_robot_attached_part_.type] + drop_height_ + 0.05,
                              SetRobotOrientation(0)));
  }

  CeilRobotMoveCartesian(waypoints, 0.1, 0.1,true);

  auto QualityCheck = CheckFaultyPart(current_order[0].GetId());
  usleep(2500);
  QualityCheck = CheckFaultyPart(current_order[0].GetId());

  if(QualityCheck[6] || QualityCheck[12] || QualityCheck[18] || QualityCheck[24]){
    ceil_robot_->setJointValueTarget(ceil_disposal_poses_[agv_num]);
    CeilRobotMovetoTarget();
    CeilRobotSetGripperState(false);
    std::string part_name = part_colors_[ceil_robot_attached_part_.color] +
                            "_" + part_types_[ceil_robot_attached_part_.type];
    ceil_robot_->detachObject(part_name);

    planning_scene_.removeCollisionObjects({part_name});
    dropped_parts_.push_back(ceil_robot_attached_part_);

    CeilRobotMoveHome();
  } else{
    CeilRobotSetGripperState(false);
    std::string part_name = part_colors_[ceil_robot_attached_part_.color] +
                            "_" + part_types_[ceil_robot_attached_part_.type];
    ceil_robot_->detachObject(part_name);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));

    CeilRobotMoveCartesian(waypoints, 0.4, 0.1,true);
  }

  if(QualityCheck[5] || QualityCheck[11] || QualityCheck[17] || QualityCheck[23]){
    //flip
  }

}

bool AriacCompetition::CeilRobotWaitForAssemble(int station, Part part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceil_robot_->getCurrentPose().pose;

  bool assembled = false;
  while (!assembled) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.type) {
      case ariac_msgs::msg::Part::BATTERY:
        assembled = assembly_station_states_[station].battery_attached;
        break;
      // case ariac_msgs::msg::Part::PUMP:
      //   assembled = assembly_station_states_[station].pump_attached;
      //   break;
      case ariac_msgs::msg::Part::SENSOR:
        assembled = assembly_station_states_[station].sensor_attached;
        break;
      case ariac_msgs::msg::Part::REGULATOR:
        assembled = assembly_station_states_[station].regulator_attached;
        break;
      default:
        RCLCPP_WARN(get_logger(), "Not a valid part type");
        return false;
    }

    double step = 0.0005;

    waypoints.clear();
    starting_pose.position.x += step * part.install_direction.x;
    starting_pose.position.y += step * part.install_direction.y;
    starting_pose.position.z += step * part.install_direction.z;
    waypoints.push_back(starting_pose);

    CeilRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(100);
    if (now() - start > rclcpp::Duration::from_seconds(8)){
      RCLCPP_ERROR(get_logger(), "Unable to assemble object");
      ceil_robot_->stop();
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Part is assembled");
  
  return true;
}

bool AriacCompetition::CeilRobotMoveToAssemblyStation(int station)
{
  switch (station) {
    case 1:
      ceil_robot_->setJointValueTarget(ceiling_as1_js_);
      break;
    case 2:
      ceil_robot_->setJointValueTarget(ceiling_as2_js_);
      break;
    case 3:
      ceil_robot_->setJointValueTarget(ceiling_as3_js_);
      break;
    case 4:
      ceil_robot_->setJointValueTarget(ceiling_as4_js_);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  return CeilRobotMovetoTarget();
}

bool AriacCompetition::CeilRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  double part_rotation = GetYaw(part.pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    dx = battery_grip_offset_*cos(part_rotation);
    dy = battery_grip_offset_*sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));
  
  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));
  
  CeilRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  CeilRobotSetGripperState(true);

  CeilRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
  AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
  ceil_robot_->attachObject(part_name);
  ceil_robot_attached_part_ = part.part;

  // Move up slightly
  auto current_pose = ceil_robot_->getCurrentPose().pose;
  current_pose.position.z += 0.2;
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  return true;

}

bool AriacCompetition::CeilRobotAssemblePart(int station, Part part)
{
  // Check that part is attached and matches part to assemble
  if (!ceil_gripper_state_.attached) {
    RCLCPP_WARN(get_logger(), "No part attached");
    return false;
  }

  
  // Calculate assembled pose in world frame
  std::string insert_frame_name;
  switch (station) {
    case 1:
      insert_frame_name = "as1_insert_frame";
      break;
    case 2:
      insert_frame_name = "as2_insert_frame";
      break;
    case 3:
      insert_frame_name = "as3_insert_frame";
      break;
    case 4:
      insert_frame_name = "as4_insert_frame";
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  // Calculate robot positions at assembly and approach
  KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

  KDL::Frame insert;
  tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

  KDL::Frame part_assemble;
  tf2::fromMsg(part.assembled_pose.pose, part_assemble);

  KDL::Frame part_to_gripper;

  // Build approach waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part.type == ariac_msgs::msg::Part::BATTERY) {
    tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    KDL::Vector up(0, 0, 0.1);
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));    

  } else {
    tf2::fromMsg(BuildPose(0, 0, part_heights_[part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  }
  
  // Move to approach position
  CeilRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  // Move to just before assembly position
  waypoints.clear();
  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
  CeilRobotMoveCartesian(waypoints, 0.1, 0.1, true);

  CeilRobotWaitForAssemble(station, part);

  CeilRobotSetGripperState(false);

  std::string part_name = part_colors_[ceil_robot_attached_part_.color] + 
    "_" + part_types_[ceil_robot_attached_part_.type];
  ceil_robot_->detachObject(part_name);

  // Move away slightly
  auto current_pose = ceil_robot_->getCurrentPose().pose;

  if (part.type == ariac_msgs::msg::Part::REGULATOR) {
    current_pose.position.x -= 0.05;
  }
  else {
    current_pose.position.z += 0.1;
  }
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilRobotMoveCartesian(waypoints, 0.3, 0.3, true);
  
  return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto ariac_competition = std::make_shared<AriacCompetition>("group3_Competitor");
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(ariac_competition);

    executor.spin();
    rclcpp::shutdown();
}