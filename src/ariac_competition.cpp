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

#include <algorithm>
#include <array>
#include <iterator>
#include <string>
#include <unistd.h>
#include <vector>

#include "../include/group3/ariac_competition.hpp"
// #include "../include/group3/floor_robot.hpp"
#include "../include/group3/tray_id_detect.hpp"
#include "../include/group3/part_type_detect.hpp"

// floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),

AriacCompetition::AriacCompetition(std::string node_name): Node(node_name),
  node_(std::make_shared<rclcpp::Node>("floor_robot")),
  executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
  // floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
  planning_scene_() 
{

  RCLCPP_INFO_STREAM(this->get_logger(),
                            "\n\nInside constructor");
  
  auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
        "floor_robot",
        "robot_description");
  floor_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, mgi_options);
  
  if (floor_robot_->startStateMonitor()) {
      RCLCPP_INFO(this->get_logger(), "Floor Robot State Monitor Started");
  } else {
      RCLCPP_ERROR(this->get_logger(), "Floor Robot State Monitor Failed to Start");
  }
  
  floor_robot_->setMaxAccelerationScalingFactor(1.0);
  floor_robot_->setMaxVelocityScalingFactor(1.0);

  competition_state_sub_ =
      this->create_subscription<ariac_msgs::msg::CompetitionState>(
          "/ariac/competition_state", 10,
          std::bind(&AriacCompetition::competition_state_cb, this,
                  std::placeholders::_1));

  order_subscriber_ = this->create_subscription<ariac_msgs::msg::Order>(
      "/ariac/orders", 10,
      std::bind(&AriacCompetition::order_callback, this,
              std::placeholders::_1));

  // bin_parts_subscriber_ = this->create_subscription<ariac_msgs::msg::BinParts>(
  //     "/ariac/bin_parts", 10,
  //     std::bind(&AriacCompetition::bin_parts_callback, this,
  //             std::placeholders::_1));

  conveyor_parts_subscriber_ = this->create_subscription<ariac_msgs::msg::ConveyorParts>(
      "/ariac/conveyor_parts", 10,
      std::bind(&AriacCompetition::conveyor_parts_callback, this,
              std::placeholders::_1)); 

  rclcpp::SubscriptionOptions options;

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = topic_cb_group_;

  kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
      "/ariac/sensors/kts1_basic_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::kts1_camera_cb, this, std::placeholders::_1), options);

  kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
      "/ariac/sensors/kts2_basic_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::kts2_camera_cb, this, std::placeholders::_1), options);

  left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
      "/ariac/sensors/left_bins_basic_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::left_bins_camera_cb, this, std::placeholders::_1), options);

  right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
      "/ariac/sensors/right_bins_basic_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::right_bins_camera_cb, this, std::placeholders::_1), options);
  
  conv_camera_sub_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
      "/ariac/sensors/conv_basic_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::conv_camera_cb, this, std::placeholders::_1), options);
  
  kts1_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/kts1_rgb_camera/rgb_image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::kts1_rgb_camera_cb, this, std::placeholders::_1), options);

  kts2_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/kts2_rgb_camera/rgb_image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::kts2_rgb_camera_cb, this, std::placeholders::_1), options);

  left_bins_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/left_bins_rgb_camera/rgb_image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::left_bins_rgb_camera_cb, this, std::placeholders::_1), options);

  right_bins_rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ariac/sensors/right_bins_rgb_camera/rgb_image", rclcpp::SensorDataQoS(),
      std::bind(&AriacCompetition::right_bins_rgb_camera_cb, this, std::placeholders::_1), options);

  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::floor_gripper_state_cb, this, std::placeholders::_1), options);

  // subscription_ = this->create_subscription<std_msgs::msg::String>(
  //     "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

  // right_part_detector_sub_ = this->create_subscription<group3::msg::Parts>(
  //       "/right_bin_part_detector", rclcpp::SensorDataQoS(),
  //       std::bind(&AriacCompetition::right_part_detector_cb, this, std::placeholders::_1), options);

  // left_part_detector_sub_ = this->create_subscription<group3::msg::Parts>(
  //       "/left_bin_part_detector", rclcpp::SensorDataQoS(),
  //       std::bind(&AriacCompetition::left_part_detector_cb, this, std::placeholders::_1), options);

  breakbeam_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
        "/ariac/sensors/breakbeam_0/status", rclcpp::SensorDataQoS(),
        std::bind(&AriacCompetition::breakbeam_cb, this, std::placeholders::_1), options);

  // Initialize service clients 
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
  // pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
  // ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");

  AddModelsToPlanningScene();

  end_competition_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AriacCompetition::end_competition_timer_callback, this)); 

  executor_->add_node(node_);
  // std::thread([this]() { this->executor_->spin(); }).detach();
  executor_thread_ = std::thread([this]()
                                   { this->executor_->spin(); });   

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
  
}

void AriacCompetition::competition_state_cb(
  const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) {
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
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }
  }
  
  if ((!orders.empty() || !current_order.empty()) && conveyor_parts_flag_)
    process_order();
}

void AriacCompetition::order_callback(ariac_msgs::msg::Order::SharedPtr msg) {
  Orders order(msg->id, msg->type, msg->priority);

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

// void AriacCompetition::bin_parts_callback(ariac_msgs::msg::BinParts::SharedPtr msg) {
//   AriacCompetition::setup_map();

//   int idx_start, idx_end;
//   for (unsigned int bin_idx = 0; bin_idx < msg->bins.size(); bin_idx++) { 
//     // Key value for bin_number bin in bin_map
//     idx_start = 9*((msg->bins[bin_idx].bin_number) - 1);
//     // Tracks the 9 locations for the bin_number bin in bin_map
//     idx_end = idx_start + 9;

//     for (unsigned int part_idx = 0; part_idx < msg->bins[bin_idx].parts.size(); part_idx++){
//       for (unsigned int qty = 0; qty < msg->bins[bin_idx].parts[part_idx].quantity; qty++){
//         for (int a = idx_start; a < idx_end; a++){
//           if (bin_map[a].part_type_clr == -1){
//             bin_map[a].part_type_clr = (msg->bins[bin_idx].parts[part_idx].part.type)*10 + (msg->bins[bin_idx].parts[part_idx].part.color);
//             break;
//             // TODO: Include pose information later
//           }
//         }
//       }
//     }
//   }
 
//   RCLCPP_INFO_STREAM(this->get_logger(), "Bin Part Information populated");
//   bin_parts_subscriber_.reset();
// }

void AriacCompetition::populate_bin_part(){
  AriacCompetition::setup_map();
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin map setup");
  while (!right_bins_rgb_camera_received_data && 
         !left_bins_rgb_camera_received_data) {}

  // std::vector<std::vector<int>> right_bin;
  // for (unsigned int i = 0; i < right_parts_.size(); i++) {
  //   std::vector<int> right_bin_part;
  //   right_bin_part.push_back(right_parts_[i].color);
  //   right_bin_part.push_back(right_parts_[i].type);
  //   right_bin_part.push_back(right_parts_[i].quad);
  //   right_bin.push_back(right_bin_part);
  // }
  std::vector<std::vector<int>> right_bin = rightbin(right_bins_rgb_camera_image_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Vector Information populated");
  std::vector<std::vector<int>> left_bin = leftbin(left_bins_rgb_camera_image_);
  // std::vector<std::vector<int>> left_bin;
  // for (unsigned int i = 0; i < left_parts_.size(); i++) {
  //   std::vector<int> left_bin_part;
  //   left_bin_part.push_back(left_parts_[i].color);
  //   left_bin_part.push_back(left_parts_[i].type);
  //   left_bin_part.push_back(left_parts_[i].quad);
  //   left_bin.push_back(left_bin_part);
  // }
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Left Vector Information populated");
  int count_right = 0;
  int count_left = 0;
  for (auto part : right_bin){
    bin_map[part[2]].part_type_clr = (part[1]*10 + part[0]);
    bin_map[part[2]].part_pose = right_bins_parts_[count_right];
    count_right++;
    RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Information populated with " << bin_map[part[2]].part_type_clr << " " << bin_map[part[2]].part_pose.position.x << " " << part[2]);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Information populated");
  for (auto part : left_bin){
    bin_map[part[2]].part_type_clr = (part[1]*10 + part[0]);
    bin_map[part[2]].part_pose = left_bins_parts_[count_left];
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

void AriacCompetition::process_order() {
  label:
  if (orders.at(0).IsPriority() == 0){
    current_order.push_back(orders.at(0));
    orders.erase(orders.begin());
  } else if (orders[0].IsPriority() == 1 && current_order.empty()){
    current_order.push_back(orders.at(0));
    orders.erase(orders.begin());
  }
  
  while(orders.empty() && !current_order.empty()){
    RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");
    RCLCPP_INFO_STREAM(this->get_logger(), "Doing Task " <<  current_order[0].GetId() << " Priority: "  << std::to_string(current_order[0].IsPriority()));
    RCLCPP_INFO_STREAM(this->get_logger(), "====================================================");

    if(current_order[0].GetType() == ariac_msgs::msg::Order::KITTING){
      do_kitting(current_order);
    }
    else if (current_order[0].GetType() == ariac_msgs::msg::Order::ASSEMBLY) {
      do_assembly(current_order);
    }
    else if (current_order[0].GetType() == ariac_msgs::msg::Order::COMBINED) {
      do_combined(current_order);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "\033[92m" + std::string("Submitting an order; Order ID: ") + current_order[0].GetId() + std::string("\033[0m"));
    submit_order(current_order[0].GetId().c_str());
    current_order.erase(current_order.begin());
    break;

    // Check priority for breaking
    // Make insert fn for Kitting, Assembly and Combined Orders
  }
  
  // Check if a new order has higher priority than the current one
  // in which case push the current one to incomplete orders.
  if (orders[0].IsPriority() == 1 && current_order[0].IsPriority() == 0){
    incomplete_orders.push_back(current_order.at(0));
    current_order.erase(current_order.begin());
    current_order.push_back(orders.at(0));
    orders.erase(orders.begin());
  } 

  while(!orders.empty() && !current_order.empty()){

    RCLCPP_INFO_STREAM(this->get_logger(),"Continuing Task " << current_order[0].GetId() << " Priority: " << current_order[0].IsPriority());
  
    if((current_order.at(0).IsPriority() != orders.at(0).IsPriority()) && (orders.at(0).IsPriority() == 1)){
      break;
    }

    if(current_order[0].GetType() == ariac_msgs::msg::Order::KITTING){
      do_kitting(current_order);
    }
    else if (current_order[0].GetType() == ariac_msgs::msg::Order::ASSEMBLY) {
      do_assembly(current_order);
    }
    else if (current_order[0].GetType() == ariac_msgs::msg::Order::COMBINED) {
      do_combined(current_order);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "\033[92m" + std::string("Submitting an order; Order ID: ") + current_order[0].GetId() + std::string("\033[0m"));

    submit_order(current_order[0].GetId().c_str());

    current_order.erase(current_order.begin());
    break;
  }

  // Finish the incomplete orders once current order is done.
  if(!incomplete_orders.empty() && current_order.empty()){
    current_order.push_back(incomplete_orders.at(0));
    incomplete_orders.erase(incomplete_orders.begin());
  }

  if(orders.empty() && current_order.empty() && incomplete_orders.empty()){
    submit_orders_ = true;
    return;
  }
  
  goto label;
}

void AriacCompetition::do_kitting(std::vector<Orders> current_order) {
  int type_color_key;  // Stores the key of the specific part in the bin map
  std::vector<std::array<int, 2>> keys;
  int type_color;   // Stores type and color info: For ex: 101 -> Battery Green

  populate_bin_part();

  for (unsigned int j =0; j<current_order[0].GetKitting().get()->GetParts().size(); j++){
    type_color = (current_order[0].GetKitting().get()->GetParts()[j][1]*10 + current_order[0].GetKitting().get()->GetParts()[j][0]);

    type_color_key = search_bin(type_color);
    if(type_color_key != -1){
      // 1 denotes part found in Bin
      keys.push_back({type_color_key, 1});
    } else if(type_color_key == -1){
      type_color_key = search_conveyor(type_color);
      if(type_color_key != -1){
        // 2 denotes part found in Conveyor
        keys.push_back({type_color_key, 2});
      }
    }

    if(type_color_key == -1){
      RCLCPP_WARN_STREAM(this->get_logger(),"The Missing Part is : " << ConvertPartColorToString(type_color%10) << " " << ConvertPartTypeToString(type_color/10));
      RCLCPP_WARN_STREAM(this->get_logger(),"This Kitting order has insufficient parts : " << current_order[0].GetId());
      // 0 denotes part not found anywhere
      keys.push_back({type_color_key, 0});
    }
  }

  std::string part_info;

  FloorRobotMoveHome();
  FloorRobotPickandPlaceTray(current_order[0].GetKitting().get()->GetTrayId(),current_order[0].GetKitting().get()->GetAgvId());

  int count = 0;
  for (auto i : keys){
    if (i[1] == 0) {
      continue;
    } else if (i[1] == 1) {
      // part_info = ConvertPartColorToString((bin_map[i[0]].part_type_clr)%10) + " " + ConvertPartTypeToString((bin_map[i[0]].part_type_clr)/10);
      RCLCPP_INFO_STREAM(this->get_logger(),"Picking Part " << ConvertPartColorToString(bin_map[i[0]].part_type_clr%10) << " " << ConvertPartTypeToString(bin_map[i[0]].part_type_clr/10));
      // floor_pick_bin_part_client((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]);
      FloorRobotPickBinPart((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]);
      // floor.PickBinPart(part_info,(int(i[0])/9)+1,(int(i[0])%9)+1);
    } else if (i[1] == 2) {
      part_info = ConvertPartColorToString((conveyor_parts[i[0]])%10) + " " + ConvertPartTypeToString((conveyor_parts[i[0]])/10);
      // floor.PickConveyorPart(part_info);
    }
    // floor.PlacePartOnKitTray(part_info, current_order[0].GetKitting().get()->GetParts()[count][2], current_order[0].GetKitting().get()->GetTrayId());
    // floor_place_part_client(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]);
    FloorRobotPlacePartOnKitTray(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]);
    count++;
  }

  move_agv(current_order[0].GetKitting().get()->GetAgvId(), current_order[0].GetKitting().get()->GetDestination());
  FloorRobotMoveHome();

}

void AriacCompetition::do_assembly(std::vector<Orders>  current_order) {
  std::string part_info;
  
  if (current_order[0].GetAssembly().get()->GetAgvNumbers().size() > 1) {
    RCLCPP_INFO_STREAM(this->get_logger(),"Parts can be found on AGVs " << current_order[0].GetAssembly().get()->GetAgvNumbers()[0] << " and " << current_order[0].GetAssembly().get()->GetAgvNumbers()[1]);
    // ceil.MoveToAssemblyStation(ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
    // move_agv(current_order[0].GetAssembly().get()->GetAgvNumbers()[0], ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
    // move_agv(current_order[0].GetAssembly().get()->GetAgvNumbers()[1], ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),"Parts can be found on AGV " << current_order[0].GetAssembly().get()->GetAgvNumbers()[0]);
    // ceil.MoveToAssemblyStation(ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
    // move_agv(current_order[0].GetAssembly().get()->GetAgvNumbers()[0], ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
  }

  for (long unsigned int i = 0; i < current_order[0].GetAssembly().get()->GetParts().size(); i++){
    part_info = ConvertPartColorToString(current_order[0].GetAssembly().get()->GetParts()[i].color) + " " + ConvertPartTypeToString(current_order[0].GetAssembly().get()->GetParts()[i].type);
    RCLCPP_INFO_STREAM(this->get_logger(),"Located " << part_info);
    // ceil.PickPartFromAGV(part_info);
    // ceil.PlacePartInInsert(part_info);
  }
  
  // ceil.SendHome();
}

void AriacCompetition::do_combined(std::vector<Orders>  current_order) {
  int agv_num = determine_agv(current_order[0].GetCombined().get()->GetStation());
  int tray_num = 1;
  std::string part_info;

  // ceil.MoveToAssemblyStation(ConvertAssemblyStationToString(current_order[0].GetCombined().get()->GetStation()));
  RCLCPP_INFO_STREAM(this->get_logger(),"Use AGV " << agv_num);
    
  // floor.ChangeGripper("Tray");
  // floor.PickandPlaceTray(tray_num, agv_num);
  // floor.ChangeGripper("Part");
  
  int type_color_key;
  std::vector<std::array<int, 2>> keys;
  int type_color;
  for (unsigned int j = 0; j < current_order[0].GetCombined().get()->GetParts().size(); j++){
    type_color = (current_order[0].GetCombined().get()->GetParts()[j].type*10 + current_order[0].GetCombined().get()->GetParts()[j].color);

    type_color_key = search_bin(type_color);
    if(type_color_key != -1){
      keys.push_back({type_color_key, 1});
    }
    else if(type_color_key == -1){
      type_color_key = search_conveyor(type_color);
      if(type_color_key != -1){
        keys.push_back({type_color_key, 2});
      }
    }
  }

  int count = 0;
  for (auto i : keys){
    if (i[1] == 0) {
      continue;
    } else if (i[1] == 1) {
      part_info = ConvertPartColorToString((bin_map[i[0]].part_type_clr)%10) + " " + ConvertPartTypeToString((bin_map[i[0]].part_type_clr)/10);
      // floor.PickBinPart(part_info,(int(i[0])/9)+1,(int(i[0])%9)+1);
    } else if (i[1] == 2) {
      part_info = ConvertPartColorToString((conveyor_parts[i[0]])%10) + " " + ConvertPartTypeToString((conveyor_parts[i[0]])/10);
      // floor.PickConveyorPart(part_info);
    }
    // floor.PlacePartOnKitTray(part_info, count+1, tray_num);
    count++;
  }

  // move_agv(agv_num, ConvertDestinationToString(ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT, agv_num));
  // floor.SendHome();

  for (long unsigned int i = 0; i < current_order[0].GetCombined().get()->GetParts().size(); i++){
    part_info = ConvertPartColorToString(current_order[0].GetCombined().get()->GetParts()[i].color) + " " + ConvertPartTypeToString(current_order[0].GetCombined().get()->GetParts()[i].type);
    RCLCPP_INFO_STREAM(this->get_logger(),"Located " << part_info);
    // ceil.PickPartFromAGV(part_info);
    // ceil.PlacePartInInsert(part_info);
  }

  // ceil.SendHome();
}

int AriacCompetition::search_bin(int part) {
  for (auto& it : bin_map) {
    if (it.second.part_type_clr == part) {
      return it.first;
    }
  }
  return -1;
}

int AriacCompetition::search_conveyor(int part) {
  auto idx = std::find(conveyor_parts.begin(), conveyor_parts.end(), part);
  if (idx != conveyor_parts.end()){ 
    return idx - conveyor_parts.begin();
  } else {
    return -1;
  }
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

void AriacCompetition::floor_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg){
  floor_gripper_state_ = *msg;
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("CB"),
  //                       "\nCALLBACK Gripper Enabled: " << floor_gripper_state_.enabled);
}

void AriacCompetition::kts1_camera_cb(
    const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg){
    if (!kts1_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
        kts1_camera_received_data = true;
    }

    kts1_trays_ = msg->tray_poses;
    kts1_camera_pose_ = msg->sensor_pose;
}

void AriacCompetition::kts2_camera_cb(
    const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg){
    if (!kts2_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
        kts2_camera_received_data = true;
    }

    kts2_trays_ = msg->tray_poses;
    kts2_camera_pose_ = msg->sensor_pose;
}

void AriacCompetition::left_bins_camera_cb(
    const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg){
    if (!left_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from left bins camera");
        left_bins_camera_received_data = true;
    }

    left_bins_parts_ = msg->part_poses;
    left_bins_camera_pose_ = msg->sensor_pose;
}

void AriacCompetition::right_bins_camera_cb(
    const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg){
    if (!right_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from right bins camera");
        right_bins_camera_received_data = true;
    }

    right_bins_parts_ = msg->part_poses;
    right_bins_camera_pose_ = msg->sensor_pose;
}

void AriacCompetition::conv_camera_cb(
    const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg){
    if (!conv_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from conveyor camera");
        conv_camera_received_data = true;
    }

    conv_parts_ = msg->part_poses;
    conv_camera_pose_ = msg->sensor_pose;
}

void AriacCompetition::kts1_rgb_camera_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!kts1_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
        kts1_rgb_camera_received_data = true;
    }
    kts1_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::kts2_rgb_camera_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!kts2_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
        kts2_rgb_camera_received_data = true;
    }
    kts2_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::left_bins_rgb_camera_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!left_bins_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from left bins camera");
        left_bins_rgb_camera_received_data = true;
    }
    left_bins_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::right_bins_rgb_camera_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg){
    if (!right_bins_rgb_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from right bins camera");
        right_bins_rgb_camera_received_data = true;
    }
    right_bins_rgb_camera_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void AriacCompetition::right_part_detector_cb(
    const group3::msg::Parts::ConstSharedPtr msg){
    if (!right_part_detector_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from Right part detector node");
        right_part_detector_received_data = true;
    }
    right_parts_ = msg->parts;
}

void AriacCompetition::left_part_detector_cb(
    const group3::msg::Parts::ConstSharedPtr msg){
    if (!left_part_detector_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from Left part detector node");
        left_part_detector_received_data = true;
    }
    left_parts_ = msg->parts;
}

void AriacCompetition::breakbeam_cb(
    const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if (!breakbeam_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from breakbeam node");
        breakbeam_received_data = true;
    }
    breakbeam_time_sec = msg->header.stamp.sec;
    breakbeam_status = msg->object_detected;
    if (breakbeam_status == true){
      RCLCPP_INFO_STREAM(get_logger(), "\033[0;91m Part detected by breakbeam at time stamp \033[0m" << breakbeam_time_sec << " Time now is " << now().seconds());
    //   wait_flag = true;
    //   while(floor_pick_conv_part_client(conv_parts_[0],conv_camera_pose_,breakbeam_time_sec)){
    //     sleep(10);
    //   }
    //   wait_flag = false;
    }
}

geometry_msgs::msg::Pose AriacCompetition::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
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

geometry_msgs::msg::Pose AriacCompetition::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
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

void AriacCompetition::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
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

bool AriacCompetition::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf){
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


void AriacCompetition::FloorRobotMoveHome() {
  floor_robot_->setNamedTarget("home");
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

  RCLCPP_INFO_STREAM(this->get_logger(),
                    "Moved inside tool changer");

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
  // bool found_tray = false;
  std::vector<int> kts2_vec;
  tray_aruco_id = tray_detect(kts1_rgb_camera_image_);
  kts2_vec = tray_detect(kts2_rgb_camera_image_);
  tray_aruco_id.insert(tray_aruco_id.end(), kts2_vec.begin(), kts2_vec.end());
  // RCLCPP_INFO_STREAM(this->get_logger(),"After insert funtion");

  auto tray_it = std::find(tray_aruco_id.begin(), tray_aruco_id.end(), tray_idx);
  // RCLCPP_INFO_STREAM(this->get_logger(),"After find");
  auto tray_id = tray_it -tray_aruco_id.begin();
  
  std::string station;
  geometry_msgs::msg::Pose tray_camera_pose;
  geometry_msgs::msg::Pose camera_pose_;
  geometry_msgs::msg::Pose tray_pose;

  if (tray_id < 3) {
      station = "kts1";
      tray_camera_pose = kts1_trays_[tray_id];
      camera_pose_ = kts1_camera_pose_;
      if (floor_gripper_state_.type != "tray_gripper") {
        FloorRobotChangeGripper("trays","kts1");
      }
  } else {
      station = "kts2";
      tray_camera_pose = kts2_trays_[tray_id-3];
      camera_pose_ = kts2_camera_pose_;
      if (floor_gripper_state_.type != "tray_gripper")
      {
        FloorRobotChangeGripper("trays","kts2");
      }
  }

  tray_pose = MultiplyPose(camera_pose_, tray_camera_pose);
  // RCLCPP_INFO_STREAM(this->get_logger(), "\n\nTray Pose: " << tray_pose.position.x << " " << tray_pose.position.y << " " << tray_pose.position.z);

  double tray_rotation = GetYaw(tray_pose); //3.14;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
  // waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
  //                               tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z, SetRobotOrientation(tray_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
  
  FloorRobotSetGripperState(true);
  // Call enable service
  // std::string srv_name = "/ariac/floor_robot_enable_gripper";
  // std::shared_ptr<rclcpp::Node> node =
  //     rclcpp::Node::make_shared("client_floor_robot_enable_gripper");
    
  // rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
  //     node->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name);
  // auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  // request->enable = true;

  // while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
  //   if (!rclcpp::ok()) {
  //   RCLCPP_ERROR(this->get_logger(),
  //                   "Interrupted while waiting for the service. Exiting.");
  //   }
  //   RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "Service not available, waiting again...");
  // }

  // auto result = client->async_send_request(request);

  // if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
  // }
  
  
  
  // RCLCPP_INFO(this->get_logger(), "\n\nBefore Attach");
  // FloorRobotWaitForAttach(1.0);
  // RCLCPP_INFO(this->get_logger(), "\n\nAfter Attach");

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

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = GetYaw(agv_tray_pose);

  // FloorRobotSetGripperState(false);
  waypoints.clear();
  // waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
  //                               agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  FloorRobotSetGripperState(false);
  // request->enable = false;

  // while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
  //   if (!rclcpp::ok()) {
  //   RCLCPP_ERROR(this->get_logger(),
  //                   "Interrupted while waiting for the service. Exiting.");
  //   }
  //   RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "Service not available, waiting again...");
  // }

  // auto result2 = client->async_send_request(request);

  // if (rclcpp::spin_until_future_complete(node, result2) != rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
  // }


  floor_robot_->detachObject(tray_name);

  // publish to robot state
  // LockAGVTray(agv_num);
  lock_agv(agv_num);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

}

bool AriacCompetition::FloorRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_camera_pose,int part_quad){
  bool found_part = false;
  std::string bin_side;
  geometry_msgs::msg::Pose camera_pose;
  std::string station;

  if (part_quad < 37) {
      bin_side = "right_bins";
      // request->part_pose = part_pose;
      camera_pose = right_bins_camera_pose_;
      // RCLCPP_INFO_STREAM(this->get_logger(),
      //                   "\nIf Gripper Type: " << floor_gripper_state_.type);

      if (floor_gripper_state_.type != "part_gripper")
      {
        FloorRobotChangeGripper("parts","kts2");
      }
  } else {
      bin_side = "left_bins";
      // request->part_pose = part_pose;
      camera_pose = left_bins_camera_pose_;
      //  RCLCPP_INFO_STREAM(this->get_logger(),
      //                   "\nElse Gripper Type: " << floor_gripper_state_.type);
      if (floor_gripper_state_.type != "part_gripper")
      {
        FloorRobotChangeGripper("parts","kts1");
      }
  }

  geometry_msgs::msg::Pose part_pose;
  part_pose = MultiplyPose(camera_pose, part_camera_pose);
  double part_rotation = GetYaw(part_pose);

  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
  //                               part_pose.position.z + 0.2, SetRobotOrientation(part_rotation)));

  // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
  //                               part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_type], SetRobotOrientation(part_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

  FloorRobotSetGripperState(true);
  // std::string srv_name = "/ariac/floor_robot_enable_gripper";
  // std::shared_ptr<rclcpp::Node> node =
  //     rclcpp::Node::make_shared("client_floor_robot_enable_gripper");
    
  // rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
  //     node->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name);

  // // Call enable service
  // auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  // request->enable = true;

  // while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
  //   if (!rclcpp::ok()) {
  //   RCLCPP_ERROR(this->get_logger(),
  //                   "Interrupted while waiting for the service. Exiting.");
  //   }
  //   RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "Service not available, waiting again...");
  // }

  // auto result = client->async_send_request(request);

  // if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
  // }

  // FloorRobotWaitForAttach(3.0);

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

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

}

// ADD LATER!!!
// bool FloorRobotPickConvPart(geometry_msgs::msg::Pose part_pose,geometry_msgs::msg::Pose camera_pose,int detection_time);

bool AriacCompetition::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant) {
  if (!floor_gripper_state_.attached) {
      RCLCPP_ERROR(this->get_logger(), "No part attached");
  }

  // Move to agv
  floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();
  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "\nAFTER MOVE 2 TARGET");

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("CB"),
  //                       "\nCALLBACK Gripper Enabled: " << floor_robot_attached_part_.type << " " << floor_robot_attached_part_.color);
  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "\nBefore Pushback");
  // waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
  //                               part_drop_pose.position.z + 0.2, SetRobotOrientation(0)));

  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                                SetRobotOrientation(0)));

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("CB"),
  //                       "\nCALLBACK Gripper Enabled: " << floor_gripper_state_.enabled);
  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "\nAfter Pushback");
  FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("CB"),
  //                       "\nCALLBACK Gripper Enabled: " << floor_gripper_state_.enabled);
  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "\nAfter Move Cartesian");
  // Drop part in quadrant
  FloorRobotSetGripperState(false);

  std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                          "_" + part_types_[floor_robot_attached_part_.type];
  floor_robot_->detachObject(part_name);

  // FloorRobotSetGripperState(false);
  // std::string srv_name = "/ariac/floor_robot_enable_gripper";
  // std::shared_ptr<rclcpp::Node> node =
  //     rclcpp::Node::make_shared("client_floor_robot_enable_gripper");
    
  // rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
  //     node->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name);

  // // Call enable service
  // auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  // request->enable = false;

  // while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
  //   if (!rclcpp::ok()) {
  //   RCLCPP_ERROR(this->get_logger(),
  //                   "Interrupted while waiting for the service. Exiting.");
  //   }
  //   RCLCPP_INFO_STREAM(this->get_logger(),
  //                       "Service not available, waiting again...");
  // }

  // auto result = client->async_send_request(request);

  // if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
  // }

  waypoints.clear();
  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + 0.3,
                                SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto ariac_competition = std::make_shared<AriacCompetition>("group3_Competitor");

//     rclcpp::spin(ariac_competition);
    
//     rclcpp::shutdown();
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto ariac_competition = std::make_shared<AriacCompetition>("group3_Competitor");
    // auto floor_robot = std::make_shared<FloorRobot>();
    rclcpp::executors::MultiThreadedExecutor executor;

    // auto spin_thread = std::make_unique<std::thread>([&executor, &ariac_competition, &floor_robot]() {
      // executor.add_node(floor_robot);
    executor.add_node(ariac_competition);
      
      // executor.remove_node(ariac_competition);
      // executor.remove_node(floor_robot);
    // });

    // spin_thread->join();
    executor.spin();
    rclcpp::shutdown();
}

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   auto test_competitor = std::make_shared<AriacCompetition>("Group3_Competitor");

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(test_competitor);
//   std::thread([&executor]() { executor.spin(); });

//   RCLCPP_INFO_STREAM(rclcpp::get_logger("shutdown"),"Shutdown");
//   rclcpp::shutdown();
// }
