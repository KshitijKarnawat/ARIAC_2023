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
#include <vector>

#include "../include/group3/ariac_competition.hpp"
#include "../include/group3/floor_robot.hpp"
#include "../include/group3/tray_id_detect.hpp"
#include "../include/group3/part_type_detect.hpp"

AriacCompetition::AriacCompetition(std::string node_name): Node(node_name) {
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

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");

  end_competition_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AriacCompetition::end_competition_timer_callback, this));    
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
  std::vector<std::vector<int>> right_bin = rightbin(right_bins_rgb_camera_image_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Vector Information populated");
  std::vector<std::vector<int>> left_bin = leftbin(left_bins_rgb_camera_image_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Left Vector Information populated");
  int count_right = 0;
  int count_left = 0;
  for (auto part : right_bin){
    bin_map[part[2]].part_type_clr = (part[1]*10 + part[0]);
    bin_map[part[2]].part_pose = right_bins_parts_[count_right];
    count_right++;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Right Information populated");
  for (auto part : left_bin){
    bin_map[part[2]].part_type_clr = (part[1]*10 + part[0]);
    bin_map[part[2]].part_pose = left_bins_parts_[count_left];
    count_left++;
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

  move_floor_robot_home_client();
  floor_picknplace_tray_client(current_order[0].GetKitting().get()->GetTrayId(),current_order[0].GetKitting().get()->GetAgvId());
  // if (floor_gripper_state_.type != "part_gripper")
  //     {
  //     floor_change_gripper_client("parts","kts2");
  //     }

  int count = 0;
  for (auto i : keys){
    if (i[1] == 0) {
      continue;
    } else if (i[1] == 1) {
      // part_info = ConvertPartColorToString((bin_map[i[0]].part_type_clr)%10) + " " + ConvertPartTypeToString((bin_map[i[0]].part_type_clr)/10);
      RCLCPP_INFO_STREAM(this->get_logger(),"Picking Part " << ConvertPartColorToString(bin_map[i[0]].part_type_clr%10) << " " << ConvertPartTypeToString(bin_map[i[0]].part_type_clr/10));
      floor_pick_bin_part_client((bin_map[i[0]].part_type_clr)%10,(bin_map[i[0]].part_type_clr)/10, bin_map[i[0]].part_pose, i[0]);
      // floor.PickBinPart(part_info,(int(i[0])/9)+1,(int(i[0])%9)+1);
    } else if (i[1] == 2) {
      part_info = ConvertPartColorToString((conveyor_parts[i[0]])%10) + " " + ConvertPartTypeToString((conveyor_parts[i[0]])/10);
      // floor.PickConveyorPart(part_info);
    }
    // floor.PlacePartOnKitTray(part_info, current_order[0].GetKitting().get()->GetParts()[count][2], current_order[0].GetKitting().get()->GetTrayId());
    floor_place_part_client(current_order[0].GetKitting().get()->GetAgvId(),current_order[0].GetKitting().get()->GetParts()[count][2]);
    count++;
  }

  move_agv(current_order[0].GetKitting().get()->GetAgvId(), current_order[0].GetKitting().get()->GetDestination());
  move_floor_robot_home_client();

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
  AriacCompetition::lock_agv(agv_num);
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

void AriacCompetition::move_floor_robot_home_client(){
  std::string srv_name = "/competitor/move_floor_robot_home";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("move_floor_robot_client");
  
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Moved FLoor Robot to Home Pose");
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
  }

}

void AriacCompetition::floor_change_gripper_client(std::string gripper_type_, std::string station_){
  std::string srv_name = "/competitor/floor_robot_change_gripper";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("floor_robot_change_gripper_client");
  
  rclcpp::Client<group3::srv::FloorChangeGripper>::SharedPtr client =
      node->create_client<group3::srv::FloorChangeGripper>(srv_name);

  auto request = std::make_shared<group3::srv::FloorChangeGripper::Request>();
  request->station = station_;
  request->gripper_type = gripper_type_;

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
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully changed gripper");
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to change gripper");
  }

}

void AriacCompetition::floor_picknplace_tray_client(int tray_id , int agv_num){

  std::string srv_name = "/competitor/floor_pick_place_tray";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("floor_pick_place_tray_client");
  
  rclcpp::Client<group3::srv::FloorPickTray>::SharedPtr client =
      node->create_client<group3::srv::FloorPickTray>(srv_name);

  auto request = std::make_shared<group3::srv::FloorPickTray::Request>();

  // bool found_tray = false;

  std::vector<int> kts2_vec;
  tray_aruco_id = tray_detect(kts1_rgb_camera_image_);
  kts2_vec = tray_detect(kts2_rgb_camera_image_);
  tray_aruco_id.insert(tray_aruco_id.end(), kts2_vec.begin(), kts2_vec.end());
  RCLCPP_INFO_STREAM(this->get_logger(),"After insert funtion");

  auto tray_it = std::find(tray_aruco_id.begin(), tray_aruco_id.end(), tray_id);
  RCLCPP_INFO_STREAM(this->get_logger(),"After find");
  auto tray_idx = tray_it -tray_aruco_id.begin();

  if (tray_idx < 3) {
      request->station = "kts1";
      request->tray_pose = kts1_trays_[tray_idx];
      request->camera_pose = kts1_camera_pose_;
      if (floor_gripper_state_.type != "tray_gripper")
      {
      floor_change_gripper_client("trays","kts1");
      }
  } else {
      request->station = "kts2";
      request->tray_pose = kts2_trays_[tray_idx-3];
      request->camera_pose = kts2_camera_pose_;
      if (floor_gripper_state_.type != "tray_gripper")
      {
      floor_change_gripper_client("trays","kts2");
      }
  }

  request->tray_id = tray_id;
  request->agv_num = agv_num;

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
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully Picked Tray");
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to pickup tray");
  }
  client.reset();
}

bool AriacCompetition::floor_pick_bin_part_client(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad){

  std::string srv_name = "/competitor/floor_pick_part_bin";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("floor_pick_part_bin_client");
  
  rclcpp::Client<group3::srv::FloorPickPartBin>::SharedPtr client =
      node->create_client<group3::srv::FloorPickPartBin>(srv_name);

  auto request = std::make_shared<group3::srv::FloorPickPartBin::Request>();

  bool found_part = false;
  std::string bin_side;

  if (part_quad < 37) {
      request->bin_side = "right_bins";
      request->part_pose = part_pose;
      request->camera_pose = right_bins_camera_pose_;
      if (floor_gripper_state_.type != "part_gripper")
      {
      floor_change_gripper_client("parts","kts1");
      }
  } else {
      request->bin_side = "left_bins";
      request->part_pose = part_pose;
      request->camera_pose = left_bins_camera_pose_;
      if (floor_gripper_state_.type != "part_gripper")
      {
      floor_change_gripper_client("parts","kts2");
      }
  }

  request->part_clr = part_clr;
  request->part_type = part_type;

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                        "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully Picked Tray");
    return true;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to pickup tray");
    return false;
  }
  client.reset();
}

bool AriacCompetition::floor_place_part_client(int agv_num, int quadrant){
  std::string srv_name = "/competitor/floor_place_part";

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("floor_place_part_client_");
  
  rclcpp::Client<group3::srv::FloorPlacePart>::SharedPtr client =
      node->create_client<group3::srv::FloorPlacePart>(srv_name);

  auto request = std::make_shared<group3::srv::FloorPlacePart::Request>();

  request->agv_num = agv_num;
  request->quadrant = quadrant;

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                        "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully Placed Part on Kit Tray");
    return true;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to place part on kit tray");
    return false;
  }
  client.reset();
}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto ariac_competition = std::make_shared<AriacCompetition>("Group3_Competitor");

//     rclcpp::spin(ariac_competition);
    
//     rclcpp::shutdown();
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto ariac_competition = std::make_shared<AriacCompetition>("Group3_Competitor");
    auto floor_robot = std::make_shared<FloorRobot>();
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &ariac_competition,  &floor_robot]() {
      executor.add_node(floor_robot);
      executor.add_node(ariac_competition);
      
      // executor.remove_node(ariac_competition);
      // executor.remove_node(floor_robot);
    });

    spin_thread->join();
    executor.spin();
    rclcpp::shutdown();
}