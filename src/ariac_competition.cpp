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
#include <algorithm>
#include <array>
#include <iterator>
#include <vector>

void AriacCompetition::setup_map(){
    for(unsigned int i = 1; i <= 72; i++){
      bin_map[i];
    }
}

void AriacCompetition::end_competition_timer_callback() {
  if (competition_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submit_orders_ == 1) {

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
  if((!orders.empty() || !current_order.empty()) &&  c_flag)
    process_order();
}

void AriacCompetition::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) {
  competition_state_ = msg->competition_state;

  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
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
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }
  }
}


void AriacCompetition::order_callback(ariac_msgs::msg::Order::SharedPtr msg) {
  Orders order(msg->id, msg->type, msg->priority);
//   order.id = msg->id;
//   order.priority = msg->priority;
//   order.type = msg->type;

  if (order.GetType() == ariac_msgs::msg::Order::KITTING) {
    // kitting_.agv_id = msg->kitting_task.agv_number;
    // kitting_.tray_id = msg->kitting_task.tray_id;
    // kitting_.destination = msg->kitting_task.destination;
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
      std::vector<unsigned int> _agv_numbers;
      for (unsigned i = 0; i < msg->assembly_task.agv_numbers.size(); i++) {
        _agv_numbers.push_back(
        static_cast<int>(msg->assembly_task.agv_numbers.at(i)));
      }
      Part part_var;
      std::vector<Part> _parts_assm;
    //   order.AssemblyTask_var.station = msg->assembly_task.station;
      for (unsigned int i = 0; i < msg->assembly_task.parts.size(); i++) {
        part_var.type =
            msg->assembly_task.parts[i].part.type;
        part_var.color =
            msg->assembly_task.parts[i].part.color;
        part_var.assembled_pose =
            msg->assembly_task.parts[i].assembled_pose;
        part_var.install_direction =
            msg->assembly_task.parts[i].install_direction;
        _parts_assm.push_back(part_var);
      }
    Assembly assembly_(_agv_numbers, msg->assembly_task.station, _parts_assm);
    order.SetAssembly(std::make_shared<Assembly> (assembly_));
  }  else if (order.GetType() == ariac_msgs::msg::Order::COMBINED) {
    Part part_var;
    std::vector<Part> _parts_comb;
    for (unsigned int i = 0; i < msg->combined_task.parts.size(); i++) {
      part_var.type =
          msg->combined_task.parts[i].part.type;
      part_var.color =
          msg->combined_task.parts[i].part.color;
      part_var.assembled_pose =
          msg->combined_task.parts[i].assembled_pose;
      part_var.install_direction =
          msg->combined_task.parts[i].install_direction;
      _parts_comb.push_back(part_var);
    }
    Combined combined_(msg->combined_task.station, _parts_comb);
    order.SetCombined(std::make_shared<Combined> (combined_));
  }

  submit_orders_ = 0;
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

void AriacCompetition::bin_parts_callback(ariac_msgs::msg::BinParts::SharedPtr msg) {
  int m,n;
  AriacCompetition::setup_map();
  for (unsigned  int i = 0; i < msg->bins.size(); i++) { 
    m = 9*((msg->bins[i].bin_number)-1);
    n = m + 9;
    for (unsigned int j =0; j < msg->bins[i].parts.size();j++){
      for (int k = 0; k < msg->bins[i].parts[j].quantity;k++){
        for (int a = m; a< n;a++){
          if (bin_map[a].part_type_clr == -1){
            bin_map[a].part_type_clr = (msg->bins[i].parts[j].part.type)*10 + (msg->bins[i].parts[j].part.color);
            break;
            // add pose later
          }
        }
      }
    }
  }
 
  RCLCPP_INFO_STREAM(this->get_logger(), "Bin Part Information populated");
  bin_parts_subscriber_.reset();
}

void AriacCompetition::conveyor_parts_callback(ariac_msgs::msg::ConveyorParts::SharedPtr msg) {
  
  for (unsigned int i = 0; i < msg->parts.size(); i++) {
    for (int j = 0; j < msg->parts[i].quantity; j++) {
      conveyor_parts.push_back((msg->parts[i].part.type)*10 + (msg->parts[i].part.color));
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Conveyor Part Information populated: " << conveyor_parts.size());
  c_flag = true;
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

int AriacCompetition::search_bin(int part){
  for (auto& it : bin_map) {
    if (it.second.part_type_clr == part) {
      return it.first;
    }
  }
  return -1;
}

int AriacCompetition::search_conveyor(int part){
  auto idx = std::find(conveyor_parts.begin(), conveyor_parts.end(), part);
  if (idx != conveyor_parts.end()){ 
    return idx - conveyor_parts.begin();
  } else {return -1;}
}

std::string AriacCompetition::ConvertPartTypeToString(int part_type){
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

std::string AriacCompetition::ConvertPartColorToString(int part_color){
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

std::string AriacCompetition::ConvertDestinationToString(int destination, int agv_num)
{
    if (agv_num == 1 || agv_num == 2)
    {
        if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
            return "Assembly Station 1";
        else if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
            return "Assembly Station 2";
    }
    else if (agv_num == 3 || agv_num == 4)
    {
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

std::string AriacCompetition::ConvertAssemblyStationToString(int station_id){
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

void AriacCompetition::do_kitting(std::vector<Orders> current_order){
  // RCLCPP_INFO_STREAM(this->get_logger(),"Doing Kitting order: " << current_order[0].GetId());
  int key;
  std::vector<std::array<int, 2>> keys;
  int t_c;
  for (unsigned int j =0; j<current_order[0].GetKitting().get()->GetParts().size(); j++){
    t_c = (current_order[0].GetKitting().get()->GetParts()[j][1]*10 + current_order[0].GetKitting().get()->GetParts()[j][0]);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Value sent is : " << t_c);
    key = search_bin(t_c);
    if(key != -1){
      keys.push_back({key,1});
    }
    else if(key == -1){
      key = search_conveyor(t_c);
      if(key != -1){
        keys.push_back({key,2});
      }
    }
    // RCLCPP_INFO_STREAM(this->get_logger(),"Key in Hash Map for kitting order is : " << key);
    if(key == -1){
      RCLCPP_WARN_STREAM(this->get_logger(),"The Missing Part is : " << ConvertPartColorToString(t_c%10) << " " << ConvertPartTypeToString(t_c/10));
      RCLCPP_WARN_STREAM(this->get_logger(),"This Kitting order has insufficient parts : " << current_order[0].GetId());
      keys.push_back({key,0});
    }
  }
  std::string part_info;
  floor.ChangeGripper("Tray");
  floor.PickandPlaceTray(current_order[0].GetKitting().get()->GetTrayId(),current_order[0].GetKitting().get()->GetAgvId());
  floor.ChangeGripper("Part");
  int count = 0;
  for (auto i : keys){
    if (i[1] == 0) {
      continue;
    } else if (i[1] == 1) {
      part_info = ConvertPartColorToString((bin_map[i[0]].part_type_clr)%10) + " " + ConvertPartTypeToString((bin_map[i[0]].part_type_clr)/10);
      floor.PickBinPart(part_info,(int(i[0])/9)+1,(int(i[0])%9)+1);
    } else if (i[1] == 2) {
      part_info = ConvertPartColorToString((conveyor_parts[i[0]])%10) + " " + ConvertPartTypeToString((conveyor_parts[i[0]])/10);
      floor.PickConveyorPart(part_info);
    }
    floor.PlacePartOnKitTray(part_info, current_order[0].GetKitting().get()->GetParts()[count][2], current_order[0].GetKitting().get()->GetTrayId());
    count++;
  }

  move_agv(current_order[0].GetKitting().get()->GetAgvId(), ConvertDestinationToString(current_order[0].GetKitting().get()->GetDestination(), current_order[0].GetKitting().get()->GetAgvId()));
  floor.SendHome();

}

void AriacCompetition::do_assembly(std::vector<Orders>  current_order){
  // RCLCPP_INFO_STREAM(this->get_logger(),"Doing Assembly order: " << current_order[0].GetId());
  std::string part_info;
  
  if (current_order[0].GetAssembly().get()->GetAgvNumbers().size() > 1){
    RCLCPP_INFO_STREAM(this->get_logger(),"Parts can be found on AGVs " << current_order[0].GetAssembly().get()->GetAgvNumbers()[0] << " and " << current_order[0].GetAssembly().get()->GetAgvNumbers()[1]);
    ceil.MoveToAssemblyStation(ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
    move_agv(current_order[0].GetAssembly().get()->GetAgvNumbers()[0], ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
    move_agv(current_order[0].GetAssembly().get()->GetAgvNumbers()[1], ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),"Parts can be found on AGV " << current_order[0].GetAssembly().get()->GetAgvNumbers()[0]);
    ceil.MoveToAssemblyStation(ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
    move_agv(current_order[0].GetAssembly().get()->GetAgvNumbers()[0], ConvertAssemblyStationToString(current_order[0].GetAssembly().get()->GetStation()));
  }

  for (long unsigned int i=0; i<current_order[0].GetAssembly().get()->GetParts().size(); i++){
    part_info = ConvertPartColorToString(current_order[0].GetAssembly().get()->GetParts()[i].color) + " " + ConvertPartTypeToString(current_order[0].GetAssembly().get()->GetParts()[i].type);
    RCLCPP_INFO_STREAM(this->get_logger(),"Located " << part_info);
    ceil.PickPartFromAGV(part_info);
    ceil.PlacePartInInsert(part_info);
  }
  
  ceil.SendHome();
}

void AriacCompetition::do_combined(std::vector<Orders>  current_order){
  // RCLCPP_INFO_STREAM(this->get_logger(),"Doing Combined order: " << current_order[0].GetId());
  

  int agv_num = determine_agv(current_order[0].GetCombined().get()->GetStation());
  int tray_num = 1;
  std::string part_info;

  ceil.MoveToAssemblyStation(ConvertAssemblyStationToString(current_order[0].GetCombined().get()->GetStation()));
  RCLCPP_INFO_STREAM(this->get_logger(),"Use AGV " << agv_num);
    
  floor.ChangeGripper("Tray");
  floor.PickandPlaceTray(tray_num, agv_num);
  floor.ChangeGripper("Part");
  
  int key;
  std::vector<std::array<int, 2>> keys;
  int t_c;
  for (unsigned int j =0; j<current_order[0].GetCombined().get()->GetParts().size(); j++){
    t_c = (current_order[0].GetCombined().get()->GetParts()[j].type*10 + current_order[0].GetCombined().get()->GetParts()[j].color);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Value sent is : " << t_c);
    key = search_bin(t_c);
    if(key != -1){
      keys.push_back({key,1});
    }
    else if(key == -1){
      key = search_conveyor(t_c);
      if(key != -1){
        keys.push_back({key,2});
      }
    }
  }

  int count = 0;
  for (auto i : keys){
    if (i[1] == 0) {
      continue;
    } else if (i[1] == 1) {
      part_info = ConvertPartColorToString((bin_map[i[0]].part_type_clr)%10) + " " + ConvertPartTypeToString((bin_map[i[0]].part_type_clr)/10);
      floor.PickBinPart(part_info,(int(i[0])/9)+1,(int(i[0])%9)+1);
    } else if (i[1] == 2) {
      part_info = ConvertPartColorToString((conveyor_parts[i[0]])%10) + " " + ConvertPartTypeToString((conveyor_parts[i[0]])/10);
      floor.PickConveyorPart(part_info);
    }
    floor.PlacePartOnKitTray(part_info, count+1, tray_num);
    count++;
  }

  move_agv(agv_num, ConvertDestinationToString(ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT, agv_num));

  floor.SendHome();

  for (long unsigned int i=0; i<current_order[0].GetCombined().get()->GetParts().size(); i++){
    part_info = ConvertPartColorToString(current_order[0].GetCombined().get()->GetParts()[i].color) + " " + ConvertPartTypeToString(current_order[0].GetCombined().get()->GetParts()[i].type);
    RCLCPP_INFO_STREAM(this->get_logger(),"Located " << part_info);
    ceil.PickPartFromAGV(part_info);
    ceil.PlacePartInInsert(part_info);
  }

  ceil.SendHome();
}

void AriacCompetition::process_order(){
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
    // if((current_order.at(0).priority != orders.at(0).priority) && (orders.at(0).priority == 1))
    //   break;
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

  if(!incomplete_orders.empty() && current_order.empty()){
    current_order.push_back(incomplete_orders.at(0));
    incomplete_orders.erase(incomplete_orders.begin());
  }

  if(orders.empty() && current_order.empty() && incomplete_orders.empty()){
    submit_orders_ = 1;
    return;
  }
  
  goto label;
}

void AriacCompetition::lock_agv(int agv_num){
  RCLCPP_INFO_STREAM(this->get_logger(),"Lock AGV " << agv_num);
}

void AriacCompetition::move_agv(int agv_num, std::string dest){
  AriacCompetition::lock_agv(agv_num);
  available_agv.erase(std::find(available_agv.begin(), available_agv.end(), agv_num));
  RCLCPP_INFO_STREAM(this->get_logger(),"Move AGV " << agv_num << " to " << dest);
}

int AriacCompetition::determine_agv(int station_num){
  std::set<int> s1;
  std::set<int> s2;
  if (station_num == ariac_msgs::msg::AssemblyTask::AS1 || station_num == ariac_msgs::msg::AssemblyTask::AS2){
    // RCLCPP_INFO_STREAM(this->get_logger(),"Use AGV 1 or 2, based on availability");
    s1 = {1,2};
  } else if (station_num == ariac_msgs::msg::AssemblyTask::AS3 || station_num == ariac_msgs::msg::AssemblyTask::AS4) {
    // RCLCPP_INFO_STREAM(this->get_logger(),"Use AGV 3 or 4, based on availability");
    s1 = {3,4};
  } 
  
  if (station_num == ariac_msgs::msg::CombinedTask::AS1 || station_num == ariac_msgs::msg::CombinedTask::AS3){
    s2 = {1,3};
  } else {
    s2 = {2,4};
  }

  std::set<int> result;
  std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(result, result.begin()));
  return *(result.begin());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto ariac_competition = std::make_shared<AriacCompetition>("Group3_Competitor");
  rclcpp::spin(ariac_competition);
  rclcpp::shutdown();
}