#include "../include/group3/ariac_competition.hpp"

Orders::Orders(void){}

void AriacCompetition::end_competition_timer_callback() {
  if (competition_state_ == 3 && submit_orders_ == 1) {
    std::string srv_name = "/ariac/end_competition";

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("end_trigger_client");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = node->create_client<std_srvs::srv::Trigger>(srv_name);;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "end_trigger_client returned result %d",
                  result.get()->success);
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Failed to call trigger service");
    }
  }
}



void AriacCompetition::competition_state_cb( const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)  {
  competition_state_ = msg->competition_state;
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
    std::string srv_name = "/ariac/start_competition";

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_trigger_client");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = node->create_client<std_srvs::srv::Trigger>(srv_name);;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "start_trigger_client returned result %d",
                  result.get()->success);
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Failed to call trigger service");
    }
  }
}

void AriacCompetition::order_callback(ariac_msgs::msg::Order::SharedPtr msg){

    // RCLCPP_INFO(this->get_logger(), "Order ID CB: ");
    Orders order;
    order.id = msg->id;
    order.priority = msg->priority;
    order.type = msg->type;

    if (order.type == 0){
        // RCLCPP_INFO(this->get_logger(), "KITTING: ");
        order.KittingTask_var.agv_id = msg->kitting_task.agv_number;
        order.KittingTask_var.tray_id = msg->kitting_task.tray_id;
        order.KittingTask_var.destination = msg->kitting_task.destination;
        for (unsigned int i = 0 ; i < msg->kitting_task.parts.size(); i++)
        {
            order.KittingTask_var.part[0] = msg->kitting_task.parts[i].part.color;
            order.KittingTask_var.part[1] = msg->kitting_task.parts[i].part.type;
            order.KittingTask_var.part[2] = msg->kitting_task.parts[i].quadrant;
            order.KittingTask_var.parts_kit.push_back(order.KittingTask_var.part);
        }
    }

    else if (order.type == 1){
        // RCLCPP_INFO(this->get_logger(), "ASSEM: ");
        // RCLCPP_INFO_STREAM(this->get_logger(), msg->assembly_task.agv_numbers);
        // RCLCPP_INFO_STREAM(this->get_logger(), "0: " << msg->assembly_task.agv_numbers[0].data);
        // RCLCPP_INFO_STREAM(this->get_logger(), "1: " << int(msg->assembly_task.agv_numbers.at(0)));
        for (unsigned i = 0; i < msg->assembly_task.agv_numbers.size(); i++){
            order.AssemblyTask_var.agv_numbers.push_back(int(msg->assembly_task.agv_numbers.at(i)));
            // RCLCPP_INFO_STREAM(this->get_logger(), int(msg->assembly_task.agv_numbers.at(i)));
        }
        
        order.AssemblyTask_var.station = msg->assembly_task.station;
        for (unsigned int i = 0 ; i < msg->assembly_task.parts.size(); i++)
        {
            order.AssemblyTask_var.part_var.type = msg->assembly_task.parts[i].part.type;
            order.AssemblyTask_var.part_var.color = msg->assembly_task.parts[i].part.color;
            order.AssemblyTask_var.part_var.assembled_pose = msg->assembly_task.parts[i].assembled_pose;
            order.AssemblyTask_var.part_var.install_direction = msg->assembly_task.parts[i].install_direction;
            // RCLCPP_INFO(this->get_logger(), "pushback");
            order.AssemblyTask_var.parts_assm.push_back(order.AssemblyTask_var.part_var);
        }
    }

    else if (order.type == 2){
        // RCLCPP_INFO(this->get_logger(), "COMB");
        order.CombinedTask_var.station = msg->combined_task.station;
        for (unsigned int i = 0 ; i < msg->combined_task.parts.size(); i++)
        {
            order.CombinedTask_var.part_var.type = msg->combined_task.parts[i].part.type;
            order.CombinedTask_var.part_var.color = msg->combined_task.parts[i].part.color;
            order.CombinedTask_var.part_var.assembled_pose = msg->combined_task.parts[i].assembled_pose;
            order.CombinedTask_var.part_var.install_direction = msg->combined_task.parts[i].install_direction;
            order.CombinedTask_var.parts_comb.push_back(order.CombinedTask_var.part_var);
        }
    }
    // RCLCPP_INFO(this->get_logger(), "Before If");
    if (order.priority == 0 || orders.size() == 0) {
        orders.push_back(order);
        // RCLCPP_INFO(this->get_logger(), "Order ID 1: %s", order.id.c_str());
    }
    else if (orders[orders.size()-1].priority == 1){
        orders.push_back(order);
        // RCLCPP_INFO(this->get_logger(), "Order ID 2: %s", order.id.c_str());
    }
    else { 
        // RCLCPP_INFO(this->get_logger(), "Order ID 3: %s", order.id.c_str());
        for (unsigned int i = 0; i<orders.size(); i++) {
            if (orders[i].priority == 0) {
                orders.insert(i+orders.begin(), order);
                break;
            }
            
        }
    }
    
    if (orders.size() == 3) {
        for (auto i = 0; i < 3; i++) {
            RCLCPP_INFO(this->get_logger(), "Submitting order: %s", orders[i].id.c_str());
            submit_order(orders[i].id.c_str());
        }
    }
    // RCLCPP_INFO(this->get_logger(), "Order ID: %s", order.id.c_str());
}

void AriacCompetition::submit_order(std::string order_id)
{
    std::string srv_name = "/ariac/submit_order";

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("end_trigger_client");
    rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client = node->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);;

    auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    request->order_id = order_id;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "end_trigger_client returned result %d",
                  result.get()->success);
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Failed to call trigger service");
    }
  
  
  
  // rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  // client = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
  // auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  // request->order_id = order_id;

  // auto result = client->async_send_request(request);
  // RCLCPP_INFO(this->get_logger(), "Submitted order: %s", order_id.c_str());
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto ariac_competition  = std::make_shared<AriacCompetition>("RWA1");
  // Start Competition
  rclcpp::spin(ariac_competition);
  rclcpp::shutdown();

}