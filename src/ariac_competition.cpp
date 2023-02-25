/**
 * @copyright Copyright (c) 2023
 * @file ariac_competition.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of RWA1 for ARIAC 2023 (Group 3)
 * @version 0.1
 * @date 2023-02-25
 * 
 * 
 */

#include "../include/group3/ariac_competition.hpp"

Orders::Orders() {}

void AriacCompetition::end_competition_timer_callback() {
  if (competition_state_ == 3 && submit_orders_ == 1) {
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
      RCLCPP_INFO_STREAM(this->get_logger(), "All Orders Submitted and Ending Competition");
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
    }
  }
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

  if (competition_state_ == 3) {
    for (auto i = 0; i < 3; i++) {
      if (orders[i].type == 0) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Submitting a Kitting order; Order ID: " << orders[i].id.c_str());
        submit_order(orders[i].id.c_str());
      } else if (orders[i].type == 1) {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Submitting an Assembly order; Order ID: " << orders[i].id.c_str());
        submit_order(orders[i].id.c_str());
      } else if (orders[i].type == 2) {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Submitting a Combined order; Order ID: " << orders[i].id.c_str());
        submit_order(orders[i].id.c_str());
      }
    }
    submit_orders_ = 1;
  }
}

void AriacCompetition::order_callback(ariac_msgs::msg::Order::SharedPtr msg) {
  Orders order;
  order.id = msg->id;
  order.priority = msg->priority;
  order.type = msg->type;

  if (order.type == 0) {
    order.KittingTask_var.agv_id = msg->kitting_task.agv_number;
    order.KittingTask_var.tray_id = msg->kitting_task.tray_id;
    order.KittingTask_var.destination = msg->kitting_task.destination;
    for (unsigned int i = 0; i < msg->kitting_task.parts.size(); i++) {
      order.KittingTask_var.part[0] = msg->kitting_task.parts[i].part.color;
      order.KittingTask_var.part[1] = msg->kitting_task.parts[i].part.type;
      order.KittingTask_var.part[2] = msg->kitting_task.parts[i].quadrant;
      order.KittingTask_var.parts_kit.push_back(order.KittingTask_var.part);
    }
  }  else if (order.type == 1) {
      for (unsigned i = 0; i < msg->assembly_task.agv_numbers.size(); i++) {
        order.AssemblyTask_var.agv_numbers.push_back(
            static_cast<int>(msg->assembly_task.agv_numbers.at(i)));
      }

      order.AssemblyTask_var.station = msg->assembly_task.station;
      for (unsigned int i = 0; i < msg->assembly_task.parts.size(); i++) {
        order.AssemblyTask_var.part_var.type =
            msg->assembly_task.parts[i].part.type;
        order.AssemblyTask_var.part_var.color =
            msg->assembly_task.parts[i].part.color;
        order.AssemblyTask_var.part_var.assembled_pose =
            msg->assembly_task.parts[i].assembled_pose;
        order.AssemblyTask_var.part_var.install_direction =
            msg->assembly_task.parts[i].install_direction;
        order.AssemblyTask_var.parts_assm.push_back(
            order.AssemblyTask_var.part_var);
      }
  }  else if (order.type == 2) {
    order.CombinedTask_var.station = msg->combined_task.station;
    for (unsigned int i = 0; i < msg->combined_task.parts.size(); i++) {
      order.CombinedTask_var.part_var.type =
          msg->combined_task.parts[i].part.type;
      order.CombinedTask_var.part_var.color =
          msg->combined_task.parts[i].part.color;
      order.CombinedTask_var.part_var.assembled_pose =
          msg->combined_task.parts[i].assembled_pose;
      order.CombinedTask_var.part_var.install_direction =
          msg->combined_task.parts[i].install_direction;
      order.CombinedTask_var.parts_comb.push_back(
          order.CombinedTask_var.part_var);
    }
  }

  if (order.priority == 0 || orders.size() == 0) {
    orders.push_back(order);
  } else if (orders[orders.size() - 1].priority == 1) {
    orders.push_back(order);
  } else {
    for (unsigned int i = 0; i < orders.size(); i++) {
      if (orders[i].priority == 0) {
        orders.insert(i + orders.begin(), order);
        break;
      }
    }
  }
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
    RCLCPP_INFO_STREAM(this->get_logger(),"submit_order_client response: " << result.get()->success << "\t" << result.get()->message);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service submit_order");
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto ariac_competition = std::make_shared<AriacCompetition>("RWA1");

  rclcpp::spin(ariac_competition);
  rclcpp::shutdown();
}
