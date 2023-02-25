#include <iostream>
#include <chrono>
#include <typeinfo>
#include "../include/group3/rwa1.hpp"

using namespace std::chrono_literals;

Orders::Orders(void){}

void OrderNode::order_callback(ariac_msgs::msg::Order::SharedPtr msg){

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
            RCLCPP_INFO(this->get_logger(), "Pushed Order ID: %s", orders[i].id.c_str());
        }
    }
    // RCLCPP_INFO(this->get_logger(), "Order ID: %s", order.id.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto subscriber_cpp = std::make_shared<OrderNode>("subscriber_cpp");
    rclcpp::spin(subscriber_cpp);
    rclcpp::shutdown();
}