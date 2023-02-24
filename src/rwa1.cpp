#include <iostream>
#include <chrono>
#include "../include/group3/rwa1.hpp"

using namespace std::chrono_literals;

void Orders::order_callback(ariac_msgs::msg::Order::SharedPtr msg){

    Orders order("orders");
    order.id = msg->id;
    order.priority = msg->priority;
    order.type = msg->type;

    if (order.type == 0){
        order.KittingTask_var.agv_id = msg->kitting_task.agv_number;
        order.KittingTask_var.tray_id = msg->kitting_task.tray_id;
        order.KittingTask_var.destination = msg->kitting_task.destination;
        for (int i = 0 ; i < msg->kitting_task.parts.size(); i++)
        {
            order.KittingTask_var.part[0] = msg->kitting_task.parts[i].part.color;
            order.KittingTask_var.part[1] = msg->kitting_task.parts[i].part.type;
            order.KittingTask_var.part[2] = msg->kitting_task.parts[i].quadrant;
            order.KittingTask_var.parts_kit.push_back(order.KittingTask_var.part);
        }
    }

    else if (order.type == 1){
        for (int i = 0; i < msg->assembly_task.agv_numbers.size(); i++){
            order.AssemblyTask_var.agv_numbers[i] = msg->assembly_task.agv_numbers[i];
        }
        order.AssemblyTask_var.station = msg->assembly_task.station;
        for (int i = 0 ; i < msg->assembly_task.parts.size(); i++)
        {
            order.AssemblyTask_var.part_var.type = msg->assembly_task.parts[i].part.type;
            order.AssemblyTask_var.part_var.color = msg->assembly_task.parts[i].part.color;
            order.AssemblyTask_var.part_var.assembled_pose = msg->assembly_task.parts[i].assembled_pose;
            order.AssemblyTask_var.part_var.install_direction = msg->assembly_task.parts[i].install_direction;
            order.AssemblyTask_var.parts_assm.push_back(order.AssemblyTask_var.part_var);
        }
    }

    else if (order.type == 2){
        order.CombinedTask_var.station = msg->combined_task.station;
        for (int i = 0 ; i < msg->combined_task.parts.size(); i++)
        {
            order.CombinedTask_var.part_var.type = msg->combined_task.parts[i].part.type;
            order.CombinedTask_var.part_var.color = msg->combined_task.parts[i].part.color;
            order.CombinedTask_var.part_var.assembled_pose = msg->combined_task.parts[i].assembled_pose;
            order.CombinedTask_var.part_var.install_direction = msg->combined_task.parts[i].install_direction;
            order.CombinedTask_var.parts_comb.push_back(order.CombinedTask_var.part_var);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Order ID: %s", order.id.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto subscriber_cpp = std::make_shared<Orders>("subscriber_cpp");
    rclcpp::spin(subscriber_cpp);
    rclcpp::shutdown();
}