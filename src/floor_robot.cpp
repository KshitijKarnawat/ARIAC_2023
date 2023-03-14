#include "floor_robot.hpp"


void FloorRobot::SendHome()
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot"),"Going Home ");
}

void FloorRobot::ChangeGripper(std::string gripper)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot"),"Change gripper to " << gripper << " gripper");
}

void FloorRobot::PickandPlaceTray(int tray_id,int agv_num)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot"),"Pick Tray " << tray_id << " and Place Tray On AGV " << agv_num);
}

void FloorRobot::PickBinPart(std::string part_info, int bin_num, int slot_num)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot"),"Pick " << part_info << " from Bin " << bin_num << " Slot " << slot_num);
}

void FloorRobot::PickConveyorPart(std::string part_info)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot"),"Pick " << part_info << " from Conveyor");
}

void FloorRobot::PlacePartOnKitTray(std::string part_info, int quad, int tray_id)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot"),"Place "<< part_info << " on Tray " << tray_id << " in Quadrant " << quad);
}
