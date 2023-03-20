/**
 * @file floor_robot.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of the FloorRobot class for ARIAC 2023 (Group 3) 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ceiling_robot.hpp"


void CeilingRobot::SendHome() {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Going Home ");
}

void CeilingRobot::ChangeGripper(std::string gripper) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Change gripper to " << gripper << " gripper");
}

void CeilingRobot::PickandPlaceTray(int tray_id,int agv_num) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Pick Tray " << tray_id << " and Place Tray On AGV " << agv_num);
}

void CeilingRobot::PickBinPart(std::string part_info, int bin_num) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Pick " << part_info << " from Bin " << bin_num);
}

void CeilingRobot::PickConveyorPart(std::string part_info) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Pick " << part_info << " from Conveyor");
}

void CeilingRobot::PlacePartOnKitTray(std::string part_info, int quad, int tray_id) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Place "<< part_info << " on Tray " << tray_id << " in Quadrant " << quad);
}

void CeilingRobot::PlacePartInInsert(std::string part_info) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Place " << part_info << " in the corresponding Insert");
}

void CeilingRobot::MoveToAssemblyStation(std::string station) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Perform Assembly at " << station);
}

void CeilingRobot::PickPartFromAGV(std::string part_info) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling Robot"),"Pick " << part_info << " from AGV");
}
