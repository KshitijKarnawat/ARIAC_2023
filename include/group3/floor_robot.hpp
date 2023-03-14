#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

class FloorRobot
{
public:
    void SendHome();
    void ChangeGripper(std::string gripper);
    void PickandPlaceTray(int tray_id,int agv_num);
    void PickBinPart(std::string part_info, int bin_num, int slot_num);
    void PickConveyorPart(std::string part_info);
    void PlacePartOnKitTray(std::string part_info, int quad, int tray_id);
};
