#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

class CeilingRobot {
    public:
        /**
        * @brief Method to send the ceiling robot to the home position
        * 
        */

        void SendHome();
        /**
        * @brief Method to change the gripper of the ceiling robot
        * 
        * @param gripper String containing the name of the gripper to change to
        */

        void ChangeGripper(std::string gripper);
        /**
        * @brief Method to pick the tray and place it on the AGV
        * 
        * @param tray_id ID of the tray to pick
        * @param agv_num Number of the AGV to place the tray on
        */
        void PickandPlaceTray(int tray_id,int agv_num);

        /**
        * @brief Method to pick the part from the bin
        * 
        * @param part_info String containing the part information
        * @param bin_num Number of the bin to pick the part from
        */
        void PickBinPart(std::string part_info, int bin_num);

        /**
        * @brief Method to pick the part from the conveyor
        * 
        * @param part_info String containing the part information
        */
        void PickConveyorPart(std::string part_info);

        /**
        * @brief Method to place the part on the kit tray 
        * 
        * @param part_info String containing the part information
        * @param quad  Quadrant of the kit tray to place the part on
        * @param tray_id ID of the kit tray to place the part on 
        */
        void PlacePartOnKitTray(std::string part_info, int quad, int tray_id);

        /**
        * @brief Method to assemble the part in the briefcase at the corresponding insert
        * 
        * @param part_info String containing the part information 
        */
        void PlacePartInInsert(std::string);

        /**
        * @brief Method to move to the assembly station 
        * 
        * @param station String containing the name of the assembly station 
        */
        void MoveToAssemblyStation(std::string);

        /**
        * @brief Method to pick the part from the AGV 
        * 
        * @param part_info  String containing the part information
        */
        void PickPartFromAGV(std::string);
};
