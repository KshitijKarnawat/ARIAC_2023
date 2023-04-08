/**
 * @copyright Copyright (c) 2023
 * @file ariac_competition.hpp
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

#pragma once
#include <unistd.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <typeinfo>
#include <vector>
#include <chrono>
#include <map>
#include <utility>
#include <algorithm>
#include <set>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

#include <ariac_msgs/msg/assembly_part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/bin_parts.hpp>
#include <ariac_msgs/msg/conveyor_parts.hpp>
#include <ariac_msgs/msg/basic_logical_camera_image.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>

#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "group3/srv/floor_change_gripper.hpp"
#include "group3/srv/floor_pick_tray.hpp"
#include "group3/srv/floor_pick_part_bin.hpp"
#include "group3/srv/floor_place_part.hpp"

class Orders;

/**
 * @brief Class definition for ARIAC Competition
 * 
 */
class AriacCompetition : public rclcpp::Node {
    public:

        bool conveyor_parts_flag_{false};
        bool submit_orders_{false};
        int competition_state_ = -1;
        bool competition_started_{false};

        std::vector<Orders> orders;
        std::vector<Orders> incomplete_orders;
        std::vector<Orders> current_order;
        std::vector<Orders> submitted_orders;

        std::vector<int> tray_aruco_id;

        struct BinQuadrant {
            int part_type_clr = -1;
            geometry_msgs::msg::PoseStamped part_pose;
        };

        std::vector<int> conveyor_parts;
        std::map<int, BinQuadrant> bin_map;    // Holds part information in 72 possible bin locations (8 bins x 9 locations)

        /**
        * @brief Construct a new Ariac Competition object
        * 
        * @param node_name Name of the node
        */
        AriacCompetition(std::string);

        /**
        * @brief Callback function for competition state subscriber and to start competition
        * 
        * @param msg CompetitionState message 
        */
        void competition_state_cb(
            const ariac_msgs::msg::CompetitionState::ConstSharedPtr);

        /**
        * @brief Callback function to end the competition
        * 
        */
        void end_competition_timer_callback();

        /**
        * @brief Callback function to store the orders
        * 
        * @param msg Order
        */
        void order_callback(const ariac_msgs::msg::Order::SharedPtr);

            /**
        * @brief Callback function to retrieve bin part information
        * 
        * @param msg 
        */
        void bin_parts_callback(const ariac_msgs::msg::BinParts::SharedPtr);

        /**
        * @brief  Callback function to retrieve conveyor part information
        * 
        * @param msg 
        */
        void conveyor_parts_callback(const ariac_msgs::msg::ConveyorParts::SharedPtr);

        /**
        * @brief Method to submit the orders
        * 
        * @param order_id Order ID
        */
        void submit_order(std::string order_id);

        /**
        * @brief Method to process the order
        * 
        */
        void process_order();

        /**
        * @brief Method to do the kitting task
        * 
        */
        void do_kitting(std::vector<Orders>);

        /**
        * @brief Method to perform the assembly task
        * 
        */
        void do_assembly(std::vector<Orders>);

        /**
        * @brief Method to carry out the combined task
        * 
        */
        void do_combined(std::vector<Orders>);

        /**
        * @brief Method to search the bin for the part
        * 
        * @return int 
        */
        int search_bin(int);

        /**
        * @brief Method to check if the conveyor has the part
        * 
        * @return int 
        */
        int search_conveyor(int);

        /**
        * @brief Set the up map object to store the bin part information
        * 
        */
        void setup_map();

        /**
        * @brief Method to convert the part type to string
        * 
        * @param int Part type from ariac_msgs::msg:Part
        * @return std::string Part type as string
        */
        std::string ConvertPartTypeToString(int);

        /**
        * @brief Method to convert the part color to string
        * 
        * @param int Part color from ariac_msgs::msg:Part
        * @return std::string Part color as string
        */
        std::string ConvertPartColorToString(int);
        
        /**
        * @brief Method to convert the destination to string
        * 
        * @param int Destination from ariac_msgs
        * @return std::string Destination as string
        */
        std::string ConvertDestinationToString(int, int);
        
        /**
        * @brief Method to convert the assembly station to string
        * 
        * @param int Assembly station from ariac_msgs
        * @return std::string Assembly station as string
        */
        std::string ConvertAssemblyStationToString(int);

        /**
        * @brief Method to lock the AGV
        * 
        * @param int AGV number
        */
        void lock_agv(int);

        /**
        * @brief Method to unlock the AGV
        * 
        * @param int AGV number
        */
        void unlock_agv(int);

        /**
        * @brief Method to move the AGV
        * 
        * @param int  AGV number
        * @param int AGV Destination
        *
        */
        void move_agv(int, int);

        /**
        * @brief Method to choose the AGV for Combined task
        * 
        * @param int Station number
        */
        int determine_agv(int);

        void move_floor_robot_home_client();

        void floor_change_gripper_client(std::string gripper_type_, std::string station_);

        void floor_picknplace_tray_client(int tray_id, int agv_num);

        bool floor_pick_bin_part_client(int part_clr,int part_type);

        bool floor_place_part_client(int agv_num, int quadrant);

    private:
        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
            competition_state_sub_;

        rclcpp::TimerBase::SharedPtr end_competition_timer_;

        ariac_msgs::msg::Order order_;

        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subscriber_;
        rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_parts_subscriber_;
        rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_parts_subscriber_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kts1_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kts2_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_bins_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_bins_rgb_camera_sub_;

        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr kts1_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr kts2_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr right_bins_camera_sub_;

        rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
        
        // Sensor Images
        cv::Mat kts1_rgb_camera_image_;
        cv::Mat kts2_rgb_camera_image_;
        cv::Mat left_bins_camera_image_;
        cv::Mat right_bins_camera_image_;

        // Sensor poses
        geometry_msgs::msg::Pose kts1_camera_pose_;
        geometry_msgs::msg::Pose kts2_camera_pose_;
        geometry_msgs::msg::Pose left_bins_camera_pose_;
        geometry_msgs::msg::Pose right_bins_camera_pose_;

        // Trays
        std::vector<geometry_msgs::msg::Pose> kts1_trays_;
        std::vector<geometry_msgs::msg::Pose> kts2_trays_;

        // Bins
        std::vector<geometry_msgs::msg::Pose> left_bins_parts_;
        std::vector<geometry_msgs::msg::Pose> right_bins_parts_;

        // Callback Groups
        rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

        // Gripper State
        ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
        ariac_msgs::msg::Part floor_robot_attached_part_;
        ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
        ariac_msgs::msg::Part ceiling_robot_attached_part_;

        // Sensor Callbacks
        bool kts1_camera_received_data = false;
        bool kts2_camera_received_data = false;
        bool left_bins_camera_received_data = false;
        bool right_bins_camera_received_data = false;

        bool kts1_rgb_camera_received_data = false;
        bool kts2_rgb_camera_received_data = false;
        bool left_bins_rgb_camera_received_data = false;
        bool right_bins_rgb_camera_received_data = false;

        void kts1_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void kts2_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void left_bins_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void right_bins_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);

        void kts1_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void kts2_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void left_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void right_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
        // Constants
        double kit_tray_thickness_ = 0.01;
        double drop_height_ = 0.002;
        double pick_offset_ = 0.003;
        double battery_grip_offset_ = -0.05;
};

/**
 * @brief Class to store Kitting Order
 * 
 */
class Kitting {
    public:
        /**
            * @brief Construct a new Kitting object
            * 
            * @param agv_number 
            * @param tray_id 
            * @param destination 
            * @param _parts_kit 
            */
        Kitting(unsigned int agv_number,
                        unsigned int tray_id,
                        unsigned int destination,
                        const std::vector<std::array<int, 3>> _parts_kit) : agv_id_(agv_number),
                                                                    tray_id_(tray_id),
                                                                    destination_(destination),
                                                                    parts_kit_(_parts_kit) {}

        /**
        * @brief Get the Agv Id object
        * 
        * @return unsigned int 
        */
        unsigned int GetAgvId() const { return agv_id_; }

        /**
        * @brief Get the Tray Id object
        * 
        * @return unsigned int 
        */
        unsigned int GetTrayId() const { return tray_id_; }

        /**
        * @brief Get the Destination object
        * 
        * @return unsigned int 
        */
        unsigned int GetDestination() const { return destination_; }

        /**
        * @brief Get the Parts object
        * 
        * @return const std::vector<std::array<int, 3>> 
        */
        const std::vector<std::array<int, 3>> GetParts() const { return parts_kit_; }

    private:
        unsigned int agv_id_;
        unsigned int tray_id_;
        unsigned int destination_;
        std::vector<std::array<int, 3>> parts_kit_;
};

/**
 * @brief Struct of type Part used in Assembly and Combined Order
 * 
 */
struct Part {
      int type;
      int color;
      geometry_msgs::msg::PoseStamped assembled_pose;
      geometry_msgs::msg::Vector3 install_direction;
};

/**
 * @brief Class to store Assembly Order
 * 
 */
class Assembly {
    public:
        /**
        * @brief Construct a new Assembly object
        * 
        * @param agv_numbers 
        * @param station 
        * @param parts_assm 
        */
        Assembly(std::vector<unsigned int> agv_numbers, unsigned int station, std::vector<Part> parts_assm) : agv_numbers_(agv_numbers),
                                                                                            station_(station),
                                                                                            parts_assm_(parts_assm) {}

        /**
        * @brief Get the Agv Numbers object
        * 
        * @return const std::vector<unsigned int> 
        */
        const std::vector<unsigned int> GetAgvNumbers() const { return agv_numbers_; }

        /**
        * @brief Get the Station object
        * 
        * @return unsigned int 
        */
        unsigned int GetStation() const { return station_; }

        /**
        * @brief Get the Parts object
        * 
        * @return const std::vector<Part> 
        */
        const std::vector<Part> GetParts() const { return parts_assm_; }

    private:
        std::vector<unsigned int> agv_numbers_;
        unsigned int station_;
        std::vector<Part> parts_assm_;
};

/**
 * @brief Class to store Combined Order
 * 
 */
class Combined {
    public:
        /**
        * @brief Construct a new Combined object
        * 
        * @param _station 
        * @param parts_comb 
        */
        Combined(unsigned int _station, std::vector<Part> parts_comb) : station_(_station),
                                                                parts_comb_(parts_comb) {}

        /**
        * @brief Get the Station object
        * 
        * @return unsigned int 
        */
        unsigned int GetStation() const { return station_; }

        /**
        * @brief Get the Parts object
        * 
        * @return const std::vector<Part> 
        */
        const std::vector<Part> GetParts() const { return parts_comb_; }

    private:
        unsigned int station_;
        std::vector<Part> parts_comb_;
};

class Orders {
    protected:
        std::string id_;
        unsigned int type_;
        bool priority_;
        std::shared_ptr<Kitting> kitting_ = nullptr;
        std::shared_ptr<Assembly> assembly_ = nullptr;
        std::shared_ptr<Combined> combined_ = nullptr;

    public:
        /**
        * @brief Construct a new Orders object
        * 
        * @param id 
        * @param type 
        * @param priority 
        */
        Orders(std::string id,
                unsigned int type,
                bool priority) : id_(id),
                                    type_(type),
                                    priority_(priority) {}
        ~Orders() = default;
        
        /**
        * @brief Get the Id object
        * 
        * @return std::string 
        */
        std::string GetId() const { return id_; }

        /**
        * @brief Get the Type object
        * 
        * @return unsigned int 
        */
        unsigned int GetType() const { return type_; }
        
        /**
        * @brief Get the Priority of the object
        * 
        * @return true 
        * @return false 
        */
        bool IsPriority() const { return priority_; }

        /**
        * @brief Get the Kitting object
        * 
        * @return std::shared_ptr<Kitting> 
        */
        std::shared_ptr<Kitting> GetKitting() const { return kitting_; }

        /**
        * @brief Set the Kitting object
        * 
        * @param _kitting 
        */
        virtual void SetKitting(std::shared_ptr<Kitting> _kitting) { kitting_ = _kitting; }

        /**
        * @brief Get the Assembly object
        * 
        * @return std::shared_ptr<Assembly> 
        */
        std::shared_ptr<Assembly> GetAssembly() const { return assembly_; }

        /**
        * @brief Set the Assembly object
        * 
        * @param _assembly 
        */
        virtual void SetAssembly(std::shared_ptr<Assembly> _assembly) { assembly_ = _assembly; }

        /**
        * @brief Get the Combined object
        * 
        * @return std::shared_ptr<Combined> 
        */
        std::shared_ptr<Combined> GetCombined() const { return combined_; }

        /**
        * @brief Set the Combined object
        * 
        * @param _combined 
        */
        virtual void SetCombined(std::shared_ptr<Combined> _combined) { combined_ = _combined; }
};
