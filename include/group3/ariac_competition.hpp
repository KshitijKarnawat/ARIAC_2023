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
#include <iterator>

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
#include <ariac_msgs/msg/break_beam_status.hpp>
#include <ariac_msgs/msg/quality_issue.hpp>


#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

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
#include <rclcpp/subscription_options.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// #include "group3/srv/floor_change_gripper.hpp"
// #include "group3/srv/floor_pick_tray.hpp"
// #include "group3/srv/floor_pick_part_bin.hpp"
// #include "group3/srv/floor_place_part.hpp"

#include "group3/msg/part.hpp"
#include "group3/msg/parts.hpp"

// #include "../include/group3/floor_robot.hpp"
#include "tray_id_detect.hpp"
#include "part_type_detect.hpp"
#include "map_poses.hpp"

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
        std::vector<int> available_agvs = {1, 2, 3, 4};

        struct BinQuadrant {
            int part_type_clr = -1;
            geometry_msgs::msg::Pose part_pose;
        };

        std::vector<int> conveyor_parts;
        std::map<int, BinQuadrant> bin_map;    // Holds part information in 72 possible bin locations (8 bins x 9 locations)


        AriacCompetition(std::string);
        

        void competition_state_cb(
            const ariac_msgs::msg::CompetitionState::ConstSharedPtr);

        void end_competition_timer_callback();

        void order_callback(const ariac_msgs::msg::Order::SharedPtr);

        // void bin_parts_callback(const ariac_msgs::msg::BinParts::SharedPtr);

        void conveyor_parts_callback(const ariac_msgs::msg::ConveyorParts::SharedPtr);

        void process_order();
        void submit_order(std::string order_id);
        void do_kitting(std::vector<Orders>);
        void do_assembly(std::vector<Orders>);
        void do_combined(std::vector<Orders>);
        int search_bin(int);
        int search_conveyor(int);
        void setup_map();

        std::string ConvertPartTypeToString(int);
        std::string ConvertPartColorToString(int);
        std::string ConvertDestinationToString(int, int);
        std::string ConvertAssemblyStationToString(int);

        void lock_agv(int);
        void unlock_agv(int);
        void move_agv(int, int);
        int determine_agv(int);

        void FloorRobotMoveHome();
        bool FloorRobotSetGripperState(bool enable);
        void FloorRobotChangeGripper(std::string gripper_type, std::string station);
        void FloorRobotPickandPlaceTray(int tray_idx, int agv_num);
        bool FloorRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad);
        bool FloorRobotPickConvPart(geometry_msgs::msg::Pose part_pose,geometry_msgs::msg::Pose camera_pose,int detection_time);
        bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);

        void CeilRobotMoveHome();
        bool CeilRobotSetGripperState(bool enable);
        void CeilRobotChangeGripper(std::string gripper_type, std::string station);
        bool CeilRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad);
        bool CeilRobotPlacePartOnKitTray(int agv_num, int quadrant);

        void populate_bin_part();
        int CheckFaultyPart(std::string order_id, int quadrant);
        void DisposePart();


    private:

        bool FloorRobotMovetoTarget();
        bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
        void FloorRobotWaitForAttach(double timeout);
        bool FloorRobotReachableWorkspace(int quadrant);

        bool CeilRobotMovetoTarget();
        bool CeilRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
        void CeilRobotWaitForAttach(double timeout);

        geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

        void LogPose(geometry_msgs::msg::Pose p);
        geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
        geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
        geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
        double GetYaw(geometry_msgs::msg::Pose pose);
        geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

        void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
        void AddModelsToPlanningScene();
        
        rclcpp::Node::SharedPtr floor_robot_node_;
        rclcpp::Node::SharedPtr ceil_robot_node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;

        // MoveIt Interfaces 
        moveit::planning_interface::MoveGroupInterfacePtr floor_robot_;
        moveit::planning_interface::MoveGroupInterfacePtr ceil_robot_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_;
        
        trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

        // TF
        std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
            competition_state_sub_;

        rclcpp::TimerBase::SharedPtr end_competition_timer_;

        ariac_msgs::msg::Order order_;

        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subscriber_;
        rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_parts_subscriber_;
        rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_parts_subscriber_;
        rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceil_gripper_state_sub_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kts1_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kts2_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_bins_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_bins_rgb_camera_sub_;

        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr kts1_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr kts2_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr conv_camera_sub_;

        rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_sub_;

        rclcpp::Subscription<group3::msg::Parts>::SharedPtr right_part_detector_sub_;
        rclcpp::Subscription<group3::msg::Parts>::SharedPtr left_part_detector_sub_;

        bool breakbeam_status;
        float breakbeam_time_sec;
        bool wait_flag = false;
        
        // Sensor Images
        cv::Mat kts1_rgb_camera_image_;
        cv::Mat kts2_rgb_camera_image_;
        cv::Mat left_bins_rgb_camera_image_;
        cv::Mat right_bins_rgb_camera_image_;

        // Sensor poses
        geometry_msgs::msg::Pose kts1_camera_pose_;
        geometry_msgs::msg::Pose kts2_camera_pose_;
        geometry_msgs::msg::Pose left_bins_camera_pose_;
        geometry_msgs::msg::Pose right_bins_camera_pose_;
        geometry_msgs::msg::Pose conv_camera_pose_;

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
        ariac_msgs::msg::VacuumGripperState ceil_gripper_state_;
        ariac_msgs::msg::Part ceil_robot_attached_part_;

        // Parts
        std::vector<group3::msg::Part> right_parts_;
        std::vector<group3::msg::Part> left_parts_;
        std::vector<ariac_msgs::msg::Part> dropped_parts_;
        std::vector<geometry_msgs::msg::Pose> conv_parts_;

        // ARIAC Services
        rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr ceil_robot_tool_changer_;
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceil_robot_gripper_enable_;

        // Sensor Callbacks
        bool kts1_camera_received_data = false;
        bool kts2_camera_received_data = false;
        bool left_bins_camera_received_data = false;
        bool right_bins_camera_received_data = false;
        bool conv_camera_received_data = false;
        bool breakbeam_received_data = false;

        bool kts1_rgb_camera_received_data = false;
        bool kts2_rgb_camera_received_data = false;
        bool left_bins_rgb_camera_received_data = false;
        bool right_bins_rgb_camera_received_data = false;

        bool right_part_detector_received_data = false;
        bool left_part_detector_received_data = false;

        void kts1_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void kts2_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void left_bins_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void right_bins_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);
        void conv_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);

        void breakbeam_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);

        void kts1_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void kts2_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void left_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void right_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
        void ceil_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
        
        void right_part_detector_cb(const group3::msg::Parts::ConstSharedPtr msg);
        void left_part_detector_cb(const group3::msg::Parts::ConstSharedPtr msg);

        // Constants
        double kit_tray_thickness_ = 0.011;
        double drop_height_ = 0.005;
        double pick_offset_ = 0.003;
        double battery_grip_offset_ = -0.05;

        std::map<int, geometry_msgs::msg::Pose> bin_quadrant_poses;
        std::map<int, geometry_msgs::msg::Pose> tray_poses;

        std::map<int, std::string> part_types_ = {
            {ariac_msgs::msg::Part::BATTERY, "battery"},
            {ariac_msgs::msg::Part::PUMP, "pump"},
            {ariac_msgs::msg::Part::REGULATOR, "regulator"},
            {ariac_msgs::msg::Part::SENSOR, "sensor"}};

        std::map<int, std::string> part_colors_ = {
            {ariac_msgs::msg::Part::RED, "red"},
            {ariac_msgs::msg::Part::BLUE, "blue"},
            {ariac_msgs::msg::Part::GREEN, "green"},
            {ariac_msgs::msg::Part::ORANGE, "orange"},
            {ariac_msgs::msg::Part::PURPLE, "purple"},
        };

        // Part heights
        std::map<int, double> part_heights_ = {
            {ariac_msgs::msg::Part::BATTERY, 0.04},
            {ariac_msgs::msg::Part::PUMP, 0.12},
            {ariac_msgs::msg::Part::REGULATOR, 0.07},
            {ariac_msgs::msg::Part::SENSOR, 0.07}};

        // Quadrant Offsets
        std::map<int, std::pair<double, double>> quad_offsets_ = {
            {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
        };

        std::map<std::string, double> rail_positions_ = {
            {"agv1", -4.5},
            {"agv2", -1.2},
            {"agv3", 1.2},
            {"agv4", 4.5},
            {"left_bins", 3},
            {"right_bins", -3}};

        std::map<std::string, double> gantry_positions_ = {
            {"agv1", -4.686},
            {"agv2", -1.078},
            {"agv3", 1.392},
            {"agv4", 4.961},
            {"left_bins", 2.8},
            {"right_bins", -2.8}};
            
            // gantry_x_axis_joint 2.6
            // gantry_y_axis_joint -2.8
            // gantry_rotation_joint -90 deg

        // Joint value targets for kitting stations
        std::map<std::string, double> floor_kts1_js_ = {
            {"linear_actuator_joint", 4.0},
            {"floor_shoulder_pan_joint", 1.57},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -1.57},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0}};

        // std::map<std::string, double> ceil_kts1_js_ = {
        //     {"gantry_x_axis_joint", 5.783},
        //     {"gantry_y_axis_joint", 6.255},
        //     {"gantry_rotation_joint", 0},
        //     {"ceiling_shoulder_pan_joint", 0.087},  // -5 deg
        //     {"ceiling_shoulder_lift_joint", -1.0821},  // -62 deg
        //     {"ceiling_elbow_joint", 1.221},  // 70 deg
        //     {"ceiling_wrist_1_joint", 0.0},
        //     {"ceiling_wrist_2_joint", 1.57},  // 90 deg
        //     {"ceiling_wrist_3_joint", 0.0}};

        std::map<std::string, double> ceil_kts1_js_ = {
            {"gantry_x_axis_joint", 4.332},
            {"gantry_y_axis_joint", 5.510},
            {"gantry_rotation_joint", -1.57},
            {"ceiling_shoulder_pan_joint", 0.0},  // -5 deg
            {"ceiling_shoulder_lift_joint", -1.57},  // -62 deg
            {"ceiling_elbow_joint", 1.57},  // 70 deg
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},  // 90 deg
            {"ceiling_wrist_3_joint", 0.0}};
        
        // std::map<std::string, double> ceil_kts2_js_ = {
        //     {"gantry_x_axis_joint", 5.783},
        //     {"gantry_y_axis_joint", -6.255},
        //     {"gantry_rotation_joint", 3.141},
        //     {"ceiling_shoulder_pan_joint", 0.087},  // -5 deg
        //     {"ceiling_shoulder_lift_joint", -1.0821},  // -62 deg
        //     {"ceiling_elbow_joint", 1.221},  // 70 deg
        //     {"ceiling_wrist_1_joint", 0.0},
        //     {"ceiling_wrist_2_joint", 1.57},  // 90 deg
        //     {"ceiling_wrist_3_joint", 0.0}};

        std::map<std::string, double> ceil_kts2_js_ = {
            {"gantry_x_axis_joint", 4.332},
            {"gantry_y_axis_joint", -5.235},
            {"gantry_rotation_joint", -1.57},
            {"ceiling_shoulder_pan_joint", 0.0},  // -5 deg
            {"ceiling_shoulder_lift_joint", -1.57},  // -62 deg
            {"ceiling_elbow_joint", 1.57},  // 70 deg
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},  // 90 deg
            {"ceiling_wrist_3_joint", 0.0}};

        std::map<std::string, double> conv_js_ = {
            {"linear_actuator_joint", 0},
            {"floor_shoulder_pan_joint", 3.14},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -1.57},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0},
            {"floor_gripper_joint",0}};

        std::map<std::string, double> floor_kts2_js_ = {
            {"linear_actuator_joint", -4.0},
            {"floor_shoulder_pan_joint", -1.57},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -1.57},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0}};

        std::map<std::string, double> ceil_conv_js_ = {
            {"gantry_x_axis_joint", 7.3},
            {"gantry_y_axis_joint", -2},
            {"gantry_rotation_joint", 1.57}, //270 deg
            {"ceiling_shoulder_pan_joint", 3.176},  // 0 deg
            {"ceiling_shoulder_lift_joint", -3.77},  // 310 deg
            {"ceiling_elbow_joint", 1.553},  // 55 deg
            {"ceiling_wrist_1_joint", -0.9}, // 170 deg
            {"ceiling_wrist_2_joint", 1.57},  // -90 deg
            {"ceiling_wrist_3_joint", 1.57}};  // 0 deg

        // ADD POSES OF ALL 72 positions in BIN
        // ALL POSES IN WORLD FRAME FROM GAZEBO, DON'T MULTPLY POSE
        // std::map<int, std::vector<float>> bin_quadrant_positions_ = {
        //     {1, std::vector<float>{0.1, -0.1, 0.05}},
        // };


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
