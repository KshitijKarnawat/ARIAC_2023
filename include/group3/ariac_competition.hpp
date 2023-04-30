/**
 * @copyright Copyright (c) 2023
 * @file ariac_competition.hpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of RWA2 for ARIAC 2023 (Group 3)
 * @version 0.3
 * @date 2023-04-30
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
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/assembly_station_state.hpp>
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

#include "group3/msg/part.hpp"
#include "group3/msg/parts.hpp"

#include "tray_id_detect.hpp"
#include "part_type_detect.hpp"
#include "map_poses.hpp"

class Orders;

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
 * @brief Class definition for ARIAC Competition
 * 
 */
class AriacCompetition : public rclcpp::Node {
    public:

        bool conveyor_parts_flag_{false};   // Flag to check if conveyor information is populated
        bool submit_orders_{false};    // Flag to track when to submit orders
        int competition_state_ = -1;  // Competition state
        bool competition_started_{false};   // Flag to check if competition is started
        int conveyor_size;  // Number of parts spawning on the conveyor 
        bool high_priority_order_{false}; // Flag to check if there is a high priority order
        bool doing_incomplete = false; // Flag to check if the robot is doing an incomplete order
        int kittingorder_count_ = 0; // Number of kitting Tasks in the order
        int kittingorder_count_incomplete_ = 0; // Number of kitting Tasks in the incomplete order


        std::vector<Orders> orders; // Vector of orders
        std::vector<Orders> incomplete_orders; // Vector of incomplete orders
        std::vector<Orders> current_order; // Vector of current order
        std::vector<Orders> submitted_orders; // Vector of submitted orders

        std::vector<int> tray_aruco_id;     // Available Trays
        std::vector<int> available_agvs = {1, 2, 3, 4}; // Available AGVs

        struct BinQuadrant {
            int part_type_clr = -1;
            geometry_msgs::msg::Pose part_pose;
        };

        std::vector<int> conveyor_parts;   // Vector of parts on the conveyor
        std::map<int, BinQuadrant> bin_map;    // Holds part information in 72 possible bin locations (8 bins x 9 locations)

        /**
        * @brief Construct a new Ariac Competition object
        * 
        * @param node_name Name of the node
        */
        AriacCompetition(std::string);
        

        ////////////////////////////////////////
        //         Competition Methods
        ////////////////////////////////////////
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
        * @brief  Callback function to retrieve conveyor part information
        * 
        * @param msg 
        */
        void conveyor_parts_callback(const ariac_msgs::msg::ConveyorParts::SharedPtr);

        /**
        * @brief Method to process the order
        * 
        */
        bool process_order();

        /**
        * @brief Method to submit the orders
        * 
        * @param order_id Order ID
        */
        void submit_order(std::string order_id);

        /**
        * @brief Method to do the kitting task
        * 
        */
        bool do_kitting(std::vector<Orders>);

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
         * @brief Method to populate the bin_map using RGB image information
         * 
         */
        void populate_bin_part();

        ////////////////////////////////////////
        //        Type Conversion Methods
        ////////////////////////////////////////
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

        ////////////////////////////////////////
        //           AGV Methods
        ////////////////////////////////////////
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
        * @param std::string AGV Destination
        *
        */
        void move_agv(int, int);

        /**
        * @brief Method to choose the AGV for Combined task
        * 
        * @param int Station number
        */
        int determine_agv(int);

        ////////////////////////////////////////
        //         Floor Robot Methods
        ////////////////////////////////////////

        /**
         * @brief Method to move the Floor Robot Home
         * 
         */
        void FloorRobotMoveHome();

        /**
         * @brief Method to move the Floor Robot near the conveyor belt
         * 
         */
        void FloorRobotMoveConveyorHome();

        /**
         * @brief Method to set the Floor Robot's gripper state
         * 
         * @param enable Gripper state
         * @return true 
         * @return false 
         */
        bool FloorRobotSetGripperState(bool enable);

        /**
         * @brief Method to change the Floor Robot's gripper
         * 
         * @param gripper_type Gripper Type (parts/tray)
         * @param station Station number
         */
        void FloorRobotChangeGripper(std::string gripper_type, std::string station);
        
        /**
         * @brief Method to make the Floor Robot pick and place the tray on the AGV.
         * 
         * @param tray_idx Tray number
         * @param agv_num AGV number
         */
        void FloorRobotPickandPlaceTray(int tray_idx, int agv_num);
        
        /**
         * @brief Method to make the Floor Robot pick the part from the bin
         * 
         * @param part_clr Color of the part
         * @param part_type Type of the part
         * @param part_pose Desired pose of the part
         * @param part_quad Quadrant in bin (1-72) 
         * @return true 
         * @return false 
         */
        bool FloorRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad);
        
        /**
         * @brief Method to make the Floor Robot pick part from the Conveyor.
         * 
         * @param part_pose Pose of the part
         * @param part_rgb Part to pick
         * @return true 
         * @return false 
         */
        bool FloorRobotPickConvPart(std::vector<geometry_msgs::msg::Pose> part_pose,group3::msg::Part part_rgb);
        
        /**
         * @brief Method to make the Floor Robot place part on the Kit tray
         * 
         * @param agv_num AGV number
         * @param quadrant Tray quadrant
         * @return true 
         * @return false 
         */
        bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);

        ////////////////////////////////////////
        //       Ceiling Robot Methods
        ////////////////////////////////////////

        /**
         * @brief Method to move the Ceiling Robot Home
         * 
         */
        void CeilRobotMoveHome();

        /**
         * @brief Method to set the Ceiling Robot's gripper state
         * 
         * @param enable Gripper state
         * @return true 
         * @return false 
         */
        bool CeilRobotSetGripperState(bool enable);

        /**
         * @brief Method to change the Ceiling Robot's gripper
         * 
         * @param gripper_type Gripper Type (parts/tray)
         * @param station Station number
         */
        void CeilRobotChangeGripper(std::string gripper_type, std::string station);
        
        /**
         * @brief Method to make the Ceiling Robot pick the part from the bin
         * 
         * @param part_clr Color of the part
         * @param part_type Type of the part
         * @param part_pose Desired pose of the part
         * @param part_quad Quadrant in bin (1-72) 
         * @return true 
         * @return false 
         */
        bool CeilRobotPickBinPart(int part_clr,int part_type,geometry_msgs::msg::Pose part_pose,int part_quad);
        
        /**
         * @brief Method to make the Ceiling Robot place part on the Kit tray
         * 
         * @param agv_num AGV number
         * @param quadrant Tray quadrant
         * @return true 
         * @return false 
         */
        bool CeilRobotPlacePartOnKitTray(int agv_num, int quadrant);

        /**
         * @brief Method to make the Ceiling Robot fine-tune the assembly of the part.
         * 
         * @param station Assembly station number 
         * @param part Part to be assembled
         * @return true 
         * @return false 
         */
        bool CeilRobotWaitForAssemble(int station, Part part);
        
        /**
         * @brief Method to make the Ceiling Robot move to the desired Assembly station
         * 
         * @param station Assembly Station number
         * @return true 
         * @return false 
         */
        bool CeilRobotMoveToAssemblyStation(int station);
        
        /**
         * @brief Method to make the Ceiling Robot pick from the AGV
         * 
         * @param part Part to be picked (type and pose) 
         * @return true 
         * @return false 
         */
        bool CeilRobotPickAGVPart(ariac_msgs::msg::PartPose part);
        
        /**
         * @brief Method to make the Ceiling Robot place part in the assembly station.
         * 
         * @param station Assmebly Station number 
         * @param part Part to be assembled
         * @return true 
         * @return false 
         */
        bool CeilRobotAssemblePart(int station, Part part);

        ////////////////////////////////////////
        //           Challenges Methods
        ////////////////////////////////////////

        /**
         * @brief Method to detect if the part on the tray is faulty.
         * 
         * @param order_id Order ID
         * @return std::vector<bool> 
         * @attention The perform_quality_check service has some bugs and needs to be called twice for it to work. 
         */
        std::vector<bool> CheckFaultyPart(std::string order_id);
        
        /**
         * @brief Method to flip a part using the Floor and Ceiling Robots.
         * 
         * @param part_clr Part color
         * @param part_type Part type
         * @param agv_num AGV number
         * @param part_quad Bin quadrant of the part (1-72)
         */
        void FlipPart(int part_clr, int part_type, int agv_num, int part_quad);

    private:

        ////////////////////////////////////////
        //     Floor Robot MoveIt Methods
        ////////////////////////////////////////
        /**
         * @brief Method to generate and execute the plan for the Floor Robot
         * 
         * @return true 
         * @return false 
         */
        bool FloorRobotMovetoTarget();

        /**
         * @brief Method to generate a Time optimal trajectory for the Floor Robot
         * 
         * @param waypoints Waypoints of the robot
         * @param vsf Velocity Scaling Factor
         * @param asf Acceleration Scaling Factor
         * @return true 
         * @return false 
         */
        bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
        
        /**
         * @brief Method to make fine-adjustments to the Floor Robot while attaching to a part
         * 
         * @param timeout Time to perform adjustments
         */
        void FloorRobotWaitForAttach(double timeout);

        /**
         * @brief Method to determine if the quadrant is within the Floor Robot's reachable workspace
         * 
         * @param quadrant 
         * @return true 
         * @return false 
         */
        bool FloorRobotReachableWorkspace(int quadrant);

        ////////////////////////////////////////
        //     Ceiling Robot MoveIt Methods
        ////////////////////////////////////////

        /**
         * @brief Method to generate and execute the plan for the Ceiling Robot
         * 
         * @return true 
         * @return false 
         */
        bool CeilRobotMovetoTarget();

        /**
         * @brief Method to generate a Time optimal trajectory for the Ceiling Robot
         * 
         * @param waypoints Waypoints of the robot
         * @param vsf Velocity Scaling Factor
         * @param asf Acceleration Scaling Factor
         * @return true 
         * @return false 
         */
        bool CeilRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
        
        /**
         * @brief Method to make fine-adjustments to the Ceiling Robot while attaching to a part
         * 
         * @param timeout Time to perform adjustments
         */
        void CeilRobotWaitForAttach(double timeout);

        ////////////////////////////////////////
        //     Kinematics based Methods
        ////////////////////////////////////////

        /**
         * @brief Method to set the robot's yaw orientation
         * 
         * @param rotation Yaw angle
         * @return geometry_msgs::msg::Quaternion 
         */
        geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);
        
        /**
         * @brief Method to print the pose 
         * 
         * @param p Pose
         */
        void LogPose(geometry_msgs::msg::Pose p);

        /**
         * @brief Method to perform coordinate frame multiplication
         * 
         * @param p1 Pose #1
         * @param p2 Pose #2
         * @return geometry_msgs::msg::Pose 
         */
        geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
        
        /**
         * @brief Method to build a geometry_msg/Pose object given individual pose information
         * 
         * @param x X coordinate of pose
         * @param y Y coordinate of pose
         * @param z Z coordinate of pose
         * @param orientation Quaternion of pose 
         * @return geometry_msgs::msg::Pose 
         */
        geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
        
        /**
         * @brief  Method to retrieve the tf2 pose information of the frame
         * 
         * @param frame_id tf2 Frame ID
         * @return geometry_msgs::msg::Pose 
         */
        geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
        
        /**
         * @brief  Method to extract the yaw angle from a pose.
         * 
         * @param pose Pose
         * @return double 
         */
        double GetYaw(geometry_msgs::msg::Pose pose);
        
        /**
         * @brief Method to convert RPY angles to Quaternion
         * 
         * @param r Roll angle 
         * @param p Pitch angle
         * @param y Yaw angle
         * @return geometry_msgs::msg::Quaternion 
         */
        geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

        /**
         * @brief Method to add the model STL into RViz Planning Scene
         * 
         * @param name Model name
         * @param mesh_file Path to STL file of model
         * @param model_pose Model pose
         */
        void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
        
        /**
         * @brief Method to add competition models to RViz Planning Scene
         * 
         */
        void AddModelsToPlanningScene();
        
        rclcpp::Node::SharedPtr floor_robot_node_;  // Floor Robot's Node object pointer
        rclcpp::Node::SharedPtr ceil_robot_node_;   // Ceiling Robot's Node object pointer
        rclcpp::Executor::SharedPtr executor_;      // Executor object for Floor & Ceiling Robot nodes
        std::thread executor_thread_;               // Thread for executor_ object 

        std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

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

        // RGB Camera subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kts1_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kts2_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_bins_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_bins_rgb_camera_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr conv_rgb_camera_sub_;

        // Conveyor Basic logical camera subscription
        rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr conv_camera_sub_;

        // Breakbeam subscriptions
        rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam1_sub_;
        rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam2_sub_;

        // OpenCV detection subscriptions
        rclcpp::Subscription<group3::msg::Parts>::SharedPtr right_part_detector_sub_;
        rclcpp::Subscription<group3::msg::Parts>::SharedPtr left_part_detector_sub_;
        rclcpp::Subscription<group3::msg::Part>::SharedPtr conv_part_detector_sub_;

        // Assembly State subscriptions
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

        // Break Beam variables
        bool breakbeam_status;
        bool breakbeam_trigger;
        bool breakbeam1_status;
        bool breakbeam2_status;
        float breakbeam_time_sec;
        bool wait_flag = false;
        
        // Sensor Images
        cv::Mat kts1_rgb_camera_image_;
        cv::Mat kts2_rgb_camera_image_;
        cv::Mat left_bins_rgb_camera_image_;
        cv::Mat right_bins_rgb_camera_image_;
        cv::Mat conv_rgb_camera_image_;

        // Sensor poses
        geometry_msgs::msg::Pose conv_camera_pose_;

        // Bins
        std::vector<geometry_msgs::msg::Pose> left_bins_parts_;
        std::vector<geometry_msgs::msg::Pose> right_bins_parts_;

        //Quadrants
        std::vector<int> occupied_quadrants;

        // Callback Groups
        rclcpp::CallbackGroup::SharedPtr order_cb_group_;
        rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
        rclcpp::CallbackGroup::SharedPtr topic_cb_group2_;

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
        group3::msg::Part pump_rgb;

        group3::msg::Part conv_rgb_parts_;

        // ARIAC Services
        rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr ceil_robot_tool_changer_;
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceil_robot_gripper_enable_;

        // Sensor Flags
        bool conv_camera_received_data = false;
        bool breakbeam_received_data = false;
        bool breakbeam1_received_data = false;
        bool breakbeam2_received_data = false;

        bool kts1_rgb_camera_received_data = false;
        bool kts2_rgb_camera_received_data = false;
        bool left_bins_rgb_camera_received_data = false;
        bool right_bins_rgb_camera_received_data = false;

        bool right_part_detector_received_data = false;
        bool left_part_detector_received_data = false;
        bool conv_part_detector_received_data = false; 

        // Sensor Callbacks
        void conv_camera_cb(const ariac_msgs::msg::BasicLogicalCameraImage::ConstSharedPtr msg);

        void breakbeam_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
        void breakbeam1_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
        void breakbeam2_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);

        void kts1_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void kts2_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void left_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void right_bins_rgb_camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
        void ceil_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
        
        void right_part_detector_cb(const group3::msg::Parts::ConstSharedPtr msg);
        void left_part_detector_cb(const group3::msg::Parts::ConstSharedPtr msg);
        void conv_part_detector_cb(const group3::msg::Part::ConstSharedPtr msg);

        void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

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
            
        // Joint value targets for kitting stations
        std::map<std::string, double> floor_kts1_js_ = {
            {"linear_actuator_joint", 4.0},
            {"floor_shoulder_pan_joint", 1.57},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -1.57},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0}};
        
        std::map<std::string, double> floor_kts2_js_ = {
            {"linear_actuator_joint", -4.0},
            {"floor_shoulder_pan_joint", -1.57},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -1.57},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0}};

        std::map<std::string, double> ceil_kts1_js_ = {
            {"gantry_x_axis_joint", 4.332},
            {"gantry_y_axis_joint", 5.510},
            {"gantry_rotation_joint", -1.57},
            {"ceiling_shoulder_pan_joint", 0.0},
            {"ceiling_shoulder_lift_joint", -1.57},
            {"ceiling_elbow_joint", 1.57}, 
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},
            {"ceiling_wrist_3_joint", 0.0}};

        std::map<std::string, double> ceil_kts2_js_ = {
            {"gantry_x_axis_joint", 4.332},
            {"gantry_y_axis_joint", -5.235},
            {"gantry_rotation_joint", -1.57},
            {"ceiling_shoulder_pan_joint", 0.0},
            {"ceiling_shoulder_lift_joint", -1.57}, 
            {"ceiling_elbow_joint", 1.57},
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},
            {"ceiling_wrist_3_joint", 0.0}};

        // Joint value targets for Conveyor
        std::map<std::string, double> conv_js_ = {
            {"linear_actuator_joint", 0},
            {"floor_shoulder_pan_joint", 3.14},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -1.57},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0},
            {"floor_gripper_joint",0}};

        std::map<std::string, double> floor_conv_home_js_ = {
            {"linear_actuator_joint", -2.75},
            {"floor_shoulder_pan_joint", 3.14},
            {"floor_shoulder_lift_joint", -0.9162979},
            {"floor_elbow_joint", 2.04204},
            {"floor_wrist_1_joint", -2.67035},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0}
        };

        std::map<std::string, double> ceil_conv_js_ = {
            {"gantry_x_axis_joint", 7.3},
            {"gantry_y_axis_joint", -2},
            {"gantry_rotation_joint", 1.57},
            {"ceiling_shoulder_pan_joint", 3.176},
            {"ceiling_shoulder_lift_joint", -3.77},
            {"ceiling_elbow_joint", 1.553},
            {"ceiling_wrist_1_joint", -0.9},
            {"ceiling_wrist_2_joint", 1.57},
            {"ceiling_wrist_3_joint", 1.57}};

        // Joint value targets for Disposal Bins
        std::map<int, std::map<std::string, double>> floor_disposal_poses_ = {
            {4 , std::map<std::string, double>{
                {"linear_actuator_joint", 4.8},
                {"floor_shoulder_pan_joint", 3.07},
                {"floor_shoulder_lift_joint", -1.57},
                {"floor_elbow_joint", 1.57},
                {"floor_wrist_1_joint", -1.57},
                {"floor_wrist_2_joint", -1.57},
                {"floor_wrist_3_joint", 0.0},
                {"floor_gripper_joint",0}}},
            {2 , std::map<std::string, double>{
                {"linear_actuator_joint", -0.132},
                {"floor_shoulder_pan_joint", 0},
                {"floor_shoulder_lift_joint", -1.57},
                {"floor_elbow_joint", 1.57},
                {"floor_wrist_1_joint", -1.57},
                {"floor_wrist_2_joint", -1.57},
                {"floor_wrist_3_joint", 0.0},
                {"floor_gripper_joint",0}}},
            {3 , std::map<std::string, double>{
                {"linear_actuator_joint", -0.132},
                {"floor_shoulder_pan_joint", 0},
                {"floor_shoulder_lift_joint", -1.57},
                {"floor_elbow_joint", 1.57},
                {"floor_wrist_1_joint", -1.57},
                {"floor_wrist_2_joint", -1.57},
                {"floor_wrist_3_joint", 0.0},
                {"floor_gripper_joint",0}}},
            {1 , std::map<std::string, double>{
                {"linear_actuator_joint", -4.8},
                {"floor_shoulder_pan_joint", 2.97},
                {"floor_shoulder_lift_joint", -1.57},
                {"floor_elbow_joint", 1.57},
                {"floor_wrist_1_joint", -1.57},
                {"floor_wrist_2_joint", -1.57},
                {"floor_wrist_3_joint", 0.0},
                {"floor_gripper_joint",0}}}
        };

        std::map<int, std::map<std::string, double>> ceil_disposal_poses_ = {
            {4 , std::map<std::string, double>{
                {"gantry_x_axis_joint", 5.392},
                {"gantry_y_axis_joint", 4.455},
                {"gantry_rotation_joint", -1.52},
                {"ceiling_shoulder_pan_joint", -0.45},
                {"ceiling_shoulder_lift_joint", 0.24},
                {"ceiling_elbow_joint", -0.86},
                {"ceiling_wrist_1_joint", 3.77},
                {"ceiling_wrist_2_joint", -1.1},
                {"ceiling_wrist_3_joint", 1.57}}},
            {2, std::map<std::string, double>{
                {"gantry_x_axis_joint", 3.903},
                {"gantry_y_axis_joint", -0.687},
                {"gantry_rotation_joint", 4.43},
                {"ceiling_shoulder_pan_joint", -3.29},
                {"ceiling_shoulder_lift_joint", -2.71},
                {"ceiling_elbow_joint", -0.44},
                {"ceiling_wrist_1_joint", 6.28},
                {"ceiling_wrist_2_joint", -4.56},
                {"ceiling_wrist_3_joint", 0.35}}},
            {3, std::map<std::string, double>{
                {"gantry_x_axis_joint", 3.903},
                {"gantry_y_axis_joint", -0.687},
                {"gantry_rotation_joint", 4.43},
                {"ceiling_shoulder_pan_joint", -3.29},
                {"ceiling_shoulder_lift_joint", -2.71},
                {"ceiling_elbow_joint", -0.44},
                {"ceiling_wrist_1_joint", 6.28},
                {"ceiling_wrist_2_joint", -4.56},
                {"ceiling_wrist_3_joint", 0.35}}},
            {1, std::map<std::string, double>{
                {"gantry_x_axis_joint", 5.396},
                {"gantry_y_axis_joint", -4.982},
                {"gantry_rotation_joint", -1.57},
                {"ceiling_shoulder_pan_joint", -0.44},
                {"ceiling_shoulder_lift_joint", 0.24},
                {"ceiling_elbow_joint", -0.87},
                {"ceiling_wrist_1_joint", 3.79},
                {"ceiling_wrist_2_joint", -1.12},
                {"ceiling_wrist_3_joint", 1.68}}}
        };

        // Joint value targets for Assembly stations
        std::map<std::string, double> ceiling_as1_js_ = {
            {"gantry_x_axis_joint", 1},
            {"gantry_y_axis_joint", -3},
            {"gantry_rotation_joint", 1.571},
            {"ceiling_shoulder_pan_joint", 0},
            {"ceiling_shoulder_lift_joint", -2.37},
            {"ceiling_elbow_joint", 2.37},
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},
            {"ceiling_wrist_3_joint", 0}
        };

        std::map<std::string, double> ceiling_as2_js_ = {
            {"gantry_x_axis_joint", -4},
            {"gantry_y_axis_joint", -3},
            {"gantry_rotation_joint", 1.571},
            {"ceiling_shoulder_pan_joint", 0},
            {"ceiling_shoulder_lift_joint", -2.37},
            {"ceiling_elbow_joint", 2.37},
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},
            {"ceiling_wrist_3_joint", 0}
        };

        std::map<std::string, double> ceiling_as3_js_ = {
            {"gantry_x_axis_joint", 1},
            {"gantry_y_axis_joint", 3},
            {"gantry_rotation_joint", 1.571},
            {"ceiling_shoulder_pan_joint", 0},
            {"ceiling_shoulder_lift_joint", -2.37},
            {"ceiling_elbow_joint", 2.37},
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},
            {"ceiling_wrist_3_joint", 0}
        };

        std::map<std::string, double> ceiling_as4_js_ = {
            {"gantry_x_axis_joint", -4},
            {"gantry_y_axis_joint", 3},
            {"gantry_rotation_joint", 1.571},
            {"ceiling_shoulder_pan_joint", 0},
            {"ceiling_shoulder_lift_joint", -2.37},
            {"ceiling_elbow_joint", 2.37},
            {"ceiling_wrist_1_joint", 3.14},
            {"ceiling_wrist_2_joint", -1.57},
            {"ceiling_wrist_3_joint", 0}
        };

        // Joint value targets for flipping parts
        std::map<std::string, double> ceil_flip_part_js_ = {
            {"gantry_x_axis_joint", 2.971},
            {"gantry_y_axis_joint", 0.359},
            {"gantry_rotation_joint", -1.57}, // -89 deg
            {"ceiling_shoulder_pan_joint", -0.069},  // 4 deg
            {"ceiling_shoulder_lift_joint", -0.872},  // -50 deg
            {"ceiling_elbow_joint", -0.994},  // -57 deg
            {"ceiling_wrist_1_joint", 3.42}, // 196 deg
            {"ceiling_wrist_2_joint", -1.57},  // -90 deg
            {"ceiling_wrist_3_joint", -0.0523}};  // -3 deg

        std::map<std::string, double> ceil_flip_part2_js_ = {
            {"gantry_x_axis_joint", 3.77001},
            {"gantry_y_axis_joint", -0.416054},
            {"gantry_rotation_joint", -1.81436}, // -89 deg
            {"ceiling_shoulder_pan_joint", -0.688833},  // 4 deg
            {"ceiling_shoulder_lift_joint", -0.34172},  // -50 deg
            {"ceiling_elbow_joint", -1.11438},  // -57 deg
            {"ceiling_wrist_1_joint", 1.45713}, // 196 deg
            {"ceiling_wrist_2_joint", -2.25962},  // -90 deg
            {"ceiling_wrist_3_joint", -1.32557}};  // -3 deg

        std::map<std::string, double> floor_flip_part_js_ = {
            {"linear_actuator_joint", 0.0},
            {"floor_shoulder_pan_joint", 0},
            {"floor_shoulder_lift_joint", -1.57},
            {"floor_elbow_joint", 1.57},
            {"floor_wrist_1_joint", -3.14},
            {"floor_wrist_2_joint", -1.57},
            {"floor_wrist_3_joint", 0.0}};

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
