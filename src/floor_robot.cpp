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

#include "floor_robot.hpp"

FloorRobot::FloorRobot()
    : Node("floor_robot_node"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      planning_scene_()
{
    // Use upper joint velocity and acceleration limits
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);

    // Subscribe to topics
    rclcpp::SubscriptionOptions options;

    topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = topic_cb_group_;

    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&FloorRobot::floor_gripper_state_cb, this, std::placeholders::_1), options);

    
    // Initialize service clients
    quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
    // floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    // floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");

    // Register services
    floor_robot_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
        "/competitor/move_floor_robot_home", 
        std::bind(
        &FloorRobot::FloorRobotMoveHome, this,
        std::placeholders::_1, std::placeholders::_2));

    floor_robot_change_gripper_srv_ = create_service<group3::srv::FloorChangeGripper>(
        "/competitor/floor_robot_change_gripper", 
        std::bind(
        &FloorRobot::FloorRobotChangeGripper, this,
        std::placeholders::_1, std::placeholders::_2));

    floor_pick_place_tray_srv_ = create_service<group3::srv::FloorPickTray>(
        "/competitor/floor_pick_place_tray", 
        std::bind(
        &FloorRobot::FloorRobotPickandPlaceTray, this,
        std::placeholders::_1, std::placeholders::_2));

    floor_pick_part_bin_srv_ = create_service<group3::srv::FloorPickPartBin>(
        "/competitor/floor_pick_part_bin",
        std::bind(
        &FloorRobot::FloorRobotPickBinPart, this,
        std::placeholders::_1, std::placeholders::_2));
    
    floor_pick_part_conv_srv_ = create_service<group3::srv::FloorPickPartConv>(
        "/competitor/floor_pick_part_conv",
        std::bind(
        &FloorRobot::FloorRobotPickConvPart, this,
        std::placeholders::_1, std::placeholders::_2));

    floor_place_part_srv_ = create_service<group3::srv::FloorPlacePart>(
        "/competitor/floor_place_part",
        std::bind(
        &FloorRobot::FloorRobotPlacePartOnKitTray, this,
        std::placeholders::_1, std::placeholders::_2));

    AddModelsToPlanningScene();

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

FloorRobot::~FloorRobot() 
{
  floor_robot_.~MoveGroupInterface();
}

void FloorRobot::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}

geometry_msgs::msg::Pose FloorRobot::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void FloorRobot::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose FloorRobot::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose FloorRobot::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double FloorRobot::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion FloorRobot::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

void FloorRobot::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

void FloorRobot::AddModelsToPlanningScene()
{
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
        {"as1_insert", std::pair<double, double>(-7.7, 3)},
        {"as2_insert", std::pair<double, double>(-12.7, 3)},
        {"as3_insert", std::pair<double, double>(-7.7, -3)},
        {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert : assembly_insert_positions)
    {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

geometry_msgs::msg::Quaternion FloorRobot::SetRobotOrientation(double rotation){
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

bool FloorRobot::FloorRobotMovetoTarget(){
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(floor_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool FloorRobot::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf){
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }
    // Retime trajectory
    // RCLCPP_INFO(this->get_logger(), "AC Node Time is %ld", this->now().nanoseconds());
    // RCLCPP_INFO(this->get_logger(), "AC Clock Time is %ld", rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds());
    // robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
    // rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
    // totg_.computeTimeStamps(rt, vsf, asf);
    // rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_.execute(trajectory));
}

void FloorRobot::FloorRobotWaitForAttach(double timeout, std::vector<geometry_msgs::msg::Pose> waypoints){
    // Wait for part to be attached
    rclcpp::Time start = now();
    geometry_msgs::msg::Pose starting_pose = waypoints.back();

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        
        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

void FloorRobot::FloorRobotMoveHome(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res){
  (void)req; // remove unused parameter warning
  (void)res; // remove unused parameter warning
  floor_robot_.setNamedTarget("home");
  FloorRobotMovetoTarget();
}

bool FloorRobot::FloorRobotSetGripperState(bool enable){
    if (floor_gripper_state_.enabled == enable)
    {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    std::string srv_name = "/ariac/floor_robot_enable_gripper";
    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("client_floor_robot_enable_gripper");
    
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
        node->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name);

    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                        "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO_STREAM(this->get_logger(),
                            "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    // floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
    // Call enable service
    // auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    // request->enable = enable;

    // auto result = floor_robot_gripper_enable_->async_send_request(request);
    // result.wait();

    // if (!result.get()->success)
    // {
    //     RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    //     return false;
    // }

    return true;
}

void FloorRobot::FloorRobotChangeGripper(
    group3::srv::FloorChangeGripper::Request::SharedPtr req,
    group3::srv::FloorChangeGripper::Response::SharedPtr res){
    std::string station = req->station;
    std::string gripper_type = req->gripper_type;

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
        floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
        floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z, SetRobotOrientation(0.0)));

    // RCLCPP_INFO_STREAM(this->get_logger(),"Waypoint 0 x: " << waypoints[0].position.x << " y: "<<waypoints[0].position.y << " z: "<< waypoints[0].position.z);
    // RCLCPP_INFO_STREAM(this->get_logger(),"Waypoint 1 x: " << waypoints[1].position.x << " y: "<< waypoints[1].position.y << " z: "<<waypoints[1].position.z);

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        res->success = false;


    std::string srv_name = "/ariac/floor_robot_change_gripper";
    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("client_floor_robot_change_gripper");
    
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr client =
        node->create_client<ariac_msgs::srv::ChangeGripper>(srv_name);

    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
    if (gripper_type == "trays")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    }
    else if (gripper_type == "parts")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    }

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                        "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO_STREAM(this->get_logger(),
                            "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to change gripper");
        res->success = false;
    }

    // Call service to change gripper
    // auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

    // if (gripper_type == "trays")
    // {
    //     request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    //     RCLCPP_INFO(get_logger(), "Changing to tray gripper");
    // }
    // else if (gripper_type == "parts")
    // {
    //     request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    //     RCLCPP_INFO(get_logger(), "Changing to part gripper");
    // }

    // auto result = floor_robot_tool_changer_->async_send_request(request);
    // result.wait();
    // if (result.get()->success){
    //     RCLCPP_INFO_STREAM(this->get_logger(), "Successfully changed gripper");
    // } else {
    //     RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    //     res->success = false;
    // }

    waypoints.clear();
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        res->success = false;

    res->success = true;
}

void FloorRobot::FloorRobotPickandPlaceTray(
    group3::srv::FloorPickTray::Request::SharedPtr req,
    group3::srv::FloorPickTray::Response::SharedPtr res){

    uint tray_id = req->tray_id;
    uint agv_num = req->agv_num;
    // std::vector<float> tray_camera_pose = tray_positions_[req->tray_pose];
    geometry_msgs::msg::Pose tray_camera_pose = req->tray_pose;
    // auto tray_pose = BuildPose(tray_camera_pose[0], tray_camera_pose[1], tray_camera_pose[2], SetRobotOrientation(3.14));
    geometry_msgs::msg::Pose camera_pose_ = req->camera_pose;
    std::string station = req->station;
    geometry_msgs::msg::Pose tray_pose;

    // tray_pose = tray_camera_pose;
    tray_pose = MultiplyPose(camera_pose_, tray_camera_pose);

    double tray_rotation = GetYaw(tray_pose); //3.14;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(3.0,waypoints);

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

    FloorRobotMovetoTarget();

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    FloorRobotSetGripperState(false);

    floor_robot_.detachObject(tray_name);

    // publish to robot state
    // LockAGVTray(agv_num);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    res->success = true;
}

void FloorRobot::FloorRobotPickBinPart(
    group3::srv::FloorPickPartBin::Request::SharedPtr req,
    group3::srv::FloorPickPartBin::Response::SharedPtr res)
{
    uint part_clr = req->part_clr;
    uint part_type = req->part_type;

    std::string bin_side = req->bin_side;

    geometry_msgs::msg::Pose camera_pose_ = req->camera_pose;
    geometry_msgs::msg::Pose part_camera_pose = req->part_pose;

    std::string station;
    
    // RCLCPP_INFO_STREAM(this->get_logger(), "Camera Pose x: " << camera_pose_.position.x << " y: " << camera_pose_.position.y << " z: " << camera_pose_.position.z);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Part Pose x: " << part_camera_pose.position.x << " y: " << part_camera_pose.position.y << " z: " << part_camera_pose.position.z);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Bin side: " << bin_side);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Attempting to pick a " << part_colors_[part_clr] << " " << part_types_[part_type]);

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;

    part_pose = MultiplyPose(camera_pose_, part_camera_pose);
    // RCLCPP_INFO_STREAM(this->get_logger(), "MPart pose x: " << part_pose.position.x << " y: " << part_pose.position.y << " z: " << part_pose.position.z);
    double part_rotation = GetYaw(part_pose);

    // Change gripper at location closest to part
    if (floor_gripper_state_.type != "part_gripper")
    {
        std::string station;
        if (part_pose.position.y < 0)
        {
            station = "kts1";
        }
        else
        {
            station = "kts2";
        }

        // Move floor robot to the corresponding kit tray table
        if (station == "kts1")
        {
            floor_robot_.setJointValueTarget(floor_kts1_js_);
        }
        else
        {
            floor_robot_.setJointValueTarget(floor_kts2_js_);
        }
        FloorRobotMovetoTarget();
    }

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0,waypoints);

    // Add part to planning scene
    std::string part_name = part_colors_[part_clr] + "_" + part_types_[part_type];
    AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    // floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    res->success = true;
}

void FloorRobot::FloorRobotPickConvPart(
    group3::srv::FloorPickPartConv::Request::SharedPtr req,
    group3::srv::FloorPickPartConv::Response::SharedPtr res)
{
    uint part_clr = req->part_clr;
    uint part_type = req->part_type;
    std::string bin_side = req->bin_side;
    geometry_msgs::msg::Pose camera_pose_ = req->camera_pose;
    geometry_msgs::msg::Pose part_camera_pose = req->part_pose;

    std::string station;
    geometry_msgs::msg::Pose part_pose;
    part_pose = MultiplyPose(camera_pose_, part_camera_pose);

    double part_rotation = GetYaw(part_pose);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0,waypoints);

    // Add part to planning scene
    std::string part_name = part_colors_[part_clr] + "_" + part_types_[part_type];
    AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    // floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    res->success = true;
}

void FloorRobot::FloorRobotPlacePartOnKitTray(
    group3::srv::FloorPlacePart::Request::SharedPtr req,
    group3::srv::FloorPlacePart::Response::SharedPtr res)
{
    int agv_num = req->agv_num;
    int quadrant = req->quadrant;

    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(this->get_logger(), "No part attached");
        res->success = false;
    }

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Drop part in quadrant
    FloorRobotSetGripperState(false);

    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type];
    floor_robot_.detachObject(part_name);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    res->success = true;
}

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   auto robot_commander = std::make_shared<FloorRobot>();

//   rclcpp::spin(robot_commander);

//   rclcpp::shutdown();
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto floor_robot = std::make_shared<FloorRobot>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(floor_robot);
//     executor.spin();

//     // floor_robot->FloorRobotSendHome();
//     // floor_robot->CompleteOrders();

//     rclcpp::shutdown();
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto floor_robot = std::make_shared<FloorRobot>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(floor_robot);
    
//     std::thread thread([&executor]() { executor.spin(); });
//     rclcpp::shutdown();
// }