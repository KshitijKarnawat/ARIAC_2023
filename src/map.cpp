#include <iostream>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(){

std::map<int, geometry_msgs::msg::Pose> m;

geometry_msgs::msg::Pose v1;
geometry_msgs::msg::Pose v2;
geometry_msgs::msg::Pose v3;
geometry_msgs::msg::Pose v4;
geometry_msgs::msg::Pose v5;
geometry_msgs::msg::Pose v6;

geometry_msgs::msg::Pose kts1_camera_pose;
geometry_msgs::msg::Pose kts2_camera_pose;


v1.position.x = 1.0650072764702059;
v1.position.y = 0.43000709753611055;
v1.position.z = 0.03999614099218002;
v1.orientation.x = 0.49999992883987115;
v1.orientation.y = -0.4999999411265107;
v1.orientation.z = -0.49999690775270145;
v1.orientation.w = 0.5000032222609632;

m[0] = v1; 

v2.position.x = 1.0650099968000726;
v2.position.y = 0.00007097561813884656;
v2.position.z = 0.040000002555061864;
v2.orientation.x = 0.49999992890919653;
v2.orientation.y = -0.4999999411838644;
v2.orientation.z = -0.4999969076833762;
v2.orientation.w = 0.5000032222036095;

m[1] = v2; 

v3.position.x = 1.0650127171073582;
v3.position.y = -0.42999290241251087;
v3.position.z = 0.040003864117645255;
v3.orientation.x = 0.4999999289866776;
v3.orientation.y = -0.49999994124796565;
v3.orientation.z=-0.49999690760589494;
v3.orientation.w = 0.5000032221395082;

m[2] = v3; 

kts1_camera_pose.position.x = -1.3;
kts1_camera_pose.position.y = -5.8;
kts1_camera_pose.position.z = 1.8;
kts1_camera_pose.orientation.x = -0.5000006633870012;
kts1_camera_pose.orientation.y = -0.49999933659210455;
kts1_camera_pose.orientation.z = 0.5000038267902595;
kts1_camera_pose.orientation.w = -0.499996173200466;

v4.position.x = 1.0650135895670279;
v4.position.y = 0.4299931162944382;
v4.position.x = 0.04000159355511179;
v4.orientation.x=0.4999989504743995;
v4.orientation.x=-0.5000003439391368;
v4.orientation.x=-0.5000028861253973;
v4.orientation.x=0.4999978194467621;

m[3] = v4; 

v5.position.x = 1.0650108688589284;
v5.position.y = -0.000006883694286974329;
v5.position.z = 0.04000001407445625;
v5.orientation.x =0.49999895054168514;
v5.orientation.y = -0.5000003439948044;
v5.orientation.z =-0.500002886058111;
v5.orientation.w =0.49999781939109533;

m[4] = v5; 

v6.position.x = 1.0650081481564746;
v6.position.y = -0.430006883683005;
v6.position.z = 0.0399984345938762;
v6.orientation.x = 0.49999895060693184;
v6.orientation.y = -0.5000003440487852;
v6.orientation.z = -0.5000028859928637;
v6.orientation.w = 0.4999978193371154;

m[5] = v6; 

kts2_camera_pose.position.x = -1.3;
kts2_camera_pose.position.y = 5.8;
kts2_camera_pose.position.z = 1.8;
kts2_camera_pose.orientation.x = -0.49999933659210455;
kts2_camera_pose.orientation.y = 0.5000006633870012;
kts2_camera_pose.orientation.z = 0.5000025000037572;
kts2_camera_pose.orientation.w = 0.49999750000375703;

}