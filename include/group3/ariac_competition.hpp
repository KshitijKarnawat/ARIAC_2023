#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/submit_order.hpp>


class AriacCompetition : public rclcpp::Node
{
 public:
    AriacCompetition(std::string node_name) : Node(node_name)
    {
        competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&AriacCompetition::competition_state_cb, this, std::placeholders::_1));
    }
    ~AriacCompetition();
 
 private:
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
    
    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);


};