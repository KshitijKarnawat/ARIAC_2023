#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

using namespace std::chrono_literals;
class AriacCompetition : public rclcpp::Node
{
 public:
    unsigned int competition_state_;
    AriacCompetition(std::string node_name) : Node(node_name)
    {
        competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&AriacCompetition::competition_state_cb, this, std::placeholders::_1));

        // this->declare_parameter("publishing_interval", 2);
        // rclcpp::Parameter pub_frequency = this->get_parameter("publishing_interval");

        end_competition_timer_ = this->create_wall_timer(100ms, std::bind(&AriacCompetition::end_competition_timer_callback, this));
    }
 
 private:
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

    rclcpp::TimerBase::SharedPtr end_competition_timer_;
    
    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

    void end_competition_timer_callback();

};