#include "ariac_competition.hpp"

// AriacCompetition::AriacCompetition() : Node ("AriacCompetition"){

//     competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
//     std::bind(&AriacCompetition::competition_state_cb, this, std::placeholders::_1));
// }

void AriacCompetition::competition_state_cb( const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) 
{
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/start_competition";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    client->async_send_request(request);
}
 
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto ariac_competition  = std::make_shared<AriacCompetition>("RWA1");
  // Start Competition
  rclcpp::spin(ariac_competition);
  rclcpp::shutdown();

}