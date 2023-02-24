#include "ariac_competition.hpp"

void AriacCompetition::end_competition_timer_callback() {
  if (competition_state_ == 3) {
    std::string srv_name = "/ariac/end_competition";

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("end_trigger_client");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = node->create_client<std_srvs::srv::Trigger>(srv_name);;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "client returned result %d",
                  result.get()->success);
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Failed to call trigger service");
    }
  }
}



void AriacCompetition::competition_state_cb( const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)  {
  competition_state_ = msg->competition_state;
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
    std::string srv_name = "/ariac/start_competition";

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_trigger_client");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = node->create_client<std_srvs::srv::Trigger>(srv_name);;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "client returned result %d",
                  result.get()->success);
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Failed to call trigger service");
    }
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