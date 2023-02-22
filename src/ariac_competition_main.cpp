#include "ariac_competition.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto ariac_competition  = std::make_shared<AriacCompetition>("RWA1");
  // Start Competition
  rclcpp::spin(ariac_competition);


  rclcpp::shutdown();

}