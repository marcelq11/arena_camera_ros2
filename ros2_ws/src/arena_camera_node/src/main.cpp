#include "ArenaCameraNode.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ArenaCameraNode>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
