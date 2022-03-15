// "Copyright [year] <Copyright Owner>"

#include "GpsTranslator.hpp"
#include "as2_core/core_functions.hpp"

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<GpsTranslator::GpsTranslator>();
//   //TODO: check if freq is needed
//   as2::spinLoop(node);

//   rclcpp::shutdown();
//   return 0;
// }

// Compile-time composition
// Use of containers as node executable
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto translator = std::make_shared<GpsTranslator::GpsTranslator>(options);
  exec.add_node(translator->get_node_base_interface());

  exec.spin();
  rclcpp::shutdown();
}