// "Copyright [year] <Copyright Owner>"

#include "GpsTranslator.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsTranslator>();
  //TODO: check if freq is needed
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}