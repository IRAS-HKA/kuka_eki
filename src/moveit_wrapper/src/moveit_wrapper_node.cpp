#include <moveit_wrapper/moveit_wrapper.h>
#include <typeinfo>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto moveit_wrapper = std::make_shared<moveit_wrapper::MoveitWrapper>(node_options);
  moveit_wrapper->init_move_group();
  exec.add_node(moveit_wrapper);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
