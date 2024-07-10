#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam() // constructor
  : Node("minimal_param_node")
  {
    // (Optional) you can set a descriptor for the parameter (ros2 param describe ...)
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";

    // create a parameter with default value "world", parameter type is inferred from the default value (string type)
    this->declare_parameter("my_parameter", "world", param_desc); 

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this)); // execute timer_callback once a second
  }

  void timer_callback()
  {
    // get the "my_parameter" from the node
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    // set the "my_parameter" back to the default string value "world"
    // in the case that the user changed the parameter externally, this ensures it is always reset back to the original
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}