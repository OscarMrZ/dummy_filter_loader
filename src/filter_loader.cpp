#include <rclcpp/utilities.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "control_toolbox/control_filters/gravity_compensation.hpp"

// Base class template with pure virtual function
class DummyFilterLoader : public rclcpp::Node
{
public:
  DummyFilterLoader()
  : Node("dummy_filter_loader"), loader_("filters",
      "filters::FilterBase<geometry_msgs::msg::WrenchStamped>")
  {
  }

  void configure_filter_chain()
  {
    filter_chain_ =
      std::make_unique<filters::FilterChain<geometry_msgs::msg::WrenchStamped>>(
      "geometry_msgs::msg::WrenchStamped");

    filter_chain_->configure(
      "sensor_filter_chain", this->get_node_logging_interface(),
      this->get_node_parameters_interface());
  }

  void configure_filter_directly()
  {
    pluginlib::UniquePtr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> loaded_filter;

    loaded_filter = loader_.createUniqueInstance("control_filters/GravityCompensationWrench");
    loaded_filter->configure(
      "sensor_filter_chain.filter1", "gravity_compensation",
      this->get_node_logging_interface(), this->get_node_parameters_interface());
  }

private:
  std::unique_ptr<filters::FilterChain<geometry_msgs::msg::WrenchStamped>> filter_chain_;
  pluginlib::ClassLoader<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> loader_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto dummy_filter = std::make_shared<DummyFilterLoader>();
  dummy_filter->configure_filter_directly();

  // rclcpp::spin(dummy_filter);

  rclcpp::shutdown();
  return 0;
}
