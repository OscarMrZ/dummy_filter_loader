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
  : Node("dummy_filter_loader"), filter_loader_("filters",
      "filters::FilterBase<geometry_msgs::msg::WrenchStamped>")
  {
  }

  void configure_filter_chain()
  {
    std::cout << "Configuring filter chain" << std::endl;

    filter_chain_ =
      std::make_unique<filters::FilterChain<geometry_msgs::msg::WrenchStamped>>(
      "geometry_msgs::msg::WrenchStamped");

    filter_chain_->configure(
      "sensor_filter_chain", this->get_node_logging_interface(),
      this->get_node_parameters_interface());
  }

  void configure_filter_directly_unique_ptr()
  {
    std::cout << "Configuring filter directly as a plugin (unique ptr)" << std::endl;

    pluginlib::UniquePtr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter;

    std::string filter_type = "control_filters/GravityCompensationWrench";
    filter = filter_loader_.createUniqueInstance(filter_type);

    filter->configure(
      "sensor_filter_chain.filter1.params", "gravity_compensation",
      this->get_node_logging_interface(), this->get_node_parameters_interface());
  }

  void configure_filter_directly_as_plugin()
  {
    std::cout << "Configuring filter directly as a plugin (shared ptr)" << std::endl;

    std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter;
    auto available_classes = filter_loader_.getDeclaredClasses();
    std::cout << "available filters:" << std::endl;
    for (const auto & available_class : available_classes) {
      std::cout << "  " << available_class << std::endl;
    }

    std::string filter_type = "control_filters/GravityCompensationWrench";
    filter = filter_loader_.createSharedInstance(filter_type);

    filter->configure(
      "sensor_filter_chain.filter1.params", "gravity_compensation",
      this->get_node_logging_interface(), this->get_node_parameters_interface());
  }

  // void configure_filter_directly()
  // {
  //   std::cout << "Configuring filter directly" << std::endl;

  //   std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter =
  //     std::make_shared<control_filters::GravityCompensation<geometry_msgs::msg::WrenchStamped>>();

  //   filter->configure(
  //     "sensor_filter_chain.filter1.params", "gravity_compensation",
  //     this->get_node_logging_interface(), this->get_node_parameters_interface());
  // }

private:
  std::unique_ptr<filters::FilterChain<geometry_msgs::msg::WrenchStamped>> filter_chain_;
  pluginlib::ClassLoader<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_loader_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto dummy_filter = std::make_shared<DummyFilterLoader>();
  dummy_filter->configure_filter_chain();
  dummy_filter->configure_filter_directly_as_plugin();
  dummy_filter->configure_filter_directly_unique_ptr();
  // dummy_filter->configure_filter_directly();

  // rclcpp::spin(dummy_filter);

  rclcpp::shutdown();
  return 0;
}
