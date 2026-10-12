// Copyright (c) 2025 quantumbeam (powder_grinding_ros2 / grinding_force_torque), MIT License
// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// Low-pass filter for geometry_msgs/WrenchStamped.
// Ported from grinding_force_torque::WrenchFilter (powder_grinding_ros2 344d0cc). Changes:
//   - samples containing NaN/Inf are dropped (not filtered, not published) so that they do not
//     corrupt the IIR state; the output simply stops, which the wrench_watchdog detects as stale
//   - the Butterworth filter is designed in the node from sampling_frequency_hz (required, no
//     default), cutoff_frequency_hz (required) and filter_order, as second-order sections in
//     cascade, instead of passing filter_b / filter_a (placeholder default coefficients removed)
//   - zeroing readiness counts samples instead of keeping a deque

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/empty.hpp>

#include "wrench_filter/butterworth.hpp"
#include "wrench_filter/iir_filter.hpp"

namespace wrench_filter
{

using Wrench6 = std::array<double, 6>;

namespace
{
Wrench6 from_msg(const geometry_msgs::msg::Wrench & w)
{
  return {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
}

geometry_msgs::msg::Wrench to_msg(const Wrench6 & a)
{
  geometry_msgs::msg::Wrench w;
  w.force.x = a[0];
  w.force.y = a[1];
  w.force.z = a[2];
  w.torque.x = a[3];
  w.torque.y = a[4];
  w.torque.z = a[5];
  return w;
}

bool all_finite(const Wrench6 & a)
{
  for (double v : a) {
    if (!std::isfinite(v)) {return false;}
  }
  return true;
}
}  // namespace

class WrenchFilter : public rclcpp::Node
{
public:
  explicit WrenchFilter(const rclcpp::NodeOptions & options)
  : Node("wrench_filter", options)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "wrench_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "wrench_filtered");
    // samples that must be received before zeroing is allowed
    data_window_ = declare_parameter<int>("data_window", 100);
    if (data_window_ < 1) {
      throw std::invalid_argument("data_window must be >= 1");
    }
    initial_zero_ = declare_parameter<bool>("initial_zero", true);
    disable_filtering_ = declare_parameter<bool>("disable_filtering", false);
    // The filter advances one step per received message, so sampling_frequency_hz must be the
    // actual message rate of input_topic. It has no default on purpose: a wrong rate silently
    // shifts the cutoff (e.g. 100 Hz assumed for a 500 Hz topic makes it 5x higher).
    const double sampling_hz = required_double("sampling_frequency_hz");
    const double cutoff_hz = required_double("cutoff_frequency_hz");
    const int order = declare_parameter<int>("filter_order", 3);
    // second-order sections in cascade (one cascade per axis)
    const auto sections = butterworth_lowpass_sos(order, cutoff_hz, sampling_hz);
    filters_.resize(6);
    for (auto & cascade : filters_) {
      for (const auto & sec : sections) {cascade.emplace_back(sec.b, sec.a);}
    }

    publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(output_topic, 10);
    subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      input_topic, 10,
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {on_wrench(*msg);});
    zero_service_ = create_service<std_srvs::srv::Empty>(
      output_topic + "/zero_ftsensor",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {zero();});

    RCLCPP_INFO(
      get_logger(), "%s -> %s (filtering %s: order %d, cutoff %.3f Hz at %.3f Hz input; "
      "initial_zero %s)", input_topic.c_str(), output_topic.c_str(),
      disable_filtering_ ? "disabled" : "enabled", order, cutoff_hz, sampling_hz,
      initial_zero_ ? "true" : "false");
  }

private:
  // a double parameter without a default value; start-up fails if it is not given
  double required_double(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = false;
    declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE, descriptor);
    try {
      return get_parameter(name).as_double();
    } catch (const std::exception &) {
      // not given (ParameterUninitializedException) or not a double
      throw std::invalid_argument("parameter " + name + " (double) is required");
    }
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped & msg)
  {
    const Wrench6 input = from_msg(msg.wrench);
    if (!all_finite(input)) {
      ++dropped_non_finite_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Dropping wrench sample with NaN/Inf (%lu dropped so far); output is not published",
        static_cast<unsigned long>(dropped_non_finite_));
      return;
    }

    for (int i = 0; i < 6; ++i) {
      double y = input[i];
      if (!disable_filtering_) {
        for (auto & f : filters_[i]) {y = f.filter(y);}
      }
      filtered_[i] = y;
    }
    ++received_;

    if (initial_zero_ && !zeroed_ && received_ >= static_cast<uint64_t>(data_window_)) {
      offset_ = filtered_;
      zeroed_ = true;
      RCLCPP_INFO(get_logger(), "Auto zeroing complete.");
    }
    if (initial_zero_ && !zeroed_) {
      return;  // do not publish an offset-less value before the initial zeroing
    }

    Wrench6 out;
    for (int i = 0; i < 6; ++i) {
      out[i] = filtered_[i] - offset_[i];
    }
    if (!all_finite(out)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Filter output is not finite; not publishing");
      return;
    }

    geometry_msgs::msg::WrenchStamped out_msg;
    out_msg.header = msg.header;
    out_msg.wrench = to_msg(out);
    publisher_->publish(out_msg);
  }

  void zero()
  {
    if (received_ < static_cast<uint64_t>(data_window_)) {
      RCLCPP_WARN(
        get_logger(), "Not enough data to compute offset. Need %d, have %lu.", data_window_,
        static_cast<unsigned long>(received_));
      return;
    }
    offset_ = filtered_;
    zeroed_ = true;
    RCLCPP_INFO(get_logger(), "FT sensor zeroed (offset updated).");
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_service_;

  std::vector<std::vector<IirFilter>> filters_;  // [axis][section]
  Wrench6 filtered_{};
  Wrench6 offset_{};
  int data_window_;
  bool initial_zero_;
  bool disable_filtering_;
  bool zeroed_ = false;
  uint64_t received_ = 0;
  uint64_t dropped_non_finite_ = 0;
};

}  // namespace wrench_filter

RCLCPP_COMPONENTS_REGISTER_NODE(wrench_filter::WrenchFilter)
