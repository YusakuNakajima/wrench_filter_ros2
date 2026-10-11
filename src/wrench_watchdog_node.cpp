// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// Watchdog for the wrench input of force-feedback controllers (e.g. cartesian_force_controller).
//
// Such controllers keep using the last received wrench when their input stops, and ignore NaN
// samples (i.e. also keep the last value). This node monitors the same topic in a separate
// process and, while any of `monitored_controllers` is active and the input is missing, stale or
// non-finite, switches the controller_manager:
//   deactivate the active monitored controllers + activate `fallback_controller` (STRICT)
// A joint trajectory controller holds the current position when it is activated, so the robot
// stops where it is. If that switch is rejected, the monitored controllers are deactivated alone
// (BEST_EFFORT). While a fault persists and a monitored controller is still active, the switch is
// retried every check period. If the switch service is unavailable or the deactivation is
// rejected, `escalation_service` (std_srvs/Trigger, e.g. the UR dashboard "stop") is called once
// per fault episode. The watchdog never re-activates a force controller by itself.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "wrench_filter/watchdog_monitor.hpp"

namespace wrench_filter
{

using namespace std::chrono_literals;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using Trigger = std_srvs::srv::Trigger;

namespace
{
std::chrono::nanoseconds from_ms(double ms)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(ms * 1e6));
}

// periods / timeouts in ms must be finite and in (0, 60 s]
double positive_ms(rclcpp::Node & node, const std::string & name, double default_ms)
{
  const double v = node.declare_parameter<double>(name, default_ms);
  if (!std::isfinite(v) || v <= 0.0 || v > 60000.0) {
    throw std::invalid_argument(name + " must be a finite value in (0, 60000] ms");
  }
  return v;
}

bool finite(const geometry_msgs::msg::Wrench & w)
{
  return std::isfinite(w.force.x) && std::isfinite(w.force.y) && std::isfinite(w.force.z) &&
         std::isfinite(w.torque.x) && std::isfinite(w.torque.y) && std::isfinite(w.torque.z);
}
}  // namespace

class WrenchWatchdog : public rclcpp::Node
{
public:
  explicit WrenchWatchdog(const rclcpp::NodeOptions & options)
  : Node("wrench_watchdog", options),
    monitor_(from_ms(positive_ms(*this, "stale_timeout_ms", 50.0)))
  {
    const auto wrench_topic = declare_parameter<std::string>("wrench_topic", "wrench");
    const auto controller_manager =
      declare_parameter<std::string>("controller_manager", "/controller_manager");
    monitored_controllers_ = declare_parameter<std::vector<std::string>>(
      "monitored_controllers",
      std::vector<std::string>{"cartesian_force_controller", "cartesian_compliance_controller"});
    // empty = only deactivate the monitored controllers
    fallback_controller_ =
      declare_parameter<std::string>("fallback_controller", "scaled_joint_trajectory_controller");
    // empty = no escalation
    const auto escalation_service = declare_parameter<std::string>("escalation_service", "");
    const double check_period_ms = positive_ms(*this, "check_period_ms", 5.0);
    // a newly activated controller is monitored only after the next poll, so this adds to the
    // worst-case reaction time right after activation
    const double state_poll_period_ms = positive_ms(*this, "state_poll_period_ms", 10.0);
    switch_timeout_ms_ = positive_ms(*this, "switch_timeout_ms", 1000.0);
    if (monitored_controllers_.empty()) {
      throw std::invalid_argument("monitored_controllers must not be empty");
    }
    if (std::find(
        monitored_controllers_.begin(), monitored_controllers_.end(), fallback_controller_) !=
      monitored_controllers_.end())
    {
      throw std::invalid_argument("fallback_controller must not be one of monitored_controllers");
    }

    subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic, 10, [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
        monitor_.on_message(WatchdogMonitor::Clock::now(), finite(msg->wrench));
      });
    list_client_ = create_client<ListControllers>(controller_manager + "/list_controllers");
    switch_client_ = create_client<SwitchController>(controller_manager + "/switch_controller");
    if (!escalation_service.empty()) {
      escalation_client_ = create_client<Trigger>(escalation_service);
    }

    poll_timer_ = create_wall_timer(from_ms(state_poll_period_ms), [this]() {poll_state();});
    check_timer_ = create_wall_timer(from_ms(check_period_ms), [this]() {check();});

    std::string names;
    for (const auto & n : monitored_controllers_) {names += n + " ";}
    RCLCPP_INFO(
      get_logger(), "Watching %s (stale timeout %.1f ms) for: %s-> fallback '%s', escalation '%s'",
      subscription_->get_topic_name(), get_parameter("stale_timeout_ms").as_double(),
      names.c_str(), fallback_controller_.c_str(), escalation_service.c_str());
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  // A request whose response did not arrive in time is dropped from the client so that a hung
  // controller_manager cannot block the watchdog forever.
  template<typename ServiceT>
  static bool expired(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    std::optional<int64_t> & request_id, SteadyClock::time_point sent,
    std::chrono::nanoseconds deadline)
  {
    if (!request_id || SteadyClock::now() - sent <= deadline) {return false;}
    client->remove_pending_request(*request_id);
    request_id.reset();
    return true;
  }

  void poll_state()
  {
    if (expired<ListControllers>(list_client_, list_request_, list_sent_, from_ms(500.0))) {
      // keep the last known state: if a monitored controller was active it is still treated as
      // active, so a fault still triggers a switch
      RCLCPP_ERROR(get_logger(), "list_controllers did not respond within 500 ms");
    }
    if (list_request_ || !list_client_->service_is_ready()) {return;}
    const auto sent = SteadyClock::now();
    list_sent_ = sent;
    list_request_ = list_client_->async_send_request(
      std::make_shared<ListControllers::Request>(),
      [this, sent](rclcpp::Client<ListControllers>::SharedFuture future) {
        list_request_.reset();
        // a response to a request sent before the switch completed cannot verify the switch
        if (awaiting_verification_ && sent < switch_completed_) {return;}
        std::vector<std::string> active;
        bool fallback_active = false;
        try {
          for (const auto & c : future.get()->controller) {
            if (c.name == fallback_controller_ && !fallback_checked_) {
              fallback_checked_ = true;
              // only a joint trajectory controller is known to hold the current position on
              // activation; anything else (velocity / effort / force controller) is refused
              if (std::find(
                  kHoldingControllerTypes.begin(), kHoldingControllerTypes.end(), c.type) ==
                kHoldingControllerTypes.end())
              {
                RCLCPP_ERROR(
                  get_logger(), "fallback_controller '%s' has type '%s', which is not a joint "
                  "trajectory controller; only deactivating on faults", c.name.c_str(),
                  c.type.c_str());
                fallback_controller_.clear();
              }
            }
            if (c.name == fallback_controller_ && c.state == "active") {
              fallback_active = true;
            }
            if (c.state == "active" &&
              std::find(monitored_controllers_.begin(), monitored_controllers_.end(), c.name) !=
              monitored_controllers_.end())
            {
              active.push_back(c.name);
            }
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "list_controllers failed: %s", e.what());
          return;  // keep the last known state
        }
        active_monitored_ = active;
        if (awaiting_verification_ && expect_fallback_active_ && !fallback_active) {
          // the switch reported ok but nothing is holding the position
          RCLCPP_ERROR(
            get_logger(), "%s is not active after the switch", fallback_controller_.c_str());
          escalate();
        }
        awaiting_verification_ = false;
        expect_fallback_active_ = false;
        last_state_update_ = SteadyClock::now();
      }).request_id;
  }

  void check()
  {
    if (escalation_client_ &&
      expired<Trigger>(
        escalation_client_, escalation_request_, escalation_sent_, from_ms(kResponseMarginMs)))
    {
      RCLCPP_ERROR(
        get_logger(), "Escalation service %s did not respond within %.0f ms; the robot may not "
        "have stopped", escalation_client_->get_service_name(), kResponseMarginMs);
    }
    if (expired<SwitchController>(
        switch_client_, switch_request_, switch_sent_,
        from_ms(switch_timeout_ms_ + kResponseMarginMs)))
    {
      RCLCPP_ERROR(get_logger(), "switch_controller did not respond in time");
      escalate();
    }

    const WrenchFault fault = monitor_.check(WatchdogMonitor::Clock::now());
    // The controller state was known once but list_controllers has not answered since: whether a
    // monitored controller is active is unknown, so a wrench fault is escalated. (Before the
    // first answer nothing is escalated, so that start-up does not stop the robot.)
    if (last_state_update_ && fault != WrenchFault::kNone &&
      SteadyClock::now() - *last_state_update_ > from_ms(kStateLostMs))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s and controller state unknown for > %.0f ms",
        to_string(fault).c_str(), kStateLostMs);
      escalate();
      return;
    }
    if (fault == WrenchFault::kNone || active_monitored_.empty()) {
      escalated_ = false;  // re-arm the escalation for the next fault episode
    }
    // after a successful switch, wait for list_controllers to confirm the new state; if the
    // monitored controller is still active then, the switch is retried
    if (active_monitored_.empty() || switch_request_ || awaiting_verification_ ||
      fault == WrenchFault::kNone)
    {
      return;
    }

    std::string action = "deactivating";
    for (const auto & n : active_monitored_) {action += " " + n;}
    if (!fallback_controller_.empty()) {action += ", activating " + fallback_controller_;}
    RCLCPP_ERROR(get_logger(), "%s -> %s", to_string(fault).c_str(), action.c_str());
    request_switch(active_monitored_, !fallback_controller_.empty());
  }

  void request_switch(const std::vector<std::string> & deactivate, bool with_fallback)
  {
    if (!switch_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s is not available", switch_client_->get_service_name());
      escalate();
      return;
    }
    auto request = std::make_shared<SwitchController::Request>();
    request->deactivate_controllers = deactivate;
    if (with_fallback) {
      request->activate_controllers = {fallback_controller_};
      request->strictness = SwitchController::Request::STRICT;
    } else {
      request->strictness = SwitchController::Request::BEST_EFFORT;
    }
    request->activate_asap = true;
    request->timeout = rclcpp::Duration(from_ms(switch_timeout_ms_));

    switch_sent_ = SteadyClock::now();
    switch_request_ = switch_client_->async_send_request(
      request,
      [this, deactivate, with_fallback](rclcpp::Client<SwitchController>::SharedFuture future) {
        switch_request_.reset();
        bool ok = false;
        try {
          ok = future.get()->ok;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "switch_controller failed: %s", e.what());
          escalate();
          return;
        }
        if (ok) {
          const std::string held =
            with_fallback ? " (" + fallback_controller_ + " should hold the position)" : "";
          RCLCPP_WARN(
            get_logger(), "Switched controllers%s. Re-activate the force controller manually "
            "after the wrench input is healthy again.", held.c_str());
          awaiting_verification_ = true;
          expect_fallback_active_ = with_fallback;
          switch_completed_ = SteadyClock::now();
          poll_state();
        } else if (with_fallback) {
          RCLCPP_ERROR(
            get_logger(), "Switch with fallback '%s' was rejected; deactivating only",
            fallback_controller_.c_str());
          request_switch(deactivate, false);
        } else {
          RCLCPP_ERROR(get_logger(), "Deactivating the monitored controllers was rejected");
          escalate();
        }
      }).request_id;
  }

  void escalate()
  {
    if (!escalation_client_ || escalated_) {return;}
    escalated_ = true;
    if (!escalation_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "Escalation service %s is not available",
        escalation_client_->get_service_name());
      return;
    }
    RCLCPP_ERROR(get_logger(), "Calling escalation service %s", escalation_client_->get_service_name());
    escalation_sent_ = SteadyClock::now();
    escalation_request_ = escalation_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        try {
          const auto & r = *future.get();
          if (r.success) {
            RCLCPP_WARN(get_logger(), "Escalation succeeded: %s", r.message.c_str());
          } else {
            RCLCPP_ERROR(get_logger(), "Escalation failed: %s", r.message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Escalation failed: %s", e.what());
        }
        escalation_request_.reset();
      }).request_id;
  }

  static constexpr double kResponseMarginMs = 500.0;
  static constexpr double kStateLostMs = 500.0;
  // controller types known to hold the current position when activated
  static inline const std::vector<std::string> kHoldingControllerTypes{
    "joint_trajectory_controller/JointTrajectoryController",
    "ur_controllers/ScaledJointTrajectoryController"};

  WatchdogMonitor monitor_;
  std::vector<std::string> monitored_controllers_;
  std::string fallback_controller_;
  double switch_timeout_ms_;

  std::vector<std::string> active_monitored_;
  std::optional<int64_t> list_request_;
  std::optional<int64_t> switch_request_;
  SteadyClock::time_point list_sent_;
  SteadyClock::time_point switch_sent_;
  bool awaiting_verification_ = false;
  bool expect_fallback_active_ = false;
  SteadyClock::time_point switch_completed_;
  bool fallback_checked_ = false;
  std::optional<SteadyClock::time_point> last_state_update_;
  std::optional<int64_t> escalation_request_;
  SteadyClock::time_point escalation_sent_;
  bool escalated_ = false;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<Trigger>::SharedPtr escalation_client_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

}  // namespace wrench_filter

RCLCPP_COMPONENTS_REGISTER_NODE(wrench_filter::WrenchWatchdog)
