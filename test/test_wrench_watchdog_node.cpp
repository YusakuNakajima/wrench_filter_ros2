// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// ROS integration test of wrench_filter::WrenchWatchdog against a fake controller_manager.
// The watchdog component is loaded from its shared library (WATCHDOG_COMPONENT_LIBRARY) the same
// way a component container loads it.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <class_loader/class_loader.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using Trigger = std_srvs::srv::Trigger;

namespace
{

constexpr char kJtcType[] = "joint_trajectory_controller/JointTrajectoryController";
constexpr char kForceType[] = "cartesian_force_controller/CartesianForceController";

struct FakeController
{
  std::string name;
  std::string type;
  std::string state;
};

// Fake controller_manager: list_controllers / switch_controller, plus an escalation service.
class FakeControllerManager
{
public:
  explicit FakeControllerManager(const std::string & ns)
  : node_(std::make_shared<rclcpp::Node>("fake_cm", ns))
  {
    list_ = node_->create_service<ListControllers>(
      "controller_manager/list_controllers",
      [this](const std::shared_ptr<ListControllers::Request>,
      std::shared_ptr<ListControllers::Response> res) {
        for (const auto & c : controllers) {
          controller_manager_msgs::msg::ControllerState s;
          s.name = c.name;
          s.type = c.type;
          s.state = c.state;
          res->controller.push_back(s);
        }
      });
    switch_ = node_->create_service<SwitchController>(
      "controller_manager/switch_controller",
      [this](const std::shared_ptr<SwitchController::Request> req,
      std::shared_ptr<SwitchController::Response> res) {
        requests.push_back(*req);
        res->ok = switch_handler(*req);
      });
    escalation_ = node_->create_service<Trigger>(
      "stop", [this](const std::shared_ptr<Trigger::Request>,
      std::shared_ptr<Trigger::Response> res) {
        ++escalations;
        res->success = true;
      });
  }

  FakeController * find(const std::string & name)
  {
    for (auto & c : controllers) {
      if (c.name == name) {return &c;}
    }
    return nullptr;
  }

  // default behaviour: apply the request like controller_manager does
  bool apply(const SwitchController::Request & req)
  {
    if (req.strictness == SwitchController::Request::STRICT) {
      for (const auto & n : req.activate_controllers) {
        if (!find(n)) {return false;}
      }
    }
    for (const auto & n : req.deactivate_controllers) {
      if (auto * c = find(n)) {c->state = "inactive";}
    }
    for (const auto & n : req.activate_controllers) {
      if (auto * c = find(n)) {c->state = "active";}
    }
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<FakeController> controllers;
  std::vector<SwitchController::Request> requests;
  int escalations = 0;
  std::function<bool(const SwitchController::Request &)> switch_handler =
    [this](const SwitchController::Request & r) {return apply(r);};

private:
  rclcpp::Service<ListControllers>::SharedPtr list_;
  rclcpp::Service<SwitchController>::SharedPtr switch_;
  rclcpp::Service<Trigger>::SharedPtr escalation_;
};

class WrenchWatchdogTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    // a unique namespace per test so that services of earlier tests are never reached
    static int counter = 0;
    ns_ = "/wd_test_" + std::to_string(counter++);
    cm_ = std::make_unique<FakeControllerManager>(ns_);
    cm_->controllers = {
      {"cartesian_force_controller", kForceType, "active"},
      {"scaled_joint_trajectory_controller", kJtcType, "inactive"}};
    publisher_node_ = std::make_shared<rclcpp::Node>("wrench_publisher", ns_);
    publisher_ = publisher_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      ns_ + "/wrench", 10);
    executor_.add_node(cm_->node_);
    executor_.add_node(publisher_node_);
  }

  void TearDown() override
  {
    if (started_) {executor_.remove_node(watchdog_.get_node_base_interface());}
    executor_.remove_node(publisher_node_);
    executor_.remove_node(cm_->node_);
    watchdog_ = rclcpp_components::NodeInstanceWrapper();
    loader_.reset();
  }

  void start_watchdog(const std::string & fallback = "scaled_joint_trajectory_controller")
  {
    loader_ = std::make_unique<class_loader::ClassLoader>(WATCHDOG_COMPONENT_LIBRARY);
    auto factory = loader_->createInstance<rclcpp_components::NodeFactory>(
      "rclcpp_components::NodeFactoryTemplate<wrench_filter::WrenchWatchdog>");
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    options.parameter_overrides({
        {"wrench_topic", ns_ + "/wrench"},
        {"controller_manager", ns_ + "/controller_manager"},
        {"monitored_controllers", std::vector<std::string>{"cartesian_force_controller"}},
        {"fallback_controller", fallback},
        {"escalation_service", ns_ + "/stop"},
        {"stale_timeout_ms", 50.0}});
    watchdog_ = factory->create_node_instance(options);
    executor_.add_node(watchdog_.get_node_base_interface());
    started_ = true;
  }

  void publish(double fz)
  {
    geometry_msgs::msg::WrenchStamped m;
    m.wrench.force.z = fz;
    publisher_->publish(m);
  }

  // spin while publishing a healthy wrench every 2 ms (500 Hz) for `duration`
  void spin_healthy(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      publish(1.0);
      executor_.spin_some(1ms);
      std::this_thread::sleep_for(2ms);
    }
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {executor_.spin_some(1ms);}
  }

  std::string ns_;
  std::unique_ptr<FakeControllerManager> cm_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<class_loader::ClassLoader> loader_;
  rclcpp_components::NodeInstanceWrapper watchdog_;
  bool started_ = false;
};

TEST_F(WrenchWatchdogTest, HealthyInputDoesNotSwitch)
{
  start_watchdog();
  spin_healthy(500ms);
  EXPECT_TRUE(cm_->requests.empty());
  EXPECT_EQ(cm_->escalations, 0);
}

TEST_F(WrenchWatchdogTest, StaleInputSwitchesToFallback)
{
  start_watchdog();
  spin_healthy(300ms);
  spin_for(500ms);  // input stops

  ASSERT_EQ(cm_->requests.size(), 1u);
  const auto & r = cm_->requests[0];
  EXPECT_EQ(r.strictness, SwitchController::Request::STRICT);
  EXPECT_EQ(r.deactivate_controllers, std::vector<std::string>{"cartesian_force_controller"});
  EXPECT_EQ(
    r.activate_controllers, std::vector<std::string>{"scaled_joint_trajectory_controller"});
  EXPECT_EQ(cm_->find("cartesian_force_controller")->state, "inactive");
  EXPECT_EQ(cm_->find("scaled_joint_trajectory_controller")->state, "active");
  EXPECT_EQ(cm_->escalations, 0);
}

TEST_F(WrenchWatchdogTest, NonFiniteInputSwitches)
{
  start_watchdog();
  const auto end = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < end) {
    publish(std::numeric_limits<double>::quiet_NaN());
    executor_.spin_some(1ms);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_GE(cm_->requests.size(), 1u);
  EXPECT_EQ(cm_->find("cartesian_force_controller")->state, "inactive");
  EXPECT_EQ(cm_->find("scaled_joint_trajectory_controller")->state, "active");
}

TEST_F(WrenchWatchdogTest, OkSwitchWithoutActiveFallbackEscalates)
{
  // the switch reports ok, but the fallback does not end up active
  cm_->switch_handler = [this](const SwitchController::Request & r) {
      for (const auto & n : r.deactivate_controllers) {cm_->find(n)->state = "inactive";}
      return true;
    };
  start_watchdog();
  spin_healthy(300ms);
  spin_for(500ms);
  ASSERT_EQ(cm_->requests.size(), 1u);
  EXPECT_EQ(cm_->escalations, 1);
}

TEST_F(WrenchWatchdogTest, RejectedStrictSwitchFallsBackToDeactivateOnly)
{
  cm_->switch_handler = [this](const SwitchController::Request & r) {
      if (r.strictness == SwitchController::Request::STRICT) {return false;}
      return cm_->apply(r);
    };
  start_watchdog();
  spin_healthy(300ms);
  spin_for(500ms);
  ASSERT_GE(cm_->requests.size(), 2u);
  EXPECT_EQ(cm_->requests[1].strictness, SwitchController::Request::BEST_EFFORT);
  EXPECT_TRUE(cm_->requests[1].activate_controllers.empty());
  EXPECT_EQ(cm_->find("cartesian_force_controller")->state, "inactive");
  EXPECT_EQ(cm_->escalations, 0);
}

TEST_F(WrenchWatchdogTest, RejectedDeactivationEscalatesOnce)
{
  cm_->switch_handler = [](const SwitchController::Request &) {return false;};
  start_watchdog();
  spin_healthy(300ms);
  spin_for(500ms);
  ASSERT_GE(cm_->requests.size(), 2u);
  EXPECT_EQ(cm_->escalations, 1);
}

TEST_F(WrenchWatchdogTest, FallbackOfWrongTypeIsNotActivated)
{
  cm_->find("scaled_joint_trajectory_controller")->type = kForceType;
  start_watchdog();
  spin_healthy(300ms);
  spin_for(500ms);
  ASSERT_GE(cm_->requests.size(), 1u);
  for (const auto & r : cm_->requests) {EXPECT_TRUE(r.activate_controllers.empty());}
  EXPECT_EQ(cm_->find("scaled_joint_trajectory_controller")->state, "inactive");
  EXPECT_EQ(cm_->find("cartesian_force_controller")->state, "inactive");
}

TEST_F(WrenchWatchdogTest, FallbackReloadedAsWrongTypeIsNotActivated)
{
  start_watchdog();
  spin_healthy(300ms);  // the fallback has been seen with the correct type
  cm_->find("scaled_joint_trajectory_controller")->type = kForceType;
  spin_healthy(100ms);  // the watchdog polls the new type
  spin_for(500ms);
  ASSERT_GE(cm_->requests.size(), 1u);
  for (const auto & r : cm_->requests) {EXPECT_TRUE(r.activate_controllers.empty());}
  EXPECT_EQ(cm_->find("scaled_joint_trajectory_controller")->state, "inactive");
}

TEST_F(WrenchWatchdogTest, InactiveMonitoredControllerIsNotSwitched)
{
  cm_->find("cartesian_force_controller")->state = "inactive";
  start_watchdog();
  spin_for(500ms);  // no input at all
  EXPECT_TRUE(cm_->requests.empty());
  EXPECT_EQ(cm_->escalations, 0);
}

TEST_F(WrenchWatchdogTest, FallbackInMonitoredControllersIsRejected)
{
  EXPECT_ANY_THROW(start_watchdog("cartesian_force_controller"));
}

}  // namespace
