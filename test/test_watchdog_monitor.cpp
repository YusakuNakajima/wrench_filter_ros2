// Copyright (c) 2026 Yusaku Nakajima, MIT License

#include <gtest/gtest.h>

#include <chrono>

#include "wrench_filter/watchdog_monitor.hpp"

using namespace std::chrono_literals;
using wrench_filter::WatchdogMonitor;
using wrench_filter::WrenchFault;

TEST(WatchdogMonitor, NeverReceived)
{
  WatchdogMonitor m(50ms);
  EXPECT_EQ(m.check(WatchdogMonitor::Clock::now()), WrenchFault::kNeverReceived);
}

TEST(WatchdogMonitor, FreshFiniteMessageIsHealthy)
{
  WatchdogMonitor m(50ms);
  const auto t0 = WatchdogMonitor::Clock::time_point{} + 1s;
  m.on_message(t0, true);
  EXPECT_EQ(m.check(t0 + 50ms), WrenchFault::kNone);
}

TEST(WatchdogMonitor, StaleAfterTimeout)
{
  WatchdogMonitor m(50ms);
  const auto t0 = WatchdogMonitor::Clock::time_point{} + 1s;
  m.on_message(t0, true);
  EXPECT_EQ(m.check(t0 + 51ms), WrenchFault::kStale);
}

TEST(WatchdogMonitor, NonFiniteUntilNextFiniteMessage)
{
  WatchdogMonitor m(50ms);
  const auto t0 = WatchdogMonitor::Clock::time_point{} + 1s;
  m.on_message(t0, false);
  EXPECT_EQ(m.check(t0 + 1ms), WrenchFault::kNonFinite);
  m.on_message(t0 + 2ms, true);
  EXPECT_EQ(m.check(t0 + 3ms), WrenchFault::kNone);
}
