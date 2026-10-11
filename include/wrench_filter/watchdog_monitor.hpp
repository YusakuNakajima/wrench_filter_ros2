// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// ROS-independent fault decision of the wrench watchdog, so that it can be unit tested.

#ifndef WRENCH_FILTER__WATCHDOG_MONITOR_HPP_
#define WRENCH_FILTER__WATCHDOG_MONITOR_HPP_

#include <chrono>
#include <optional>
#include <string>

namespace wrench_filter
{

enum class WrenchFault
{
  kNone,
  kNeverReceived,  // no message since the watchdog started
  kStale,          // last message is older than the timeout
  kNonFinite,      // last message contained NaN / Inf
};

inline std::string to_string(WrenchFault fault)
{
  switch (fault) {
    case WrenchFault::kNone: return "none";
    case WrenchFault::kNeverReceived: return "no wrench message received";
    case WrenchFault::kStale: return "wrench message is stale";
    case WrenchFault::kNonFinite: return "wrench message contains NaN/Inf";
  }
  return "unknown";
}

class WatchdogMonitor
{
public:
  using Clock = std::chrono::steady_clock;

  explicit WatchdogMonitor(Clock::duration stale_timeout)
  : stale_timeout_(stale_timeout) {}

  void on_message(Clock::time_point stamp, bool finite)
  {
    last_message_ = stamp;
    last_finite_ = finite;
  }

  WrenchFault check(Clock::time_point now) const
  {
    if (!last_message_) {return WrenchFault::kNeverReceived;}
    if (!last_finite_) {return WrenchFault::kNonFinite;}
    if (now - *last_message_ > stale_timeout_) {return WrenchFault::kStale;}
    return WrenchFault::kNone;
  }

private:
  Clock::duration stale_timeout_;
  std::optional<Clock::time_point> last_message_;
  bool last_finite_ = true;
};

}  // namespace wrench_filter

#endif  // WRENCH_FILTER__WATCHDOG_MONITOR_HPP_
