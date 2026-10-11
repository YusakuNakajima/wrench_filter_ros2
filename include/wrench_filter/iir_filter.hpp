// Copyright (c) 2025 quantumbeam (powder_grinding_ros2 / grinding_force_torque), MIT License
// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// IIR filter (Direct Form II Transposed) applied one sample at a time.
// Ported from grinding_force_torque::ButterworthLowPass (powder_grinding_ros2 344d0cc).

#ifndef WRENCH_FILTER__IIR_FILTER_HPP_
#define WRENCH_FILTER__IIR_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace wrench_filter
{

class IirFilter
{
public:
  // b: numerator, a: denominator (same convention as scipy.signal.butter)
  IirFilter(std::vector<double> b, std::vector<double> a)
  : b_(std::move(b)), a_(std::move(a))
  {
    if (a_.empty() || std::abs(a_[0]) < 1e-12) {
      throw std::invalid_argument("filter_a must be non-empty and a[0] must be non-zero");
    }
    if (b_.size() != a_.size()) {
      throw std::invalid_argument("filter_b and filter_a must have the same length");
    }
    for (double v : b_) {
      if (!std::isfinite(v)) {throw std::invalid_argument("filter_b contains a non-finite value");}
    }
    for (double v : a_) {
      if (!std::isfinite(v)) {throw std::invalid_argument("filter_a contains a non-finite value");}
    }
    const double a0 = a_[0];
    for (auto & v : b_) {v /= a0;}
    for (auto & v : a_) {v /= a0;}
    for (double v : b_) {
      if (!std::isfinite(v)) {throw std::invalid_argument("filter_b / a[0] is not finite");}
    }
    for (double v : a_) {
      if (!std::isfinite(v)) {throw std::invalid_argument("filter_a / a[0] is not finite");}
    }
    if (!is_stable(a_)) {
      throw std::invalid_argument("filter_a has a pole on or outside the unit circle (unstable)");
    }
    z_.assign(a_.size() - 1, 0.0);
  }

  // Schur-Cohn (step-down) test: true if all roots of z^n + a[1] z^(n-1) + ... + a[n]
  // lie strictly inside the unit circle. `a` must be normalized so that a[0] == 1.
  static bool is_stable(std::vector<double> a)
  {
    if (a.empty()) {return false;}
    for (size_t k = a.size() - 1; k >= 1; --k) {
      const double r = a[k];  // reflection coefficient
      if (!(std::abs(r) < 1.0)) {return false;}
      const double d = 1.0 - r * r;
      std::vector<double> next(k);
      next[0] = 1.0;
      for (size_t i = 1; i < k; ++i) {
        next[i] = (a[i] - r * a[k - i]) / d;
      }
      a = next;
    }
    return true;
  }

  void reset() {std::fill(z_.begin(), z_.end(), 0.0);}

  // Callers must not pass a non-finite x: it would corrupt the internal state permanently.
  double filter(double x)
  {
    const double y = b_[0] * x + (z_.empty() ? 0.0 : z_[0]);
    if (!z_.empty()) {
      for (size_t i = 0; i + 1 < z_.size(); ++i) {
        z_[i] = b_[i + 1] * x + z_[i + 1] - a_[i + 1] * y;
      }
      z_.back() = b_.back() * x - a_.back() * y;
    }
    return y;
  }

private:
  std::vector<double> b_;
  std::vector<double> a_;
  std::vector<double> z_;
};

}  // namespace wrench_filter

#endif  // WRENCH_FILTER__IIR_FILTER_HPP_
