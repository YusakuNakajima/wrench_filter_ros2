// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// Digital Butterworth low-pass design, equivalent to
//   scipy.signal.butter(order, cutoff_hz / (0.5 * sampling_hz))
// (analog prototype -> pre-warped cutoff -> bilinear transform).

#ifndef WRENCH_FILTER__BUTTERWORTH_HPP_
#define WRENCH_FILTER__BUTTERWORTH_HPP_

#include <cmath>
#include <complex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wrench_filter
{

constexpr int kMaxButterworthOrder = 8;

// Returns {b, a} (a[0] == 1). sampling_hz: rate at which samples are fed to the filter [Hz].
inline std::pair<std::vector<double>, std::vector<double>> butterworth_lowpass(
  int order, double cutoff_hz, double sampling_hz)
{
  if (order < 1 || order > kMaxButterworthOrder) {
    throw std::invalid_argument("filter_order must be in [1, 8]");
  }
  if (!std::isfinite(sampling_hz) || sampling_hz <= 0.0) {
    throw std::invalid_argument("sampling_frequency_hz must be a finite positive value");
  }
  if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0 || cutoff_hz >= 0.5 * sampling_hz) {
    throw std::invalid_argument(
            "cutoff_frequency_hz must be in (0, sampling_frequency_hz / 2)");
  }

  using C = std::complex<double>;
  constexpr double kPi = 3.14159265358979323846;
  // scipy works with the normalized frequency Wn and fs = 2 for the bilinear transform
  const double wn = cutoff_hz / (0.5 * sampling_hz);
  const double fs2 = 4.0;  // 2 * fs
  const double warped = fs2 * std::tan(kPi * wn / 2.0);

  // analog prototype poles (unit cutoff), scaled to the warped cutoff, then bilinear-mapped
  std::vector<C> poles;
  C denominator = 1.0;
  for (int k = 0; k < order; ++k) {
    const double theta = kPi * (2.0 * k - order + 1) / (2.0 * order);
    const C p = -std::exp(C(0.0, theta)) * warped;
    denominator *= fs2 - p;
    poles.push_back((fs2 + p) / (fs2 - p));
  }
  const double gain = std::pow(warped, order) * (C(1.0) / denominator).real();

  // a = poly(poles); all zeros are at z = -1, so b = gain * binomial coefficients
  std::vector<C> a_c(order + 1, C(0.0));
  a_c[0] = 1.0;
  for (int i = 0; i < order; ++i) {
    for (int j = i + 1; j >= 1; --j) {a_c[j] -= poles[i] * a_c[j - 1];}
  }
  std::vector<double> a(order + 1), b(order + 1);
  double binomial = 1.0;
  for (int i = 0; i <= order; ++i) {
    a[i] = a_c[i].real();
    b[i] = gain * binomial;
    binomial = binomial * (order - i) / (i + 1);
  }
  return {b, a};
}

}  // namespace wrench_filter

#endif  // WRENCH_FILTER__BUTTERWORTH_HPP_
