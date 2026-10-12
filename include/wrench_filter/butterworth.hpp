// Copyright (c) 2026 Yusaku Nakajima, MIT License
//
// Digital Butterworth low-pass design as second-order sections (SOS), with the same poles,
// zeros and overall gain as
//   scipy.signal.butter(order, cutoff_hz / (0.5 * sampling_hz), output="sos")
// (analog prototype -> pre-warped cutoff -> bilinear transform).
//
// The transfer-function form (one polynomial b / a) is not used: with a high order and a low
// cutoff its coefficients lose precision (e.g. order 6, 0.5 Hz at 500 Hz: DC gain 0.985).
// Each section here has its DC gain set to exactly 1 from its own coefficients.

#ifndef WRENCH_FILTER__BUTTERWORTH_HPP_
#define WRENCH_FILTER__BUTTERWORTH_HPP_

#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>

namespace wrench_filter
{

constexpr int kMaxButterworthOrder = 8;

struct Section
{
  std::vector<double> b;  // numerator
  std::vector<double> a;  // denominator, a[0] == 1
};

// Returns ceil(order / 2) sections: second-order ones (a conjugate pole pair, double zero at
// z = -1) and, for an odd order, one first-order section (real pole, zero at z = -1).
// sampling_hz: rate at which samples are fed to the filter [Hz].
inline std::vector<Section> butterworth_lowpass_sos(
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

  // analog prototype pole k: -exp(j * pi * (2k - order + 1) / (2 order)), scaled and mapped.
  // k and order - 1 - k are a conjugate pair; for an odd order, k = (order - 1) / 2 is real.
  auto digital_pole = [&](int k) {
      const double theta = kPi * (2.0 * k - order + 1) / (2.0 * order);
      const C p = -std::exp(C(0.0, theta)) * warped;
      return (fs2 + p) / (fs2 - p);
    };

  std::vector<Section> sections;
  for (int k = 0; k < order / 2; ++k) {
    const C z = digital_pole(k);
    Section s;
    s.a = {1.0, -2.0 * z.real(), std::norm(z)};
    const double dc = (s.a[0] + s.a[1] + s.a[2]) / 4.0;  // b = dc * [1, 2, 1] -> unity DC gain
    s.b = {dc, 2.0 * dc, dc};
    sections.push_back(s);
  }
  if (order % 2 == 1) {
    const double z = digital_pole((order - 1) / 2).real();
    Section s;
    s.a = {1.0, -z};
    const double dc = (1.0 - z) / 2.0;  // b = dc * [1, 1]
    s.b = {dc, dc};
    sections.push_back(s);
  }
  return sections;
}

}  // namespace wrench_filter

#endif  // WRENCH_FILTER__BUTTERWORTH_HPP_
