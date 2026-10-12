// Copyright (c) 2026 Yusaku Nakajima, MIT License

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "wrench_filter/butterworth.hpp"
#include "wrench_filter/iir_filter.hpp"

using wrench_filter::butterworth_lowpass_sos;
using wrench_filter::IirFilter;
using wrench_filter::Section;

namespace
{

std::vector<double> convolve(const std::vector<double> & x, const std::vector<double> & y)
{
  std::vector<double> r(x.size() + y.size() - 1, 0.0);
  for (size_t i = 0; i < x.size(); ++i) {
    for (size_t j = 0; j < y.size(); ++j) {r[i + j] += x[i] * y[j];}
  }
  return r;
}

struct Case
{
  int order;
  double cutoff_hz;
  double sampling_hz;
  std::vector<double> b;
  std::vector<double> a;
};

// references: scipy 1.15.3, scipy.signal.butter(order, cutoff_hz / (0.5 * sampling_hz)).
// Only configurations whose transfer-function form is well conditioned are compared this way.
const std::vector<Case> kCases{
  {3, 20.0, 500.0,
    {0.0015670103505882685, 0.004701031051764806, 0.004701031051764806, 0.0015670103505882685},
    {1.0, -2.4986083446911773, 2.1152541270031584, -0.6041096995072747}},
  {3, 2.5, 500.0,
    {3.7568380197512637e-06, 1.1270514059253792e-05, 1.1270514059253792e-05,
      3.7568380197512637e-06},
    {1.0, -2.9371707284498907, 2.8762997234793315, -0.939098940325283}},
  {1, 10.0, 100.0, {0.24523727525278557, 0.24523727525278557}, {1.0, -0.5095254494944288}},
  {2, 50.0, 1000.0, {0.020083365564211232, 0.040166731128422464, 0.020083365564211232},
    {1.0, -1.5610180758007182, 0.6413515380575631}},
  {5, 150.0, 500.0,
    {0.10837370258747996, 0.5418685129373998, 1.0837370258747996, 1.0837370258747996,
      0.5418685129373998, 0.10837370258747996},
    {1.0, 0.9853252392792378, 0.9738493318367639, 0.3863565586484487, 0.11116384057834201,
      0.011263512456565873}},
};

// the response of the cascade to a constant input after `samples` samples
double cascade_step(const std::vector<Section> & sections, int samples)
{
  std::vector<IirFilter> filters;
  for (const auto & s : sections) {filters.emplace_back(s.b, s.a);}
  double y = 0.0;
  for (int i = 0; i < samples; ++i) {
    y = 1.0;
    for (auto & f : filters) {y = f.filter(y);}
  }
  return y;
}

}  // namespace

TEST(Butterworth, CascadeMatchesScipyTransferFunction)
{
  for (const auto & c : kCases) {
    const auto sections = butterworth_lowpass_sos(c.order, c.cutoff_hz, c.sampling_hz);
    ASSERT_EQ(sections.size(), static_cast<size_t>((c.order + 1) / 2));
    std::vector<double> b{1.0}, a{1.0};
    for (const auto & s : sections) {
      b = convolve(b, s.b);
      a = convolve(a, s.a);
    }
    ASSERT_EQ(b.size(), c.b.size());
    for (size_t i = 0; i < b.size(); ++i) {
      EXPECT_NEAR(b[i], c.b[i], 1e-9 * std::abs(c.b[i])) << "order " << c.order << " b" << i;
      EXPECT_NEAR(a[i], c.a[i], 1e-9 * std::max(1.0, std::abs(c.a[i])))
        << "order " << c.order << " a" << i;
    }
  }
}

TEST(Butterworth, UnityDcGainOverTheWholeAllowedRange)
{
  // includes configurations where the transfer-function form fails
  // (order 6, 0.5 Hz at 500 Hz: DC gain 0.985; order 8, 1 Hz: DC gain 2.2)
  for (int order = 1; order <= wrench_filter::kMaxButterworthOrder; ++order) {
    for (double cutoff_hz : {0.1, 0.5, 1.0, 2.5, 5.0, 20.0, 200.0}) {
      for (const auto & s : butterworth_lowpass_sos(order, cutoff_hz, 500.0)) {
        double sb = 0.0, sa = 0.0;
        for (double v : s.b) {sb += v;}
        for (double v : s.a) {sa += v;}
        EXPECT_NEAR(sb / sa, 1.0, 1e-12) << "order " << order << ", " << cutoff_hz << " Hz";
        EXPECT_TRUE(IirFilter::is_stable(s.a)) << "order " << order << ", " << cutoff_hz;
      }
    }
  }
}

TEST(Butterworth, CascadeSettlesToDcInput)
{
  // order 6, 0.5 Hz at 500 Hz is the configuration the transfer-function form got 1.5 % wrong
  EXPECT_NEAR(cascade_step(butterworth_lowpass_sos(6, 0.5, 500.0), 20000), 1.0, 1e-9);
  EXPECT_NEAR(cascade_step(butterworth_lowpass_sos(8, 0.1, 500.0), 100000), 1.0, 1e-9);
  EXPECT_NEAR(cascade_step(butterworth_lowpass_sos(3, 20.0, 500.0), 2000), 1.0, 1e-12);
}

TEST(Butterworth, RejectsInvalidArguments)
{
  EXPECT_THROW(butterworth_lowpass_sos(0, 2.5, 500.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass_sos(9, 2.5, 500.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass_sos(3, 0.0, 500.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass_sos(3, 250.0, 500.0), std::invalid_argument);  // Nyquist
  EXPECT_THROW(butterworth_lowpass_sos(3, 2.5, 0.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass_sos(3, 2.5, NAN), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass_sos(3, NAN, 500.0), std::invalid_argument);
}
