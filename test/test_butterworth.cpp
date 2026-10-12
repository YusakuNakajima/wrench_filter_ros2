// Copyright (c) 2026 Yusaku Nakajima, MIT License

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "wrench_filter/butterworth.hpp"

using wrench_filter::butterworth_lowpass;

namespace
{

struct Case
{
  int order;
  double cutoff_hz;
  double sampling_hz;
  std::vector<double> b;
  std::vector<double> a;
};

// references: scipy 1.15.3, scipy.signal.butter(order, cutoff_hz / (0.5 * sampling_hz))
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
  {8, 5.0, 500.0,
    {8.098259786747707e-13, 6.478607829398165e-12, 2.267512740289358e-11, 4.535025480578716e-11,
      5.6687818507233944e-11, 4.535025480578716e-11, 2.267512740289358e-11,
      6.478607829398165e-12, 8.098259786747707e-13},
    {1.0, -7.677940205392836, 25.797219528171233, -49.54122563778755, 59.47613197003973,
      -45.7087344779167, 21.960120132116103, -6.0301722352443194, 0.7246009262216517}},
};

}  // namespace

TEST(Butterworth, MatchesScipy)
{
  for (const auto & c : kCases) {
    const auto [b, a] = butterworth_lowpass(c.order, c.cutoff_hz, c.sampling_hz);
    ASSERT_EQ(b.size(), c.b.size());
    ASSERT_EQ(a.size(), c.a.size());
    for (size_t i = 0; i < b.size(); ++i) {
      // relative tolerance: the order-8 coefficients span 1e-13 .. 60
      EXPECT_NEAR(b[i], c.b[i], 1e-9 * std::abs(c.b[i])) << "order " << c.order << " b" << i;
      EXPECT_NEAR(a[i], c.a[i], 1e-9 * std::max(1.0, std::abs(c.a[i])))
        << "order " << c.order << " a" << i;
    }
  }
}

TEST(Butterworth, UnityDcGain)
{
  for (const auto & c : kCases) {
    const auto [b, a] = butterworth_lowpass(c.order, c.cutoff_hz, c.sampling_hz);
    double sb = 0.0, sa = 0.0;
    for (double v : b) {sb += v;}
    for (double v : a) {sa += v;}
    // a high order with a low cutoff is ill-conditioned in transfer-function form
    // (order 8, 5 Hz at 500 Hz: sum(a) ~ 2e-10; scipy's own coefficients give 1 + 3e-6)
    const double tolerance = c.order <= 5 ? 1e-6 : 1e-4;
    EXPECT_NEAR(sb / sa, 1.0, tolerance) << "order " << c.order;
  }
}

TEST(Butterworth, RejectsInvalidArguments)
{
  EXPECT_THROW(butterworth_lowpass(0, 2.5, 500.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass(9, 2.5, 500.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass(3, 0.0, 500.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass(3, 250.0, 500.0), std::invalid_argument);  // at Nyquist
  EXPECT_THROW(butterworth_lowpass(3, 2.5, 0.0), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass(3, 2.5, NAN), std::invalid_argument);
  EXPECT_THROW(butterworth_lowpass(3, NAN, 500.0), std::invalid_argument);
}
