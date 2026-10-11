// Copyright (c) 2026 Yusaku Nakajima, MIT License

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "wrench_filter/iir_filter.hpp"

using wrench_filter::IirFilter;

// scipy.signal.butter(3, 20 / 250)  (fs = 500 Hz, fc = 20 Hz)
const std::vector<double> kB{
  0.0015670103505882685, 0.004701031051764806, 0.004701031051764806, 0.0015670103505882685};
const std::vector<double> kA{1.0, -2.4986083446911773, 2.1152541270031584, -0.6041096995072747};

TEST(IirFilter, ConvergesToDcInput)
{
  IirFilter f(kB, kA);
  double y = 0.0;
  for (int i = 0; i < 2000; ++i) {y = f.filter(10.0);}
  EXPECT_NEAR(y, 10.0, 1e-6);
}

TEST(IirFilter, MatchesScipyLfilterStepResponse)
{
  // scipy.signal.lfilter(kB, kA, np.ones(4))
  const std::vector<double> expected{
    0.0015670103505882685, 0.01018338654055037, 0.03309874193041435, 0.0746430713360703};
  IirFilter f(kB, kA);
  for (double e : expected) {EXPECT_NEAR(f.filter(1.0), e, 1e-9);}
}

TEST(IirFilter, NormalizesByA0)
{
  std::vector<double> b2, a2;
  for (double v : kB) {b2.push_back(2.0 * v);}
  for (double v : kA) {a2.push_back(2.0 * v);}
  IirFilter f1(kB, kA), f2(b2, a2);
  for (int i = 0; i < 50; ++i) {EXPECT_DOUBLE_EQ(f1.filter(1.0 + i), f2.filter(1.0 + i));}
}

TEST(IirFilter, RejectsInvalidCoefficients)
{
  EXPECT_THROW(IirFilter({1.0}, {}), std::invalid_argument);
  EXPECT_THROW(IirFilter({1.0, 1.0}, {0.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(IirFilter({1.0}, {1.0, 0.5}), std::invalid_argument);
  EXPECT_THROW(IirFilter({NAN, 1.0}, {1.0, 0.5}), std::invalid_argument);
}

TEST(IirFilter, RejectsUnstableDenominator)
{
  // pole at z = 1.1
  EXPECT_THROW(IirFilter({1.0, 0.0}, {1.0, -1.1}), std::invalid_argument);
  // poles at z = 0.5 +/- 1.0j (|z| = 1.118)
  EXPECT_THROW(IirFilter({1.0, 0.0, 0.0}, {1.0, -1.0, 1.25}), std::invalid_argument);
  // pole on the unit circle (integrator)
  EXPECT_THROW(IirFilter({1.0, 0.0}, {1.0, -1.0}), std::invalid_argument);
}

TEST(IirFilter, StabilityTest)
{
  EXPECT_TRUE(IirFilter::is_stable(kA));
  EXPECT_TRUE(IirFilter::is_stable({1.0, -0.9}));
  EXPECT_TRUE(IirFilter::is_stable({1.0, -1.0, 0.89}));  // |z| = 0.943
  EXPECT_FALSE(IirFilter::is_stable({1.0, -2.0, 1.0}));  // double pole at z = 1
  EXPECT_FALSE(IirFilter::is_stable({}));
  // poles at 0.5, 0.5, 1.2
  EXPECT_FALSE(IirFilter::is_stable({1.0, -2.2, 1.45, -0.3}));
}
