#include <cstdint>

#include "gtest/gtest.h"

#include "perception/traffic_light_stability_filter.hpp"

namespace
{

constexpr std::uint8_t kUnknown = 0;
constexpr std::uint8_t kRed = 1;
constexpr std::uint8_t kGreen = 3;

TEST(TrafficLightStabilityFilter, ConfirmsUnambiguousGreenInTwoFrames)
{
  perception::TrafficLightStabilityFilter filter(4);

  EXPECT_EQ(filter.update(kGreen, 0, 40), kUnknown);
  EXPECT_EQ(filter.update(kGreen, 0, 40), kGreen);
}

TEST(TrafficLightStabilityFilter, AmbiguousGreenStillRequiresFourFrames)
{
  perception::TrafficLightStabilityFilter filter(4);

  EXPECT_EQ(filter.update(kGreen, 36, 48), kUnknown);
  EXPECT_EQ(filter.update(kGreen, 36, 48), kUnknown);
  EXPECT_EQ(filter.update(kGreen, 36, 48), kUnknown);
  EXPECT_EQ(filter.update(kGreen, 36, 48), kGreen);
}

TEST(TrafficLightStabilityFilter, NonGreenImmediatelyRevokesPermission)
{
  perception::TrafficLightStabilityFilter filter(4);
  filter.update(kGreen, 0, 40);
  filter.update(kGreen, 0, 40);
  filter.update(kGreen, 0, 40);
  ASSERT_EQ(filter.update(kGreen, 0, 40), kGreen);

  EXPECT_EQ(filter.update(kRed, 40, 0), kRed);
}

TEST(TrafficLightStabilityFilter, ConfidentFramesMustBeConsecutive)
{
  perception::TrafficLightStabilityFilter filter(4);

  EXPECT_EQ(filter.update(kGreen, 0, 40), kUnknown);
  EXPECT_EQ(filter.update(kUnknown, 0, 0), kUnknown);
  EXPECT_EQ(filter.update(kGreen, 0, 40), kUnknown);
  EXPECT_EQ(filter.update(kGreen, 0, 40), kGreen);
}

}  // namespace
