#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "perception/yolo_postprocess.hpp"

namespace
{

cv::Mat DarkTrafficLightRoi()
{
  return cv::Mat(46, 22, CV_8UC3, cv::Scalar(20, 20, 20));
}

}  // namespace

TEST(PredictedRoiGreenLampScore, AcceptsCompactLowerGreenBulb)
{
  cv::Mat image = DarkTrafficLightRoi();
  cv::circle(image, cv::Point(11, 31), 2, cv::Scalar(0, 255, 0), -1);

  EXPECT_GE(perception::yolo::PredictedRoiGreenLampScore(image), 30);
}


TEST(PredictedRoiGreenLampScore, RejectsRedBulb)
{
  cv::Mat image = DarkTrafficLightRoi();
  cv::circle(image, cv::Point(11, 14), 3, cv::Scalar(0, 0, 255), -1);

  EXPECT_EQ(perception::yolo::PredictedRoiGreenLampScore(image), 0);
}

TEST(PredictedRoiGreenLampScore, RejectsGreenRegionTouchingCropBoundary)
{
  cv::Mat image = DarkTrafficLightRoi();
  cv::rectangle(image, cv::Rect(17, 25, 5, 8), cv::Scalar(0, 255, 0), -1);

  EXPECT_EQ(perception::yolo::PredictedRoiGreenLampScore(image), 0);
}

TEST(PredictedRoiGreenLampScore, RejectsLargeGreenRoadSign)
{
  cv::Mat image = DarkTrafficLightRoi();
  cv::rectangle(image, cv::Rect(3, 22, 16, 18), cv::Scalar(0, 255, 0), -1);

  EXPECT_EQ(perception::yolo::PredictedRoiGreenLampScore(image), 0);
}
