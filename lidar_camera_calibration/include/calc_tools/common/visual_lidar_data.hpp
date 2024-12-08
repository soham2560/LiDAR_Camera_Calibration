#pragma once

#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <calc_tools/common/frame_cpu.hpp>

namespace calc_tools {

struct VisualLiDARData {
public:
  using Ptr = std::shared_ptr<VisualLiDARData>;
  using ConstPtr = std::shared_ptr<const VisualLiDARData>;

  VisualLiDARData() {}
  VisualLiDARData(const cv::Mat& image, const FrameCPU::Ptr& points) : image(image), points(points) {}
  VisualLiDARData(const std::string& data_path, const std::string& bag_name);
  ~VisualLiDARData();

public:
  cv::Mat image;
  FrameCPU::Ptr points;
};

}  // namespace calc_tools
