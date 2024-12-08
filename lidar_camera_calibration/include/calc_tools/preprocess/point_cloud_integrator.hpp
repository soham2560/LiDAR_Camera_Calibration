#pragma once

#include <calc_tools/common/frame.hpp>

namespace calc_tools {

class PointCloudIntegrator {
public:
  virtual ~PointCloudIntegrator() {}

  virtual void insert_points(const Frame::ConstPtr& raw_points) = 0;
  virtual Frame::ConstPtr get_points() = 0;
};

}