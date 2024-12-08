#pragma once

#include <random>
#include <thread>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <calc_tools/common/frame_cpu.hpp>
#include <calc_tools/common/vector3i_hash.hpp>
#include <calc_tools/common/concurrent_queue.hpp>
#include <calc_tools/preprocess/point_cloud_integrator.hpp>


namespace calc_tools {

/**
 * @brief Static LiDAR point integrator for non-repetitive scan LiDARs
 */
struct StaticPointCloudIntegratorParams {
public:
  StaticPointCloudIntegratorParams();
  ~StaticPointCloudIntegratorParams();

  bool visualize;
  double voxel_resolution;
  double min_distance;
};

class StaticPointCloudIntegrator : public PointCloudIntegrator {
public:
  StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params = StaticPointCloudIntegratorParams());
  ~StaticPointCloudIntegrator();

  virtual void insert_points(const Frame::ConstPtr& raw_points) override;
  virtual Frame::ConstPtr get_points() override;

private:
  const StaticPointCloudIntegratorParams params;

  std::unordered_map<Eigen::Vector3i, Eigen::Vector4d, Vector3iHash> voxelgrid;
};
}