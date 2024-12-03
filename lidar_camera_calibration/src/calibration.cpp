#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>

class CalibrationOptimizer {
public:
    struct ReprojectionError {
        ReprojectionError(const cv::Point2f& observed, const cv::Point3f& point)
            : observed_(observed), point_(point) {}

        template <typename T>
        bool operator()(const T* const axis_angle,
                        const T* const translation,
                        const T* const intrinsics,
                        const T* const dist_params,
                        T* residuals) const {
            const T fx = intrinsics[0];
            const T fy = intrinsics[1];
            const T cx = intrinsics[2];
            const T cy = intrinsics[3];
            const T k1 = dist_params[0];
            const T k2 = dist_params[1];
            const T p1 = dist_params[2];
            const T p2 = dist_params[3];
            const T k3 = dist_params[4];

            Eigen::Matrix<T, 3, 1> point_3d;
            point_3d << T(point_.x), T(point_.y), T(point_.z);

            Eigen::Matrix<T, 3, 3> rotation_matrix;
            ceres::AngleAxisToRotationMatrix(axis_angle, rotation_matrix.data());

            Eigen::Matrix<T, 3, 1> translation_vec;
            translation_vec << translation[0], translation[1], translation[2];

            Eigen::Matrix<T, 3, 1> transformed_point = rotation_matrix * point_3d + translation_vec;
            
            T x = transformed_point(0) / transformed_point(2);
            T y = transformed_point(1) / transformed_point(2);

            T r2 = x * x + y * y;
            T r4 = r2 * r2;
            T r6 = r4 * r2;

            T radial_dist = T(1) + k1 * r2 + k2 * r4 + k3 * r6;
            T tangential_x = T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
            T tangential_y = p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;

            T x_dist = x * radial_dist + tangential_x;
            T y_dist = y * radial_dist + tangential_y;

            T projected_x = fx * x_dist + cx;
            T projected_y = fy * y_dist + cy;

            residuals[0] = projected_x - T(observed_.x);
            residuals[1] = projected_y - T(observed_.y);
            return true;
        }

    private:
        cv::Point2f observed_;
        cv::Point3f point_;
    };

    static std::string MatrixToString(const Eigen::Matrix4d& mat) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                ss << std::setw(10) << mat(i,j) << " ";
            }
            ss << "\n";
        }
        return ss.str();
    }

    static void OptimizeTransform(
        double* axis_angle, 
        double* translation, 
        double* intrinsics, 
        double* dist_params,
        const std::vector<cv::Point2f>& points2D,
        const std::vector<cv::Point3f>& points3D,
        rclcpp::Node* node
    ) {
        ceres::Problem problem;
        for (size_t i = 0; i < points2D.size(); ++i) {
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 4, 5>(
                    new ReprojectionError(points2D[i], points3D[i])
                );
            problem.AddResidualBlock(cost_function, nullptr, axis_angle, translation, intrinsics, dist_params);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 200;
        
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (node) {
            RCLCPP_INFO(node->get_logger(), "Optimization Summary:");
            RCLCPP_INFO(node->get_logger(), "Final cost: %f", summary.final_cost);
            RCLCPP_INFO(node->get_logger(), "Iterations: %d", (int)summary.iterations.size());
            
            RCLCPP_INFO(node->get_logger(), "Optimized Transformation:");
            RCLCPP_INFO(node->get_logger(), "Axis Angle: [%f, %f, %f]", 
                        axis_angle[0], axis_angle[1], axis_angle[2]);
            RCLCPP_INFO(node->get_logger(), "Translation: [%f, %f, %f]", 
                        translation[0], translation[1], translation[2]);
        }
    }
};

class LiDARCameraCalibration : public rclcpp::Node {
private:
    struct CalibrationData {
        std::vector<cv::Point2f> points2D;
        std::vector<cv::Point3f> points3D;
        std::vector<cv::Point2f> inlier_points2D;
        std::vector<cv::Point3f> inlier_points3D;
        std::vector<int> inliers;
        cv::Mat rvec, tvec, K, dist_coeffs;
        std::string camera_base_frame;
        std::string camera_sensor_frame;
        std::string lidar_base_frame;
        std::string lidar_sensor_frame;
        Eigen::Matrix4d T_camera_base_to_sensor;
        Eigen::Matrix4d T_lidar_base_to_sensor;
        Eigen::Matrix4d T_lidar_sensor_to_camera_sensor;
    };

    CalibrationData calibration_data_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> transform_broadcaster_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr transform_timer_;

    void LoadCorrespondences(const std::string& path) {
        RCLCPP_INFO(get_logger(), "Loading correspondences from %s", path.c_str());
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open correspondence file: %s", path.c_str());
            return;
        }

        double px, py, X, Y, Z;
        while (file >> px >> py >> X >> Y >> Z) {
            calibration_data_.points2D.emplace_back(px, py);
            calibration_data_.points3D.emplace_back(X, Y, Z);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu point correspondences", calibration_data_.points2D.size());
    }

    void LoadCameraMatrix(const std::string& path) {
        RCLCPP_INFO(get_logger(), "Loading camera matrix from %s", path.c_str());
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera matrix file: %s", path.c_str());
            return;
        }

        calibration_data_.K = cv::Mat::zeros(3, 3, CV_64F);
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                file >> calibration_data_.K.at<double>(i, j);
            }
        }

        Eigen::Matrix4d camera_matrix_eigen = Eigen::Matrix4d::Zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                camera_matrix_eigen(i, j) = calibration_data_.K.at<double>(i, j);
            }
        }
        RCLCPP_INFO(get_logger(), "Camera Matrix:\n%s", 
            CalibrationOptimizer::MatrixToString(camera_matrix_eigen).c_str());
    }

    void LoadDistortionCoefficients(const std::string& path) {
        RCLCPP_INFO(get_logger(), "Loading distortion coefficients from %s", path.c_str());
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open distortion coefficients file: %s", path.c_str());
            return;
        }

        calibration_data_.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
        
        for (int i = 0; i < 5; ++i) {
            file >> calibration_data_.dist_coeffs.at<double>(0, i);
        }

        std::stringstream coeffs_str;
        for (int i = 0; i < 5; ++i) {
            coeffs_str << calibration_data_.dist_coeffs.at<double>(0, i) << " ";
        }
        RCLCPP_INFO(get_logger(), "Distortion Coefficients: %s", coeffs_str.str().c_str());
    }

    Eigen::Matrix4d ConvertTransformToEigenMatrix(const geometry_msgs::msg::TransformStamped& transform) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(transform.transform.rotation.w, 
                              transform.transform.rotation.x, 
                              transform.transform.rotation.y, 
                              transform.transform.rotation.z);
        mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        mat(0, 3) = transform.transform.translation.x;
        mat(1, 3) = transform.transform.translation.y;
        mat(2, 3) = transform.transform.translation.z;
        
        RCLCPP_INFO(get_logger(), "Conversion to Eigen Matrix:\n%s", 
                    CalibrationOptimizer::MatrixToString(mat).c_str());
        
        return mat;
    }

    void LookupInitialTransforms() {
        constexpr std::chrono::seconds wait_for_transform_period_seconds(10);

        try {
            geometry_msgs::msg::TransformStamped camera_base_to_sensor = 
                tf_buffer_.lookupTransform(
                    calibration_data_.camera_base_frame, 
                    calibration_data_.camera_sensor_frame, 
                    tf2::TimePointZero,
                    wait_for_transform_period_seconds
                );
            
            geometry_msgs::msg::TransformStamped lidar_base_to_sensor = 
                tf_buffer_.lookupTransform(
                    calibration_data_.lidar_base_frame, 
                    calibration_data_.lidar_sensor_frame, 
                    tf2::TimePointZero,
                    wait_for_transform_period_seconds
                );

            calibration_data_.T_camera_base_to_sensor = ConvertTransformToEigenMatrix(camera_base_to_sensor);
            calibration_data_.T_lidar_base_to_sensor = ConvertTransformToEigenMatrix(lidar_base_to_sensor);

            RCLCPP_INFO(get_logger(), "Camera Base to Sensor Transform:\n%s", 
                        CalibrationOptimizer::MatrixToString(calibration_data_.T_camera_base_to_sensor).c_str());
            RCLCPP_INFO(get_logger(), "LiDAR Base to Sensor Transform:\n%s", 
                        CalibrationOptimizer::MatrixToString(calibration_data_.T_lidar_base_to_sensor).c_str());
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "Transform lookup failed after waiting: %s", e.what());
            throw std::runtime_error("Required transforms not available.");
        }
    }

    void PerformCalibration()
    {
        if (calibration_data_.points3D.empty() || calibration_data_.points2D.empty()) {
            RCLCPP_WARN(get_logger(), "No correspondence points available for calibration");
            return;
        }

        LookupInitialTransforms();

        cv::solvePnPRansac(
            calibration_data_.points3D, 
            calibration_data_.points2D, 
            calibration_data_.K, 
            calibration_data_.dist_coeffs, 
            calibration_data_.rvec, 
            calibration_data_.tvec, 
            false, 10000, 2.0, 0.999, 
            calibration_data_.inliers
        );

        RCLCPP_INFO(get_logger(), "RANSAC found %zu inliers out of %zu points", 
                    calibration_data_.inliers.size(), 
                    calibration_data_.points2D.size());

        for (int idx : calibration_data_.inliers) {
            calibration_data_.inlier_points2D.push_back(calibration_data_.points2D[idx]);
            calibration_data_.inlier_points3D.push_back(calibration_data_.points3D[idx]);
        }

        cv::solvePnPRefineLM(
            calibration_data_.inlier_points3D, 
            calibration_data_.inlier_points2D, 
            calibration_data_.K, 
            calibration_data_.dist_coeffs, 
            calibration_data_.rvec, 
            calibration_data_.tvec
        );

        double axis_angle[3] = {
            calibration_data_.rvec.at<double>(0), 
            calibration_data_.rvec.at<double>(1), 
            calibration_data_.rvec.at<double>(2)
        };
        
        double translation[3] = {
            calibration_data_.tvec.at<double>(0), 
            calibration_data_.tvec.at<double>(1), 
            calibration_data_.tvec.at<double>(2)
        };
        
        double intrinsics[4] = {
            calibration_data_.K.at<double>(0,0), 
            calibration_data_.K.at<double>(1,1), 
            calibration_data_.K.at<double>(0,2), 
            calibration_data_.K.at<double>(1,2)
        };
        
        double dist_params[5] = {
            calibration_data_.dist_coeffs.at<double>(0), 
            calibration_data_.dist_coeffs.at<double>(1), 
            calibration_data_.dist_coeffs.at<double>(2), 
            calibration_data_.dist_coeffs.at<double>(3), 
            calibration_data_.dist_coeffs.at<double>(4)
        };

        CalibrationOptimizer::OptimizeTransform(
            axis_angle, translation, intrinsics, dist_params,
            calibration_data_.inlier_points2D, 
            calibration_data_.inlier_points3D,
            this
        );

        cv::Mat rotation_matrix;
        cv::Rodrigues(calibration_data_.rvec, rotation_matrix);
        
        Eigen::Matrix4d T_lidar_sensor_to_camera_sensor = Eigen::Matrix4d::Identity();
        
        Eigen::Matrix3d eigen_rotation;
        cv::cv2eigen(rotation_matrix, eigen_rotation);
        
        T_lidar_sensor_to_camera_sensor.block<3, 3>(0, 0) = eigen_rotation;
        T_lidar_sensor_to_camera_sensor.block<3, 1>(0, 3) << 
            calibration_data_.tvec.at<double>(0),
            calibration_data_.tvec.at<double>(1),
            calibration_data_.tvec.at<double>(2);

        calibration_data_.T_lidar_sensor_to_camera_sensor = T_lidar_sensor_to_camera_sensor;

        RCLCPP_INFO(get_logger(), "LiDAR Sensor to Camera Sensor Transform:\n%s", 
                    CalibrationOptimizer::MatrixToString(T_lidar_sensor_to_camera_sensor).c_str());

        transform_stamped_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
        transform_stamped_->header.stamp = this->get_clock()->now();
        transform_stamped_->header.frame_id = calibration_data_.lidar_sensor_frame;
        transform_stamped_->child_frame_id = calibration_data_.camera_sensor_frame;
        
        Eigen::Quaterniond q(eigen_rotation);
        transform_stamped_->transform.rotation.x = q.x();
        transform_stamped_->transform.rotation.y = q.y();
        transform_stamped_->transform.rotation.z = q.z();
        transform_stamped_->transform.rotation.w = q.w();

        transform_stamped_->transform.translation.x = translation[0];
        transform_stamped_->transform.translation.y = translation[1];
        transform_stamped_->transform.translation.z = translation[2];
    }

    void PublishTransform()
    {
        transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "Publishing calibration transforms");
        Eigen::Matrix4d T_lidar_base_to_camera_base = 
            calibration_data_.T_lidar_base_to_sensor.inverse() * 
            calibration_data_.T_lidar_sensor_to_camera_sensor * 
            calibration_data_.T_camera_base_to_sensor;
        
        RCLCPP_INFO(get_logger(), "Final LiDAR Base to Camera Base Transform:\n%s", 
                    CalibrationOptimizer::MatrixToString(T_lidar_base_to_camera_base).c_str());

        geometry_msgs::msg::TransformStamped final_transform = 
            ConvertEigenMatrixToTransform(
                T_lidar_base_to_camera_base, 
                calibration_data_.lidar_base_frame, 
                calibration_data_.camera_base_frame
            );
        
        transform_broadcaster_->sendTransform(final_transform);
        RCLCPP_INFO(get_logger(), "Sent transform from %s to %s", 
                    calibration_data_.lidar_base_frame.c_str(), 
                    calibration_data_.camera_base_frame.c_str());
    }
    geometry_msgs::msg::TransformStamped ConvertEigenMatrixToTransform(
            const Eigen::Matrix4d& matrix, 
            const std::string& parent_frame, 
            const std::string& child_frame
        ) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = this->get_clock()->now();
            transform_stamped.header.frame_id = parent_frame;
            transform_stamped.child_frame_id = child_frame;

            Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);
            Eigen::Quaterniond q(rotation_matrix);

            transform_stamped.transform.rotation.x = q.x();
            transform_stamped.transform.rotation.y = q.y();
            transform_stamped.transform.rotation.z = q.z();
            transform_stamped.transform.rotation.w = q.w();

            transform_stamped.transform.translation.x = matrix(0, 3);
            transform_stamped.transform.translation.y = matrix(1, 3);
            transform_stamped.transform.translation.z = matrix(2, 3);

            return transform_stamped;
        }

    void SaveCalibrationData(const std::string& output_file) {
        std::ofstream file(output_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open file: %s", output_file.c_str());
            return;
        }

        file << "LiDAR Sensor to Camera Sensor Transformation Matrix:\n";
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                file << calibration_data_.T_lidar_sensor_to_camera_sensor(i, j) << " ";
            }
            file << "\n";
        }

        file << "\nCamera Matrix (K):\n";
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                file << calibration_data_.K.at<double>(i, j) << " ";
            }
            file << "\n";
        }

        file << "\nDistortion Coefficients:\n";
        for (int i = 0; i < 5; ++i) {
            file << calibration_data_.dist_coeffs.at<double>(0, i) << " ";
        }
        file << "\n";

        file.close();
        RCLCPP_INFO(get_logger(), "Calibration data saved to: %s", output_file.c_str());
    }


public:
    LiDARCameraCalibration() : Node("lidar_camera_calibration"), 
                                tf_buffer_(get_clock()), 
                                tf_listener_(tf_buffer_) {
        RCLCPP_INFO(get_logger(), "Initializing LiDAR-Camera Calibration Node");

        auto correspondence_path = declare_parameter<std::string>("correspondence_path");
        auto k_matrix_path = declare_parameter<std::string>("k_matrix_path");
        auto dist_coeffs_path = declare_parameter<std::string>("dist_coeffs_path");
        auto output_path = declare_parameter<std::string>("output_path");
        
        calibration_data_.camera_base_frame = declare_parameter<std::string>("camera_base_frame");
        calibration_data_.camera_sensor_frame = declare_parameter<std::string>("camera_sensor_frame");
        calibration_data_.lidar_base_frame = declare_parameter<std::string>("lidar_base_frame");
        calibration_data_.lidar_sensor_frame = declare_parameter<std::string>("lidar_sensor_frame");

        RCLCPP_INFO(get_logger(), "Camera Base Frame: %s", calibration_data_.camera_base_frame.c_str());
        RCLCPP_INFO(get_logger(), "Camera Sensor Frame: %s", calibration_data_.camera_sensor_frame.c_str());
        RCLCPP_INFO(get_logger(), "LiDAR Base Frame: %s", calibration_data_.lidar_base_frame.c_str());
        RCLCPP_INFO(get_logger(), "LiDAR Sensor Frame: %s", calibration_data_.lidar_sensor_frame.c_str());

        LoadCorrespondences(correspondence_path);
        LoadCameraMatrix(k_matrix_path);
        LoadDistortionCoefficients(dist_coeffs_path);
        PerformCalibration();
        PublishTransform();
        SaveCalibrationData(output_path);

        RCLCPP_INFO(get_logger(), "Node initialization complete");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<LiDARCameraCalibration>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Unhandled exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}