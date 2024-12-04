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

class LiDARCameraCalibration : public rclcpp::Node {
private:
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat cameraMatrix;
    std::vector<cv::Point2f> points2D;
    std::vector<cv::Point3f> points3D;
    std::string camera_base_frame;
    std::string camera_sensor_frame;
    std::string lidar_base_frame;
    std::string lidar_sensor_frame;
    Eigen::Matrix4d T_camera_base_to_sensor;
    Eigen::Matrix4d T_lidar_base_to_sensor;
    Eigen::Matrix4d T_lidar_sensor_to_camera_sensor;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> transform_broadcaster_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr transform_timer_;
    
    // Intermediate storage for optimization results
    double rvex[3] = {0, 0, 0};
    double tvec_data[3] = {0, 0, 0};
    double* translation = tvec_data;
    double axis_angle[3] = {0, 0, 0};

    void LoadCorrespondences(const std::string& path) {
        RCLCPP_INFO(get_logger(), "Loading correspondences from %s", path.c_str());
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open correspondence file: %s", path.c_str());
            return;
        }

        double px, py, X, Y, Z;
        while (file >> px >> py >> X >> Y >> Z) {
            points2D.emplace_back(px, py);
            points3D.emplace_back(X, Y, Z);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu point correspondences", points2D.size());
    }

    void LoadCameraMatrix(const std::string& path) {
        RCLCPP_INFO(get_logger(), "Loading camera matrix from %s", path.c_str());
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera matrix file: %s", path.c_str());
            return;
        }

        cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                file >> cameraMatrix.at<double>(i, j);
            }
        }

        Eigen::Matrix4d camera_matrix_eigen = Eigen::Matrix4d::Zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                camera_matrix_eigen(i, j) = cameraMatrix.at<double>(i, j);
            }
        }
        RCLCPP_INFO(get_logger(), "Camera Matrix:\n%s", 
            MatrixToString(camera_matrix_eigen).c_str());
    }

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

    Eigen::Matrix4d ConvertTransformToEigenMatrix(const geometry_msgs::msg::TransformStamped& transform)
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(transform.transform.rotation.w, 
                              transform.transform.rotation.x, 
                              transform.transform.rotation.y, 
                              transform.transform.rotation.z);
        mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        mat(0, 3) = transform.transform.translation.x;
        mat(1, 3) = transform.transform.translation.y;
        mat(2, 3) = transform.transform.translation.z;
        
        return mat;
    }

    geometry_msgs::msg::TransformStamped ConvertEigenMatrixToTransform(const Eigen::Matrix4d& matrix, const std::string& parent_frame, const std::string& child_frame) 
    {
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

    void LookupInitialTransforms() {
        constexpr std::chrono::seconds wait_for_transform_period_seconds(10);

        try {
            geometry_msgs::msg::TransformStamped camera_base_to_sensor = 
                tf_buffer_.lookupTransform(
                    camera_base_frame, 
                    camera_sensor_frame, 
                    tf2::TimePointZero,
                    wait_for_transform_period_seconds
                );
            
            geometry_msgs::msg::TransformStamped lidar_base_to_sensor = 
                tf_buffer_.lookupTransform(
                    lidar_base_frame, 
                    lidar_sensor_frame, 
                    tf2::TimePointZero,
                    wait_for_transform_period_seconds
                );

            T_camera_base_to_sensor = ConvertTransformToEigenMatrix(camera_base_to_sensor);
            T_lidar_base_to_sensor = ConvertTransformToEigenMatrix(lidar_base_to_sensor);

            RCLCPP_DEBUG(get_logger(), "Camera Base to Sensor Transform:\n%s", 
                        MatrixToString(T_camera_base_to_sensor).c_str());
            RCLCPP_DEBUG(get_logger(), "LiDAR Base to Sensor Transform:\n%s", 
                        MatrixToString(T_lidar_base_to_sensor).c_str());
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "Transform lookup failed after waiting: %s", e.what());
            throw std::runtime_error("Required transforms not available.");
        }
    }

    void PublishTransform()
    {
        transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "Publishing calibration transforms");
        Eigen::Matrix4d T_lidar_base_to_camera_base = 
            T_lidar_base_to_sensor.inverse() * 
            T_lidar_sensor_to_camera_sensor * 
            T_camera_base_to_sensor;
        
        RCLCPP_DEBUG(get_logger(), "Final LiDAR Base to Camera Base Transform:\n%s", 
                    MatrixToString(T_lidar_base_to_camera_base).c_str());

        geometry_msgs::msg::TransformStamped final_transform = 
            ConvertEigenMatrixToTransform(
                T_lidar_base_to_camera_base, 
                lidar_base_frame, 
                camera_base_frame
            );
        
        transform_broadcaster_->sendTransform(final_transform);
        RCLCPP_INFO(get_logger(), "Sent transform from %s to %s", 
                    lidar_base_frame.c_str(), 
                    camera_base_frame.c_str());
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
                file << T_lidar_sensor_to_camera_sensor(i, j) << " ";
            }
            file << "\n";
        }

        file << "\nCamera Matrix (K):\n";
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                file << cameraMatrix.at<double>(i, j) << " ";
            }
            file << "\n";
        }

        file.close();
        RCLCPP_INFO(get_logger(), "Calibration data saved to: %s", output_file.c_str());
    }

    struct ReprojectionError {
        ReprojectionError(const cv::Point2f& observed, const cv::Point3f& point)
            : observed_(observed), point_(point) {}
        
        template <typename T>
        bool operator()(const T* const axis_angle,
                        const T* const translation,
                        const T* const intrinsics,
                        T* residuals) const {
            const T fx = intrinsics[0];
            const T fy = intrinsics[1];
            const T cx = intrinsics[2];
            const T cy = intrinsics[3];

            T point[3] = {T(point_.x), T(point_.y), T(point_.z)};
            T transformed_point[3];

            ceres::AngleAxisRotatePoint(axis_angle, point, transformed_point);
            transformed_point[0] += translation[0];
            transformed_point[1] += translation[1];
            transformed_point[2] += translation[2];

            T x = transformed_point[0] / transformed_point[2];
            T y = transformed_point[1] / transformed_point[2];
            T projected_x = fx * x + cx;
            T projected_y = fy * y + cy;

            residuals[0] = projected_x - T(observed_.x);
            residuals[1] = projected_y - T(observed_.y);
            return true;
        }

    private:
        cv::Point2f observed_;
        cv::Point3f point_;
    };

    void PerformCalibration()
    {
        if (points3D.empty() || points2D.empty()) {
            RCLCPP_WARN(get_logger(), "No correspondence points available for calibration");
            return;
        }
        estimateInitialPose();

        refineCalibration();

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        T_lidar_sensor_to_camera_sensor = Eigen::Matrix4d::Identity();
        
        Eigen::Matrix3d eigen_rotation;
        cv::cv2eigen(rotation_matrix, eigen_rotation);
        
        T_lidar_sensor_to_camera_sensor.block<3, 3>(0, 0) = eigen_rotation;
        T_lidar_sensor_to_camera_sensor.block<3, 1>(0, 3) << 
            tvec.at<double>(0),
            tvec.at<double>(1),
            tvec.at<double>(2);

        RCLCPP_INFO(get_logger(), "LiDAR Sensor to Camera Sensor Transform:\n%s", 
                    MatrixToString(T_lidar_sensor_to_camera_sensor).c_str());

        transform_stamped_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
        transform_stamped_->header.stamp = this->get_clock()->now();
        transform_stamped_->header.frame_id = lidar_sensor_frame;
        transform_stamped_->child_frame_id = camera_sensor_frame;
        
        Eigen::Quaterniond q(eigen_rotation);
        transform_stamped_->transform.rotation.x = q.x();
        transform_stamped_->transform.rotation.y = q.y();
        transform_stamped_->transform.rotation.z = q.z();
        transform_stamped_->transform.rotation.w = q.w();

        transform_stamped_->transform.translation.x = translation[0];
        transform_stamped_->transform.translation.y = translation[1];
        transform_stamped_->transform.translation.z = translation[2];
    }

    void estimateInitialPose() {
        if (points3D.size() < 4 || points3D.size() != points2D.size()) {
            throw std::runtime_error("Insufficient or mismatched point correspondences");
        }

        Eigen::MatrixXd A(2 * points3D.size(), 12);
        A.setZero();

        for (size_t i = 0; i < points3D.size(); ++i) {
            double X = points3D[i].x;
            double Y = points3D[i].y;
            double Z = points3D[i].z;
            
            double fx = cameraMatrix.at<double>(0, 0);
            double fy = cameraMatrix.at<double>(1, 1);
            double cx = cameraMatrix.at<double>(0, 2);
            double cy = cameraMatrix.at<double>(1, 2);

            double x = (points2D[i].x - cx) / fx;
            double y = (points2D[i].y - cy) / fy;

            A.block<2, 12>(2*i, 0) << 
                X, Y, Z, 1, 0, 0, 0, 0, -x*X, -x*Y, -x*Z, -x,
                0, 0, 0, 0, X, Y, Z, 1, -y*X, -y*Y, -y*Z, -y;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
        Eigen::VectorXd v = svd.matrixV().col(11);

        Eigen::Matrix<double, 3, 4> P;
        P.row(0) << v(0), v(1), v(2), v(3);
        P.row(1) << v(4), v(5), v(6), v(7);
        P.row(2) << v(8), v(9), v(10), v(11);

        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        
        Eigen::MatrixXd P_block = P.block<3,3>(0,0);
        Eigen::JacobiSVD<Eigen::MatrixXd> P_svd(P_block, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        R = P_svd.matrixU() * P_svd.matrixV().transpose();
        
        if (R.determinant() < 0) {
            R = -R;
        }

        Eigen::Vector3d singular_values = P_svd.singularValues();
        double scale = (singular_values(0) + singular_values(1)) / 2.0;
        
        t = P.block<3,1>(0,3) / scale;
        t = -R.transpose() * t;

        cv::Mat R_cv = cv::Mat(3, 3, CV_64F);
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_cv.at<double>(i, j) = R(i, j);
            }
        }

        cv::Rodrigues(R_cv, rvec);
        for (int i = 0; i < 3; ++i) {
            tvec.at<double>(i, 0) = t(i);
        }

        std::stringstream rvec_ss, tvec_ss;
        rvec_ss << "[" 
                << rvec.at<double>(0, 0) << ", " 
                << rvec.at<double>(1, 0) << ", " 
                << rvec.at<double>(2, 0) << "]";

        tvec_ss << "[" 
                << tvec.at<double>(0, 0) << ", " 
                << tvec.at<double>(1, 0) << ", " 
                << tvec.at<double>(2, 0) << "]";
        RCLCPP_INFO(get_logger(), "Estimated Rotation Vector: %s", rvec_ss.str().c_str());
        RCLCPP_INFO(get_logger(), "Estimated Translation Vector: %s", tvec_ss.str().c_str());
    }

    void refineCalibration()
    {
        ceres::Problem problem;
        
        double intrinsics[4] = {
            cameraMatrix.at<double>(0, 0),  // fx
            cameraMatrix.at<double>(1, 1),  // fy
            cameraMatrix.at<double>(0, 2),  // cx
            cameraMatrix.at<double>(1, 2)   // cy
        };
        for (int i = 0; i < 3; ++i) {
            rvex[i] = rvec.at<double>(i, 0);
            tvec_data[i] = tvec.at<double>(i, 0);
        }

        for (size_t i = 0; i < points2D.size(); ++i) {
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 4>(
                    new ReprojectionError(points2D[i], points3D[i])
                );
            
            problem.AddResidualBlock(
                cost_function, 
                nullptr,
                rvex,
                tvec_data,
                intrinsics
            );
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 200;
        
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        RCLCPP_INFO(get_logger(), "Optimization Summary:");
        RCLCPP_INFO(get_logger(), "Final cost: %f", summary.final_cost);
        RCLCPP_INFO(get_logger(), "Iterations: %d", (int)summary.iterations.size());
        
        RCLCPP_INFO(get_logger(), "Optimized Transformation:");
        RCLCPP_INFO(get_logger(), "Axis Angle: [%f, %f, %f]", 
                    rvex[0], rvex[1], rvex[2]);
        RCLCPP_INFO(get_logger(), "Translation: [%f, %f, %f]", 
                    tvec_data[0], tvec_data[1], tvec_data[2]);
        for (int i = 0; i < 3; ++i) {
            rvec.at<double>(i, 0) = rvex[i];
            tvec.at<double>(i, 0) = tvec_data[i];
            axis_angle[i] = rvex[i];
            translation[i] = tvec_data[i];
        }

        computeReprojectionError();
    }

    void computeReprojectionError() {
        cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, projected_points);
        
        double total_error = 0.0;
        for (size_t i = 0; i < points3D.size(); ++i) {
            double err = cv::norm(points2D[i] - projected_points[i]);
            total_error += err;
        }
        double mean_reprojection_error = total_error / points3D.size();

        // Create a string representation of the rotation and translation vectors
        std::stringstream rvec_ss, tvec_ss;
        rvec_ss << "[" 
                << rvec.at<double>(0, 0) << ", " 
                << rvec.at<double>(1, 0) << ", " 
                << rvec.at<double>(2, 0) << "]";

        tvec_ss << "[" 
                << tvec.at<double>(0, 0) << ", " 
                << tvec.at<double>(1, 0) << ", " 
                << tvec.at<double>(2, 0) << "]";

        RCLCPP_INFO(get_logger(), "Mean Reprojection Error: %f pixels", mean_reprojection_error);
        RCLCPP_INFO(get_logger(), "Refined Rotation Vector: %s", rvec_ss.str().c_str());
        RCLCPP_INFO(get_logger(), "Refined Translation Vector: %s", tvec_ss.str().c_str());
    }

public:
    LiDARCameraCalibration() : Node("lidar_camera_calibration"), 
                                tf_buffer_(get_clock()), 
                                tf_listener_(tf_buffer_) {
        RCLCPP_INFO(get_logger(), "Initializing LiDAR-Camera Calibration Node");

        auto correspondence_path = declare_parameter<std::string>("correspondence_path");
        auto k_matrix_path = declare_parameter<std::string>("k_matrix_path");
        auto output_path = declare_parameter<std::string>("output_path");
        
        camera_base_frame = declare_parameter<std::string>("camera_base_frame");
        camera_sensor_frame = declare_parameter<std::string>("camera_sensor_frame");
        lidar_base_frame = declare_parameter<std::string>("lidar_base_frame");
        lidar_sensor_frame = declare_parameter<std::string>("lidar_sensor_frame");

        RCLCPP_INFO(get_logger(), "Camera Base Frame: %s", camera_base_frame.c_str());
        RCLCPP_INFO(get_logger(), "Camera Sensor Frame: %s", camera_sensor_frame.c_str());
        RCLCPP_INFO(get_logger(), "LiDAR Base Frame: %s", lidar_base_frame.c_str());
        RCLCPP_INFO(get_logger(), "LiDAR Sensor Frame: %s", lidar_sensor_frame.c_str());

        LoadCorrespondences(correspondence_path);
        LoadCameraMatrix(k_matrix_path);
        LookupInitialTransforms();

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