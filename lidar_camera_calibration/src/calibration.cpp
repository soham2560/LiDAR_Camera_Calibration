#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LiDARCameraCalibration : public rclcpp::Node {
private:
    std::vector<cv::Point2f> points2D;
    std::vector<cv::Point3f> points3D;
    std::vector<cv::Point2f> inlier_points2D;
    std::vector<cv::Point3f> inlier_points3D;
    std::vector<int> inliers;
    cv::Mat rvec, tvec, K, dist_coeffs;
    std::string camera_frame, lidar_frame, camera_bottom_screw_frame;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    rclcpp::TimerBase::SharedPtr transform_timer_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    struct FullReprojectionError {
        FullReprojectionError(const cv::Point2f& observed, const cv::Point3f& point)
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

    void loadCorrespondences(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open correspondence file: %s", path.c_str());
            return;
        }
        
        double px, py, X, Y, Z;
        while (file >> px >> py >> X >> Y >> Z) {
            points2D.emplace_back(px, py);
            points3D.emplace_back(X, Y, Z);
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu correspondence points from %s", points2D.size(), path.c_str());
    }

    void loadCalibrationParameters() {
        try {
            std::string correspondence_path = declare_parameter<std::string>("correspondence_path");
            std::string k_matrix_path = declare_parameter<std::string>("k_matrix_path");
            std::string dist_coeffs_path = declare_parameter<std::string>("dist_coeffs_path");
            camera_frame = declare_parameter<std::string>("camera_frame");
            lidar_frame = declare_parameter<std::string>("lidar_frame");
            camera_bottom_screw_frame = "camera_bottom_screw_frame";

            RCLCPP_INFO(this->get_logger(), "Loading calibration parameters:");
            RCLCPP_INFO(this->get_logger(), "Correspondence path: %s", correspondence_path.c_str());
            RCLCPP_INFO(this->get_logger(), "K matrix path: %s", k_matrix_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Distortion coeffs path: %s", dist_coeffs_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Camera frame: %s", camera_frame.c_str());
            RCLCPP_INFO(this->get_logger(), "LiDAR frame: %s", lidar_frame.c_str());

            loadCorrespondences(correspondence_path);
            loadKMatrix(k_matrix_path);
            loadDistCoeffs(dist_coeffs_path);
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
            RCLCPP_ERROR(this->get_logger(), "Parameter not declared: %s", e.what());
            throw;
        }
    }

    void loadKMatrix(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open K matrix file: %s", path.c_str());
            return;
        }
        
        K = cv::Mat::zeros(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                file >> K.at<double>(i, j);
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Loaded K matrix from %s", path.c_str());
    }

    void loadDistCoeffs(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open distortion coefficients file: %s", path.c_str());
            return;
        }
        
        dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
        for (int i = 0; i < 5; ++i) {
            file >> dist_coeffs.at<double>(0, i);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Loaded distortion coefficients from %s", path.c_str());
    }

    Eigen::Matrix4d toEigenMatrix(const geometry_msgs::msg::TransformStamped& transform) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
        mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        mat(0, 3) = transform.transform.translation.x;
        mat(1, 3) = transform.transform.translation.y;
        mat(2, 3) = transform.transform.translation.z;
        return mat;
    }

    geometry_msgs::msg::TransformStamped fromEigenMatrix(const Eigen::Matrix4d& mat, const std::string& parent_frame, const std::string& child_frame) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        Eigen::Quaterniond q(Eigen::Matrix3d(mat.block<3, 3>(0, 0)));
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        transform.transform.translation.x = mat(0, 3);
        transform.transform.translation.y = mat(1, 3);
        transform.transform.translation.z = mat(2, 3);
        return transform;
    }

    void publishTransform() {
        RCLCPP_INFO(this->get_logger(), "Starting transform publication process");

        cv::solvePnPRansac(points3D, points2D, K, dist_coeffs, rvec, tvec, false, 10000, 2.0, 0.999, inliers, cv::SOLVEPNP_SQPNP);
        
        RCLCPP_INFO(this->get_logger(), "RANSAC found %zu inliers", inliers.size());

        for (int idx : inliers) {
            inlier_points2D.push_back(points2D[idx]);
            inlier_points3D.push_back(points3D[idx]);
        }

        cv::solvePnPRefineLM(inlier_points3D, inlier_points2D, K, dist_coeffs, rvec, tvec);
        
        double axis_angle[3] = {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)};
        double translation[3] = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
        double intrinsics[4] = {K.at<double>(0,0), K.at<double>(1,1), K.at<double>(0,2), K.at<double>(1,2)};
        double dist_params[5] = {dist_coeffs.at<double>(0), dist_coeffs.at<double>(1), dist_coeffs.at<double>(2), dist_coeffs.at<double>(3), dist_coeffs.at<double>(4)};

        RCLCPP_DEBUG(this->get_logger(), "Initial transform: translation [%f, %f, %f], axis angle [%f, %f, %f]", 
                     translation[0], translation[1], translation[2],
                     axis_angle[0], axis_angle[1], axis_angle[2]);

        ceres::Problem problem;
        for (size_t i = 0; i < inlier_points2D.size(); ++i) {
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FullReprojectionError, 2, 3, 3, 4, 5>(new FullReprojectionError(inlier_points2D[i], inlier_points3D[i]));
            problem.AddResidualBlock(cost_function, nullptr, axis_angle, translation, intrinsics, dist_params);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 200;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        RCLCPP_INFO(this->get_logger(), "Ceres optimization summary:");
        RCLCPP_INFO(this->get_logger(), "Successful: %s", summary.IsSolutionUsable() ? "Yes" : "No");
        RCLCPP_INFO(this->get_logger(), "Initial cost: %f", summary.initial_cost);
        RCLCPP_INFO(this->get_logger(), "Final cost: %f", summary.final_cost);

        transform_stamped_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
        transform_stamped_->header.stamp = this->get_clock()->now();
        transform_stamped_->header.frame_id = lidar_frame;
        transform_stamped_->child_frame_id = camera_frame;
        transform_stamped_->transform.translation.x = translation[0];
        transform_stamped_->transform.translation.y = translation[1];
        transform_stamped_->transform.translation.z = translation[2];

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        Eigen::Matrix3d eigen_rotation;
        cv::cv2eigen(rotation_matrix, eigen_rotation);
        Eigen::Quaterniond q(eigen_rotation);
        transform_stamped_->transform.rotation.x = q.x();
        transform_stamped_->transform.rotation.y = q.y();
        transform_stamped_->transform.rotation.z = q.z();
        transform_stamped_->transform.rotation.w = q.w();

        transform_broadcaster_->sendTransform(*transform_stamped_);

        RCLCPP_INFO(this->get_logger(), "Published transform from %s to %s", lidar_frame.c_str(), camera_frame.c_str());

        try {
            geometry_msgs::msg::TransformStamped screw_to_color = tf_buffer_.lookupTransform(camera_frame, camera_bottom_screw_frame, tf2::TimePointZero);
            
            Eigen::Matrix4d T_lidar_to_color = toEigenMatrix(*transform_stamped_);
            Eigen::Matrix4d T_screw_to_color = toEigenMatrix(screw_to_color);
            Eigen::Matrix4d T_lidar_to_screw = T_lidar_to_color * T_screw_to_color.inverse();
            
            geometry_msgs::msg::TransformStamped final_transform = fromEigenMatrix(T_lidar_to_screw, lidar_frame, camera_bottom_screw_frame);
            transform_broadcaster_->sendTransform(final_transform);
            RCLCPP_INFO(this->get_logger(), "Published final transform from %s to %s", lidar_frame.c_str(), camera_bottom_screw_frame.c_str());
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", e.what());
        }
    }

public:
    LiDARCameraCalibration() : Node("lidar_camera_calibration"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        RCLCPP_INFO(this->get_logger(), "Initializing LiDAR-Camera Calibration Node");
        
        try {
            loadCalibrationParameters();
            transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            transform_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&LiDARCameraCalibration::publishTransform, this));
            
            RCLCPP_INFO(this->get_logger(), "LiDAR-Camera Calibration Node initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
            throw;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<LiDARCameraCalibration>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Node initialization failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}