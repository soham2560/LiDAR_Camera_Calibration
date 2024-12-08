#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>

class ZhangCalibrationNode : public rclcpp::Node {
public:
    ZhangCalibrationNode() : Node("zhang_calibration"), calibration_done_(false) {
        input_folder = this->declare_parameter<std::string>("input_folder", "");
        output_file = this->declare_parameter<std::string>("output_file", "");
        square_size = this->declare_parameter<double>("square_size", 0.0835);
        pattern_width = this->declare_parameter<int>("pattern_width", 8);
        pattern_height = this->declare_parameter<int>("pattern_height", 6);
        
        perform_calibration();
    }

    bool is_calibration_done() const { return calibration_done_; }

private:
    std::string input_folder, output_file;
    double square_size;
    int pattern_width, pattern_height;
    bool calibration_done_;

    void zhangs_calib(
        const std::vector<std::vector<cv::Point3f>>& object_points,
        const std::vector<std::vector<cv::Point2f>>& image_points,
        cv::Mat& camera_matrix,
        int pattern_height,
        int pattern_width
    ) {
        int N = object_points.size();

        std::vector<cv::Mat> homographies;
        for (int i = 0; i < (int)N; ++i) {
            cv::Mat A(2 * pattern_height * pattern_width, 9, CV_64F, cv::Scalar(0));
            
            for (int j = 0; j < pattern_height * pattern_width; ++j) {
                double X = object_points[i][j].x;
                double Y = object_points[i][j].y;
                double x = image_points[i][j].x;
                double y = image_points[i][j].y;

                A.at<double>(2*j, 0) = X;
                A.at<double>(2*j, 1) = Y;
                A.at<double>(2*j, 2) = 1;
                A.at<double>(2*j, 6) = -x * X;
                A.at<double>(2*j, 7) = -x * Y;
                A.at<double>(2*j, 8) = -x;

                A.at<double>(2*j + 1, 3) = X;
                A.at<double>(2*j + 1, 4) = Y;
                A.at<double>(2*j + 1, 5) = 1;
                A.at<double>(2*j + 1, 6) = -y * X;
                A.at<double>(2*j + 1, 7) = -y * Y;
                A.at<double>(2*j + 1, 8) = -y;
            }

            cv::Mat w, u, vt;
            cv::SVD::compute(A, w, u, vt);
            cv::Mat H = vt.row(8).reshape(0, 3);
            H /= H.at<double>(2, 2);
            homographies.push_back(H);
        }

        cv::Mat V(2 * N, 6, CV_64F);
        for (int i = 0; i < (int)N; ++i) {
            cv::Mat H = homographies[i];
            
            V.at<double>(2*i, 0) = H.at<double>(0,0) * H.at<double>(0,1);
            V.at<double>(2*i, 1) = H.at<double>(0,0) * H.at<double>(1,1) + H.at<double>(1,0) * H.at<double>(0,1);
            V.at<double>(2*i, 2) = H.at<double>(1,0) * H.at<double>(1,1);
            V.at<double>(2*i, 3) = H.at<double>(2,0) * H.at<double>(0,1) + H.at<double>(0,0) * H.at<double>(2,1);
            V.at<double>(2*i, 4) = H.at<double>(2,0) * H.at<double>(1,1) + H.at<double>(1,0) * H.at<double>(2,1);
            V.at<double>(2*i, 5) = H.at<double>(2,0) * H.at<double>(2,1);
            
            V.at<double>(2*i+1, 0) = H.at<double>(0,0) * H.at<double>(0,0) - H.at<double>(0,1) * H.at<double>(0,1);
            V.at<double>(2*i+1, 1) = 2 * (H.at<double>(0,0) * H.at<double>(1,0) - H.at<double>(0,1) * H.at<double>(1,1));
            V.at<double>(2*i+1, 2) = H.at<double>(1,0) * H.at<double>(1,0) - H.at<double>(1,1) * H.at<double>(1,1);
            V.at<double>(2*i+1, 3) = 2 * (H.at<double>(2,0) * H.at<double>(0,0) - H.at<double>(2,1) * H.at<double>(0,1));
            V.at<double>(2*i+1, 4) = 2 * (H.at<double>(2,0) * H.at<double>(1,0) - H.at<double>(2,1) * H.at<double>(1,1));
            V.at<double>(2*i+1, 5) = H.at<double>(2,0) * H.at<double>(2,0) - H.at<double>(2,1) * H.at<double>(2,1);
        }

        cv::Mat w, u, vt;
        cv::SVD::compute(V, w, u, vt);

        cv::Mat b = vt.row(vt.rows-1).t();

        double v0 = (b.at<double>(1) * b.at<double>(3) - b.at<double>(0) * b.at<double>(4)) / 
                    (b.at<double>(0) * b.at<double>(2) - b.at<double>(1) * b.at<double>(1));
        
        double lambda = b.at<double>(5) - (b.at<double>(3) * b.at<double>(3) + v0 * (b.at<double>(1) * b.at<double>(3) - b.at<double>(0) * b.at<double>(4))) / b.at<double>(0);
        
        double alpha = std::sqrt(lambda / b.at<double>(0));
        double beta = std::sqrt(lambda * b.at<double>(0) / (b.at<double>(0) * b.at<double>(2) - b.at<double>(1) * b.at<double>(1)));
        
        double gamma = -b.at<double>(1) * alpha * alpha * beta / lambda;
        double u0 = (gamma * v0 / beta) - (b.at<double>(3) * alpha * alpha / lambda);

        camera_matrix = (cv::Mat_<double>(3,3) << 
            alpha,  gamma, u0,
            0,      beta,  v0,
            0,      0,     1);
    }

    void perform_calibration() {
        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> image_points;
        cv::Size pattern_size(pattern_width, pattern_height);

        std::vector<cv::Point3f> obj;
        for (int i = 0; i < (int)pattern_height; ++i)
            for (int j = 0; j < pattern_width; ++j)
                obj.emplace_back(j * square_size, i * square_size, 0.0);

        std::vector<cv::Mat> processed_images;

        int successful_images = 0;
        int total_images = 0;
        for (const auto& entry : std::filesystem::directory_iterator(input_folder)) {
            if (!std::filesystem::is_regular_file(entry.status()) || 
                !std::filesystem::path(entry.path()).has_extension() || 
                (entry.path().extension() != ".jpg" && 
                 entry.path().extension() != ".png" && 
                 entry.path().extension() != ".bmp")) {
                continue;
            }

            total_images++;
            cv::Mat img = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
            if (img.empty()) {
                RCLCPP_WARN(this->get_logger(), "Could not read image: %s", entry.path().string().c_str());
                continue;
            }

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(img, pattern_size, corners, 
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
                
                image_points.push_back(corners);
                object_points.push_back(obj);
                processed_images.push_back(img);
                successful_images++;
            }
        }

        if (successful_images < 5) {
            RCLCPP_ERROR(this->get_logger(), "Not enough images processed for calibration. Found %d out of %d", 
                         successful_images, total_images);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Processed %d successful images out of %d total", 
                    successful_images, total_images);

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                                    0, 1, 0,
                                                    0, 0, 1);
        zhangs_calib(object_points, image_points, camera_matrix, pattern_height, pattern_width );

        RCLCPP_INFO(this->get_logger(), "Camera Matrix:\n%s", 
            format_matrix(camera_matrix).c_str());

        save_calibration(camera_matrix);
        calibration_done_ = true;
    }

    std::string format_matrix(const cv::Mat& mat) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4);
        for (int i = 0; i < (int)mat.rows; ++i) {
            for (int j = 0; j < mat.cols; ++j) {
                ss << std::setw(10) << mat.at<double>(i, j) << " ";
            }
            ss << "\n";
        }
        return ss.str();
    }

    void save_calibration(const cv::Mat& camera_matrix) {
        std::filesystem::path output_path(output_file);

        try {
            std::ofstream file(output_path);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", 
                            output_path.string().c_str());
                return;
            }
            for (int i = 0; i < (int)camera_matrix.rows; ++i) {
                for (int j = 0; j < camera_matrix.cols; ++j) {
                    file << std::fixed << std::setprecision(6) << camera_matrix.at<double>(i, j) << " ";
                }
                file << "\n";
            }

            file.close();
            RCLCPP_INFO(this->get_logger(), "Calibration results saved to %s", 
                        output_path.string().c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving calibration: %s", e.what());
        }
    }
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ZhangCalibrationNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        
        while (rclcpp::ok() && !node->is_calibration_done()) {
            executor.spin_some();
        }

        rclcpp::shutdown();
        return node->is_calibration_done() ? 0 : 1;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}