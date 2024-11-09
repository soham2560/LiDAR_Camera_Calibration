#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using namespace message_filters;

class SyncNode : public rclcpp::Node {
public:
    SyncNode() : Node("sync_node") {
        auto topic_image = declare_parameter("topic_image", "/image_raw");
        auto topic_pointcloud = declare_parameter("topic_pointcloud", "/velodyne_points");
        auto queue_size = declare_parameter("queue_size", 10);
        auto use_approximate_sync = declare_parameter("use_approximate_sync", true);

        sub_image_.subscribe(this, topic_image);
        sub_pointcloud_.subscribe(this, topic_pointcloud);

        if (use_approximate_sync) {
            approximate_sync_ = std::make_shared<Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queue_size), sub_image_, sub_pointcloud_);
            approximate_sync_->registerCallback([this](const Image::ConstSharedPtr &image_msg, const PointCloud2::ConstSharedPtr &pointcloud_msg) {
                publishSyncMessages(image_msg, pointcloud_msg);
            });
        } else {
            exact_sync_ = std::make_shared<Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queue_size), sub_image_, sub_pointcloud_);
            exact_sync_->registerCallback([this](const Image::ConstSharedPtr &image_msg, const PointCloud2::ConstSharedPtr &pointcloud_msg) {
                publishSyncMessages(image_msg, pointcloud_msg);
            });
        }

        pub_image_sync_ = create_publisher<Image>(topic_image + "_sync", 10);
        pub_pointcloud_sync_ = create_publisher<PointCloud2>(topic_pointcloud + "_sync", 10);
    }

private:
    void publishSyncMessages(const Image::ConstSharedPtr &image_msg, const PointCloud2::ConstSharedPtr &pointcloud_msg) {
        pub_image_sync_->publish(*image_msg);
        pub_pointcloud_sync_->publish(*pointcloud_msg);
    }

    Subscriber<Image> sub_image_;
    Subscriber<PointCloud2> sub_pointcloud_;
    using ApproximateSyncPolicy = sync_policies::ApproximateTime<Image, PointCloud2>;
    using ExactSyncPolicy = sync_policies::ExactTime<Image, PointCloud2>;
    std::shared_ptr<Synchronizer<ApproximateSyncPolicy>> approximate_sync_;
    std::shared_ptr<Synchronizer<ExactSyncPolicy>> exact_sync_;
    rclcpp::Publisher<Image>::SharedPtr pub_image_sync_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_sync_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncNode>());
    rclcpp::shutdown();
    return 0;
}
