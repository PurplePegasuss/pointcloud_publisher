#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

class PointCloudPublisherNode : public rclcpp::Node {
public:
    PointCloudPublisherNode() : Node("pointcloud_publisher") {
        // Declare parameters
        this->declare_parameter<std::string>("file_path", "/root/Downloads/LOAM/cloudCorner.pcd");
        this->declare_parameter<std::string>("publish_topic", "/pointcloud");

        // Get parameters
        file_path_ = this->get_parameter("file_path").as_string();
        publish_topic_ = this->get_parameter("publish_topic").as_string();

        // Check if file path is provided
        if (file_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "File path not provided. Exiting node.");
            rclcpp::shutdown();
            return;
        }

        // Load point cloud from file
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path_, *pcl_cloud) == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to load point cloud from file.");
            rclcpp::shutdown();
            return;
        }

        // Convert PCL point cloud to ROS PointCloud2 message
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*pcl_cloud, pcl_pc2);

        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl_conversions::fromPCL(pcl_pc2, ros_cloud);

        ros_cloud.header.frame_id = "map"; 

        // Set up publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, 10);

        
        // Publish the point cloud
        publisher_->publish(ros_cloud);
    }

private:
    std::string file_path_;
    std::string publish_topic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisherNode>());
    rclcpp::shutdown();
    return 0;
}