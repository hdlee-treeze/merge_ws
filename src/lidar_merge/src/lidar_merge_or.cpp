#include "utils.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <typeinfo>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Process : public rclcpp::Node {
private:
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_fc;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_r;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_fr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_fl;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_rr;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_rl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud;

    pcl::PassThrough<pcl::PointXYZI> pass;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMerged;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTransformed;
    
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;

    std_msgs::msg::Header cloudHeader;
    std::string frameID_;
    bool isRearL; // Frame ID가 RearL일 때 true로 설정하는 변수
    int maximum_queue_size_;

public:

    void copyPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg, const std::string& id) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*laserCloudMsg, *cloud);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        if (id == "PA64_LowF_lidar") {
            // translation << 0.0f, 6.37f, 1.043f;
            translation << 0.0f,0.0f,0.0f;
            rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());

        } else if (id == "FrontL") {
            // translation << -1.2f, 4.13f, 0.643f;
            translation << 0.0f,0.0f,0.0f;
            // rotation = Eigen::AngleAxisf(-M_PI /2, Eigen::Vector3f::UnitZ()); 
            rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()); 
            
        } else if (id == "RearL") {
            // translation << -1.2f, -2.32f, 0.643f;
            translation << 0.0f,0.0f,0.0f;
            // rotation = Eigen::AngleAxisf(-58.8f * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
            rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()); 

            isRearL = true;
            mergedCloud->clear();

        } else if (id == "RearR") {
            // translation << 1.2f, -2.32f, 0.643f;
            translation << 0.0f,0.0f,0.0f;
            // rotation = Eigen::AngleAxisf(62.0f * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
            rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()); 
            

        } else if (id == "FrontR") {
            // translation << 1.2f, 4.13f, 0.643f;
            translation << 0.0f,0.0f,0.0f;
            // rotation = Eigen::AngleAxisf(M_PI /2, Eigen::Vector3f::UnitZ());
            rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()); 

        }

        RCLCPP_INFO(this->get_logger(), "Translation: [%f, %f, %f]", translation.x(), translation.y(), translation.z());
 

        transform.translation() = translation;
        transform.rotate(rotation);

        pcl::transformPointCloud(*cloud, *cloud_transformed, transform);
        
        *transformedCloud = *cloud_transformed;
        // *transformedCloud = *cloud;

        cloudHeader.stamp = this->get_clock()->now();
        cloudHeader.frame_id = laserCloudMsg->header.frame_id;
    }

    void cloudPublish() {
        RCLCPP_DEBUG(this->get_logger(), "cloudPublish called"); // Debug log added
        auto laserCloudTemp = sensor_msgs::msg::PointCloud2();
        auto mergedCloudTemp = sensor_msgs::msg::PointCloud2(); // Merged cloud message

        // subscribe 수 확인
        size_t subscription_count = pubMerged->get_subscription_count();
        RCLCPP_INFO(this->get_logger(), "Number of subscribers to pubMerged: %zu", subscription_count);

        if (subscription_count > 0) { // 구독자가 있을 때만 발행
            pcl::toROSMsg(*transformedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = frameID_; // pubframeID에서 frameID_로 변경

            if (mergedCloud->size() > 0) { // mergedCloud가 비어있지 않을 때만 발행
                pcl::toROSMsg(*mergedCloud, mergedCloudTemp);
                mergedCloudTemp.header.stamp = cloudHeader.stamp;
                mergedCloudTemp.header.frame_id = pubframeID;
                pubMerged->publish(mergedCloudTemp); // Publish merged cloud
            }

            RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 with frame ID: %s", laserCloudTemp.header.frame_id.c_str());
            pubTransformed->publish(laserCloudTemp);
        } else {
            RCLCPP_WARN(this->get_logger(), "No subscribers to pubTransformed, not publishing.");
        }
    }

    void mergeClouds(const std::string& id) {
        // transformedCloud를 mergedCloud에 복사
        *mergedCloud += *transformedCloud; // 현재 변환된 클라우드를 병합

    }

    void allocateMemory() {
        transformedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mergedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_transformed.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received a point cloud message");
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", msg->header.frame_id.c_str());
        copyPointCloud(msg, msg->header.frame_id);
        mergeClouds(msg->header.frame_id); // Merge clouds after copying
        cloudPublish();
    }

    Process() : Node("lidar_fusion") {
        maximum_queue_size_ = static_cast<int>(declare_parameter("max_queue_size", 5));

        // subpointCloudTopic_fc = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_fc, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_r = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_r, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_fr = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_fr,  rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_fl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_fl,  rclcpp::SensorDataQoS().keep_last(5), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_rr = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_rr, rclcpp::SensorDataQoS(), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_rl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_rl, rclcpp::SensorDataQoS(), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        
        pubMerged = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubMergedTopic, 1);
        pubTransformed = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubTransformedTopic, 1);
        // CLCPP_INFO(this->get_logger(), "Publishing message to topic: %s", pubTransformed->get_topic_name());
        frameID_ = pubframeID;
        RCLCPP_INFO(this->get_logger(), "Start fusion");

        allocateMemory(); 
        isRearL = false; // 초기값 설정
    }

    ~Process() {}
};

int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing ROS 2...");
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROS 2 initialized");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating node...");
    auto node = std::make_shared<Process>();
    RCLCPP_INFO(node->get_logger(), "Node has been initialized");

    RCLCPP_INFO(node->get_logger(), "Spinning node...");
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}