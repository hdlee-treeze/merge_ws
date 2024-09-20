#include "utils.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iomanip>  // 소수점 자릿수 조절을 위한 헤더

class Process : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_r;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_fr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_fl;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_rr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_rl;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subpointCloudTopic_f;


    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud;

    pcl::PassThrough<pcl::PointXYZI> pass;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMerged;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTransformed;

    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;

    std_msgs::msg::Header cloudHeader;
    std::string frameID_;
    int maximum_queue_size_;

    rclcpp::TimerBase::SharedPtr timer;

public:
    void copyPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg, const std::string& id) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*laserCloudMsg, *cloud);

        *transformedCloud = *cloud;

        cloudHeader.stamp = this->get_clock()->now();
        cloudHeader.frame_id = laserCloudMsg->header.frame_id;
    }

    void cloudPublish() {
        auto laserCloudTemp = sensor_msgs::msg::PointCloud2();
        auto mergedCloudTemp = sensor_msgs::msg::PointCloud2();

        if (pubMerged->get_subscription_count() > 0) {
            if (!mergedCloud->empty()) {
                pcl::toROSMsg(*mergedCloud, mergedCloudTemp);
                mergedCloudTemp.header.stamp = cloudHeader.stamp;
                mergedCloudTemp.header.frame_id = frameID_;
                pubMerged->publish(mergedCloudTemp);
            }

            if (!transformedCloud->empty()) {
                pcl::toROSMsg(*transformedCloud, laserCloudTemp);
                laserCloudTemp.header.stamp = cloudHeader.stamp;
                laserCloudTemp.header.frame_id = frameID_;
                pubTransformed->publish(laserCloudTemp);
            }
        }
    }

    void mergeClouds() {
        if (!transformedCloud->empty()) {
            *mergedCloud += *transformedCloud;
        }
    }

    void allocateMemory() {
        transformedCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        mergedCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        static bool subscribed = false;  // 처음 한 번만 출력하기 위한 플래그
        if (!subscribed) {
            RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud from frame: %s", msg->header.frame_id.c_str());
            subscribed = true;
        }

        copyPointCloud(msg, msg->header.frame_id);
        mergeClouds();
    }

    void timerCallback() {
        cloudPublish();
        mergedCloud->clear();  // Clear mergedCloud after publishing

        // FPS 계산 및 ROS 로그로 출력
        static int frameCount = 0;
        static auto lastTime = std::chrono::steady_clock::now();
        frameCount++;

        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastTime;

        if (elapsed.count() >= 1.0) {  // 1초마다 FPS 출력
            double fps = static_cast<double>(frameCount) / elapsed.count();
            RCLCPP_INFO(this->get_logger(), "Merged Cloud FPS: %.2f", fps);
            frameCount = 0;
            lastTime = currentTime;
        }
    }

    Process() : Node("lidar_fusion") {
        // maximum_queue_size_ = this->declare_parameter("max_queue_size", 5);

        // subpointCloudTopic_r = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_r, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_fr = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_fr, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_fl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_fl, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_rr = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_rr, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_rl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_rl, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        // subpointCloudTopic_f = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointCloudTopic_f, rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));

        subpointCloudTopic_r = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_r, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_fr = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_fr, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_fl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_fl, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_rr = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_rr, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_rl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_rl, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));
        subpointCloudTopic_f = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic_f, 10, std::bind(&Process::pointCloudCallback, this, std::placeholders::_1));

        pubMerged = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubMergedTopic, 1);
        pubTransformed = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubTransformedTopic, 1);

        frameID_ = pubframeID;

        allocateMemory();

        // Timer callback at higher frequency for reduced latency
        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust period for faster publishing
            std::bind(&Process::timerCallback, this)
        );
    }

    ~Process() {}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Process>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}