#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

// Publish_topic
extern const string pubMergedTopic = "/merged";
extern const string pubTransformedTopic = "/transformed";
extern const string pubframeID = "lidar";

// Front_center
extern const string pointCloudTopic_f = "/Front_points";
extern const string frameID_f = "lidar";

// Front_top
extern const string pointCloudTopic_r = "/Rear_points";
extern const string frameID_r = "lidar";

// Front_right
extern const string pointCloudTopic_fr = "/FR_points";
extern const string frameID_fr = "lidar";

// Front_left
extern const string pointCloudTopic_fl = "/FL_points";
extern const string frameID_fl = "lidar";

// Rear_right
extern const string pointCloudTopic_rr = "/RR_points";
extern const string frameID_rr = "lidar";

// Rear_left
extern const string pointCloudTopic_rl = "/RL_points";
extern const string frameID_rl = "lidar";

