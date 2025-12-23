// lidar_obstacle_perception_node.cpp
// LiDAR Obstacle Perception Node - gap 찾기 및 장애물 인식

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <algorithm>
#include <vector>

// -------------------- 전역 상태 --------------------
static std::string g_scan_topic;
static std::string g_best_gap_topic;
static std::string g_min_distance_topic;
static std::string g_has_obstacle_topic;

static ros::Subscriber g_scan_sub;
static ros::Publisher g_best_gap_pub;
static ros::Publisher g_min_distance_pub;
static ros::Publisher g_has_obstacle_pub;

// LiDAR parameters
static double g_scan_angle = 30.0;
static double g_safe_distance = 0.5;
static double g_stop_distance = 0.2;
static int g_num_sectors = 12;

// Sector data
static std::vector<double> g_sector_distances;
static std::vector<double> g_sector_angles;

// -------------------- Helper functions --------------------
static void initSectors()
{
  double total_angle = g_scan_angle * 2.0;
  double sector_size = total_angle / g_num_sectors;
  
  g_sector_angles.clear();
  g_sector_distances.clear();
  
  for (int i = 0; i < g_num_sectors; i++) {
    double angle = g_scan_angle - sector_size * (i + 0.5);
    g_sector_angles.push_back(angle);
    g_sector_distances.push_back(10.0);
  }
}

static void calculateSectorDistances(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  const std::vector<float>& ranges = msg->ranges;
  double angle_increment = msg->angle_increment;
  
  if (ranges.empty() || angle_increment == 0) {
    return;
  }
  
  int total_points = ranges.size();
  int center_idx = 0;
  double points_per_degree = 1.0 / (angle_increment * 180.0 / M_PI);
  double sector_size_deg = (g_scan_angle * 2.0) / g_num_sectors;
  
  for (int i = 0; i < g_num_sectors; i++) {
    double start_angle = -g_scan_angle + sector_size_deg * i;
    double end_angle = start_angle + sector_size_deg;
    
    int start_idx = static_cast<int>(center_idx + start_angle * points_per_degree);
    int end_idx = static_cast<int>(center_idx + end_angle * points_per_degree);
    
    start_idx = std::max(0, std::min(start_idx, total_points - 1));
    end_idx = std::max(0, std::min(end_idx, total_points - 1));
    
    if (start_idx > end_idx) {
      std::swap(start_idx, end_idx);
    }
    
    double min_dist = 10.0;
    for (int j = start_idx; j <= end_idx; j++) {
      float range = ranges[j];
      if (range > 0.01 && range < 10.0) {
        min_dist = std::min(min_dist, static_cast<double>(range));
      }
    }
    
    g_sector_distances[i] = min_dist;
  }
}

struct Gap {
  int start_idx;
  int end_idx;
};

static Gap findBestGap()
{
  std::vector<bool> is_open(g_num_sectors);
  for (int i = 0; i < g_num_sectors; i++) {
    is_open[i] = (g_sector_distances[i] >= g_safe_distance);
  }
  
  std::vector<Gap> gaps;
  int gap_start = -1;
  
  for (int i = 0; i < g_num_sectors; i++) {
    if (is_open[i] && gap_start == -1) {
      gap_start = i;
    } else if (!is_open[i] && gap_start != -1) {
      gaps.push_back({gap_start, i - 1});
      gap_start = -1;
    }
  }
  
  if (gap_start != -1) {
    gaps.push_back({gap_start, g_num_sectors - 1});
  }
  
  if (gaps.empty()) {
    // No open gap - find widest sector
    int max_idx = 0;
    double max_dist = g_sector_distances[0];
    for (int i = 1; i < g_num_sectors; i++) {
      if (g_sector_distances[i] > max_dist) {
        max_dist = g_sector_distances[i];
        max_idx = i;
      }
    }
    return {max_idx, max_idx};
  }
  
  // Find widest gap
  Gap best_gap = gaps[0];
  int best_width = best_gap.end_idx - best_gap.start_idx;
  
  for (const auto& gap : gaps) {
    int width = gap.end_idx - gap.start_idx;
    if (width > best_width) {
      best_width = width;
      best_gap = gap;
    }
  }
  
  return best_gap;
}

static double getGapCenterAngle(const Gap& gap)
{
  double center_idx = (gap.start_idx + gap.end_idx) / 2.0;
  
  if (center_idx == static_cast<int>(center_idx)) {
    return g_sector_angles[static_cast<int>(center_idx)];
  }
  
  int low_idx = static_cast<int>(center_idx);
  int high_idx = std::min(low_idx + 1, g_num_sectors - 1);
  double ratio = center_idx - low_idx;
  
  return g_sector_angles[low_idx] * (1.0 - ratio) + 
         g_sector_angles[high_idx] * ratio;
}

static double getMinFrontDistance()
{
  int center_sectors = g_num_sectors / 3;
  int start = (g_num_sectors - center_sectors) / 2;
  int end = start + center_sectors;
  
  double min_dist = 10.0;
  for (int i = start; i < end; i++) {
    if (i >= 0 && i < g_num_sectors) {
      min_dist = std::min(min_dist, g_sector_distances[i]);
    }
  }
  
  return min_dist;
}

static bool hasObstacleInFront()
{
  return getMinFrontDistance() < g_safe_distance;
}

// -------------------- Callback --------------------
static void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  calculateSectorDistances(msg);
  
  // Find best gap
  Gap best_gap = findBestGap();
  double gap_angle = getGapCenterAngle(best_gap);
  
  // Get minimum distance
  double min_dist = getMinFrontDistance();
  bool has_obstacle = hasObstacleInFront();
  
  // Publish results
  geometry_msgs::PointStamped gap_msg;
  gap_msg.header.stamp = ros::Time::now();
  gap_msg.header.frame_id = "laser";
  gap_msg.point.x = gap_angle;
  gap_msg.point.y = best_gap.end_idx - best_gap.start_idx + 1;  // gap width
  gap_msg.point.z = 0;
  g_best_gap_pub.publish(gap_msg);
  
  std_msgs::Float32 dist_msg;
  dist_msg.data = static_cast<float>(min_dist);
  g_min_distance_pub.publish(dist_msg);
  
  std_msgs::Int32 obstacle_msg;
  obstacle_msg.data = has_obstacle ? 1 : 0;
  g_has_obstacle_pub.publish(obstacle_msg);
  
  ROS_DEBUG_THROTTLE(1.0,
    "[lidar_perception] gap_angle=%.1f gap_width=%d min_dist=%.2f obstacle=%d",
    gap_angle, (int)(best_gap.end_idx - best_gap.start_idx + 1),
    min_dist, has_obstacle ? 1 : 0);
}

// -------------------- Init function --------------------
void lidar_perception_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[lidar_perception] lidar_perception_init()");
  
  // Load parameters
  pnh.param<std::string>("scan_topic", g_scan_topic, "/scan");
  pnh.param<std::string>("best_gap_topic", g_best_gap_topic, "/webot/obstacle/best_gap");
  pnh.param<std::string>("min_distance_topic", g_min_distance_topic, "/webot/obstacle/min_distance");
  pnh.param<std::string>("has_obstacle_topic", g_has_obstacle_topic, "/webot/obstacle/has_obstacle");
  
  pnh.param<double>("scan_angle", g_scan_angle, 30.0);
  pnh.param<double>("safe_distance", g_safe_distance, 0.5);
  pnh.param<double>("stop_distance", g_stop_distance, 0.2);
  pnh.param<int>("num_sectors", g_num_sectors, 12);
  
  // Initialize sectors
  initSectors();
  
  // Subscribe
  g_scan_sub = nh.subscribe(g_scan_topic, 10, scanCB);
  
  // Advertise
  g_best_gap_pub = nh.advertise<geometry_msgs::PointStamped>(g_best_gap_topic, 10);
  g_min_distance_pub = nh.advertise<std_msgs::Float32>(g_min_distance_topic, 10);
  g_has_obstacle_pub = nh.advertise<std_msgs::Int32>(g_has_obstacle_topic, 10);
  
  ROS_INFO("[lidar_perception] Scan topic: %s",
          ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[lidar_perception] Publishing best_gap: %s",
          ros::names::resolve(g_best_gap_topic).c_str());
  ROS_INFO("[lidar_perception] scan_angle=%.1f, safe_distance=%.2f, num_sectors=%d",
          g_scan_angle, g_safe_distance, g_num_sectors);
  
  ROS_INFO("[lidar_perception] lidar_perception_init done");
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_obstacle_perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  lidar_perception_init(nh, pnh);
  
  ros::spin();
  
  return 0;
}
