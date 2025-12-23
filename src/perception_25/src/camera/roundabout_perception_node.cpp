// roundabout_perception_node.cpp
// Roundabout Perception Node - LiDAR 기반 전방 장애물 감지

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cmath>

// -------------------- 전역 상태 --------------------
// LiDAR 감지 영역 파라미터
static double g_detect_x_min = 0.2;
static double g_detect_x_max = 0.8;
static double g_detect_y_min = -0.3;
static double g_detect_y_max = 0.3;
static int g_obstacle_threshold = 3;

// 발행자
static ros::Publisher g_pub_obstacle_detected;
static ros::Publisher g_pub_obstacle_count;

// 내부 상태
static std::vector<std::pair<double, double>> g_obstacle_points;
static bool g_obstacle_detected = false;

// -------------------- LiDAR 처리 함수 --------------------

/**
 * LiDAR 데이터 처리 - laser_link 180도 회전 보정
 * 극좌표(r, theta) → 직교좌표(x, y)
 * x: 전방 거리, y: 좌우 거리
 */
void process_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  g_obstacle_points.clear();
  
  double angle = msg->angle_min;
  
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    double r = msg->ranges[i];
    
    // 유효한 거리 범위 확인
    if (r < msg->range_min || r > msg->range_max)
    {
      angle += msg->angle_increment;
      continue;
    }
    
    // === laser_link가 180도 회전되어 있으므로 보정 ===
    double corrected_angle = angle + M_PI;
    
    // -π ~ π 범위로 정규화
    while (corrected_angle > M_PI)
      corrected_angle -= 2 * M_PI;
    while (corrected_angle < -M_PI)
      corrected_angle += 2 * M_PI;
    
    // === 극좌표 → 직교좌표 변환 ===
    // x: 전방 거리 (앞 방향 양수)
    // y: 좌우 거리 (왼쪽 양수)
    double x = r * std::cos(corrected_angle);
    double y = r * std::sin(corrected_angle);
    
    // === 감지 영역 내 포인트 필터링 ===
    // 전방(x) 범위와 좌우(y) 범위 모두 만족하는 포인트만
    if ((x >= g_detect_x_min && x <= g_detect_x_max) &&
        (y >= g_detect_y_min && y <= g_detect_y_max))
    {
      g_obstacle_points.push_back(std::make_pair(x, y));
    }
    
    angle += msg->angle_increment;
  }
  
  // === 장애물 판정 (포인트 수로 임계값 설정) ===
  int point_count = static_cast<int>(g_obstacle_points.size());
  g_obstacle_detected = (point_count >= g_obstacle_threshold);
  
  if (g_obstacle_detected)
  {
    ROS_INFO_THROTTLE(1.0, "[roundabout_perception] Obstacle detected: %d points", point_count);
  }
}

// -------------------- 메인 --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "roundabout_perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // === 파라미터 로드 ===
  pnh.param<double>("detect_x_min", g_detect_x_min, 0.2);
  pnh.param<double>("detect_x_max", g_detect_x_max, 0.8);
  pnh.param<double>("detect_y_min", g_detect_y_min, -0.3);
  pnh.param<double>("detect_y_max", g_detect_y_max, 0.3);
  pnh.param<int>("obstacle_threshold", g_obstacle_threshold, 3);

  // === 발행자 ===
  g_pub_obstacle_detected = nh.advertise<std_msgs::Bool>("/webot/roundabout/obstacle_detected", 1);
  g_pub_obstacle_count = nh.advertise<std_msgs::Int32>("/webot/roundabout/obstacle_count", 1);

  // === 구독자 ===
  ros::Subscriber scan_sub = nh.subscribe("/scan", 10, process_lidar);

  ROS_INFO("[roundabout_perception] Node started");
  ROS_INFO("[roundabout_perception] Detect area: X(%.2f-%.2f)m Y(%.2f-%.2f)m",
           g_detect_x_min, g_detect_x_max, g_detect_y_min, g_detect_y_max);
  ROS_INFO("[roundabout_perception] Threshold: %d points", g_obstacle_threshold);

  // === 발행 루프 (LiDAR 수신 시마다 자동 발행) ===
  ros::Rate rate(30);  // 30 Hz
  while (ros::ok())
  {
    ros::spinOnce();

    // 현재 장애물 상태 발행
    std_msgs::Bool obstacle_msg;
    obstacle_msg.data = g_obstacle_detected;
    g_pub_obstacle_detected.publish(obstacle_msg);

    std_msgs::Int32 count_msg;
    count_msg.data = static_cast<int>(g_obstacle_points.size());
    g_pub_obstacle_count.publish(count_msg);

    rate.sleep();
  }

  return 0;
}
