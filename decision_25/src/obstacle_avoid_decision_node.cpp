// obstacle_avoid_decision_node.cpp
// Obstacle Avoidance Decision Node - LiDAR 기반 장애물 회피 제어

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <algorithm>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------
static std::string g_motor_topic;
static std::string g_servo_topic;

// Servo parameters
static double g_servo_center = 0.57;
static double g_servo_min = 0.0;
static double g_servo_max = 1.0;
static double g_steer_sign = -1.0;

// Obstacle avoidance parameters
static double g_avoid_speed = 0.2;
static double g_max_steering_obstacle = 0.5;
static double g_steering_gain_obstacle = 0.02;
static int g_clear_threshold = 20;
static double g_safe_distance = 0.5;
static double g_stop_distance = 0.2;

// Motor parameters
static double g_motor_min_cmd = 0.0;
static double g_motor_max_cmd = 1200.0;
static double g_motor_gain = 300.0;

// Internal state
static double g_best_gap_angle = 0.0;
static double g_min_obstacle_distance = 10.0;
static bool g_has_obstacle = false;
static bool g_avoiding = false;
static int g_clear_count = 0;
static std::string g_current_state = "IDLE";
static ros::Time g_last_state_change_time;

// Subscribers/Publishers
static ros::Subscriber g_best_gap_sub;
static ros::Subscriber g_min_distance_sub;
static ros::Subscriber g_has_obstacle_sub;
static ros::Publisher g_motor_pub;
static ros::Publisher g_servo_pub;
static ros::Publisher g_state_pub;
static ros::Publisher g_avoiding_pub;

// -------------------- 콜백: best_gap (LiDAR) --------------------
static void bestGapCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_best_gap_angle = msg->point.x;
}

// -------------------- 콜백: min_distance (LiDAR) --------------------
static void minDistanceCB(const std_msgs::Float32::ConstPtr& msg)
{
  g_min_obstacle_distance = msg->data;
}

// -------------------- 콜백: has_obstacle (LiDAR) --------------------
static void hasObstacleCB(const std_msgs::Int32::ConstPtr& msg)
{
  g_has_obstacle = (msg->data != 0);
}

// -------------------- 초기화 함수 --------------------
void obstacle_avoid_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[obstacle_avoid_decision] obstacle_avoid_decision_init()");
  
  // Load parameters
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         "/commands/motor/speed");
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         "/commands/servo/position");
  
  // Servo parameters
  pnh.param<double>("servo_center", g_servo_center, 0.57);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);
  pnh.param<double>("steer_sign", g_steer_sign, -1.0);
  
  // Obstacle avoidance parameters
  pnh.param<double>("avoid_speed", g_avoid_speed, 0.2);
  pnh.param<double>("max_steering_obstacle", g_max_steering_obstacle, 0.5);
  pnh.param<double>("steering_gain_obstacle", g_steering_gain_obstacle, 0.02);
  pnh.param<int>("clear_threshold", g_clear_threshold, 20);
  pnh.param<double>("safe_distance", g_safe_distance, 0.5);
  pnh.param<double>("stop_distance", g_stop_distance, 0.2);
  
  // Motor parameters
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 1200.0);
  pnh.param<double>("motor_gain", g_motor_gain, 300.0);
  
  // Subscribe to LiDAR perception results
  g_best_gap_sub = nh.subscribe("/webot/obstacle/best_gap", 10, bestGapCB);
  g_min_distance_sub = nh.subscribe("/webot/obstacle/min_distance", 10, minDistanceCB);
  g_has_obstacle_sub = nh.subscribe("/webot/obstacle/has_obstacle", 10, hasObstacleCB);
  
  // Advertise control commands
  g_motor_pub = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_servo_pub = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);
  g_state_pub = nh.advertise<std_msgs::String>("/webot/obstacle_avoid/state", 10);
  g_avoiding_pub = nh.advertise<std_msgs::Bool>("/webot/obstacle_avoid/is_avoiding", 10);
  
  ROS_INFO("[obstacle_avoid_decision] Subscribe best_gap='/webot/obstacle/best_gap'");
  ROS_INFO("[obstacle_avoid_decision] Subscribe min_distance='/webot/obstacle/min_distance'");
  ROS_INFO("[obstacle_avoid_decision] Subscribe has_obstacle='/webot/obstacle/has_obstacle'");
  ROS_INFO("[obstacle_avoid_decision] Publish motor='%s', servo='%s'",
          ros::names::resolve(g_motor_topic).c_str(),
          ros::names::resolve(g_servo_topic).c_str());
  ROS_INFO("[obstacle_avoid_decision] safe_distance=%.2f, stop_distance=%.2f, clear_threshold=%d",
          g_safe_distance, g_stop_distance, g_clear_threshold);
  
  ROS_INFO("[obstacle_avoid_decision] obstacle_avoid_decision_init done");
}

// -------------------- 제어 루프 (매 주기마다 호출) --------------------
void obstacle_avoid_decision_step()
{
  std::string state_msg = "IDLE";
  double steer_cmd = 0.0;
  double speed_cmd = 0.0;
  
  // ================ 장애물 회피 로직 ================
  
  // 1) 너무 가까우면 정지
  if (g_min_obstacle_distance < g_stop_distance) {
    state_msg = "TOO_CLOSE";
    steer_cmd = 0.0;
    speed_cmd = 0.0;
    g_avoiding = true;
    g_clear_count = 0;
  }
  
  // 2) 장애물 감지 -> 회피 시작
  else if (g_has_obstacle && !g_avoiding) {
    state_msg = "AVOIDING_START";
    double steering = -g_steering_gain_obstacle * g_best_gap_angle;
    steer_cmd = clamp(steering, -g_max_steering_obstacle, g_max_steering_obstacle);
    speed_cmd = g_avoid_speed;
    g_avoiding = true;
    g_clear_count = 0;
  }
  
  // 3) 회피 중 -> 계속 회피
  else if (g_avoiding) {
    double steering = -g_steering_gain_obstacle * g_best_gap_angle;
    steer_cmd = clamp(steering, -g_max_steering_obstacle, g_max_steering_obstacle);
    speed_cmd = g_avoid_speed;
    
    // 장애물 없음 카운트 증가
    g_clear_count++;
    
    if (g_clear_count >= g_clear_threshold) {
      // 충분히 오래 장애물 없음 -> 회피 종료
      g_avoiding = false;
      g_clear_count = 0;
      state_msg = "CLEAR";
      steer_cmd = 0.0;
      speed_cmd = 0.0;
    } else {
      state_msg = "AVOIDING";
    }
  }
  
  // 4) 평상시 (회피 안 함)
  else {
    state_msg = "CLEAR";
    steer_cmd = 0.0;
    speed_cmd = 0.0;
  }
  
  // ================ 제어값 변환 ================
  
  // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);
  
  double servo_range = std::min(g_servo_center - g_servo_min,
                               g_servo_max - g_servo_center);
  double servo_hw = g_servo_center + steer_norm * servo_range;
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);
  
  // ---- 2) 속도 변환: m/s -> 모터 명령 ----
  double motor_cmd = g_motor_gain * speed_cmd;
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);
  
  // ---- 3) Publish ----
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;
  
  g_motor_pub.publish(motor_msg);
  g_servo_pub.publish(servo_msg);
  
  // Publish state
  std_msgs::String state_str;
  state_str.data = state_msg;
  g_state_pub.publish(state_str);
  
  std_msgs::Bool avoiding_bool;
  avoiding_bool.data = g_avoiding;
  g_avoiding_pub.publish(avoiding_bool);
  
  if (state_msg != g_current_state) {
    ROS_INFO("[obstacle_avoid_decision] State changed: %s", state_msg.c_str());
    g_current_state = state_msg;
    g_last_state_change_time = ros::Time::now();
  }
  
  ROS_DEBUG_THROTTLE(0.5,
    "[obstacle_avoid_decision] state=%s gap_angle=%.1f min_dist=%.2f steer=%.3f motor=%.1f obstacle=%d",
    state_msg.c_str(), g_best_gap_angle, g_min_obstacle_distance,
    steer_cmd, motor_cmd, g_has_obstacle ? 1 : 0);
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoid_decision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  obstacle_avoid_decision_init(nh, pnh);
  
  ros::Rate loop_rate(30);  // 30 Hz control loop
  
  while (ros::ok()) {
    ros::spinOnce();
    obstacle_avoid_decision_step();
    loop_rate.sleep();
  }
  
  return 0;
}
