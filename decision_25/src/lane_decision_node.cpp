// lane_decision_node.cpp
// Decision node: perception 토픽 구독 -> 제어 계산 -> 모터/서보 명령 발행

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <algorithm>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------
static std::string g_center_topic;
static std::string g_curvature_topic;
static std::string g_center_color_topic;
static std::string g_motor_topic;
static std::string g_servo_topic;

// BEV parameters
static double g_bev_center_x_px = 320.0;

// Servo parameters
static double g_servo_center = 0.57;
static double g_servo_min = 0.0;
static double g_servo_max = 1.0;
static double g_steer_sign = -1.0;

// Control parameters
static double g_max_abs_dx_px = 60.0;
static double g_dx_tolerance = 8.0;
static double g_steer_gain_base = 0.8;
static double g_steer_gain_min = 0.6;
static double g_steer_gain_max = 2.2;
static double g_steer_gain = 0.8;
static double g_alpha_ema = 0.2;
static double g_max_delta = 0.08;

// Speed parameters
static double g_base_speed_mps = 7.0;
static double g_min_speed_mps = 0.8;
static double g_speed_drop_gain = 0.5;

// Motor parameters
static double g_motor_min_cmd = 0.0;
static double g_motor_max_cmd = 1200.0;
static double g_motor_max_cmd_red = 900.0;
static double g_motor_max_cmd_blue = 2000.0;
static double g_motor_gain = 300.0;

// PID parameters
static double g_pid_kp = 0.01;
static double g_pid_ki = 0.0;
static double g_pid_kd = 0.0001;
static double g_pid_weight = 0.0;

// Internal state
static double g_prev_steer = 0.0;
static double g_latest_steer_cmd = 0.0;
static double g_latest_speed_cmd = 0.0;
static double g_pid_integral = 0.0;
static double g_pid_prev_error = 0.0;
static ros::Time g_pid_prev_time;
static bool g_pid_has_prev_t = false;

static double g_prev_dx = 0.0;
static bool g_has_prev_dx = false;
static int g_lane_color_code = 0;
static ros::Time g_last_cb_time;
static bool g_have_cb_time = false;
static double g_dx_timeout_sec = 1.0;

// Subscribers/Publishers
static ros::Subscriber g_center_sub;
static ros::Subscriber g_curvature_sub;
static ros::Subscriber g_center_color_sub;
static ros::Publisher g_motor_pub;
static ros::Publisher g_servo_pub;

// -------------------- PID 계산 --------------------
static double runPID(double error)
{
  ros::Time now = ros::Time::now();
  double dt = 0.0;
  
  if (g_pid_has_prev_t) {
    dt = (now - g_pid_prev_time).toSec();
  }
  g_pid_prev_time = now;
  g_pid_has_prev_t = true;
  
  double P = g_pid_kp * error;
  
  g_pid_integral += error * dt;
  double I = g_pid_ki * g_pid_integral;
  
  double D = 0.0;
  if (dt > 1e-4) {
    D = g_pid_kd * (error - g_pid_prev_error) / dt;
  }
  g_pid_prev_error = error;
  
  return P + I + D;
}

// -------------------- dx 처리 로직 --------------------
static void processDx(double dx)
{
  double abs_dx = std::fabs(dx);
  double err_norm = 0.0;
  
  if (abs_dx <= g_dx_tolerance) {
    err_norm = 0.0;
  } else {
    err_norm = clamp(dx / g_max_abs_dx_px, -1.0, 1.0);
  }
  
  // Non-linear gain
  double gain_scale = 1.0;
  if (abs_dx < 15.0) {
    gain_scale = 0.5;
  } else if (abs_dx > 35.0) {
    gain_scale = 1.35;
  }
  double steer_raw_base = std::tanh(g_steer_gain * gain_scale * err_norm);
  
  // PID correction
  double u_pid = runPID(dx);
  double steer_pid = clamp(u_pid / g_max_abs_dx_px, -1.0, 1.0);
  
  const double w_base = 1.0;
  double steer_raw = w_base * steer_raw_base + g_pid_weight * steer_pid;
  steer_raw = clamp(steer_raw, -1.0, 1.0);
  
  // Smoothing
  double local_alpha = g_alpha_ema;
  double local_delta = g_max_delta;
  
  if (abs_dx < 15.0) {
    local_alpha = 0.15;
    local_delta = 0.05;
  } else if (abs_dx > 35.0) {
    local_alpha = 0.28;
    local_delta = 0.12;
  }
  
  double steer_smooth = local_alpha * steer_raw + (1.0 - local_alpha) * g_prev_steer;
  
  // Rate limiting
  double delta = clamp(steer_smooth - g_prev_steer, -local_delta, local_delta);
  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;
  
  // Speed control
  double speed_cmd = clamp(g_base_speed_mps - g_speed_drop_gain * std::fabs(err_norm),
                          g_min_speed_mps, g_base_speed_mps);
  
  g_latest_steer_cmd = steer_cmd;
  g_latest_speed_cmd = speed_cmd;
  
  ROS_DEBUG_THROTTLE(0.5,
    "[decision] dx=%.1fpx err_norm=%.3f steer=%.3f v=%.2f",
    dx, err_norm, steer_cmd, speed_cmd);
}

// -------------------- 콜백: center_point_px --------------------
static void centerCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;
  
  double cx = msg->point.x;
  double dx_raw = cx - g_bev_center_x_px;
  
  // Spike filtering
  double dx = dx_raw;
  const double spike_thresh = 80.0;
  if (g_has_prev_dx && std::fabs(dx_raw - g_prev_dx) > spike_thresh) {
    dx = g_prev_dx;  // Ignore spike frame
  }
  g_prev_dx = dx;
  g_has_prev_dx = true;
  
  processDx(dx);
}

// -------------------- 콜백: curvature_center --------------------
static void curvatureCB(const std_msgs::Float32::ConstPtr& msg)
{
  double curv = msg->data;
  double k = std::fabs(curv);
  
  // Normalize curvature and update steering gain
  double k_clamped = clamp(k, 3e-6, 1e-3);
  double t = (k_clamped - 3e-6) / (1e-3 - 3e-6);
  t = clamp(t, 0.0, 1.0);
  
  g_steer_gain = g_steer_gain_min + t * (g_steer_gain_max - g_steer_gain_min);
  
  ROS_DEBUG_THROTTLE(0.5,
    "[decision] curv=%.4e k=%.4e t=%.3f steer_gain=%.3f",
    curv, k, t, g_steer_gain);
}

// -------------------- 콜백: center_color --------------------
static void centerColorCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_lane_color_code = static_cast<int>(msg->point.x);
}

// -------------------- 초기화 함수 --------------------
void decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[decision] decision_init()");
  
  // Load parameters
  pnh.param<std::string>("topic_center_point", g_center_topic,
                         "/webot/lane_center");
  pnh.param<std::string>("topic_curvature_center", g_curvature_topic,
                         "/webot/lane_curvature");
  pnh.param<std::string>("topic_center_color", g_center_color_topic,
                         "/webot/lane_color");
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         "/commands/motor/speed");
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         "/commands/servo/position");
  
  // BEV center
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);
  
  // Servo parameters
  pnh.param<double>("servo_center", g_servo_center, 0.57);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);
  pnh.param<double>("steer_sign", g_steer_sign, -1.0);
  
  // Control parameters
  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 60.0);
  pnh.param<double>("dx_tolerance", g_dx_tolerance, 8.0);
  pnh.param<double>("steer_gain", g_steer_gain_base, 0.8);
  pnh.param<double>("steer_gain_min", g_steer_gain_min, 0.6);
  pnh.param<double>("steer_gain_max", g_steer_gain_max, 2.2);
  g_steer_gain = g_steer_gain_base;
  pnh.param<double>("steer_smoothing_alpha", g_alpha_ema, 0.2);
  pnh.param<double>("max_steer_delta_per_cycle", g_max_delta, 0.08);
  
  // Speed parameters
  pnh.param<double>("base_speed_mps", g_base_speed_mps, 7.0);
  pnh.param<double>("min_speed_mps", g_min_speed_mps, 0.8);
  pnh.param<double>("speed_drop_gain", g_speed_drop_gain, 0.5);
  
  // Motor parameters
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 1200.0);
  pnh.param<double>("motor_max_cmd_red", g_motor_max_cmd_red, 900.0);
  pnh.param<double>("motor_max_cmd_blue", g_motor_max_cmd_blue, 2000.0);
  pnh.param<double>("motor_gain", g_motor_gain, 300.0);
  
  // PID parameters
  pnh.param<double>("pid_kp", g_pid_kp, 0.01);
  pnh.param<double>("pid_ki", g_pid_ki, 0.0);
  pnh.param<double>("pid_kd", g_pid_kd, 0.0001);
  pnh.param<double>("pid_weight", g_pid_weight, 0.0);
  
  // Timeout
  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);
  
  // Subscribe
  g_center_sub = nh.subscribe(g_center_topic, 20, centerCB);
  g_curvature_sub = nh.subscribe(g_curvature_topic, 10, curvatureCB);
  g_center_color_sub = nh.subscribe(g_center_color_topic, 10, centerColorCB);
  
  // Advertise
  g_motor_pub = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_servo_pub = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);
  
  ROS_INFO("[decision] Subscribe center='%s'",
          ros::names::resolve(g_center_topic).c_str());
  ROS_INFO("[decision] Publish motor='%s', servo='%s'",
          ros::names::resolve(g_motor_topic).c_str(),
          ros::names::resolve(g_servo_topic).c_str());
  
  ROS_INFO("[decision] decision_init done");
}

// -------------------- 제어 루프 (매 주기마다 호출) --------------------
void Lane_step()
{
  ros::Time now = ros::Time::now();
  bool have_dx = false;
  
  if (g_have_cb_time) {
    double dt = (now - g_last_cb_time).toSec();
    have_dx = (dt <= g_dx_timeout_sec);
  }
  
  double steer_cmd = 0.0;
  double speed_cmd = 0.0;
  
  if (have_dx) {
    steer_cmd = g_latest_steer_cmd;
    speed_cmd = g_latest_speed_cmd;
  } else {
    steer_cmd = 0.0;
    speed_cmd = 0.0;
  }
  
  // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);
  
  double servo_range = std::min(g_servo_center - g_servo_min,
                               g_servo_max - g_servo_center);
  double servo_hw = g_servo_center + steer_norm * servo_range;
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);
  
  // ---- 2) 속도 변환: m/s -> 모터 명령 ----
  double motor_cmd = g_motor_gain * speed_cmd;
  
  // Color-based motor command override
  if (g_lane_color_code == 1) {          // Red
    motor_cmd = g_motor_max_cmd_red;
  } else if (g_lane_color_code == 2) {   // Blue
    motor_cmd = g_motor_max_cmd_blue;
  } else {
    motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);
  }
  
  // ---- 3) Publish ----
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;
  
  g_motor_pub.publish(motor_msg);
  g_servo_pub.publish(servo_msg);
  
  ROS_DEBUG_THROTTLE(0.5,
    "[decision] steer=%.3f servo=%.3f motor=%.1f speed=%.2f",
    steer_cmd, servo_hw, motor_cmd, speed_cmd);
}

// -------------------- Init & Step Functions --------------------
void lane_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  decision_init(nh, pnh);
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_decision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  lane_decision_init(nh, pnh);
  
  ros::Rate loop_rate(30);  // 30 Hz control loop
  
  while (ros::ok()) {
    ros::spinOnce();
    Lane_step();
