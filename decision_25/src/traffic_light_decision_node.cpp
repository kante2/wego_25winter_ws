// traffic_light_decision_node.cpp
// Traffic Light Decision Node - 신호등 기반 속도 제어

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// -------------------- 전역 상태 --------------------
static std::string g_traffic_light_state_topic;
static std::string g_lane_steering_topic;
static std::string g_lane_speed_topic;
static std::string g_motor_topic;
static std::string g_servo_topic;

static ros::Subscriber g_state_sub;
static ros::Subscriber g_lane_steering_sub;
static ros::Subscriber g_lane_speed_sub;
static ros::Publisher g_motor_pub;
static ros::Publisher g_servo_pub;
static ros::Publisher g_stop_pub;
static ros::Publisher g_passed_pub;
static ros::Publisher g_control_state_pub;

// Servo parameters
static double g_servo_center = 0.57;
static double g_servo_min = 0.0;
static double g_servo_max = 1.0;
static double g_steer_sign = -1.0;

// Motor parameters
static double g_motor_gain = 300.0;
static double g_motor_min_cmd = 0.0;
static double g_motor_max_cmd = 1200.0;

// Internal state
static std::string g_traffic_state = "UNKNOWN";
static double g_lane_steering = 0.0;
static double g_lane_speed = 0.3;
static bool g_green_passed = false;
static std::string g_last_state = "UNKNOWN";

// -------------------- Helper --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 콜백: traffic_light state --------------------
static void trafficStateCB(const std_msgs::String::ConstPtr& msg)
{
  g_traffic_state = msg->data;
}

// -------------------- 콜백: lane steering --------------------
static void laneSteeringCB(const std_msgs::Float32::ConstPtr& msg)
{
  g_lane_steering = msg->data;
}

// -------------------- 콜백: lane speed --------------------
static void laneSpeedCB(const std_msgs::Float32::ConstPtr& msg)
{
  g_lane_speed = msg->data;
}

// -------------------- 초기화 함수 --------------------
void traffic_light_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[traffic_light_decision] traffic_light_decision_init()");
  
  // Load parameters
  pnh.param<std::string>("traffic_light_state_topic", g_traffic_light_state_topic,
                         "/webot/traffic_light/state");
  pnh.param<std::string>("lane_steering_topic", g_lane_steering_topic,
                         "/webot/steering_offset");
  pnh.param<std::string>("lane_speed_topic", g_lane_speed_topic,
                         "/webot/lane_speed");
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         "/commands/motor/speed");
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         "/commands/servo/position");
  
  // Servo parameters
  pnh.param<double>("servo_center", g_servo_center, 0.57);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);
  pnh.param<double>("steer_sign", g_steer_sign, -1.0);
  
  // Motor parameters
  pnh.param<double>("motor_gain", g_motor_gain, 300.0);
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 1200.0);
  
  // Subscribe
  g_state_sub = nh.subscribe(g_traffic_light_state_topic, 10, trafficStateCB);
  g_lane_steering_sub = nh.subscribe(g_lane_steering_topic, 10, laneSteeringCB);
  g_lane_speed_sub = nh.subscribe(g_lane_speed_topic, 10, laneSpeedCB);
  
  // Advertise
  g_motor_pub = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_servo_pub = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);
  g_stop_pub = nh.advertise<std_msgs::Bool>("/webot/traffic_stop", 10);
  g_passed_pub = nh.advertise<std_msgs::Bool>("/webot/traffic_passed", 10);
  g_control_state_pub = nh.advertise<std_msgs::String>("/webot/traffic_light/decision_state", 10);
  
  ROS_INFO("[traffic_light_decision] Subscribe state='%s'",
          ros::names::resolve(g_traffic_light_state_topic).c_str());
  ROS_INFO("[traffic_light_decision] Publish motor='%s', servo='%s'",
          ros::names::resolve(g_motor_topic).c_str(),
          ros::names::resolve(g_servo_topic).c_str());
  ROS_INFO("[traffic_light_decision] RED: Stop | GREEN/UNKNOWN: Lane tracing");
  
  ROS_INFO("[traffic_light_decision] traffic_light_decision_init done");
}

// -------------------- 제어 루프 (매 주기마다 호출) --------------------
void traffic_light_decision_step()
{
  double steer_cmd = 0.0;
  double speed_cmd = 0.0;
  bool stop_signal = false;
  std::string control_state = "UNKNOWN";
  
  // ================ 신호등 상태 기반 제어 ================
  
  if (g_traffic_state == "RED") {
    // RED: 정지
    steer_cmd = 0.0;
    speed_cmd = 0.0;
    stop_signal = true;
    control_state = "RED_STOP";
    g_green_passed = false;  // 다시 초기화
  }
  else if (g_traffic_state == "GREEN") {
    // GREEN: 차선 추적
    steer_cmd = g_lane_steering;
    speed_cmd = g_lane_speed;
    stop_signal = false;
    control_state = "GREEN_DRIVING";
    
    // GREEN 신호 통과 알림 (한 번만)
    if (!g_green_passed) {
      g_green_passed = true;
      std_msgs::Bool passed_msg;
      passed_msg.data = true;
      g_passed_pub.publish(passed_msg);
      ROS_INFO("[traffic_light_decision] GREEN light passed!");
    }
  }
  else {
    // UNKNOWN: 차선 추적 (안전하게)
    steer_cmd = g_lane_steering;
    speed_cmd = g_lane_speed;
    stop_signal = false;
    control_state = "UNKNOWN_DRIVING";
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
  
  // Publish stop signal
  std_msgs::Bool stop_msg;
  stop_msg.data = stop_signal;
  g_stop_pub.publish(stop_msg);
  
  // Publish control state
  std_msgs::String state_msg;
  state_msg.data = control_state;
  g_control_state_pub.publish(state_msg);
  
  if (control_state != g_last_state) {
    ROS_INFO("[traffic_light_decision] Control state changed: %s -> %s",
            g_last_state.c_str(), control_state.c_str());
    g_last_state = control_state;
  }
  
  ROS_DEBUG_THROTTLE(0.5,
    "[traffic_light_decision] traffic=%s control=%s stop=%d steer=%.3f motor=%.1f speed=%.2f",
    g_traffic_state.c_str(), control_state.c_str(), stop_signal ? 1 : 0,
    steer_cmd, motor_cmd, speed_cmd);
}

// -------------------- Main --------------------
int main(int argc, char** argv)
    loop_rate.sleep();
  }
  
  return 0;
}
