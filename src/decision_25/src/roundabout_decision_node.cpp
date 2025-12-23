// roundabout_decision_node.cpp
// Roundabout Decision Node - 전방 장애물 기반 정지 제어
// 단순 로직: 장애물 감지 → 정지 / 장애물 없음 → main에서 라인트레이싱

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

// -------------------- 전역 상태 --------------------
// 입력 토픽
static std::string g_obstacle_detected_topic = "/webot/roundabout/obstacle_detected";
static std::string g_obstacle_count_topic = "/webot/roundabout/obstacle_count";

// 출력 토픽
static std::string g_motor_topic = "/commands/motor/speed";
static std::string g_servo_topic = "/commands/servo/position";
static std::string g_roundabout_state_topic = "/webot/roundabout/state";

// 발행자
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;
static ros::Publisher g_pub_state;

// 제어 파라미터
static double g_servo_center = 0.57;
static double g_steer_sign = -1.0;

// 내부 상태
static bool g_obstacle_detected = false;
static int g_obstacle_count = 0;
static std::string g_state = "IDLE";

// -------------------- 콜백 함수 --------------------

/**
 * 장애물 감지 콜백
 */
void obstacle_detected_callback(const std_msgs::Bool::ConstPtr& msg)
{
  g_obstacle_detected = msg->data;
}

/**
 * 장애물 포인트 수 콜백
 */
void obstacle_count_callback(const std_msgs::Int32::ConstPtr& msg)
{
  g_obstacle_count = msg->data;
}

// -------------------- 제어 로직 --------------------

/**
 * Roundabout 제어 step
 * - 장애물 감지 → 정지 (속도=0, 조향=0)
 * - 장애물 없음 → main에서 라인트레이싱 (publish 안함)
 */
void roundabout_decision_step()
{
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  std_msgs::String state_msg;

  // === 장애물 감지 시 정지 ===
  if (g_obstacle_detected)
  {
    motor_msg.data = 0.0;        // 정지
    servo_msg.data = g_servo_center;  // 중립 조향
    g_state = "STOPPED";
  }
  else
  {
    // 장애물 없음 → 제어권을 main에 반환 (publish 안함)
    g_state = "CLEAR";
    return;
  }

  // === 메시지 발행 ===
  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  state_msg.data = g_state;
  g_pub_state.publish(state_msg);

  ROS_DEBUG_THROTTLE(0.5,
    "[roundabout_decision] state=%s obstacle=%d count=%d motor=%.2f servo=%.3f",
    g_state.c_str(), (int)g_obstacle_detected, g_obstacle_count,
    motor_msg.data, servo_msg.data);
}

// -------------------- 메인 --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "roundabout_decision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[roundabout_decision] Node started");

  // === 파라미터 로드 ===
  pnh.param<std::string>("obstacle_detected_topic", g_obstacle_detected_topic,
                         "/webot/roundabout/obstacle_detected");
  pnh.param<std::string>("obstacle_count_topic", g_obstacle_count_topic,
                         "/webot/roundabout/obstacle_count");
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         "/commands/motor/speed");
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         "/commands/servo/position");
  pnh.param<std::string>("roundabout_state_topic", g_roundabout_state_topic,
                         "/webot/roundabout/state");
  
  pnh.param<double>("servo_center", g_servo_center, 0.57);
  pnh.param<double>("steer_sign", g_steer_sign, -1.0);

  // === 구독자 ===
  ros::Subscriber obstacle_det_sub = 
      nh.subscribe(g_obstacle_detected_topic, 1, obstacle_detected_callback);
  ros::Subscriber obstacle_cnt_sub = 
      nh.subscribe(g_obstacle_count_topic, 1, obstacle_count_callback);

  // === 발행자 ===
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 1);
  g_pub_state = nh.advertise<std_msgs::String>(g_roundabout_state_topic, 1);

  ROS_INFO("[roundabout_decision] Subscribing to: %s",
           ros::names::resolve(g_obstacle_detected_topic).c_str());
  ROS_INFO("[roundabout_decision] Publishing motor/servo to: %s, %s",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  // === 제어 루프 (30 Hz) ===
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    roundabout_decision_step();
    loop_rate.sleep();
  }

  return 0;
}
