// main_node.cpp
// Decision 25 Main Node
// perception_25의 감지 결과를 받아서 우선순위를 정해 각 decision step을 실행

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>

// -------------------- Decision Node 함수 선언 --------------------
// lane_decision_node.cpp
void lane_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void Lane_step();

// obstacle_avoid_decision_node.cpp
void obstacle_avoid_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void obstacle_avoid_decision_step();

// traffic_light_decision_node.cpp
void traffic_light_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void traffic_light_decision_step();

// crosswalk_decision_node.cpp
void crosswalk_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void crosswalk_decision_step();

// -------------------- Decision 상태 Enum --------------------
enum DecisionState
{
  STATE_LANE = 0,
  STATE_CROSSWALK,
  STATE_TRAFFIC_LIGHT,
  STATE_OBSTACLE_AVOID
};

DecisionState g_current_state = STATE_LANE;

// -------------------- 감지 플래그 --------------------
bool g_crosswalk_detected = false;
bool g_traffic_light_red = false;
bool g_obstacle_detected = false;

// 상태 히스테리시스
const double k_crosswalk_confirm_sec = 0.3;
const double k_crosswalk_release_sec = 0.3;
bool g_crosswalk_confirmed = false;
ros::Time g_crosswalk_on_start_time;
ros::Time g_crosswalk_last_seen;

const double k_obstacle_confirm_sec = 0.2;
const double k_obstacle_release_sec = 0.5;
bool g_obstacle_confirmed = false;
ros::Time g_obstacle_on_start_time;
ros::Time g_obstacle_last_seen;

// -------------------- 콜백: 횡단보도 감지 --------------------
void CB_CrosswalkDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_crosswalk_detected = msg->data;
  if (msg->data) {
    g_crosswalk_last_seen = ros::Time::now();
  }
}

// -------------------- 콜백: 신호등 상태 --------------------
void CB_TrafficLightState(const std_msgs::String::ConstPtr& msg)
{
  g_traffic_light_red = (msg->data == "RED");
}

// -------------------- 콜백: 장애물 감지 (LiDAR) --------------------
void CB_ObstacleDetected(const std_msgs::Int32::ConstPtr& msg)
{
  g_obstacle_detected = (msg->data != 0);
  if (msg->data) {
    g_obstacle_last_seen = ros::Time::now();
  }
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_25_main");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[main_node] Decision 25 main_node started");

  // ===== 감지 토픽 이름 파라미터 =====
  std::string topic_crosswalk_detected;
  std::string topic_traffic_light_state;
  std::string topic_obstacle_has;

  pnh.param<std::string>("crosswalk_detected_topic", topic_crosswalk_detected, 
                         std::string("/webot/crosswalk/detected"));
  pnh.param<std::string>("traffic_light_state_topic", topic_traffic_light_state,
                         std::string("/webot/traffic_light/state"));
  pnh.param<std::string>("obstacle_has_topic", topic_obstacle_has,
                         std::string("/webot/obstacle/has_obstacle"));

  // ===== 감지 토픽 구독 =====
  ros::Subscriber sub_crosswalk = nh.subscribe(topic_crosswalk_detected, 1, CB_CrosswalkDetected);
  ros::Subscriber sub_traffic = nh.subscribe(topic_traffic_light_state, 1, CB_TrafficLightState);
  ros::Subscriber sub_obstacle = nh.subscribe(topic_obstacle_has, 1, CB_ObstacleDetected);

  // ===== 각 Decision Node 초기화 =====
  lane_decision_init(nh, pnh);
  obstacle_avoid_decision_init(nh, pnh);
  traffic_light_decision_init(nh, pnh);
  crosswalk_decision_init(nh, pnh);

  ROS_INFO("[main_node] all decision nodes init done");

  ros::Rate rate(30.0);  // 30 Hz (perception과 동기화)

  DecisionState prev_state = g_current_state;

  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();

    // =====================================
    // 1) 횡단보도 감지 히스테리시스 (연속 on/off 시간 요구)
    // =====================================
    if (g_crosswalk_detected)
    {
      if (g_crosswalk_on_start_time.isZero())
      {
        g_crosswalk_on_start_time = now;
      }
      double on_elapsed = (now - g_crosswalk_on_start_time).toSec();
      if (on_elapsed >= k_crosswalk_confirm_sec)
      {
        g_crosswalk_confirmed = true;
      }
      g_crosswalk_last_seen = now;
    }
    else
    {
      if (!g_crosswalk_last_seen.isZero())
      {
        double off_elapsed = (now - g_crosswalk_last_seen).toSec();
        if (off_elapsed >= k_crosswalk_release_sec)
        {
          g_crosswalk_confirmed = false;
          g_crosswalk_on_start_time = ros::Time(0);
        }
      }
      else
      {
        g_crosswalk_confirmed = false;
      }
    }

    // =====================================
    // 2) 장애물 감지 히스테리시스
    // =====================================
    if (g_obstacle_detected)
    {
      if (g_obstacle_on_start_time.isZero())
      {
        g_obstacle_on_start_time = now;
      }
      double on_elapsed = (now - g_obstacle_on_start_time).toSec();
      if (on_elapsed >= k_obstacle_confirm_sec)
      {
        g_obstacle_confirmed = true;
      }
      g_obstacle_last_seen = now;
    }
    else
    {
      if (!g_obstacle_last_seen.isZero())
      {
        double off_elapsed = (now - g_obstacle_last_seen).toSec();
        if (off_elapsed >= k_obstacle_release_sec)
        {
          g_obstacle_confirmed = false;
          g_obstacle_on_start_time = ros::Time(0);
        }
      }
      else
      {
        g_obstacle_confirmed = false;
      }
    }

    // 디버깅: 상태 결정 전에 주요 플래그 로그 (1초 주기)
    ROS_INFO_THROTTLE(1.0,
                      "[main_node] flags: crosswalk=%d traffic_red=%d obstacle=%d | "
                      "confirmed: cw=%d obs=%d",
                      (int)g_crosswalk_detected, (int)g_traffic_light_red, (int)g_obstacle_detected,
                      (int)g_crosswalk_confirmed, (int)g_obstacle_confirmed);

    // =====================================
    // 3) Decision 상태 결정 로직
    //    우선순위: Crosswalk > TrafficLight(RED) > Obstacle > Lane
    // =====================================
    if (g_crosswalk_confirmed)
    {
      g_current_state = STATE_CROSSWALK;
    }
    else if (g_traffic_light_red)
    {
      g_current_state = STATE_TRAFFIC_LIGHT;
    }
    else if (g_obstacle_confirmed)
    {
      g_current_state = STATE_OBSTACLE_AVOID;
    }
    else
    {
      g_current_state = STATE_LANE;
    }

    // =====================================
    // 4) 상태 변경 시 로그
    // =====================================
    if (g_current_state != prev_state)
    {
      const char* state_name = "LANE";
      if (g_current_state == STATE_CROSSWALK)
        state_name = "CROSSWALK";
      else if (g_current_state == STATE_TRAFFIC_LIGHT)
        state_name = "TRAFFIC_LIGHT";
      else if (g_current_state == STATE_OBSTACLE_AVOID)
        state_name = "OBSTACLE_AVOID";

      ROS_INFO("[main_node] Decision State Changed -> %s", state_name);

      prev_state = g_current_state;
    }

    // =====================================
    // 5) 현재 상태에 맞는 Decision Step 실행
    // =====================================
    switch (g_current_state)
    {
      case STATE_LANE:
        ROS_DEBUG("[main_node] STATE_LANE");
        Lane_step();
        break;

      case STATE_CROSSWALK:
        ROS_DEBUG("[main_node] STATE_CROSSWALK");
        crosswalk_decision_step();
        break;

      case STATE_TRAFFIC_LIGHT:
        ROS_DEBUG("[main_node] STATE_TRAFFIC_LIGHT");
        traffic_light_decision_step();
        break;

      case STATE_OBSTACLE_AVOID:
        ROS_DEBUG("[main_node] STATE_OBSTACLE_AVOID");
        obstacle_avoid_decision_step();
        break;

      default:
        ROS_DEBUG("[main_node] STATE_LANE (default)");
        Lane_step();
        break;
    }

    rate.sleep();
  }

  return 0;
}
