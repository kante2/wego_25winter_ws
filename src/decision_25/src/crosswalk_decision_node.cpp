#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

// -------------------- Global State --------------------
// Input subscriptions
std::string g_crosswalk_detected_topic = "/webot/crosswalk/detected";
std::string g_stripe_ratio_topic = "/webot/crosswalk/stripe_ratio";
std::string g_lane_steering_topic = "/webot/steering_offset";
std::string g_lane_speed_topic = "/webot/lane_speed";

// Output publications
std::string g_motor_speed_topic = "/commands/motor/speed";
std::string g_servo_position_topic = "/commands/servo/position";
std::string g_crosswalk_stop_topic = "/webot/crosswalk/stop";
std::string g_crosswalk_state_topic = "/webot/crosswalk/state";

ros::Publisher g_pub_motor_speed;
ros::Publisher g_pub_servo_position;
ros::Publisher g_pub_crosswalk_stop;
ros::Publisher g_pub_crosswalk_state;

// State variables
bool g_crosswalk_detected = false;
double g_stripe_ratio = 0.0;
double g_lane_steering = 0.0;      // radians
double g_lane_speed = 0.0;         // m/s

// Control parameters
double g_stop_speed_cmd = 0.0;     // Motor command when stopped (typically 0)
double g_servo_center = 0.57;
double g_servo_range = 0.5;        // servo value range
double g_steer_sign = -1.0;        // steering sign correction
double g_motor_gain = 300.0;       // speed (m/s) -> motor command

// State machine
enum class CrosswalkState {
  CLEAR,           // No crosswalk detected, driving normally
  DETECTED,        // Crosswalk detected, initiating stop
  STOPPED,         // Fully stopped at crosswalk
  CROSSING_WAIT    // Waiting for further instruction to cross
};

CrosswalkState g_state = CrosswalkState::CLEAR;
int g_stop_counter = 0;            // Frame counter for stop detection
int g_stop_threshold = 5;          // Frames to confirm stop (30 Hz -> ~167ms)

// -------------------- Callbacks --------------------

void crosswalkDetectedCB(const std_msgs::BoolConstPtr& msg)
{
  g_crosswalk_detected = msg->data;
}

void stripeRatioCB(const std_msgs::Float32ConstPtr& msg)
{
  g_stripe_ratio = msg->data;
}

void laneSteeringCB(const std_msgs::Float32ConstPtr& msg)
{
  g_lane_steering = msg->data;
}

void laneSpeedCB(const std_msgs::Float32ConstPtr& msg)
{
  g_lane_speed = msg->data;
}

// Alternative: If lane perception publishes PointStamped
void laneSteeringPointCB(const geometry_msgs::PointStampedConstPtr& msg)
{
  g_lane_steering = msg->point.x;  // Store x as steering angle
}

void laneSpeedDoubleCB(const std_msgs::Float64ConstPtr& msg)
{
  g_lane_speed = msg->data;
}

// -------------------- Control Logic --------------------

/**
 * Convert steering angle (radians) to servo PWM value (0.0-1.0)
 */
double steeringToServo(double steer_rad)
{
  double servo_val = g_servo_center + g_steer_sign * (steer_rad / M_PI) * g_servo_range;
  servo_val = std::max(0.0, std::min(1.0, servo_val));  // Clamp to [0, 1]
  return servo_val;
}

/**
 * Convert speed (m/s) to motor command (0-1200 typically)
 */
double speedToMotor(double speed_mps)
{
  double motor_cmd = speed_mps * g_motor_gain;
  motor_cmd = std::max(0.0, std::min(1200.0, motor_cmd));  // Clamp to typical motor range
  return motor_cmd;
}

/**
 * Main crosswalk decision control step
 * Implements state machine:
 *   CLEAR -> DETECTED (when crosswalk_detected=true)
 *   DETECTED -> STOPPED (when velocity approaches zero)
 *   STOPPED -> CROSSING_WAIT (ready to continue)
 *   CROSSING_WAIT -> CLEAR (when crosswalk no longer detected)
 */
void crosswalk_decision_step()
{
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  std_msgs::Bool stop_msg;
  std_msgs::String state_msg;

  switch (g_state) {
    case CrosswalkState::CLEAR:
      // Driving normally, following lane perception
      if (g_crosswalk_detected) {
        g_state = CrosswalkState::DETECTED;
        g_stop_counter = 0;
        ROS_INFO("[crosswalk_decision] Crosswalk detected! Initiating stop.");
      } else {
        // Normal lane following
        servo_msg.data = steeringToServo(g_lane_steering);
        motor_msg.data = speedToMotor(g_lane_speed);
        
        g_pub_servo_position.publish(servo_msg);
        g_pub_motor_speed.publish(motor_msg);
        
        stop_msg.data = false;
        g_pub_crosswalk_stop.publish(stop_msg);
        
        state_msg.data = "CLEAR";
        g_pub_crosswalk_state.publish(state_msg);
      }
      break;

    case CrosswalkState::DETECTED:
      // Crosswalk detected, execute emergency stop
      motor_msg.data = g_stop_speed_cmd;
      servo_msg.data = steeringToServo(0.0);  // Center steering
      
      g_pub_servo_position.publish(servo_msg);
      g_pub_motor_speed.publish(motor_msg);
      
      stop_msg.data = true;
      g_pub_crosswalk_stop.publish(stop_msg);
      
      state_msg.data = "DETECTED";
      g_pub_crosswalk_state.publish(state_msg);
      
      g_stop_counter++;
      if (g_stop_counter >= g_stop_threshold) {
        g_state = CrosswalkState::STOPPED;
        ROS_INFO("[crosswalk_decision] Vehicle stopped at crosswalk.");
      }
      break;

    case CrosswalkState::STOPPED:
      // Fully stopped at crosswalk, wait for signal
      motor_msg.data = g_stop_speed_cmd;
      servo_msg.data = steeringToServo(0.0);
      
      g_pub_servo_position.publish(servo_msg);
      g_pub_motor_speed.publish(motor_msg);
      
      stop_msg.data = true;
      g_pub_crosswalk_stop.publish(stop_msg);
      
      state_msg.data = "STOPPED";
      g_pub_crosswalk_state.publish(state_msg);
      
      if (!g_crosswalk_detected) {
        g_state = CrosswalkState::CROSSING_WAIT;
        ROS_INFO("[crosswalk_decision] Crosswalk area cleared, ready to continue.");
      }
      break;

    case CrosswalkState::CROSSING_WAIT:
      // Crosswalk passed, return to normal driving
      servo_msg.data = steeringToServo(g_lane_steering);
      motor_msg.data = speedToMotor(g_lane_speed);
      
      g_pub_servo_position.publish(servo_msg);
      g_pub_motor_speed.publish(motor_msg);
      
      stop_msg.data = false;
      g_pub_crosswalk_stop.publish(stop_msg);
      
      state_msg.data = "CLEAR";
      g_pub_crosswalk_state.publish(state_msg);
      
      g_state = CrosswalkState::CLEAR;
      ROS_INFO("[crosswalk_decision] Returning to normal lane following.");
      break;

    default:
      ROS_WARN("[crosswalk_decision] Unknown state!");
      g_state = CrosswalkState::CLEAR;
      break;
  }
}

// -------------------- Init & Step Functions --------------------
void crosswalk_decision_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  // Load parameters
  pnh.param<std::string>("crosswalk_detected_topic", g_crosswalk_detected_topic,
                         "/webot/crosswalk/detected");
  pnh.param<std::string>("stripe_ratio_topic", g_stripe_ratio_topic,
                         "/webot/crosswalk/stripe_ratio");
  pnh.param<std::string>("lane_steering_topic", g_lane_steering_topic,
                         "/webot/steering_offset");
  pnh.param<std::string>("lane_speed_topic", g_lane_speed_topic,
                         "/webot/lane_speed");
  
  pnh.param<std::string>("motor_speed_topic", g_motor_speed_topic,
                         "/commands/motor/speed");
  pnh.param<std::string>("servo_position_topic", g_servo_position_topic,
                         "/commands/servo/position");
  pnh.param<std::string>("crosswalk_stop_topic", g_crosswalk_stop_topic,
                         "/webot/crosswalk/stop");
  pnh.param<std::string>("crosswalk_state_topic", g_crosswalk_state_topic,
                         "/webot/crosswalk/state");
  
  // Control parameters
  pnh.param<double>("servo_center", g_servo_center, 0.57);
  pnh.param<double>("servo_range", g_servo_range, 0.5);
  pnh.param<double>("steer_sign", g_steer_sign, -1.0);
  pnh.param<double>("motor_gain", g_motor_gain, 300.0);
  pnh.param<double>("stop_speed_cmd", g_stop_speed_cmd, 0.0);
  pnh.param<int>("stop_threshold", g_stop_threshold, 5);

  // Subscriptions
  nh.subscribe(g_crosswalk_detected_topic, 1, crosswalkDetectedCB);
  nh.subscribe(g_stripe_ratio_topic, 1, stripeRatioCB);
  nh.subscribe(g_lane_steering_topic, 1, laneSteeringCB);
  nh.subscribe(g_lane_speed_topic, 1, laneSpeedCB);

  // Publications
  g_pub_motor_speed = nh.advertise<std_msgs::Float64>(g_motor_speed_topic, 1);
  g_pub_servo_position = nh.advertise<std_msgs::Float64>(g_servo_position_topic, 1);
  g_pub_crosswalk_stop = nh.advertise<std_msgs::Bool>(g_crosswalk_stop_topic, 1);
  g_pub_crosswalk_state = nh.advertise<std_msgs::String>(g_crosswalk_state_topic, 1);

  ROS_INFO("[decision] Crosswalk decision init done");
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "crosswalk_decision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  crosswalk_decision_init(nh, pnh);

  ROS_INFO("[decision] Crosswalk decision node started");

  // Control loop at 30 Hz
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    crosswalk_decision_step();
    loop_rate.sleep();
  }

  return 0;
}
