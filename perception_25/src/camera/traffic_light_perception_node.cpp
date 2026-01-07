// traffic_light_perception_node.cpp
// Traffic Light Detection Node - 신호등 인식

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

// -------------------- 전역 상태 --------------------
static std::string g_camera_topic;
static std::string g_traffic_light_state_topic;

static ros::Subscriber g_image_sub;
static ros::Publisher g_state_pub;
static ros::Publisher g_debug_pub;

// Detection parameters
static double g_roi_top = 0;
static double g_roi_bottom = 240;
static double g_roi_left = 0;
static double g_roi_right = 640;

// Red detection
static int g_red_h_low1 = 0;
static int g_red_h_high1 = 10;
static int g_red_h_low2 = 170;
static int g_red_h_high2 = 180;
static int g_red_s_low = 100;
static int g_red_s_high = 255;
static int g_red_v_low = 100;
static int g_red_v_high = 255;

// Green detection
static int g_green_h_low = 35;
static int g_green_h_high = 85;
static int g_green_s_low = 80;
static int g_green_s_high = 255;
static int g_green_v_low = 80;
static int g_green_v_high = 255;

// Shape detection
static int g_min_area = 500;
static double g_circularity_thresh = 0.5;

// State
static std::string g_current_state = "UNKNOWN";
static std::string g_last_state = "UNKNOWN";

// -------------------- Helper functions --------------------
static cv::Mat detectColor(const cv::Mat& hsv, 
                          int h_low1, int h_high1, int s_low, int s_high, 
                          int v_low, int v_high,
                          int h_low2 = -1, int h_high2 = -1)
{
  cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U);
  
  // First range
  cv::Mat mask1;
  cv::inRange(hsv, cv::Scalar(h_low1, s_low, v_low), 
              cv::Scalar(h_high1, s_high, v_high), mask1);
  
  // Second range (for red wrap-around)
  if (h_low2 >= 0 && h_high2 >= 0) {
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(h_low2, s_low, v_low),
                cv::Scalar(h_high2, s_high, v_high), mask2);
    mask = mask1 | mask2;
  } else {
    mask = mask1;
  }
  
  return mask;
}

static bool isRectangular(const std::vector<cv::Point>& contour)
{
  double area = cv::contourArea(contour);
  if (area < g_min_area) {
    return false;
  }
  
  double perimeter = cv::arcLength(contour, true);
  if (perimeter < 1e-6) {
    return false;
  }
  
  double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
  return circularity > g_circularity_thresh;
}

// -------------------- Callback --------------------
static void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = 
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;
    
    if (frame.empty()) {
      ROS_WARN("[traffic_light_perception] Empty image received");
      return;
    }
    
    // Extract ROI
    cv::Mat roi = frame(cv::Range(static_cast<int>(g_roi_top), static_cast<int>(g_roi_bottom)),
                       cv::Range(static_cast<int>(g_roi_left), static_cast<int>(g_roi_right)));
    
    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    
    // Detect red (with wrap-around)
    cv::Mat red_mask = detectColor(hsv, 
                                   g_red_h_low1, g_red_h_high1,
                                   g_red_s_low, g_red_s_high,
                                   g_red_v_low, g_red_v_high,
                                   g_red_h_low2, g_red_h_high2);
    
    // Detect green
    cv::Mat green_mask = detectColor(hsv,
                                     g_green_h_low, g_green_h_high,
                                     g_green_s_low, g_green_s_high,
                                     g_green_v_low, g_green_v_high);
    
    // Find contours
    std::vector<std::vector<cv::Point>> red_contours, green_contours;
    cv::findContours(red_mask.clone(), red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(green_mask.clone(), green_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Check for valid detections
    bool red_detected = false;
    bool green_detected = false;
    
    for (const auto& contour : red_contours) {
      if (isRectangular(contour)) {
        red_detected = true;
        break;
      }
    }
    
    for (const auto& contour : green_contours) {
      if (isRectangular(contour)) {
        green_detected = true;
        break;
      }
    }
    
    // State machine
    if (red_detected && !green_detected) {
      g_current_state = "RED";
    } else if (green_detected && !red_detected) {
      g_current_state = "GREEN";
    } else {
      g_current_state = "UNKNOWN";
    }
    
    // Publish state
    std_msgs::String state_msg;
    state_msg.data = g_current_state;
    g_state_pub.publish(state_msg);
    
    // Log state changes
    if (g_current_state != g_last_state) {
      ROS_INFO("[traffic_light_perception] State changed: %s -> %s",
              g_last_state.c_str(), g_current_state.c_str());
      g_last_state = g_current_state;
    }
    
    // Debug visualization
    if (g_debug_pub.get_num_connections() > 0) {
      cv::Mat debug_image = frame.clone();
      
      // Draw ROI
      cv::rectangle(debug_image,
                   cv::Point(static_cast<int>(g_roi_left), static_cast<int>(g_roi_top)),
                   cv::Point(static_cast<int>(g_roi_right), static_cast<int>(g_roi_bottom)),
                   cv::Scalar(255, 255, 0), 2);
      
      // Draw detections
      int offset_x = static_cast<int>(g_roi_left);
      int offset_y = static_cast<int>(g_roi_top);
      
      for (const auto& contour : red_contours) {
        if (isRectangular(contour)) {
          cv::Rect rect = cv::boundingRect(contour);
          cv::rectangle(debug_image,
                       cv::Point(rect.x + offset_x, rect.y + offset_y),
                       cv::Point(rect.x + rect.width + offset_x, rect.y + rect.height + offset_y),
                       cv::Scalar(0, 0, 255), 3);
        }
      }
      
      for (const auto& contour : green_contours) {
        if (isRectangular(contour)) {
          cv::Rect rect = cv::boundingRect(contour);
          cv::rectangle(debug_image,
                       cv::Point(rect.x + offset_x, rect.y + offset_y),
                       cv::Point(rect.x + rect.width + offset_x, rect.y + rect.height + offset_y),
                       cv::Scalar(0, 255, 0), 3);
        }
      }
      
      // State text
      cv::Scalar color;
      if (g_current_state == "RED") {
        color = cv::Scalar(0, 0, 255);
      } else if (g_current_state == "GREEN") {
        color = cv::Scalar(0, 255, 0);
      } else {
        color = cv::Scalar(128, 128, 128);
      }
      
      cv::putText(debug_image, std::string("State: ") + g_current_state, 
                 cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
      
      g_debug_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg());
    }
    
    ROS_DEBUG_THROTTLE(1.0, "[traffic_light_perception] state=%s red=%d green=%d",
                      g_current_state.c_str(), red_detected ? 1 : 0, green_detected ? 1 : 0);
    
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("[traffic_light_perception] cv_bridge exception: %s", e.what());
  }
}

// -------------------- Init function --------------------
void traffic_light_perception_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[traffic_light_perception] traffic_light_perception_init()");
  
  // Load parameters
  pnh.param<std::string>("camera_topic", g_camera_topic, "/usb_cam/image_rect_color");
  pnh.param<std::string>("traffic_light_state_topic", g_traffic_light_state_topic,
                         "/webot/traffic_light/state");
  
  // ROI parameters
  pnh.param<double>("roi_top", g_roi_top, 0.0);
  pnh.param<double>("roi_bottom", g_roi_bottom, 240.0);
  pnh.param<double>("roi_left", g_roi_left, 0.0);
  pnh.param<double>("roi_right", g_roi_right, 640.0);
  
  // Red detection
  pnh.param<int>("red_h_low1", g_red_h_low1, 0);
  pnh.param<int>("red_h_high1", g_red_h_high1, 10);
  pnh.param<int>("red_h_low2", g_red_h_low2, 170);
  pnh.param<int>("red_h_high2", g_red_h_high2, 180);
  pnh.param<int>("red_s_low", g_red_s_low, 100);
  pnh.param<int>("red_s_high", g_red_s_high, 255);
  pnh.param<int>("red_v_low", g_red_v_low, 100);
  pnh.param<int>("red_v_high", g_red_v_high, 255);
  
  // Green detection
  pnh.param<int>("green_h_low", g_green_h_low, 35);
  pnh.param<int>("green_h_high", g_green_h_high, 85);
  pnh.param<int>("green_s_low", g_green_s_low, 80);
  pnh.param<int>("green_s_high", g_green_s_high, 255);
  pnh.param<int>("green_v_low", g_green_v_low, 80);
  pnh.param<int>("green_v_high", g_green_v_high, 255);
  
  // Shape detection
  pnh.param<int>("min_area", g_min_area, 500);
  pnh.param<double>("circularity_thresh", g_circularity_thresh, 0.5);
  
  // Subscribe to camera
  g_image_sub = nh.subscribe(g_camera_topic, 5, imageCB);
  
  // Advertise state
  g_state_pub = nh.advertise<std_msgs::String>(g_traffic_light_state_topic, 10);
  g_debug_pub = nh.advertise<sensor_msgs::Image>("/webot/traffic_light/debug", 10);
  
  ROS_INFO("[traffic_light_perception] Camera topic: %s",
          ros::names::resolve(g_camera_topic).c_str());
  ROS_INFO("[traffic_light_perception] Publishing state: %s",
          ros::names::resolve(g_traffic_light_state_topic).c_str());
  ROS_INFO("[traffic_light_perception] ROI: (%d,%d) - (%d,%d)",
          (int)g_roi_left, (int)g_roi_top, (int)g_roi_right, (int)g_roi_bottom);
  
  ROS_INFO("[traffic_light_perception] traffic_light_perception_init done");
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "traffic_light_perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  traffic_light_perception_init(nh, pnh);
  
  ros::spin();
  
  return 0;
}
