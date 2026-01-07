#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// -------------------- Global State --------------------
ros::Publisher g_pub_crosswalk_detected;  // /perception/crosswalk/detected (Bool)
ros::Publisher g_pub_stripe_ratio;        // /perception/crosswalk/stripe_ratio (Float32)

std::string g_camera_topic = "/usb_cam/image_rect_color";
std::string g_crosswalk_detected_topic = "/perception/crosswalk/detected";
std::string g_stripe_ratio_topic = "/perception/crosswalk/stripe_ratio";

bool g_show_window = false;
std::string g_win_name = "crosswalk_bev";

// ROI parameters (trapezoid perspective)
double g_roi_top_y_ratio = 0.60;
double g_roi_left_top_ratio = 0.22;
double g_roi_right_top_ratio = 0.78;
double g_roi_left_bot_ratio = -0.40;
double g_roi_right_bot_ratio = 1.40;

// HSV range for white/yellow stripes
cv::Scalar g_yellow_lower(10, 80, 60);
cv::Scalar g_yellow_upper(45, 255, 255);
cv::Scalar g_white_lower(0, 0, 170);
cv::Scalar g_white_upper(179, 60, 255);

// Detection parameters
bool g_use_yellow = false;
bool g_use_white = true;
double g_stripe_ratio_threshold = 0.30;

// -------------------- Helper Functions --------------------

/**
 * Create trapezoid ROI polygon for perspective transform
 * Returns 4 points: [bottom-left, top-left, top-right, bottom-right]
 */
void makeRoiPolygon(int h, int w, std::vector<cv::Point>& poly_out)
{
  int y_top = static_cast<int>(h * g_roi_top_y_ratio);
  int y_bot = h - 1;
  int x_lt = static_cast<int>(w * g_roi_left_top_ratio);
  int x_rt = static_cast<int>(w * g_roi_right_top_ratio);
  int x_lb = static_cast<int>(w * g_roi_left_bot_ratio);
  int x_rb = static_cast<int>(w * g_roi_right_bot_ratio);

  poly_out.clear();
  poly_out.emplace_back(x_lb, y_bot);  // bottom-left
  poly_out.emplace_back(x_lt, y_top);  // top-left
  poly_out.emplace_back(x_rt, y_top);  // top-right
  poly_out.emplace_back(x_rb, y_bot);  // bottom-right
}

/**
 * Apply perspective transform (trapezoid ROI -> rectangular BEV)
 */
cv::Mat warpToBev(const cv::Mat& bgr, const std::vector<cv::Point>& roi_poly)
{
  int h = bgr.rows;
  int w = bgr.cols;

  cv::Point2f BL = roi_poly[0];
  cv::Point2f TL = roi_poly[1];
  cv::Point2f TR = roi_poly[2];
  cv::Point2f BR = roi_poly[3];

  // Clamp y coordinates
  BL.y = std::max(0.f, std::min(static_cast<float>(h - 1), BL.y));
  TL.y = std::max(0.f, std::min(static_cast<float>(h - 1), TL.y));
  TR.y = std::max(0.f, std::min(static_cast<float>(h - 1), TR.y));
  BR.y = std::max(0.f, std::min(static_cast<float>(h - 1), BR.y));

  std::vector<cv::Point2f> src, dst;
  src.push_back(BL);
  src.push_back(TL);
  src.push_back(TR);
  src.push_back(BR);

  dst.push_back(cv::Point2f(0, h - 1));
  dst.push_back(cv::Point2f(0, 0));
  dst.push_back(cv::Point2f(w - 1, 0));
  dst.push_back(cv::Point2f(w - 1, h - 1));

  cv::Mat M = cv::getPerspectiveTransform(src, dst);
  cv::Mat bev;
  cv::warpPerspective(bgr, bev, M, cv::Size(w, h),
                      cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT,
                      cv::Scalar(0, 0, 0));
  return bev;
}

/**
 * Create binary mask for white/yellow stripes
 */
cv::Mat binarizeStripes(const cv::Mat& bgr)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask_y, mask_w, mask;

  // Yellow stripes
  if (g_use_yellow) {
    cv::inRange(hsv, g_yellow_lower, g_yellow_upper, mask_y);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_y, mask_y, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  }

  // White stripes
  if (g_use_white) {
    cv::inRange(hsv, g_white_lower, g_white_upper, mask_w);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_w, mask_w, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  }

  // Combine masks
  if (!mask_y.empty() && !mask_w.empty()) {
    cv::bitwise_or(mask_y, mask_w, mask);
  } else if (!mask_y.empty()) {
    mask = mask_y;
  } else if (!mask_w.empty()) {
    mask = mask_w;
  } else {
    mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
  }

  return mask;
}

/**
 * Compute white/yellow stripe ratio in the binary image
 */
double computeStripeRatio(const cv::Mat& binary)
{
  int h = binary.rows;
  int w = binary.cols;

  if (w <= 0 || h <= 0) {
    return 0.0;
  }

  const int total_pixels = w * h;
  const int total_white = cv::countNonZero(binary);
  const double stripe_ratio = static_cast<double>(total_white) / static_cast<double>(total_pixels);

  return stripe_ratio;
}

/**
 * Detect crosswalk from camera image
 */
void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat bgr = cv_ptr->image.clone();
    
    if (bgr.empty()) return;

    // Convert grayscale to BGR if needed
    if (bgr.channels() == 1) {
      cv::cvtColor(bgr, bgr, cv::COLOR_GRAY2BGR);
    }

    int h = bgr.rows;
    int w = bgr.cols;

    // Create ROI polygon
    std::vector<cv::Point> roi_poly_pts;
    makeRoiPolygon(h, w, roi_poly_pts);

    // Warp to BEV (Bird's Eye View)
    cv::Mat bev_bgr = warpToBev(bgr, roi_poly_pts);

    // Binarize stripes
    cv::Mat bev_binary = binarizeStripes(bev_bgr);

    // Compute stripe ratio
    double stripe_ratio = computeStripeRatio(bev_binary);

    // Publish stripe ratio
    std_msgs::Float32 ratio_msg;
    ratio_msg.data = static_cast<float>(stripe_ratio);
    g_pub_stripe_ratio.publish(ratio_msg);

    // Detect crosswalk (simple threshold)
    bool crosswalk_detected = (stripe_ratio > g_stripe_ratio_threshold);

    // Publish detection result
    std_msgs::Bool crosswalk_msg;
    crosswalk_msg.data = crosswalk_detected;
    g_pub_crosswalk_detected.publish(crosswalk_msg);

    // Optional: Debug visualization
    if (g_show_window) {
      cv::imshow(g_win_name, bev_binary);
      cv::waitKey(1);
    }

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("[crosswalk_perception] cv_bridge exception: %s", e.what());
    return;
  }
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "crosswalk_perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Load parameters
  pnh.param<std::string>("camera_topic", g_camera_topic, 
                         "/usb_cam/image_rect_color");
  pnh.param<std::string>("crosswalk_detected_topic", g_crosswalk_detected_topic,
                         "/webot/crosswalk/detected");
  pnh.param<std::string>("stripe_ratio_topic", g_stripe_ratio_topic,
                         "/webot/crosswalk/stripe_ratio");
  
  pnh.param<bool>("show_window", g_show_window, false);
  pnh.param<double>("stripe_ratio_threshold", g_stripe_ratio_threshold, 0.30);
  
  pnh.param<double>("roi_top_y_ratio", g_roi_top_y_ratio, 0.60);
  pnh.param<double>("roi_left_top_ratio", g_roi_left_top_ratio, 0.22);
  pnh.param<double>("roi_right_top_ratio", g_roi_right_top_ratio, 0.78);
  pnh.param<double>("roi_left_bot_ratio", g_roi_left_bot_ratio, -0.40);
  pnh.param<double>("roi_right_bot_ratio", g_roi_right_bot_ratio, 1.40);
  
  pnh.param<bool>("use_yellow_lanes", g_use_yellow, false);
  pnh.param<bool>("use_white_lanes", g_use_white, true);

  // Subscribe to camera
  ros::Subscriber img_sub = nh.subscribe(g_camera_topic, 2, imageCB);

  // Advertise output topics
  g_pub_crosswalk_detected = 
      nh.advertise<std_msgs::Bool>(g_crosswalk_detected_topic, 1);
  g_pub_stripe_ratio = 
      nh.advertise<std_msgs::Float32>(g_stripe_ratio_topic, 1);

  if (g_show_window) {
    cv::namedWindow(g_win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_name, 960, 540);
  }

  ROS_INFO("[perception] Crosswalk perception node started");
  ROS_INFO("[perception] Camera topic: %s",
           ros::names::resolve(g_camera_topic).c_str());
  ROS_INFO("[perception] Publishing detected: %s",
           ros::names::resolve(g_crosswalk_detected_topic).c_str());
  ROS_INFO("[perception] Publishing stripe_ratio: %s",
           ros::names::resolve(g_stripe_ratio_topic).c_str());
  ROS_INFO("[perception] Stripe detection: Yellow=%s, White=%s",
           g_use_yellow ? "true" : "false", g_use_white ? "true" : "false");
  ROS_INFO("[perception] Stripe ratio threshold: %.3f", g_stripe_ratio_threshold);

  ros::spin();
  return 0;
}
