#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

// Global variables
image_transport::Publisher image_pub;
ros::Publisher grid_pub;
std::string video_path;

// Function to convert an image frame to a binary OccupancyGrid message
void frameToGrid(const cv::Mat& frame, nav_msgs::OccupancyGrid& grid) {
  // Convert the image frame to grayscale
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  // Threshold the grayscale image to create a binary image
  cv::Mat binary;
  cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

  // Set the dimensions of the OccupancyGrid message
  grid.header.frame_id = "map";
  grid.info.width = binary.cols;
  grid.info.height = binary.rows;
  grid.info.resolution = 0.1; // 10 cm per grid cell
  grid.data.resize(grid.info.width * grid.info.height);

  // Copy the binary image data into the OccupancyGrid message
  for (int y = 0; y < binary.rows; y++) {
    for (int x = 0; x < binary.cols; x++) {
      if (binary.at<uchar>(y, x) == 0) {
        // Black pixel, set grid cell to occupied
        grid.data[y * binary.cols + x] = 100;
      } else {
        // White pixel, set grid cell to free
        grid.data[y * binary.cols + x] = 0;
      }
    }
  }
}

// Function to load the Bad Apple music video and extract its frames
std::vector<cv::Mat> loadBadAppleVideo() {
  // Open the video file
  cv::VideoCapture capture(video_path);

  // Check if the video file was opened successfully
  if (!capture.isOpened()) {
    throw std::runtime_error("Failed to open video file");
  }

  // Vector to store the extracted frames
  std::vector<cv::Mat> frames;

  // Extract the frames from the video
  cv::Mat frame;
  while (capture.read(frame)) {
    frames.push_back(frame.clone());
  }

  // Return the extracted frames
  return frames;
}

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "bad_apple_publisher");
  ros::NodeHandle nh;

  // Get the video path from the ROS parameter server
  nh.param<std::string>("video_path", video_path, "bad_apple.mp4"); // Default value is "bad_apple.mp4"

  // Load the Bad Apple video frames
  std::vector<cv::Mat> frames = loadBadAppleVideo();

  // Create publishers for the image and OccupancyGrid messages
  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("bad_apple_image", 1);
  grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("bad_apple_grid", 1);

  // Loop through the video frames and publish the image and OccupancyGrid
  ros::Rate loop_rate(30); // Publish at 30 Hz
  for (const cv::Mat& frame : frames) {
    // Convert the image frame to an OccupancyGrid message
    nav_msgs::OccupancyGrid grid;
    frameToGrid(frame, grid);

    // Publish the image and OccupancyGrid messages
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(msg);
    grid_pub.publish(grid);

    // Sleep for the remainder of the loop period
    loop_rate.sleep();
  }

  // Spin until the node is shut down
  ros::spin();
}
