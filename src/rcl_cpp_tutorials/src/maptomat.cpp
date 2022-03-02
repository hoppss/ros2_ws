#include <iostream>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

  cv::Mat mapToMat(const nav_msgs::msg::OccupancyGrid& grid) {
    /**
     *
     * **/
    int grid_size = grid.data.size();
    int grid_height = grid.info.height;
    int grid_width = grid.info.width;

    std::cout << "grid_width " << grid_width << " grid_height " << grid_height
              << std::endl;

    cv::Mat mat = cv::Mat(grid_height, grid_width, CV_8UC1, cv::Scalar(255));

    // for (int k = 0; k < grid_size; k++) {
    //   int pt_y = grid_height - 1 - k / grid_width;
    //   int pt_x = k % grid_width;
    //   int value = grid.data[k];

    //   unsigned char img_value;
    //   if (value == -1)
    //     img_value = 120;
    //   else if (value >= 100)
    //     img_value = 0;
    //   else if (value < 50)
    //     img_value = 255;
    //   mat.at<uchar>(pt_y, pt_x) = img_value;  //(y, x)
    // }

    for (int h = 0; h < grid_height; ++h) {
      for (int r = 0; r < grid_width; ++r) {
        //
        int value = grid.data[grid_width * (grid_height - 1 - h) + r];
        unsigned char img_value;
        if (value == -1)
          img_value = 120;
        else if (value >= 100)
          img_value = 0;
        else if (value < 50)
          img_value = 255;
        mat.at<uchar>(h, r) = img_value;  //(y, x)
      }
    }
    return mat;
  }

  void display(const cv::Mat& mat) {
    cv::namedWindow("static_map", cv::WINDOW_AUTOSIZE);
    cv::imshow("static_map", mat);

    auto k = cv::waitKey(0);

    if (k == 27) {
      cv::destroyAllWindows();
      exit(0);
    }

    cv::imwrite("/home/mi/local.jpg", mat);
  }

  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "callback %ld", msg->header.stamp.nanosec);
    nav_msgs::msg::OccupancyGrid grid = *msg;
    auto mat = mapToMat(grid);
    display(mat);
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

  //   cv::Mat mat_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
