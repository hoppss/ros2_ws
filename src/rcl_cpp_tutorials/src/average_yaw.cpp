#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>  // getYaw

#include <angles/angles.h>

#include <iostream>
#include <string>
#include <vector>

void show(std::vector<double>& data){
    for(int i=0; i < data.size(); ++i) {
        std::cout << "i[" <<i <<"]: " <<data[i] << ", ";
    }
    std::cout << std::endl;
}

void averageYaw(std::vector<double>& angles)
{
    std::vector<double> copy = angles;

    show(copy);

    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for(int i=0; i < angles.size(); ++i) {

    //    if(copy[i] < 0.0) copy[i] += M_PI * 2;
    sum_sin += std::sin(copy[i]);
    sum_cos += std::cos(copy[i]);
    }

    show(copy);

    double avg = std::atan2(sum_sin, sum_cos);

    std::cout << "average is " << avg << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("averaging");

  std::vector<double> v;
  v.push_back(10 / 180.0 * M_PI);
  v.push_back(20 / 180.0 * M_PI);
  v.push_back(30 / 180.0 * M_PI);
  averageYaw(v);




//    std::vector<double> v;
//    v.push_back(10 / 180.0 * M_PI);
//    v.push_back(0 / 180.0 * M_PI);
//    v.push_back(-11 / 180.0 * M_PI);

//    averageYaw(v);
  /*
   i[0]: 0.174533, i[1]: 0, i[2]: -0.191986,
   i[0]: 0.174533, i[1]: 0, i[2]: 6.0912,
   average is 2.08858
   */


//    std::vector<double> v;
//    v.push_back(179 / 180.0 * M_PI);
//    v.push_back(-179 / 180.0 * M_PI);
//    v.push_back(-170 / 180.0 * M_PI);

//   averageYaw(v);



}