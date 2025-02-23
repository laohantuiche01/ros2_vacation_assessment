#ifndef ALGORITHM_H
#define ALGORITHM_H


#include "rclcpp/rclcpp.hpp"
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"

class Algorithm : public rclcpp::Node
{
  public:
    Algorithm();
  private:
      std::vector<std::vector<cv::Point>> runing_way_function(cv::Point began_point,cv::Point end_point);
      bool if_wall_towards();

      std::vector<std::vector<int>> map_data;
};











#endif //ALGORITHM_H
