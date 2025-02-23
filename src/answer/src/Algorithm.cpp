#include<answer/Algorithm.h>
#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>

std::vector<std::vector<cv::Point>> Algorithm::runing_way_function(cv::Point began_point,cv::Point end_point) //计算要走的路径
{
      update_basic_map();
      remember_the_way[end_point.x][end_point.y] = 0;
      reflash_map(end_point,0);
}

void Algorithm::reflash_map(cv::Point basic_point,int num_to_fill) {
    num_to_fill = num_to_fill+1;
      for(int i=0;i<4;i++) {
          if (if_can_go(basic_point.x,basic_point.y,i)) //判断某个方向有没有墙，并且有没有被填充
          {
              cv::Point input_point;
              input_point.x = basic_point.x+direction::dir[i][0];
              input_point.y = basic_point.y+direction::dir[i][1];
              remember_the_way[input_point.x][input_point.y] = num_to_fill;
              reflash_map(input_point,num_to_fill);
          }
      }
}

