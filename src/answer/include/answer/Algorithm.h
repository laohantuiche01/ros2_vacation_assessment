#ifndef ALGORITHM_H
#define ALGORITHM_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<answer_infos/msg/map.hpp>
#include<answer_infos/msg/map_point.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include "answer/packages_connect.h"
#include<example_interfaces/msg/bool.hpp>
#include<answer/colcor_select.h>
#include <answer_infos/msg/robot_location.hpp>

namespace direction {
    std::vector<std::vector<int>> dir={
        {0,-1},//向上走 0
        {1,0},//向右走 1
        {0,1},//向下走 2
        {-1,0}//向左走 3
    };
}


class Algorithm : public rclcpp::Node
{
  public:
    Algorithm():Node("algo") {
        map_point_sub=this->create_subscription<answer_infos::msg::MapPoint>("map_point_",10,
            std::bind(&Algorithm::callback_test,this,std::placeholders::_1)
            );
    }

  private:
      std::vector<std::vector<cv::Point>> runing_way_function(cv::Point began_point,cv::Point end_point);
    void update_basic_map();
        void reflash_map(cv::Point basic_point,int num_to_fill);

    void callback_test(const answer_infos::msg::MapPoint::SharedPtr map_point_msg) {
        std::cout<<map_point_msg->password.x<<" "<<map_point_msg->password.y<<std::endl;
    };


      rclcpp::Subscription<answer_infos::msg::MapPoint>::SharedPtr map_point_sub;
        int remember_the_way[17][33];

      bool if_can_go(int x,int y,int direction);
      std::vector<std::vector<int>> map_data;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Algorithm>());
    rclcpp::shutdown();
    return 0;
}









#endif //ALGORITHM_H
