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
    std::vector<std::vector<int> > dir = {
        {0, -1}, //向上走 0
        {1, 0}, //向右走 1
        {0, 1}, //向下走 2
        {-1, 0} //向左走 3
    };
}


class Algorithm : public rclcpp::Node {
public:
    Algorithm(): Node("algo") {
        RCLCPP_INFO(this->get_logger(), "Algorithm::Algorithm");

        map_point_sub = this->create_subscription<answer_infos::msg::MapPoint>("map_point_", 10,
            std::bind(&Algorithm::callback_test_1,this, std::placeholders::_1)
            );
        map_base_sub = this->create_subscription<answer_infos::msg::Map>("map_base", 10,
        std::bind(&Algorithm::callback_test,this, std::placeholders::_1)
        );
    }

private:
    //std::vector<std::vector<cv::Point> > runing_way_function(cv::Point began_point, cv::Point end_point);
    rclcpp::Subscription<answer_infos::msg::MapPoint>::SharedPtr map_point_sub; //受到图中不动点的坐标
    rclcpp::Subscription<answer_infos::msg::Map>::SharedPtr map_base_sub;//受到地图信息
    rclcpp::Subscription<answer_infos::msg::RobotLocation>::SharedPtr map_img;//受到地图画面
    int  map_data[33][17];

    void update_basic_map();

    void reflash_map(cv::Point basic_point, int num_to_fill);

    void callback_test_1(const answer_infos::msg::MapPoint::SharedPtr msg) {
        //std::cout<<msg->destination.x<<" "<<msg->destination.y<<std::endl;
    }

    void callback_test(const answer_infos::msg::Map::SharedPtr map_point_msg) {
        //map_point_msg->map_abstract.resize(16);
        RCLCPP_INFO(this->get_logger(), "map_point_msg received");
        for (int i = 0; i < 33; i++) {
            //map_point_msg->map_abstract[i].map_abstracts.resize(32);
            for (int j = 0; j < 17; j++) {
                std::cout<<map_point_msg->map_abstract[i].map_abstracts[j]<<" ";
            }
            std::cout<<std::endl;
        }
    };


    int remember_the_way[17][33]; //记录步数

    bool if_can_go(int x, int y, int direction); //撞墙了没

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Algorithm>());
    rclcpp::shutdown();
    return 0;
}


#endif //ALGORITHM_H
