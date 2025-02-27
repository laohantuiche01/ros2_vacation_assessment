#ifndef ALGORITHM_H
#define ALGORITHM_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<stack>
#include<answer_infos/msg/map.hpp>
#include<answer_infos/msg/map_point.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include "answer/packages_connect.h"
#include<example_interfaces/msg/bool.hpp>
#include<answer/colcor_select.h>
#include <answer_infos/msg/robot_location.hpp>
#include<answer_infos/srv/if_can_go.hpp>


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
        map_Initialization();
        // map_point_sub = this->create_subscription<answer_infos::msg::MapPoint>("map_point_", 10,
        //     std::bind(&Algorithm::callback_test_1,this, std::placeholders::_1)
        //     );
        if_can_go = create_client<answer_infos::srv::IfCanGo>("if_can_go");
        timer = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&Algorithm::callbask, this)
        );
    }

private:
    //std::vector<std::vector<cv::Point> > runing_way_function(cv::Point began_point, cv::Point end_point);
    rclcpp::Subscription<answer_infos::msg::MapPoint>::SharedPtr map_point_sub; //受到图中不动点的坐标
    rclcpp::Client<answer_infos::srv::IfCanGo>::SharedPtr if_can_go;
    rclcpp::TimerBase::SharedPtr timer;
    int remember_the_way[33][17]; //记录步数
    bool judgment;

    void map_Initialization();

    void cout_map();

    void update_basic_map();

    std::stack<cv::Point> reflash_map(cv::Point basic_point, cv::Point end_point);

    void callback_test_1(const answer_infos::msg::MapPoint::SharedPtr msg) {
        std::cout << msg->destination.x << " " << msg->destination.y << std::endl;
    }

    void callbask() {
        std::stack<cv::Point> Point_queue;
        Point_queue = reflash_map(cv::Point(5, 5), cv::Point(5, 10));
//         while (!Point_queue.empty()) {
//             cv::Point cout_point = Point_queue.top();
//             std::cout << cout_point.x << " " << cout_point.y << std::endl;
//             Point_queue.pop();
//         }
        RCLCPP_INFO(this->get_logger(), "callbask");
    }

    bool IfCanGo(int x, int y, int direction); //撞墙了没
    void recive_bool(rclcpp::Client<answer_infos::srv::IfCanGo>::SharedFuture recive_msg);
};


#endif //ALGORITHM_H
