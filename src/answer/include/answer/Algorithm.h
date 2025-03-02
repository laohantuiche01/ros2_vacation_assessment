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
#include<answer_infos/srv/way_service.hpp>


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
        arrayInitialization();
        PosePub=this->create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);
        way_service_client=this->create_client<answer_infos::srv::WayService>("WayPoints");
        robot_sub=this->create_subscription<answer_infos::msg::RobotLocation>("map",
            10,
            std::bind(&Algorithm::recieve_robot_location,this,std::placeholders::_1));
        map_point_sub=this->create_subscription<answer_infos::msg::MapPoint>("MapPoints",
            10,
            std::bind(&Algorithm::recieve_map_point,this,std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::seconds(5),
                                         std::bind(&Algorithm::send_point, this)
        );
        // control_timer = this->create_wall_timer(std::chrono::seconds(1),
        //     std::bind(&Algorithm::move_control, this));
    }

private:
    rclcpp::Subscription<answer_infos::msg::RobotLocation>::SharedPtr robot_sub;
    rclcpp::Subscription<answer_infos::msg::MapPoint>::SharedPtr map_point_sub; //受到图中不动点的坐标
    rclcpp::Client<answer_infos::srv::WayService>::SharedPtr way_service_client;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr PosePub; //移动
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr BoolPub; //攻击
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr control_timer;

    cv::Point Points[20];//储存点的数组
    cv::Point pricise[3];//精确坐标
    cv::Point way_points[100];//储存路径
    std::queue<cv::Point> Point_queue;//储存路径

    int q;

    void move(int dx, int dy);
    void arrayInitialization();
    void recieve_map_point(answer_infos::msg::MapPoint::SharedPtr msg);
    void recieve_robot_location(answer_infos::msg::RobotLocation::SharedPtr msg);
    void send_point();
    void way_handle(rclcpp::Client<answer_infos::srv::WayService>::SharedFuture msg);
    void move_control();

};


#endif //ALGORITHM_H
