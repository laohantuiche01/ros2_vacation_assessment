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
#include<example_interfaces/msg/int64.hpp>


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
        Int64Sub=this->create_subscription<example_interfaces::msg::Int64>("password_segment",
            10,
            std::bind(&Algorithm::password_handle,this,std::placeholders::_1)
            );
        Int64Pub=this->create_publisher<example_interfaces::msg::Int64>("password",10);
        PosePub=this->create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);
        BoolPub = this->create_publisher<example_interfaces::msg::Bool>("shoot", 10);
        way_service_client=this->create_client<answer_infos::srv::WayService>("WayPoints");

        map_point_sub=this->create_subscription<answer_infos::msg::MapPoint>("MapPoints",
            10,
            std::bind(&Algorithm::recieve_map_point,this,std::placeholders::_1));
        robot_sub=this->create_subscription<answer_infos::msg::RobotLocation>("map",
            10,
            std::bind(&Algorithm::recieve_robot_location,this,std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(740),
                                         std::bind(&Algorithm::send_point, this)
        );
    }

private:
    rclcpp::Subscription<answer_infos::msg::RobotLocation>::SharedPtr robot_sub;
    rclcpp::Subscription<answer_infos::msg::MapPoint>::SharedPtr map_point_sub; //受到图中不动点的坐标
    rclcpp::Client<answer_infos::srv::WayService>::SharedPtr way_service_client;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr PosePub; //移动
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr BoolPub; //攻击
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr Int64Sub;//接受密码
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr Int64Pub;//发送密码
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr control_timer;

    cv::Point Points[20];//储存点的数组
    cv::Point pricise[3];//精确坐标
    cv::Point way_points[100];//储存路径
    std::queue<cv::Point> Point_queue;//储存路径

    cv::Point decide_which_next();

    int num=0;
    int decide_if_go_to_recover=0;
    int decide_if_go_to_recover_again=1;
    int green_or_purple_=1;
    int has_in=0;//是否已经进入传送门
    int has_gone_password=0;
    int has_gone_out=0;
    int if_can_destroy_base=0;
    int decide_how_to_finish=0;
    int aneme_size;
    int pause_condition=1;
    int count_=0;
    int has_fire=0;
    long int password[3];//要发送password[2]
    bool check_if_right(cv::Point targetPoint);
    void fight(int is_who);
    void password_handle(example_interfaces::msg::Int64 msg);
    void move(int dx, int dy ,double dtheta);
    void arrayInitialization();
    void recieve_map_point(answer_infos::msg::MapPoint::SharedPtr msg);
    void recieve_robot_location(answer_infos::msg::RobotLocation::SharedPtr msg);
    void send_point();
    void way_handle(rclcpp::Client<answer_infos::srv::WayService>::SharedFuture msg);
    void move_control(std::queue<cv::Point> Point_queue_);


};


#endif //ALGORITHM_H
