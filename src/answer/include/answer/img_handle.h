#ifndef IMG_HANDLE_H
#define IMG_HANDLE_H

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
#include<answer_infos/msg/way_points.hpp>


class ImageHandle : public rclcpp::Node {
public:
    ImageHandle(): Node("image_handle_node") {
        mapInitialization();
        imgSub = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name::image,
            10,
            std::bind(&ImageHandle::imageCallback, this, std::placeholders::_1) //回调函数
        );
        timer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ImageHandle::publish_try, this));
        Map_unmove_Point_ = this->create_publisher<answer_infos::msg::MapPoint>("MapPoints", 10);
        Map_move_Point_ = this->create_publisher<answer_infos::msg::RobotLocation>("map", 10);
        WayPub=this->create_service<answer_infos::srv::WayService>("WayPoints",
            std::bind(&ImageHandle::WayCallback, this, std::placeholders::_1,std::placeholders::_2));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub; //接受来自判题机的图像
    rclcpp::Publisher<answer_infos::msg::MapPoint>::SharedPtr Map_unmove_Point_; //发送不动点的消息
    rclcpp::Publisher<answer_infos::msg::RobotLocation>::SharedPtr Map_move_Point_; //发送动电消息
    rclcpp::Service<answer_infos::srv::WayService>::SharedPtr WayPub;//发布路径信息

    rclcpp::TimerBase::SharedPtr timer;

    cv::Mat img;
    cv::Point myself;

    answer_infos::msg::MapPoint map_unmove_point;


    static std::vector<colcor_select::point_and_area> getContours(const cv::Mat &img_find_point); //得到大概轮廓，从而更好的找到点
    std::stack<cv::Point> reflash_way(cv::Point beginPoint ,cv::Point endPoint);

    bool IfCanGo(cv::Point basicPoint,int PointDir);
    bool hasRun=false;

    static int transform_abstract_point(int location); //转换成粗略的坐标
    int mapPoint[32][16];
    int green_purple=-1;

    void WayCallback(const answer_infos::srv::WayService::Request::SharedPtr request,
        const answer_infos::srv::WayService::Response::SharedPtr response);
    void reflesh();
    void reflesh_point();
    void mapInitialization();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg); //接收到图像(测试)
    void find_pricise_point_unmove(); //确定精准的坐标
    void publish_try() {
        find_pricise_point_unmove();
        reflesh_point();
    }
};


#endif //IMG_HANDLE_H
