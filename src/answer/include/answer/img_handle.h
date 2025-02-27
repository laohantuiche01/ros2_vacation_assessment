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
            std::chrono::seconds(1),
            std::bind(&ImageHandle::publish_try, this));
        PosePub = this->create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);
        BoolPub = this->create_publisher<example_interfaces::msg::Bool>("shoot", 10);
        Map_unmove_Point_ = this->create_publisher<answer_infos::msg::MapPoint>("map_point_", 10);
        Map_move_Point_ = this->create_publisher<answer_infos::msg::RobotLocation>("map", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub; //接受来自判题机的图像
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr PosePub; //发送移动信息
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr BoolPub; //发送发射弹丸的消息
    rclcpp::Publisher<answer_infos::msg::MapPoint>::SharedPtr Map_unmove_Point_; //发送不动点的消息
    rclcpp::Publisher<answer_infos::msg::RobotLocation>::SharedPtr Map_move_Point_; //发送动电消息
    rclcpp::Service<answer_infos::srv::IfCanGo>::SharedPtr if_can_go_service_; //判断是否能走

    rclcpp::TimerBase::SharedPtr timer;

    cv::Mat img;

    answer_infos::msg::MapPoint map_unmove_point;
    answer_infos::msg::RobotLocation map_move_point;

    int mapPoint[33][17];

    std::vector<colcor_select::point_and_area> getContours(const cv::Mat &img_find_point); //得到大概轮廓，从而更好的找到点
    std::stack<cv::Point> reflash_way(cv::Point beginPoint ,cv::Point endPoint);
    bool IfCanGo(cv::Point basicPoint,int PointDir);

    void mapInitialization();
    void IfCanGoCallback(const answer_infos::srv::IfCanGo::Request::SharedPtr request_,
                         const answer_infos::srv::IfCanGo::Response::SharedPtr response_);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg); //接收到图像(测试)
    void find_pricise_point_unmove(); //确定精准的坐标
    int transform_abstract_point(int location); //转换成粗略的坐标
    void build_Map(int x, int y); //构建地图
    void publish_try() {
        geometry_msgs::msg::Pose2D pose;
        pose.x = 1000;
        pose.y = 1000;
        pose.theta = 2;
        RCLCPP_INFO(this->get_logger(), "publish try");
        PosePub->publish(pose);

        RCLCPP_INFO(this->get_logger(), "bool try");
        example_interfaces::msg::Bool msg_;
        msg_.data = true;
        //BoolPub->publish(msg_);

        find_pricise_point_unmove();
        reflash_way(cv::Point(2,0),cv::Point(2,4));
    }
};


#endif //IMG_HANDLE_H
