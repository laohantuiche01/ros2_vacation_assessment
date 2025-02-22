#ifndef IMG_HANDLE_H
#define IMG_HANDLE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<answer_infos/msg/map.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include "answer/packages_connect.h"
#include<std_msgs/msg/bool.hpp>
#include<example_interfaces/msg/bool.hpp>


class ImageHandle: public rclcpp::Node{
public:
    ImageHandle():Node("image_handle_node"){
        imgSub=this->create_subscription<sensor_msgs::msg::Image>(
          	topic_name::image,
          	10,
          	std::bind(&ImageHandle::imageCallback,this,std::placeholders::_1) //回调函数
        );
    	PosePub=create_publisher<geometry_msgs::msg::Pose2D>("pose",10
    		);
    	BoolPub=create_publisher<example_interfaces::msg::Bool>("shoot",10);

    	timer=this->create_wall_timer(
    		std::chrono::seconds(1),
    		std::bind(&ImageHandle::publish_try,this));

    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub; //接受来自判题机的图像
	rclcpp::Publisher<answer_infos::msg::Map>::SharedPtr MapPub;//发送地图信息
	rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr PosePub;
	rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr BoolPub;
	rclcpp::TimerBase::SharedPtr timer;
	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) ;//接收到图像(测试)
	void find_pricise_point_unmove(cv::Mat img_find_colcor) ;//确定精准的坐标
	cv::Point getContours(cv::Mat img_find_point) ; //得到大概轮廓，从而更好的找到点
	int transform_abstract_point(int location) ; //转换成粗略的坐标
	void build_Map(int x,int y);//构建地图
	void publish_try() {
		geometry_msgs::msg::Pose2D pose;
		pose.set__x(1000);
		pose.set__y(1000);
		pose.set__theta(2);
		// pose.x=1000;
		// pose.y=1000;
		// pose.theta=2;
		RCLCPP_INFO(this->get_logger(),"publish try");
		PosePub->publish(pose);

		RCLCPP_INFO(this->get_logger(),"bool try");
		example_interfaces::msg::Bool msg_;
		msg_.data=true;
		BoolPub->publish(msg_);
	}
};






#endif //IMG_HANDLE_H
