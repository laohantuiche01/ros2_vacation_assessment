#ifndef IMG_HANDLE_H
#define IMG_HANDLE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<answer_infos/msg/map.hpp>

#include "answer/packages_connect.h"


class ImageHandle: public rclcpp::Node{
public:
    ImageHandle():Node("image_handle_node"){
        imgSub=this->create_subscription<sensor_msgs::msg::Image>(
          	topic_name::image,
          	10,
          	std::bind(&ImageHandle::imageCallback,this,std::placeholders::_1) //回调函数
        );

    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub; //接受来自判题机的图像
	rclcpp::Publisher<answer_infos::msg::Map>::SharedPtr MapPub;//发送地图信息
	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) ;//接收到图像(测试)
	void find_pricise_point_unmove(cv::Mat img_find_colcor) ;//确定精准的坐标
	cv::Point getContours(cv::Mat img_find_point) ; //得到大概轮廓，从而更好的找到点
	int transform_abstract_point(int location) ; //转换成粗略的坐标
	void build_Map(int x,int y);//构建地图
};






#endif //IMG_HANDLE_H
