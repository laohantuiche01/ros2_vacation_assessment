#include "answer/img_handle.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<answer/colcor_select.h>
#include<answer_infos/msg/map.hpp>

void ImageHandle::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) //接收到图像(测试)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat img = cv_ptr->image.clone();
    cv::Mat imgGtey;

    //找到粗略的坐标
    for(int i=0;i<16;i++) {
        cv::line(img,cv::Point(0,64*i),cv::Point(3000,64*i),cv::Scalar(0,0,255),2);
    }
    for(int i=0;i<32;i++) {
        cv::line(img,cv::Point(64*i,0),cv::Point(64*i,1000),cv::Scalar(0,0,255),2);
    }

    //颜色匹配
    //cv::line(img,cv::Point(0,14*64-20),cv::Point(3000,14*64-20),cv::Scalar(175,175,175),2);

    //灰度图的测试
    //cv::cvtColor(img,imgGtey,CV_BGR2GRAY);

    //找各个颜色的RGB值
    //cv::Vec3b pixelRGB = img.at<cv::Vec3b>(30, 4*64);
    //std::cout << (int)pixelRGB.val[0]<<" "<<(int)pixelRGB.val[1]<<" "<<(int)pixelRGB.val[2] << std::endl;

    find_pricise_point_unmove(img);

    //std::cout<<transform_abstract_point((int)temp.x)<<" "<<transform_abstract_point((int)temp.y)<<std::endl;
    //std::cout<<temp.x<<" "<<temp.y<<std::endl;
    cv::imshow("image",img);
    cv::waitKey(1);
}

void ImageHandle::find_pricise_point_unmove(cv::Mat img_find_colcor) //测试输出函数
{
    answer_infos::msg::Map map_data;
    map_data.map_abstract.resize(16);
    for (int i = 0; i < 16; i++) //地图探测
    {
        map_data.map_abstract[i].map_abstracts.resize(32);
        for (int j = 0; j < 32; j++) //地图探测
        {
            cv::Vec3b pixelRGB = img_find_colcor.at<cv::Vec3b>(i * 64 + 32, j * 64 + 32);
            if (pixelRGB[0] == 175 && pixelRGB[1] == 175 && pixelRGB[2] == 175) {
                map_data.map_abstract[i].map_abstracts[j] = 0; //0为无障碍
            } else if (pixelRGB[0] == 58 && pixelRGB[1] == 58 && pixelRGB[2] == 58) {
                map_data.map_abstract[i].map_abstracts[j] = 1; //1为有障碍
            } else {
                map_data.map_abstract[i].map_abstracts[j] = -1; //-1为未知
            }
        }
    }
    RCLCPP_INFO(this->get_logger(),"map abstract map");
    for (int i = 0; i < 7; i++) {
        cv::Mat Mask;

        cv::Scalar lower = cv::Scalar(colcor_select::myColcor[i][0], colcor_select::myColcor[i][1]-1, colcor_select::myColcor[i][2]);
        cv::Scalar upper = cv::Scalar(colcor_select::myColcor[i][0], colcor_select::myColcor[i][1]+1, colcor_select::myColcor[i][2]);

        cv::inRange(img_find_colcor, lower, upper, Mask);
        cv::Point my_point = getContours(Mask);

        switch (i) {
            case 0:
                std::cout << transform_abstract_point(my_point.x )<< " " << transform_abstract_point(my_point.y) << std::endl;
                return;
            case 1:
                std::cout << my_point.x << " " << my_point.y << std::endl;
                return;
            case 2:
                std::cout << my_point.x << " " << my_point.y << std::endl;
                return;
            case 3:
                std::cout << my_point.x << " " << my_point.y << std::endl;
                return;
            case 4:
                std::cout << my_point.x << " " << my_point.y << std::endl;
                return;
            case 5:
                std::cout << my_point.x << " " << my_point.y << std::endl;
                return;
            case 6:
                std::cout << my_point.x << " " << my_point.y << std::endl;
                return;

        }
    }


}

cv::Point ImageHandle::getContours(cv::Mat img_find_point) //寻找点
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_find_point, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > conPoly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size()); //用长方形框选以确定其中心
    cv::Point myPoint(0, 0);

    //auto area=contourArea(contours[0]);
    float peri = arcLength(contours[0], true);
    approxPolyDP(contours[0], conPoly[0], 0.01 * peri, true);
    //std::cout << conPoly[0].size() << std::endl;
    boundRect[0] = boundingRect(conPoly[0]);
    myPoint.x = boundRect[0].x + boundRect[0].width / 2;
    myPoint.y = boundRect[0].y + boundRect[0].height / 2;


    return myPoint;
}

int ImageHandle::transform_abstract_point(int location) {
    int temp = location / 64;
    return temp + 1;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageHandle>());
    rclcpp::shutdown();
    return 0;
}
