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
#include<answer_infos/msg/map_point.hpp>
#include<answer_infos/msg/robot_location.hpp>



void ImageHandle::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) //接收到图像(测试)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    img = cv_ptr->image.clone();
    //cv::Mat imgGtey;

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

    //find_pricise_point_unmove(img);

    //std::cout<<transform_abstract_point((int)temp.x)<<" "<<transform_abstract_point((int)temp.y)<<std::endl;
    //std::cout<<temp.x<<" "<<temp.y<<std::endl;
    cv::imshow("image",img);
    cv::waitKey(1);
}

void ImageHandle::find_pricise_point_unmove(cv::Mat img_find_colcor) //测试输出函数
{
    answer_infos::msg::MapPoint map_unmove_point;
    answer_infos::msg::RobotLocation map_move_point;

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

        std::vector<colcor_select::point_and_area> my_points = getContours(Mask);

        switch (i) {
            case 0://绿色传送门
                if (my_points[0].area>1000) //发布者录入信息
                    {
                    map_unmove_point.green_in.x=my_points[0].point.x;
                    map_unmove_point.green_in.y=my_points[0].point.y;
                    map_unmove_point.green_out.x=my_points[1].point.x;
                    map_unmove_point.green_out.y=my_points[1].point.y;
                }
                map_unmove_point.green_in.x=my_points[1].point.x;
                map_unmove_point.green_in.y=my_points[1].point.y;
                map_unmove_point.green_out.x=my_points[0].point.x;
                map_unmove_point.green_out.y=my_points[0].point.y;
                continue;
            case 1://紫色传送门
                if (my_points[0].area>1000) {
                    map_unmove_point.purple_in.x=my_points[0].point.x;
                    map_unmove_point.purple_in.y=my_points[0].point.y;
                    map_unmove_point.purple_out.x=my_points[1].point.x;
                    map_unmove_point.purple_out.y=my_points[1].point.y;
                }
                map_unmove_point.purple_in.x=my_points[1].point.x;
                map_unmove_point.purple_in.y=my_points[1].point.y;
                map_unmove_point.purple_out.x=my_points[0].point.x;
                map_unmove_point.purple_out.y=my_points[0].point.y;
                continue;
            case 2://补给
                // std::cout<<my_points[0].area<<std::endl;
                    //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_unmove_point.recover.x=transform_abstract_point(my_points[0].point.x);
                map_unmove_point.recover.y=transform_abstract_point(my_points[0].point.y);
            continue;
            case 3://基地
                //std::cout<<my_points[0].area<<std::endl;
                    //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_unmove_point.destination.x=transform_abstract_point(my_points[0].point.x);
                map_unmove_point.destination.y=transform_abstract_point(my_points[0].point.y);
            continue;
            case 4://密码
                //std::cout<<my_points[0].area<<std::endl;
                    //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_unmove_point.password.x=transform_abstract_point(my_points[0].point.x);
                map_unmove_point.password.y=transform_abstract_point(my_points[0].point.y);
            continue;
            case 5://自己
                //std::cout<<my_points[0].area<<std::endl;
                    //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                    // map_move_point.myself.x=transform_abstract_point(my_points[0].point.x);
                    // map_move_point.myself.y=transform_abstract_point(my_points[0].point.y);
                    // map_move_point.myself_x=my_points[0].point.x;
                    // map_move_point.myself_y=my_points[0].point.y;
                    continue;
            case 6://敌人
                std::cout<<my_points.size()<<std::endl;
            continue;

        }
    }


}

std::vector<colcor_select::point_and_area> ImageHandle::getContours(const cv::Mat& img_find_point) //寻找点
{
    std::vector<colcor_select::point_and_area> return_vector;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_find_point, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > conPoly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size()); //用长方形框选以确定其中心
    cv::Point myPoint(0, 0);

    for(int i=0;i<contours.size();i++) //处理如果有多个颜色的情况，用vector承载
    {
        colcor_select::point_and_area temp_contour;
        auto area=contourArea(contours[i]); //要返回面积以判断为入口还是出口
        double peri = arcLength(contours[i], true);
        approxPolyDP(contours[i], conPoly[i], 0.01 * peri, true);
        //std::cout << conPoly[0].size() << std::endl;
        boundRect[i] = boundingRect(conPoly[i]);//用矩形包括，中心即为其坐标
        myPoint.x = boundRect[i].x + boundRect[i].width / 2;
        myPoint.y = boundRect[i].y + boundRect[i].height / 2;
        temp_contour.area = area;//返回数值
        temp_contour.point=myPoint;
        return_vector.push_back(temp_contour);
    }
    return return_vector;
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
