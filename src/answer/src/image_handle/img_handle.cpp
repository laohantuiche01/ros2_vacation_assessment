#include "answer/img_handle.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.h>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<answer/colcor_select.h>
#include<answer_infos/msg/map.hpp>
#include<answer_infos/msg/map_point.hpp>
#include<answer_infos/msg/robot_location.hpp>
#include<answer/Algorithm.h>

void ImageHandle::mapInitialization() {
    for (int i = 0; i <= 32; i++) {
        for (int j = 0; j <= 16; j++) {
            mapPoint[i][j] = -1;
        }
    }
}

bool ImageHandle::IfCanGo(cv::Point basicPoint, int PointDir) {
    int tempx=basicPoint.x * 64 + 32 + 32 * direction::dir[PointDir][0];
    int tempy=basicPoint.y * 64 + 32 + 32 * direction::dir[PointDir][1];
    cv::Vec3b pixelRGB = img.at<cv::Vec3b>(tempy, tempx);
    //cv::Vec3b pixelRGB1 = img.at<cv::Vec3b>(64*5, 64*2+32);
    // std::cout << (int)pixelRGB1.val[0] << " " << (int)pixelRGB1.val[1] << " " << (int)pixelRGB1.val[2] << std::endl;

    if ((int)pixelRGB.val[0] == 175 && (int)pixelRGB.val[1] ==175 && (int)pixelRGB.val[2] == 175) {
        return true;
    } else { return false; }
}


std::stack<cv::Point> ImageHandle::reflash_way(cv::Point beginPoint, cv::Point endPoint) {
    std::queue<cv::Point> Point_queue;
    Point_queue.push(beginPoint);
    mapPoint[beginPoint.x][beginPoint.y] = 4;

    while (!Point_queue.empty()) //dfs找路
    {
        int x = Point_queue.front().x;
        int y = Point_queue.front().y;
        Point_queue.pop();

        for (int i = 0; i <= 3; i++) {
            cv::Point temp_point;
            temp_point.x = x + direction::dir[i][0];
            temp_point.y = y + direction::dir[i][1];
            if (IfCanGo(cv::Point(x,y), i) && mapPoint[temp_point.x][temp_point.y] == -1 &&
                temp_point.x>=0 &&temp_point.y>=0 &&temp_point.x<32 &&temp_point.y<16) {
                Point_queue.push(temp_point);
                mapPoint[temp_point.x][temp_point.y] = i;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "remember_the_way");
    std::stack<cv::Point> path;
    int temp_x = endPoint.x;
    int temp_y = endPoint.y;
    path.push(endPoint);
    while (temp_x != beginPoint.x || temp_y != beginPoint.y) {
        cv::Point temp_point;
        temp_point.x = temp_x - direction::dir[mapPoint[temp_x][temp_y]][0];
        temp_point.y = temp_y - direction::dir[mapPoint[temp_x][temp_y]][1];
        path.push(temp_point);
        temp_x = temp_point.x;
        temp_y = temp_point.y;
    }
    while (!path.empty()) {
        cv::Point cout_point = path.top();
        cv::circle(img,cv::Point(cout_point.x*64+32,cout_point.y*64+32),10,cv::Scalar(0,0,255),-1);
        std::cout << cout_point.x << " " << cout_point.y << std::endl;
        path.pop();
    }

    return path;
}


void ImageHandle::IfCanGoCallback(const answer_infos::srv::IfCanGo::Request::SharedPtr request_,
                                  const answer_infos::srv::IfCanGo::Response::SharedPtr response_) {
    cv::Vec3b pixelRGB = img.at<cv::Vec3b>(request_->point_x * 64 + 32 + 32 * direction::dir[request_->dir][0],
                                           request_->point_y * 64 + 32 + 32 * direction::dir[request_->dir][1]);
    if (pixelRGB.val[0] == 58 && pixelRGB.val[1] == 58 && pixelRGB.val[2] == 58) {
        response_->result_bool = false;
    } else { response_->result_bool = true; }
}


void ImageHandle::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) //接收到图像(测试)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    img = cv_ptr->image.clone();
    //cv::Mat imgGtey;

    //找到粗略的坐标
    // for (int i = 0; i < 16; i++) {
    //     cv::line(img, cv::Point(0, 64 * i), cv::Point(3000, 64 * i), cv::Scalar(0, 0, 255), 2);
    // }
    // for (int i = 0; i < 32; i++) {
    //     cv::line(img, cv::Point(64 * i, 0), cv::Point(64 * i, 1000), cv::Scalar(0, 0, 255), 2);
    // }

    //颜色匹配
    //cv::line(img,cv::Point(0,14*64-20),cv::Point(3000,14*64-20),cv::Scalar(58,58,58),2);

    //灰度图的测试
    //cv::cvtColor(img,imgGtey,CV_BGR2GRAY);

    //找各个颜色的RGB值
    // cv::Vec3b pixelRGB = img.at<cv::Vec3b>(64*5+32, 4*64+32);
    // std::cout << (int)pixelRGB.val[0]<<" "<<(int)pixelRGB.val[1]<<" "<<(int)pixelRGB.val[2] << std::endl;

    //find_pricise_point_unmove(img);

    //std::cout<<transform_abstract_point((int)temp.x)<<" "<<transform_abstract_point((int)temp.y)<<std::endl;
    //std::cout<<temp.x<<" "<<temp.y<<std::endl;
    //cv::circle(img,cv::Point(160,0),10,cv::Scalar(0,0,255),-1);
    cv::imshow("image", img);
    cv::waitKey(1);
}

void ImageHandle::find_pricise_point_unmove() //测试输出函数
{
    //要发布的信息：
    //地图上不动点
    //地图上移动的机器人
    //地图概况

    RCLCPP_INFO(this->get_logger(), "find pricise point");

    for (int i = 0; i < 7; i++) {
        cv::Mat Mask;

        cv::Scalar lower = cv::Scalar(colcor_select::myColcor[i][0], colcor_select::myColcor[i][1] - 1,
                                      colcor_select::myColcor[i][2]);
        cv::Scalar upper = cv::Scalar(colcor_select::myColcor[i][0], colcor_select::myColcor[i][1] + 1,
                                      colcor_select::myColcor[i][2]);

        cv::inRange(img, lower, upper, Mask);

        std::vector<colcor_select::point_and_area> my_points = getContours(Mask);

        switch (i) {
            case 0: //绿色传送门
                if (my_points[0].area > 500) //发布者录入信息
                {
                    map_unmove_point.green_in.x = my_points[0].point.x;
                    map_unmove_point.green_in.y = my_points[0].point.y;
                    map_unmove_point.green_out.x = my_points[1].point.x;
                    map_unmove_point.green_out.y = my_points[1].point.y;
                }
                map_unmove_point.green_in.x = my_points[1].point.x;
                map_unmove_point.green_in.y = my_points[1].point.y;
                map_unmove_point.green_out.x = my_points[0].point.x;
                map_unmove_point.green_out.y = my_points[0].point.y;
                continue;
            case 1: //紫色传送门
                if (my_points[0].area > 1000) {
                    map_unmove_point.purple_in.x = my_points[0].point.x;
                    map_unmove_point.purple_in.y = my_points[0].point.y;
                    map_unmove_point.purple_out.x = my_points[1].point.x;
                    map_unmove_point.purple_out.y = my_points[1].point.y;
                }
                map_unmove_point.purple_in.x = my_points[1].point.x;
                map_unmove_point.purple_in.y = my_points[1].point.y;
                map_unmove_point.purple_out.x = my_points[0].point.x;
                map_unmove_point.purple_out.y = my_points[0].point.y;
                continue;
            case 2: //补给
                // std::cout<<my_points[0].area<<std::endl;
                //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_unmove_point.recover.x = transform_abstract_point(my_points[0].point.x);
                map_unmove_point.recover.y = transform_abstract_point(my_points[0].point.y);
                continue;
            case 3: //基地
                //std::cout<<my_points[0].area<<std::endl;
                //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_unmove_point.destination.x = transform_abstract_point(my_points[0].point.x);
                map_unmove_point.destination.y = transform_abstract_point(my_points[0].point.y);
                continue;
            case 4: //密码
                //std::cout<<my_points[0].area<<std::endl;
                //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_unmove_point.password.x = transform_abstract_point(my_points[0].point.x);
                map_unmove_point.password.y = transform_abstract_point(my_points[0].point.y);
                continue;
            case 5: //自己
                //std::cout<<my_points[0].area<<std::endl;
                //std::cout<<transform_abstract_point(my_points[0].point.x)<<" "<<transform_abstract_point(my_points[0].point.y)<<std::endl;
                map_move_point.myself.x = transform_abstract_point(my_points[0].point.x);
                map_move_point.myself.y = transform_abstract_point(my_points[0].point.y);
                map_move_point.myself_x = my_points[0].point.x;
                map_move_point.myself_y = my_points[0].point.y;
                continue;
            case 6: //敌人
                map_move_point.enemy.resize(my_points.size() - 1);
                map_move_point.enemy_precise.resize(my_points.size() - 1);
                int temp_num = 0; //计数，记录录入的敌人信息
                for (int h = 0; h < my_points.size(); h++) //处理多个返回值的情况
                {
                    if (my_points[h].point.x <= 30 && temp_num == 0) {
                        map_move_point.enemy[temp_num].x = transform_abstract_point(my_points[h].point.x);
                        map_move_point.enemy[temp_num].y = transform_abstract_point(my_points[h].point.y);
                        map_move_point.enemy_precise[temp_num].x = my_points[h].point.x;
                        map_move_point.enemy_precise[temp_num].y = my_points[h].point.y;
                        temp_num++;
                    } else if (my_points[h].point.x <= 30 && temp_num == 1) {
                        map_move_point.enemy[temp_num].x = transform_abstract_point(my_points[h].point.x);
                        map_move_point.enemy[temp_num].y = transform_abstract_point(my_points[h].point.y);
                        map_move_point.enemy_precise[temp_num].x = my_points[h].point.x;
                        map_move_point.enemy_precise[temp_num].y = my_points[h].point.y;
                    }
                }
                continue;
        }
    }
    Map_move_Point_->publish(map_move_point); //发布消息
    Map_unmove_Point_->publish(map_unmove_point); //发布消息
}

std::vector<colcor_select::point_and_area> ImageHandle::getContours(const cv::Mat &img_find_point) //寻找点
{
    std::vector<colcor_select::point_and_area> return_vector;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_find_point, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > conPoly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size()); //用长方形框选以确定其中心
    cv::Point myPoint(0, 0);

    for (int i = 0; i < contours.size(); i++) //处理如果有多个颜色的情况，用vector承载
    {
        colcor_select::point_and_area temp_contour;
        auto area = contourArea(contours[i]); //要返回面积以判断为入口还是出口
        if (area > 300) {
            double peri = arcLength(contours[i], true);
            approxPolyDP(contours[i], conPoly[i], 0.01 * peri, true);
            //std::cout << conPoly[0].size() << std::endl;
            boundRect[i] = boundingRect(conPoly[i]); //用矩形包括，中心即为其坐标
            myPoint.x = boundRect[i].x + boundRect[i].width / 2;
            myPoint.y = boundRect[i].y + boundRect[i].height / 2;
            temp_contour.area = area; //返回数值
            temp_contour.point = myPoint;
            return_vector.push_back(temp_contour); //将找到的情况放到容器中
        }
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
