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
#include <bits/fs_fwd.h>

void ImageHandle::WayCallback(const answer_infos::srv::WayService::Request::SharedPtr request,
                              const answer_infos::srv::WayService::Response::SharedPtr response) {
    cv::Point begin_point;
    cv::Point end_point;
    begin_point.x = request->input_point[0].x;
    begin_point.y = request->input_point[0].y;
    end_point.x = request->input_point[1].x;
    end_point.y = request->input_point[1].y;

    std::stack<cv::Point> tempstack;
    while (!tempstack.empty()) {
        tempstack.pop();
    }

    reflesh_point();
    std::cout << myself.x << " " << myself.y << std::endl;
    tempstack = reflash_way(myself, end_point);

    int num_ji = 0; //计数器
    response->point_way.resize(tempstack.size()); //初始化大小
    while (!tempstack.empty()) //将路径参数传输到服务中
    {
        cv::Point temp_point;
        temp_point.x = tempstack.top().x;
        temp_point.y = tempstack.top().y;
        response->point_way[num_ji].x = temp_point.x;
        response->point_way[num_ji].y = temp_point.y;
        num_ji++;
        tempstack.pop();
    }
}


void ImageHandle::mapInitialization() {
    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 16; j++) {
            mapPoint[i][j] = -1;
        }
    }
}

bool ImageHandle::IfCanGo(cv::Point basicPoint, int PointDir) {
    int tempx = (basicPoint.x - 1) * 64 + 32 + 32 * direction::dir[PointDir][0] + 2;
    int tempy = (basicPoint.y - 1) * 64 + 32 + 32 * direction::dir[PointDir][1] + 2;
    cv::Vec3b pixelRGB = img.at<cv::Vec3b>(tempy, tempx);
    if ((int) pixelRGB.val[0] == 58 && (int) pixelRGB.val[1] == 58 && (int) pixelRGB.val[2] == 58) {
        return false;
    }
    return true;
}


std::stack<cv::Point> ImageHandle::reflash_way(cv::Point beginPoint, cv::Point endPoint) {
    std::queue<cv::Point> Point_queue;
    Point_queue.push(beginPoint);
    mapPoint[beginPoint.x][beginPoint.y] = 4;
    mapInitialization();
    while (!Point_queue.empty()) //dfs找路
    {
        int x = Point_queue.front().x;
        int y = Point_queue.front().y;
        Point_queue.pop();

        for (int i = 0; i <= 3; i++) {
            cv::Point temp_point;
            temp_point.x = x + direction::dir[i][0];
            temp_point.y = y + direction::dir[i][1];
            if (IfCanGo(cv::Point(x, y), i) && mapPoint[temp_point.x][temp_point.y] == -1 &&
                temp_point.x >= 0 && temp_point.y >= 0 && temp_point.x < 32 && temp_point.y < 16) {
                Point_queue.push(temp_point);
                mapPoint[temp_point.x][temp_point.y] = i;
            }
        }
    }
    if (endPoint.x >= 1 && endPoint.y >= 1 && endPoint.x < 32 && endPoint.y < 16) {
        if (mapPoint[endPoint.x][endPoint.y] == -1) {
            RCLCPP_ERROR(this->get_logger(), "终点不可达");
            return {};
        }
    }
    else {
        RCLCPP_INFO(this->get_logger(),"输入错误");
        return {};
    }

    //RCLCPP_INFO(this->get_logger(), "remember_the_way");
    std::stack<cv::Point> path;
    int temp_x = endPoint.x;
    int temp_y = endPoint.y;
    path.push(endPoint);
    while (temp_x != beginPoint.x || temp_y != beginPoint.y) {
        int dir_ = mapPoint[temp_x][temp_y];
        cv::Point temp_point;
        if (dir_ >= 0 && dir_ <= 3) {
            temp_point.x = temp_x - direction::dir[dir_][0];
            temp_point.y = temp_y - direction::dir[dir_][1];
            path.push(temp_point);
            temp_x = temp_point.x;
            temp_y = temp_point.y;
        } else {
            break;
        }
    }
    return path;
}

void ImageHandle::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) //接收到图像(测试)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    img = cv_ptr->image.clone();
    // cv::imshow("image", img);
    // cv::waitKey(1);
}

void ImageHandle::find_pricise_point_unmove() //测试输出函数
{
    if (!hasRun) {
        for (int i = 0; i < 5; i++) {
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
                        map_unmove_point.green_in.x = transform_abstract_point(my_points[0].point.x);
                        map_unmove_point.green_in.y = transform_abstract_point(my_points[0].point.y);
                        map_unmove_point.green_out.x = transform_abstract_point(my_points[1].point.x);
                        map_unmove_point.green_out.y = transform_abstract_point(my_points[1].point.y);
                    } else {
                        map_unmove_point.green_in.x = transform_abstract_point(my_points[1].point.x);
                        map_unmove_point.green_in.y = transform_abstract_point(my_points[1].point.y);
                        map_unmove_point.green_out.x = transform_abstract_point(my_points[0].point.x);
                        map_unmove_point.green_out.y = transform_abstract_point(my_points[0].point.y);
                    }
                    continue;
                case 1: //紫色传送门
                    if (my_points[0].area > 500) {
                        map_unmove_point.purple_in.x = transform_abstract_point(my_points[0].point.x);
                        map_unmove_point.purple_in.y = transform_abstract_point(my_points[0].point.y);
                        map_unmove_point.purple_out.x = transform_abstract_point(my_points[1].point.x);
                        map_unmove_point.purple_out.y = transform_abstract_point(my_points[1].point.y);
                    } else {
                        map_unmove_point.purple_in.x = transform_abstract_point(my_points[1].point.x);
                        map_unmove_point.purple_in.y = transform_abstract_point(my_points[1].point.y);
                        map_unmove_point.purple_out.x = transform_abstract_point(my_points[0].point.x);
                        map_unmove_point.purple_out.y = transform_abstract_point(my_points[0].point.y);
                    }
                    continue;
                case 2: //补给
                    map_unmove_point.recover.x = transform_abstract_point(my_points[0].point.x);
                    map_unmove_point.recover.y = transform_abstract_point(my_points[0].point.y);
                    continue;
                case 3: //基地
                    map_unmove_point.destination.x = transform_abstract_point(my_points[0].point.x);
                    map_unmove_point.destination.y = transform_abstract_point(my_points[0].point.y);
                    continue;
                case 4: //密码
                    map_unmove_point.password.x = transform_abstract_point(my_points[0].point.x);
                    map_unmove_point.password.y = transform_abstract_point(my_points[0].point.y);
            }
        }

        cv::Point a, b; //用来判断那个是进去的
        a.x = map_unmove_point.recover.x;
        a.y = map_unmove_point.recover.y;
        b.x = map_unmove_point.green_in.x;
        b.y = map_unmove_point.green_in.y;
        std::stack<cv::Point> my_;
        my_ = reflash_way(a, b);
        if (green_purple == -1) {
            if (my_.empty()) {
                map_unmove_point.green_or_purple = 0; //等于1是紫色进
            } else {
                map_unmove_point.green_or_purple = 1; //等于0是紫色进
            }
        }
        hasRun = true;
    }
    Map_unmove_Point_->publish(map_unmove_point); //发布消息
}

void ImageHandle::reflesh_point() {
    answer_infos::msg::RobotLocation map_move_point;
    for (int i = 5; i < 7; i++) {
        cv::Mat Mask;

        cv::Scalar lower = cv::Scalar(colcor_select::myColcor[i][0], colcor_select::myColcor[i][1] - 1,
                                      colcor_select::myColcor[i][2]);
        cv::Scalar upper = cv::Scalar(colcor_select::myColcor[i][0], colcor_select::myColcor[i][1] + 1,
                                      colcor_select::myColcor[i][2]);

        inRange(img, lower, upper, Mask);

        std::vector<colcor_select::point_and_area> my_points_ = getContours(Mask);

        switch (i) {
            case 5: //自己
                map_move_point.myself_x = my_points_[0].point.x;
                map_move_point.myself_y = my_points_[0].point.y;
                map_move_point.myself.x = transform_abstract_point(my_points_[0].point.x);
                map_move_point.myself.y = transform_abstract_point(my_points_[0].point.y);
                myself.x = transform_abstract_point(my_points_[0].point.x);
                myself.y = transform_abstract_point(my_points_[0].point.y);
                continue;
            case 6: //敌人
                if (my_points_.size() == 1) {
                } else if (my_points_.size() == 0) {
                } else {
                    map_move_point.enemy.resize(my_points_.size() - 1);
                    map_move_point.enemy_precise.resize(my_points_.size() - 1);
                    int num_ji = 0;
                    for (u_int j = 0; j <= my_points_.size(); j++) //处理多个返回值的情况
                    {
                        if (transform_abstract_point(my_points_[j].point.x) <= 30) {
                            map_move_point.enemy[num_ji].x = transform_abstract_point(my_points_[j].point.x);
                            map_move_point.enemy[num_ji].y = transform_abstract_point(my_points_[j].point.y);
                            map_move_point.enemy_precise[num_ji].x = my_points_[j].point.x;
                            map_move_point.enemy_precise[num_ji].y = my_points_[j].point.y;
                            num_ji++;
                        }
                    }
                }
        }
    }
    Map_move_Point_->publish(map_move_point); //发布消息
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

    for (u_int i = 0; i < contours.size(); i++) //处理如果有多个颜色的情况，用vector承载
    {
        colcor_select::point_and_area temp_contour;
        auto area = contourArea(contours[i]); //要返回面积以判断为入口还是出口
        if (area > 100) {
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
