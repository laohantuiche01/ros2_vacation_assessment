#include<answer/Algorithm.h>
#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<queue>
#include<stack>

void Algorithm::move_control(std::queue<cv::Point> Point_queue_) {
    if (Points[7].x == Points[8].x && Points[7].y == Points[8].y) {
        move(0, 0, 0);
        fight(1);
        Points[8].x = -1;
        Points[8].y = -1;
    }
    if (Points[7].x == Points[9].x && Points[7].y == Points[9].y) {
        move(0, 0, 0);
        fight(2);
        Points[9].x = -1;
        Points[9].y = -1;
    } else {
        pause_condition = 1;
        while (Point_queue_.size() != 1 && pause_condition != 4) {
            if (Points[7].x == Points[8].x && Points[7].y == Points[8].y) {
                move(0, 0, 0);
                fight(1);
                Points[8].x = -1;
                Points[8].y = -1;
                break;
            }
            if (Points[7].x == Points[9].x && Points[7].y == Points[9].y) {
                move(0, 0, 0);
                fight(2);
                Points[9].x = -1;
                Points[9].y = -1;
                break;
            }
            pause_condition++;
            cv::Point m = Point_queue_.front();
            Point_queue_.pop();
            int temp_dx;
            int temp_dy;
            temp_dx = Point_queue_.front().x - m.x;
            temp_dy = Point_queue_.front().y - m.y;
            if (temp_dx > 0) //向右走
            {
                move(1000, 0, 0);
            } else if (temp_dx < 0) //向左走
            {
                move(-1000, 0, 0);
            } else if (temp_dx == 0) //数值方向
            {
                if (temp_dy > 0) //向下走
                {
                    move(0, 1000, 0);
                } else if (temp_dy < 0) //向上走
                {
                    move(0, -1000, 0);
                } else if (temp_dy == 0) //更新路径
                {
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(210));
        }
        move(1, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        move(0, 1, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        move(0, 0, 0);
    }
}

void Algorithm::fight(int is_who) {
    double dx;
    double dy;
    int num_ji = 0;
    example_interfaces::msg::Bool fire;
    fire.data = true;
    dx = pricise[is_who].x - pricise[0].x;
    dy = pricise[is_who].y - pricise[0].y;
    double theta = atan2(dy, dx);
    move(0, 0, theta);
    while (num_ji != 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        num_ji++;
        BoolPub->publish(fire);
    }
}


void Algorithm::move(int dx, int dy, double dtheta) //移动
{
    geometry_msgs::msg::Pose2D tempPose;
    tempPose.x = dx;
    tempPose.y = dy;
    tempPose.theta = dtheta;
    PosePub->publish(tempPose);
}

void Algorithm::arrayInitialization() //数组的初始化
{
    for (int i = 0; i <= 19; i++) {
        Points[i].x = -1;
        Points[i].y = -1;
    }
    for (int i = 0; i <= 99; i++) {
        way_points[i].x = -1;
        way_points[i].y = -1;
    }
}


void Algorithm::recieve_map_point(answer_infos::msg::MapPoint::SharedPtr msg) //接受不动点信息
{
    green_or_purple_ = msg->green_or_purple;
    Points[0].x = msg->destination.x - 2;
    Points[0].y = msg->destination.y;
    Points[1].x = msg->recover.x;
    Points[1].y = msg->recover.y;
    Points[2].x = msg->password.x;
    Points[2].y = msg->password.y;
    Points[3].x = msg->green_in.x;
    Points[3].y = msg->green_in.y;
    Points[4].x = msg->green_out.x;
    Points[4].y = msg->green_out.y;
    Points[5].x = msg->purple_in.x;
    Points[5].y = msg->purple_in.y;
    Points[6].x = msg->purple_out.x;
    Points[6].y = msg->purple_out.y; //存储不动点
}

void Algorithm::recieve_robot_location(answer_infos::msg::RobotLocation::SharedPtr msg) //更新位置
{
    auto request = msg.get();
    Points[7].x = msg->myself.x;
    Points[7].y = msg->myself.y;
    pricise[0].x = msg->myself_x;
    pricise[0].y = msg->myself_y;
    aneme_size = msg->enemy.size();
    if (aneme_size == 2) {
        Points[8].x = msg->enemy[0].x;
        Points[8].y = msg->enemy[0].y;
        pricise[1].x = msg->enemy_precise[0].x;
        pricise[1].y = msg->enemy_precise[0].y;
        Points[9].x = msg->enemy[1].x;
        Points[9].y = msg->enemy[1].y;
        pricise[2].x = msg->enemy_precise[1].x;
        pricise[2].y = msg->enemy_precise[1].y;
    } else if (aneme_size == 1) {
        Points[8].x = msg->enemy[0].x;
        Points[8].y = msg->enemy[0].y;
        pricise[1].x = msg->enemy_precise[0].x;
        pricise[1].y = msg->enemy_precise[0].y;
    } else if (aneme_size == 0) {
        return;
    }
}


void Algorithm::send_point() //发送给处理器的
{
    while (!way_service_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "wait_for_service failed");
    }

    auto request = std::make_shared<answer_infos::srv::WayService::Request>();
    request->input_point.resize(2);
    request->input_point[0].x = Points[4].x;
    request->input_point[0].y = Points[4].y;
    cv::Point point__ = decide_which_next();
    request->input_point[1].x = point__.x;
    request->input_point[1].y = point__.y;
    way_service_client->async_send_request(request,
                                           std::bind(&Algorithm::way_handle, this, std::placeholders::_1));
}

cv::Point Algorithm::decide_which_next() {
    RCLCPP_INFO(this->get_logger(), "decide_which_next");
    if (aneme_size == 2) //省两个敌人
    {
        RCLCPP_INFO(this->get_logger(), "fight to enemy1");
        return Points[9];
    }
    if (aneme_size == 1) //剩一个敌人
    {
        if (decide_if_go_to_recover == 0) //回到recover治疗
        {
            if (Points[7].x == Points[1].x && Points[7].y == Points[1].y) //到了变换条件
            {
                decide_if_go_to_recover = 1;
            }
            if (decide_if_go_to_recover == 0) {
                RCLCPP_INFO(this->get_logger(), "first to recover");
                return Points[1]; //回到治疗点
            }
        }
        RCLCPP_INFO(this->get_logger(), "fight to enemy2");
        return Points[8]; //找第二个敌人
    }
    if (aneme_size == 0) //没敌人了
    {
        if (decide_if_go_to_recover_again == 0) {
            if (Points[7].x == Points[1].x && Points[7].y == Points[1].y) {
                decide_if_go_to_recover_again = 1;
            } else {
                RCLCPP_INFO(this->get_logger(), "second to recover");
                return Points[1];
            }
        }
        if (decide_if_go_to_recover_again == 1) {
            if (has_in == 0) //是否已经进传送门
            {
                if (green_or_purple_ == 0) //紫色进
                {
                    if (Points[7].x == Points[5].x && Points[7].y == Points[5].y) has_in = 1; //是否到达了
                    if (has_in == 0) {
                        RCLCPP_INFO(this->get_logger(), "go to pur_in");
                        return Points[5]; //没到的话，返回pur in坐标
                    }
                } else if (green_or_purple_ == 1) //绿色进
                {
                    if (Points[7].x == Points[3].x && Points[7].y == Points[3].y) has_in = 1; //是否到达了
                    if (has_in == 0) {
                        RCLCPP_INFO(this->get_logger(), "go to gre_in");
                        return Points[3]; //没到的话，返回gre in坐标
                    }
                }
            } else if (has_in == 1) //已经进传送门
            {
                if (has_gone_password == 0) //还没到密码
                {
                    if (Points[7].x == Points[2].x && Points[7].y == Points[2].y) {
                        has_gone_password = 1;
                        example_interfaces::msg::Int64 temp;
                        temp.data = password[2];
                        Int64Pub->publish(temp);
                        RCLCPP_INFO(this->get_logger(), "password changed");
                    } else if (has_gone_password == 0) {
                        RCLCPP_INFO(this->get_logger(), "go to password");
                        return Points[2]; //如果没到，就返回其值
                    }
                }
                if (has_gone_password == 1) //到过密码
                {
                    if (has_gone_out == 0) //每到过出口
                    {
                        if (green_or_purple_ == 0) //绿色出
                        {
                            if (Points[7].x == Points[3].x && Points[7].y == Points[3].y) has_gone_out = 1; //是否到达了
                            if (has_gone_out == 0) {
                                RCLCPP_INFO(this->get_logger(), "go to gre_out");
                                return Points[3]; //没到的话，返回pur in坐标
                            }
                        } else if (green_or_purple_ == 1) //紫色出
                        {
                            if (Points[7].x == Points[5].x && Points[7].y == Points[5].y) has_gone_out = 1; //是否到达了
                            if (has_gone_out == 0) {
                                RCLCPP_INFO(this->get_logger(), "go to pur_out");
                                return Points[5]; //没到的话，返回gre in坐标
                            }
                        }
                    } else if (has_gone_out == 1) {
                        if (Points[7].x == Points[0].x && Points[7].y == Points[0].y) {
                            RCLCPP_INFO(this->get_logger(), "has reach the basic");
                            if (has_fire <= 8) {
                                move(0, 0, 0);
                                example_interfaces::msg::Bool fire;
                                fire.data = true;
                                BoolPub->publish(fire);
                                has_fire ++;
                            }
                        } else {
                            RCLCPP_INFO(this->get_logger(), "go to basic");
                            return Points[0];
                        }
                    }
                }
            }
        }
    }
}


void Algorithm::way_handle(rclcpp::Client<answer_infos::srv::WayService>::SharedFuture msg) //受到道路信息后处理
{
    auto result = msg.get();
    while (!Point_queue.empty()) {
        Point_queue.pop();
    }
    RCLCPP_INFO(this->get_logger(), "Got the way");
    if (result->point_way.size() > 0) {
        for (int i = 0; i <= result->point_way.size() - 1; i++) {
            cv::Point p;
            p.x = result->point_way[i].x;
            p.y = result->point_way[i].y;
            Point_queue.push(p);
        }
        move_control(Point_queue);
    }
}

void Algorithm::password_handle(example_interfaces::msg::Int64 msg) //处理passage
{
    password[count_] = msg.data;
    RCLCPP_INFO(this->get_logger(), "password[%ld]--------------------------------------", password[count_]);
    count_++;
    if (count_ == 2) {
        password[2] = password[1] + password[0];
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Algorithm>());
    rclcpp::shutdown();
    return 0;
}
