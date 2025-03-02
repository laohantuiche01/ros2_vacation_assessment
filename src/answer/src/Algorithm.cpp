#include<answer/Algorithm.h>
#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<queue>
#include<stack>

void Algorithm::move_control(std::queue<cv::Point> Point_queue_) {
    // if ((Points[7].x == Points[8].x && Points[7].y == Points[8].y)
    //     || (Points[7].x == Points[9].x && Points[7].y == Points[9].y)) {
    //     move(0, 0, 0);
    //     fight();
    // } else {
        pause_condition = 1;
        while (Point_queue_.size() != 1 && pause_condition != 4) {
            // if ((Points[7].x == Points[8].x && Points[7].y == Points[8].y)
            // || (Points[7].x == Points[9].x && Points[7].y == Points[9].y)) {
            //     move(0, 0, 0);
            //     fight();
            //     break;
            // }
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
            std::this_thread::sleep_for(std::chrono::milliseconds(235));
        }
        move(0, 0, 0);
    //}
}

void Algorithm::fight() {
    double dx;
    double dy;
    int num_ji = 0;
    example_interfaces::msg::Bool fire;
    fire.data = true;
    dx = pricise[1].x - pricise[0].x;
    dy = pricise[1].y - pricise[0].y;
    double theta = atan2(dy, dx);
    move(0, 0, theta);
    while (num_ji != 7) {
        num_ji++;
        BoolPub->publish(fire);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
    Points[0].x = msg->destination.x;
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
    q = msg->enemy.size();
    if (q == 2) {
        Points[8].x = msg->enemy[0].x;
        Points[8].y = msg->enemy[0].y;
        pricise[1].x = msg->enemy_precise[0].x;
        pricise[1].y = msg->enemy_precise[0].y;
        Points[9].x = msg->enemy[1].x;
        Points[9].y = msg->enemy[1].y;
        pricise[2].x = msg->enemy_precise[1].x;
        pricise[2].y = msg->enemy_precise[1].y;
    } else if (q == 1) {
        Points[8].x = msg->enemy[0].x;
        Points[8].y = msg->enemy[0].y;
        pricise[1].x = msg->enemy_precise[0].x;
        pricise[1].y = msg->enemy_precise[0].y;
    } else if (q == 0) {
        return;
    }
}


void Algorithm::send_point() //发送给处理器的
{
    while (!way_service_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "wait_for_service failed");
    }
    //RCLCPP_INFO(this->get_logger(), "wait_for_service success");

    auto request = std::make_shared<answer_infos::srv::WayService::Request>();
    request->input_point.resize(2);
    request->input_point[0].x = Points[4].x;
    request->input_point[0].y = Points[4].y;
    // request->input_point[0].x = 3;
    // request->input_point[0].y = 1;
    request->input_point[1].x = 27;
    request->input_point[1].y = 3;
    // request->input_point[1].x = Points[8].x;
    // request->input_point[1].y = Points[8].y;
    // cv::Point point__=decide_which_next();
    // request->input_point[1].x = point__.x;
    // request->input_point[1].y = point__.y;
    way_service_client->async_send_request(request,
                                           std::bind(&Algorithm::way_handle, this, std::placeholders::_1));
}

cv::Point Algorithm::decide_which_next() {
    if (num <= 6) {
        num++;
        return Points[num];
    }
    return Points[0];
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
    RCLCPP_INFO(this->get_logger(), "password[%ld]", password[count_]);
    count_++;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Algorithm>());
    rclcpp::shutdown();
    return 0;
}
