#include<answer/Algorithm.h>
#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<queue>
#include<stack>

void Algorithm::move_control() {
    // if (1) {
    //     int temp_dx;
    //     int temp_dy;
    //     temp_dx=way_points[1].x-way_points[0].x;
    //     temp_dy=way_points[1].y-way_points[0].y;
    //     if (temp_dx >0)//向右走
    //     {
    //         move(1000,0);
    //     }
    //     else if (temp_dx<0)//向左走
    //         {
    //         move(-1000,0);
    //     }
    //     else if (temp_dx==0)//数值方向
    //     {
    //         if (temp_dy>0) //向下走
    //             {move(0,-1000);}
    //         else if (temp_dy<0) //向上走
    //             {move(0,1000);}
    //         else if (temp_dy==0)//更新路径
    //             {
    //             send_point();
    //         }
    //     }
    // }
    while (Point_queue.size() != 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        cv::Point m = Point_queue.front();
        Point_queue.pop();
        int temp_dx;
        int temp_dy;
        temp_dx = Point_queue.front().x - m.x;
        temp_dy = Point_queue.front().y - m.y;
        if (temp_dx > 0) //向右走
        {
            move(1000, 0);
        } else if (temp_dx < 0) //向左走
        {
            move(-1000, 0);
        } else if (temp_dx == 0) //数值方向
        {
            if (temp_dy > 0) //向下走
            {
                move(0, 1000);
            } else if (temp_dy < 0) //向上走
            {
                move(0, -1000);
            } else if (temp_dy == 0) //更新路径
            {
                //send_point();
            }
        }
    }
}

void Algorithm::move(int dx, int dy) {
    int dx_ = std::abs(pricise[0].x - way_points[0].x * 64 + 32);
    int dy_ = std::abs(pricise[0].y - way_points[0].y * 64 + 32);
    if (dx_ > 10 && dy_ > 10) {
        geometry_msgs::msg::Pose2D tempPose;
        tempPose.x = dx;
        tempPose.y = dy;
        tempPose.theta = atan2(tempPose.y, tempPose.x);
        PosePub->publish(tempPose);
    }
}


void Algorithm::arrayInitialization() {
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

void Algorithm::recieve_robot_location(answer_infos::msg::RobotLocation::SharedPtr msg) {
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


void Algorithm::send_point() {
    while (!way_service_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "wait_for_service failed");
    }
    auto request = std::make_shared<answer_infos::srv::WayService::Request>();
    request->input_point.resize(2);
    request->input_point[0].x = Points[7].x;
    request->input_point[0].y = Points[7].y;
    // request->input_point[0].x = 3;
    // request->input_point[0].y = 1;
    request->input_point[1].x = 6;
    request->input_point[1].y = 5;
    // request->input_point[1].x = Points[1].x;
    // request->input_point[1].y = Points[1].y;
    way_service_client->async_send_request(request,
                                           std::bind(&Algorithm::way_handle, this, std::placeholders::_1));
}

void Algorithm::way_handle(rclcpp::Client<answer_infos::srv::WayService>::SharedFuture msg) {
    auto result = msg.get();
    while (!Point_queue.empty()) {
        Point_queue.pop();
    }
    // for (int i = 0; i < result->point_way.size(); i++) {
    //     way_points[i].x = result->point_way[i].x;
    //     way_points[i].y = result->point_way[i].y;
    //     std::cout << way_points[i].x << " " << way_points[i].y << std::endl;
    // }
    for (int i = 0; i <= result->point_way.size() - 1; i++) {
        cv::Point p;
        p.x = result->point_way[i].x;
        p.y = result->point_way[i].y;
        Point_queue.push(p);
    }
    move_control();
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Algorithm>());
    rclcpp::shutdown();
    return 0;
}
