#include<answer/Algorithm.h>
#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<queue>
#include<stack>

bool Algorithm::IfCanGo(int x, int y, int direction_input) {
    while (!if_can_go->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "wait_for_service failed");
    }
    auto input_request=std::make_shared<answer_infos::srv::IfCanGo::Request>();
    input_request->point_x=x;
    input_request->point_y=y;
    input_request->dir=direction_input;
    if_can_go->async_send_request(input_request,
        std::bind(&Algorithm::recive_bool,this,std::placeholders::_1)
        );
    rclcpp::Client<answer_infos::srv::IfCanGo>::SharedFuture recive_msg;
    return judgment;
}
void Algorithm::recive_bool(rclcpp::Client<answer_infos::srv::IfCanGo>::SharedFuture recive_msg) {
    auto result=recive_msg.get();
    judgment=result->result_bool;
}


std::stack<cv::Point> Algorithm::reflash_map(cv::Point basic_point, cv::Point end_point) {

    std::queue<cv::Point> Point_queue;
    Point_queue.push(basic_point);
    remember_the_way[basic_point.x][basic_point.y] = 4;

    while (!Point_queue.empty())  //dfs找路
        {
        int x = Point_queue.front().x;
        int y = Point_queue.front().y;
        Point_queue.pop();

        for (int i=0;i<=3;i++) {
            cv::Point temp_point ;
            temp_point.x = x +direction::dir[i][0];
            temp_point.y = y +direction::dir[i][1];
            if (IfCanGo(temp_point.x,temp_point.y,i)) {
                Point_queue.push(temp_point);
                remember_the_way[temp_point.x][temp_point.y] = i;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(),"remember_the_way");
    std::stack<cv::Point> path;
    int temp_x=end_point.x;
    int temp_y=end_point.y;
    while (temp_x != end_point.x || temp_y != end_point.y) {
        cv::Point temp_point ;
        temp_point.x=temp_x-direction::dir[remember_the_way[temp_x][temp_y]][0];
        temp_point.y=temp_y-direction::dir[remember_the_way[temp_x][temp_y]][1];
        path.push(temp_point);
        temp_x=temp_point.x;
        temp_y=temp_point.y;
    }
    path.push(end_point);
     while(!path.empty())
    {
        cv::Point cout_point=path.top();
        std::cout<<cout_point.x<<" "<<cout_point.y<<std::endl;
        path.pop();
    }  

    return path;
}

void Algorithm::map_Initialization() {
    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 15; j++) {
            remember_the_way[i][j] = -1; //表示未知
        }
    }
}

void Algorithm::cout_map() {
    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 15; j++) {
            std::cout << remember_the_way[i][j] << " ";
        }
        std::cout << std::endl;
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Algorithm>());
    rclcpp::shutdown();
    return 0;
}