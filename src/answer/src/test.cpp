#include <rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<filesystem>
#include<bits/stdc++.h>

#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
//#include<opencv2/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include <answer/img_handle.h>

class Test1 : public rclcpp::Node {
public:
    Test1():Node("receive_img_msg") {
        imgSub=this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            10,
            std::bind(&Test1::imgCallback,this,std::placeholders::_1)
            );
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),"Image received");
        //cv_bridge::CvImagePtr cv_ptr;
        // try {
        //     cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        // } catch (cv_bridge::Exception& e) {
        //     RCLCPP_ERROR(this->get_logger(),"cv_bridge::Exception: %s", e.what());
        //     // 处理转换失败的情况
        //     return;
        // }
        // cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        // // 现在 cv_ptr->image 是一个 cv::Mat 类型的图像，你可以像使用普通 OpenCV 图像一样使用它
        // cv::Mat image = cv_ptr->image;

        cv_bridge::CvImagePtr cv_ptr=cv_bridge::toCvCopy(msg,msg->encoding);
        cv::Mat img=cv_ptr->image.clone();

        cv::imshow("image",img);
        cv::waitKey(1);


    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageHandle>());
    rclcpp::shutdown();
    return 0;
}