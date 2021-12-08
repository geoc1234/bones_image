#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>

#include <ros/ros.h>

int main(){

    cv::VideoCapture cap("/dev/video0");
    cv::Mat img;
    while (true){
    bool ret = cap.read(img);
    if(ret) cv::imshow("window", img);
    if(cv::waitKey(1) == 27) break;
    }

    return 0;
}

