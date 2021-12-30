#include <iostream>
#include "myIO.hpp"
#include "myFace.hpp"
#include <vector>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

static
void visualize(cv::Mat& input, cv::Mat& faces, double fps);
void targetName_cb(const std_msgs::String &msg);
std_msgs::String targetName;
bool newTarget=true;
sensor_msgs::Range targetLoc;

int main(int argc, char** argv)
{    
	ros::init(argc, argv, "bones_image");

	ros::NodeHandle nh;
	ros::Publisher target_pub = nh.advertise<sensor_msgs::Range>("targetRange", 5);
  	ros::Subscriber targetName_ = nh.subscribe("targetName", 1, targetName_cb);

// IMPORT THE TRAINING PHOTOS; A SINGLE FACE PER PHOTO
    //const cv::String vid_device = "playbin udpsrc port=1234 ! appsink, NULL";
    const char* path2faces = "/home/geo/datasets/face_training";
    if(argc > 1) path2faces = argv[1];
    //if(argc > 2) vid_device = argv[2];

    targetName.data = "juju";
    bool verbose = false;
    float targetDistance;
    cv::Mat targetProfile;
    cv::Rect2i box;
    std::vector<cv::Mat> targetImages;
    //cv::VideoCapture cap(vid_device);
    //cv::VideoCapture *cap = new cv::VideoCapture();
    //bool camera = cap->open('udpsrc port=1234 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv::CAP_GSTREAMER);
    cv::Mat im;
    ros::Rate sleep(1.0);
    for (;;)

    {
        if(newTarget == true)
        {
            std::stringstream targetPath;
            std::string target_path;
            targetPath << path2faces << "/" << targetName.data;
            targetPath >> target_path;
            ROS_INFO_STREAM("TARGET: " << targetName.data);
            ROS_INFO_STREAM("TARGETPATH: " << target_path);
            targetImages = my_getfiles(target_path, verbose);
            if(!getTargetProfile(targetImages.at(0), targetProfile,verbose))
            {
                std::cout << "profile not acquired..." << std::endl;
            }
            else
            {
                newTarget = false;
            }
        }            
        cv::Mat imi;
        bool ret;
        try
        {
            //ret = cap->read(imi);
            imi = cv::imread("/home/geo/datasets/tmp.png");
            ret = true;
        }
        catch(const std::exception& e)
        {
          //  std::cerr << e.what() << '\n';
        }
        
        if(ret && !imi.empty())
        {
            //imi = ic.cvImage;
            int frameHeight = imi.rows;
            int frameWidth = imi.cols;
            cv::Mat im;
            cv::resize(imi,im,cv::Size2i(320,240));
           if(isTargetPresent(im, targetProfile, box, verbose))
           {
                cv::rectangle(im, cv::Rect2i(box.x,box.y, box.width, box.height), cv::Scalar(0, 255, 0), 2);
                cv::resize(im,imi,cv::Size2i(frameWidth,frameHeight));
                targetDistance = 1.0/(box.height / 240.0); //assumes a face ~ 1ft from camera fills the screen
                targetLoc.header.frame_id = "camera";
                targetLoc.header.stamp = ros::Time::now();
                targetLoc.radiation_type = targetLoc.INFRARED;
                targetLoc.max_range = 6.0;
                targetLoc.min_range = 0.0;
                targetLoc.field_of_view = 0.30;
                targetLoc.range = targetDistance * 0.33; // meters
                target_pub.publish(targetLoc);
                if(verbose) std::cout << "target found @ " << targetDistance << " feet" << std::endl;
           }

        cv::imshow("window",imi);
        int key = cv::waitKey(4);
        if(key == 27) break;
        ros::spinOnce();
        sleep.sleep();
        }
    }
    return 0;
}

static
void visualize(cv::Mat& input, cv::Mat& faces, double fps)
{
    int thickness = 2;
    std::string fpsString = cv::format("FPS : %.2f", (float)fps);
        for (int i = 0; i < faces.rows; i++) 
        {  // Draw bounding box
            cv::rectangle(input, cv::Rect2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)), int(faces.at<float>(i, 2)), int(faces.at<float>(i, 3))), cv::Scalar(0, 255, 0), thickness);
            // Draw landmarks
            cv::circle(input, cv::Point2i(int(faces.at<float>(i, 4)), int(faces.at<float>(i, 5))), 2, cv::Scalar(255, 0, 0), thickness);
            cv::circle(input, cv::Point2i(int(faces.at<float>(i, 6)), int(faces.at<float>(i, 7))), 2, cv::Scalar(0, 0, 255), thickness);
            cv::circle(input, cv::Point2i(int(faces.at<float>(i, 8)), int(faces.at<float>(i, 9))), 2, cv::Scalar(0, 255, 0), thickness);
            cv::circle(input, cv::Point2i(int(faces.at<float>(i, 10)), int(faces.at<float>(i, 11))), 2, cv::Scalar(255, 0, 255), thickness);
            cv::circle(input, cv::Point2i(int(faces.at<float>(i, 12)), int(faces.at<float>(i, 13))), 2, cv::Scalar(0, 255, 255), thickness);
        }
    cv::putText(input, fpsString, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
}

void targetName_cb(const std_msgs::String &msg)
{
    ROS_INFO_STREAM("NAME: " << msg.data);
    targetName = msg;
    newTarget = true;
}