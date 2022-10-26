#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "bones_image/bones_image.h"
#include "myFace.hpp"
#include "myIO.hpp"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>

class Bones_image_analyzer
{
  private:

  ros::NodeHandle nh_;
  const char* vid_device_ = "-v udpsrc port=1234 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 !  rtph264depay ! decodebin ! videoconvert ! appsink";
  const char* path2faces_ = "/home/geo/datasets/face_training";
  cv::Mat targetProfile_;
  cv::Rect2i box_;
  std::vector<cv::Mat> targetImages_;
  std::string loaded_target_ = "";
  bool targetAquired_ = false;
  bool verbose = false;
  float targetDistance_;
  float targetAngle_;
  geometry_msgs::TransformStamped camera_to_map_;

  public:

  tf2_ros::Buffer tfBuffer;
  cv::VideoCapture cap;
  cv::Mat imi;
  bool ret;


  Bones_image_analyzer()
  {
    //tf2_ros::TransformListener tfListener(tfBuffer);
    cap = cv::VideoCapture(vid_device_);
  }

  bool server(bones_image::bones_image::Request & req, bones_image::bones_image::Response & resp)
  {
    ROS_INFO("IMAGE SERVER CALLED");
    //load new profile if target identity has been changed
    if(req.target != loaded_target_)
    {
        std::stringstream targetPath;
        std::string target_path;
        targetPath << path2faces_ << "/" << req.target;
        targetPath >> target_path;
        ROS_INFO_STREAM("TARGET: " << req.target);
        ROS_INFO_STREAM("TARGETPATH: " << target_path);
        targetImages_ = my_getfiles(target_path, verbose);
        if(targetImages_.size() < 1){
          ROS_INFO_STREAM("TARGET LIST IS EMPTY");
          return false;
        }
        if(!getTargetProfile(targetImages_.at(0), targetProfile_, verbose))
        {
          ROS_INFO("target's profile not acquired...");
          return false;
        }
        else
        {
          loaded_target_ = req.target;
        }
    }
   
        resp.status = false;
        if(ret && !imi.empty())
        {
            int frameHeight = imi.rows;
            int frameWidth = imi.cols;
            cv::Mat im;
            cv::resize(imi,im,cv::Size2i(320,240));
            if(isTargetPresent(im, targetProfile_, box_, verbose))
            {
                cv::rectangle(im, cv::Rect2i(box_.x,box_.y, box_.width, box_.height), cv::Scalar(0, 255, 0), 2);
                cv::resize(im,imi,cv::Size2i(frameWidth,frameHeight));
                targetDistance_ = 0.4/(box_.height / 240.0);
                targetAngle_ = ((320*0.5)-(box_.x + box_.width/2))/320.0 * 60.0; // assumes camera fov = 60 degrees

               resp.pose.pose.position.x = targetDistance_ * cos(targetAngle_*3.14/180.0);
               resp.pose.pose.position.y = targetDistance_ * sin(targetAngle_*3.14/180.0);
               resp.pose.pose.position.z = 0.0;
               resp.pose.pose.orientation.z = 0.0;
               resp.pose.pose.orientation.x = 0.0;
               resp.pose.pose.orientation.y = 0.0;
               resp.pose.pose.orientation.w = 1.0;
               resp.pose.header.stamp = ros::Time::now();
               camera_to_map_ = this->tfBuffer.lookupTransform("map", "camera", ros::Time(0), ros::Duration(1.0) );
               ROS_INFO_STREAM("CAMERA_TO_MAP TRANSFORM: " << camera_to_map_);
               tf2::doTransform(resp.pose, resp.pose, camera_to_map_);
               resp.status = true;
               resp.pose.pose.position.z = 0.0;
               ROS_INFO_STREAM("TARGET LOCATION: " << resp.pose);
               resp.pose.pose.orientation.z = 0.0;
               resp.pose.pose.orientation.x = 0.0;
               resp.pose.pose.orientation.y = 0.0;
               resp.pose.pose.orientation.w = 1.0;
               resp.pose.header.stamp = ros::Time::now()+ros::Duration(0.5);
              }
    }
    return true; 
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bones_vision");
  ros::NodeHandle nh;
  Bones_image_analyzer bones_image_analyzer;
  tf2_ros::TransformListener tfListener(bones_image_analyzer.tfBuffer);


  ros::ServiceServer service = nh.advertiseService("image_analysis", &Bones_image_analyzer::server,&bones_image_analyzer);
  
  for(;;){

        bones_image_analyzer.ret = bones_image_analyzer.cap.read(bones_image_analyzer.imi);
        if(bones_image_analyzer.ret){
          cv::imshow("window",bones_image_analyzer.imi);
          int key = cv::waitKey(4);
        }
        ros::spinOnce();

  }
  return 0;
}