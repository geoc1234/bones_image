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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_msgs/MoveBaseResult.h>
#include <actionlib/client/simple_action_client.h>

static
void visualize(cv::Mat& input, cv::Mat& faces, double fps);
void targetName_cb(const std_msgs::String &msg);
std_msgs::String targetName;
geometry_msgs::Twist cmd_vel;
bool newTarget=true;
sensor_msgs::Range targetLoc;
mbf_msgs::MoveBaseGoal goal_;
mbf_msgs::MoveBaseGoal previous_goal_;

bool goalChanged(mbf_msgs::MoveBaseGoal goal_, mbf_msgs::MoveBaseGoal previous_goal_);

int main(int argc, char** argv)
{    
	ros::init(argc, argv, "bones_image");

	ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Publisher target_pub = nh.advertise<sensor_msgs::Range>("targetRange", 5);
	ros::Publisher target_pub_ = nh.advertise<geometry_msgs::PoseStamped>("target_location", 5);
  	ros::Subscriber targetName_ = nh.subscribe("targetName", 1, targetName_cb);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped camera_to_map;

// create an instance of the client which the movebaseaction node will use.
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> ac_m("move_base_flex/move_base", true);
    ac_m.waitForServer();
    ROS_INFO("CONNECTED TO MOVEBASE SERVER");

    
// IMPORT THE TRAINING PHOTOS; A SINGLE FACE PER PHOTO
    const char* vid_device = "-v udpsrc port=1234 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 !  rtph264depay ! decodebin ! videoconvert ! appsink";
    const char* path2faces = "/home/geo/datasets/face_training";
    if(argc > 1) path2faces = argv[1];
    //if(argc > 2) vid_device = argv[2];

    targetName.data = "geo";
    bool verbose = true;
    bool targetAquired = false;
    unsigned long aquisitionTime;
    float targetDistance;
    float targetAngle;
    geometry_msgs::PoseStamped target_loc_;
    cv::Mat targetProfile;
    cv::Rect2i box;
    std::vector<cv::Mat> targetImages;
    cv::VideoCapture cap(vid_device);
    cv::Mat im;
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
                ROS_INFO("profile not acquired...");
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
            ret = cap.read(imi);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            ROS_INFO("NO IMAGE READ");
        }
        
        if(ret && !imi.empty())
        {
            ROS_INFO("IMAGE AQUIRED");
            int frameHeight = imi.rows;
            int frameWidth = imi.cols;
            cv::Mat im;
            cv::resize(imi,im,cv::Size2i(320,240));
           if(isTargetPresent(im, targetProfile, box, verbose))
           {
                cv::rectangle(im, cv::Rect2i(box.x,box.y, box.width, box.height), cv::Scalar(0, 255, 0), 2);
                cv::resize(im,imi,cv::Size2i(frameWidth,frameHeight));
                targetDistance = 0.4/(box.height / 240.0);
                targetAngle = ((320*0.5)-(box.x + box.width/2))/320.0 * 60.0; // assumes camera fov = 60 degrees
                targetLoc.header.frame_id = "camera";
                targetLoc.header.stamp = ros::Time::now();
                targetLoc.radiation_type = targetLoc.INFRARED;
                targetLoc.max_range = 6.0;
                targetLoc.min_range = 0.0;
                targetLoc.field_of_view = 0.30;
                targetLoc.range = targetDistance * 0.33; // meters
                target_pub.publish(targetLoc);
                if(verbose){ std::cout << "target found @ " 
                    << targetDistance << " meters " 
                    << targetAngle << " degrees"
                    << std::endl;                 
               }
               target_loc_.pose.position.x = targetDistance * 0.8;
               target_loc_.pose.position.y = 0;
               target_loc_.pose.position.z = 0;
               target_loc_.pose.orientation.z = sin(targetAngle*3.14/180.0/2.0);
               target_loc_.pose.orientation.x = 0.0;
               target_loc_.pose.orientation.y = 0.0;
               target_loc_.pose.orientation.w = cos(targetAngle*3.14/180.0/2.0);
               target_loc_.header.stamp = ros::Time::now();
               camera_to_map = tf_buffer.lookupTransform("map", "camera", ros::Time(0), ros::Duration(1.0) );
               tf2::doTransform(target_loc_, target_loc_, camera_to_map);
               target_pub_.publish(target_loc_);
               goal_.target_pose = target_loc_;
               if(goalChanged(goal_, previous_goal_) || !targetAquired){
                   targetAquired = true;
                    ac_m.sendGoal(goal_);
                    previous_goal_ = goal_;
               }
           }
           else if (!targetAquired){
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = 0.2;
              cmd_vel_pub.publish(cmd_vel);
           }

        cv::imshow("window",imi);
        int key = cv::waitKey(4);
        if(key == 27) break;
        ros::spinOnce();
        }
        else{
            ROS_INFO_STREAM("image empty? " << imi.empty());
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

bool goalChanged(mbf_msgs::MoveBaseGoal goal_, mbf_msgs::MoveBaseGoal previous_goal_)
{
    double delx = goal_.target_pose.pose.position.x - previous_goal_.target_pose.pose.position.x;
    double dely = goal_.target_pose.pose.position.y - previous_goal_.target_pose.pose.position.y;
    double dist = sqrt(pow(delx,2) + pow(dely,2));
    if(dist > 0.5) return true;
    return false;
}

void targetName_cb(const std_msgs::String &msg)
{
    ROS_INFO_STREAM("NAME: " << msg.data);
    targetName = msg;
    newTarget = true;
}