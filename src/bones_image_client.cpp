#include "ros/ros.h"
#include "bones_image/bones_image.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bones_image_client");
  if (argc != 2)
  {
    ROS_INFO("usage: bones_image_client targetName");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<bones_image::bones_image>("image_analysis");
  bones_image::bones_image srv;
  srv.request.target = argv[1];
  if (client.call(srv))
  {
    ROS_INFO("Target Acquired? : %ld", (bool)srv.response.status);
    if(srv.response.status){
          ROS_INFO_STREAM("Target Pose " << srv.response.pose);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
