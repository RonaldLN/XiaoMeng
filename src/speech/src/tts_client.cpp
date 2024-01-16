#include "ros/ros.h"
#include "speech/speech.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tts_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<speech::speech>("tts");
  speech::speech srv;
  if (client.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service tts");
    return 1;
  }

  return 0;
}

