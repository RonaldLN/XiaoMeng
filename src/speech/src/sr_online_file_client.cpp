#include "ros/ros.h"
#include "speech/speech.h"
#include <cstdlib>
#include <string.h>
#include <typeinfo>
#include "string"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_online_file_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<speech::speech>("sr_online_file");
  speech::speech srv;
  srv.request.source="START!";
  //ROS_INFO("%s",srv.response.output);
  //cout<<"hello"<<endl;
  //cout<<srv.response.output<<endl;
  if (client.call(srv))
  {
    cout<<srv.response.output<<endl;
    //ROS_INFO("hey");
  }
  else
  {
    ROS_ERROR("Failed to call service online speech_recognition");
    return 1;
  }

  return 0;
}

