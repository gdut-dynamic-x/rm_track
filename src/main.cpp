//
// Created by chenzheng on 2021/12/5.
//

#include "rm_track/rm_track.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_track");
  ros::NodeHandle nh("~");
  rm_track::RmTrack track(nh);

  ros::Rate loop_rate(200);
  while (ros::ok())
  {
    ros::spinOnce();
    track.run();
    loop_rate.sleep();
  }
  return 0;
}