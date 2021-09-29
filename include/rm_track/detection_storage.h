//
// Created by qiayuan on 2021/9/22.
//

#pragma once
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_track
{
struct Target
{
  tf2::Transform transform;
  double confidence;
};

class DetectionStorage
{
public:
  DetectionStorage(ros::Time stamp, const std::vector<geometry_msgs::Transform>& datas,
                   const std::vector<double>& confidence)
    : stamp_(stamp)
  {
    for (int i = 0; i < datas.size(); ++i)
    {
      tf2::Transform transform;
      tf2::fromMsg(datas[i].translation, transform);
      targets_.push_back(Target{ .transform = transform, .confidence = confidence[i] });
    }
  }
  ros::Time stamp_;
  std::vector<Target> targets_;
};

}  // namespace rm_track