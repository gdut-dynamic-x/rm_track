//
// Created by qiayuan on 2021/9/22.
//

#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_track
{
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
      transforms.push_back(transform);
      confidences_.push_back(confidence[i]);
    }
  }
  ros::Time stamp_;
  std::vector<tf2::Transform> transforms;
  std::vector<double> confidences_;
};

}  // namespace rm_track