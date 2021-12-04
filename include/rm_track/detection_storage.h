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
  enum STATE
  {
    APPEAR,
    EXIST,
    DISAPPEAR,
    NOT_EXIST,
    EXPIRED
  } state;

  tf2::Transform transform;
  double confidence;
};

class DetectionStorage
{
public:
  enum STATE
  {
    STATIC,
    TRANSLATION,
    GYRO,
    UNKNOWN
  } state_;

  DetectionStorage(const ros::Time& stamp) : stamp_(stamp)
  {
  }

  void insertData(geometry_msgs::Pose data, double confidence)
  {
    tf2::Transform transform;
    tf2::fromMsg(data, transform);
    targets_.push_back(Target{ .transform = transform, .confidence = confidence });
  }

  std::vector<Target> eraseUselessData()
  {
    auto target_it = targets_.begin();
    while (target_it != targets_.end())
    {
      if (target_it->state == Target::APPEAR || target_it->state == Target::EXPIRED ||
          target_it->state == Target::NOT_EXIST)
        targets_.erase(target_it);
      target_it++;
    }
    return targets_;
  }

  DetectionStorage(ros::Time stamp, const std::vector<geometry_msgs::Pose>& datas,
                   const std::vector<double>& confidences)
    : stamp_(stamp)
  {
    for (int i = 0; i < datas.size(); ++i)
      insertData(datas[i], confidences[i]);
  }

  ros::Time stamp_;
  std::vector<Target> targets_;
};

}  // namespace rm_track