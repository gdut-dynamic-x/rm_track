//
// Created by qiayuan on 2021/9/29.
//
#pragma once

#include "receiver.h"
#include "logic_filter.h"

namespace rm_track
{
class RmTrack
{
public:
  explicit RmTrack(ros::NodeHandle& nh);
  void run();

private:
  Buffer buffer_;
  std::vector<LogicFilterBase> logic_filters_;
  std::shared_ptr<AprilTagReceiver> apriltag_receiver_;
};

}  // namespace rm_track