//
// Created by qiayuan on 2021/9/29.
//
#pragma once

#include "receiver.h"
#include "logic_filter.h"
#include <rm_msgs/TargetDetectionArray.h>

namespace rm_track
{
class RmTrack
{
public:
  RmTrack();

private:
  Buffer buffer_;
  std::vector<LogicFilterBase> logic_filters;
};

}  // namespace rm_track