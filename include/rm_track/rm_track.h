//
// Created by qiayuan on 2021/9/29.
//
#pragma once

#include "receiver.h"
#include "logic_filter.h"
#include "logic_selector.h"
#include "ekf/linear_kf.h"
#include "ekf/armor_ekf.h"
#include <visualization_msgs/Marker.h>
#include <realtime_tools/realtime_publisher.h>

namespace rm_track
{
class RmTrack
{
public:
  explicit RmTrack(ros::NodeHandle& nh);
  void run();

private:
  void updateTrackerState();
  std::unordered_map<int, std::shared_ptr<Trackers>> id2trackers_;
  std::vector<LogicFilterBase*> logic_filters_;
  std::vector<LogicSelectorBase*> logic_selectors_;
  tf2_ros::Buffer* tf_buffer_;
  std::shared_ptr<AprilTagReceiver> apriltag_receiver_;
  std::shared_ptr<RmDetectionReceiver> rm_detection_receiver_;
  ros::Publisher track_pub_;

  visualization_msgs::Marker marker_targets_pos_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> marker_targets_pub_;

  mutable std::mutex mutex_;
};

}  // namespace rm_track
