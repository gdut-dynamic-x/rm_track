//
// Created by qiayuan on 2021/9/29.
//
#pragma once

#include "receiver.h"
#include "logic_filter.h"
#include "logic_selector.h"
#include "ekf/linear_kf.h"
#include "ekf/armor_ekf.h"

namespace rm_track
{
class RmTrack
{
public:
  explicit RmTrack(ros::NodeHandle& nh);
  void run();
  bool matchTarget(Buffer buffer, int id, double predict_z[3], double match_z[3]);

private:
  Buffer buffer_;
  std::vector<LogicFilterBase*> logic_filters_;
  std::vector<LogicSelectorBase> logic_selectors_;
  LinearKf predictor_;
  int track_target_id_ = 0;
  double max_match_distance_ = 0.2;
  int lost_threshold_ = 10;
  int lost_count_ = 0;
  bool update_flag_ = false;
  tf2_ros::Buffer* tf_buffer_;
  ros::Time last_predict_time_;
  std::shared_ptr<AprilTagReceiver> apriltag_receiver_;
  std::shared_ptr<RmDetectionReceiver> rm_detection_receiver_;
  ros::Publisher track_pub_;
};

}  // namespace rm_track
