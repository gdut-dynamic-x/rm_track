//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/rm_track.h"
#include <rm_msgs/TrackData.h>

namespace rm_track
{
RmTrack::RmTrack(ros::NodeHandle& nh)
{
  tf_buffer_ = new tf2_ros::Buffer(ros::Duration(10));
  nh.param("max_match_distance", max_match_distance_, 0.2);
  nh.param("lost_threshold", lost_threshold_, 10);
  XmlRpc::XmlRpcValue filters;
  if (nh.getParam("filters", filters))
    for (int i = 0; i < filters.size(); ++i)
    {
      if (filters[i]["type"] == "height_filter")
        logic_filters_.push_back(new HeightFilter(filters[i], tf_buffer_));
      else if (filters[i]["type"] == "distance_filter")
        logic_filters_.push_back(new DistanceFilter(filters[i], tf_buffer_));
      else if (filters[i]["type"] == "confidence_filter")
        logic_filters_.push_back(new ConfidenceFilter(filters[i]));
      else
        ROS_ERROR("Filter '%s' does not exist", filters[i].toXml().c_str());
    }
  else
    ROS_ERROR("No filters are defined (namespace %s)", nh.getNamespace().c_str());

  XmlRpc::XmlRpcValue selectors;
  if (nh.getParam("selectors", selectors))
    for (int i = 0; i < selectors.size(); ++i)
    {
      if (selectors[i] == "last_armor")
        logic_selectors_.push_back(LastArmorSelector());
      else if (selectors[i] == "same_id_armor")
        logic_selectors_.push_back(SameIDArmorSelector());
      else if (selectors[i] == "static_armor")
        logic_selectors_.push_back(StaticArmorSelector());
      else if (selectors[i] == "closest_armor")
        logic_selectors_.push_back(ClosestArmorSelector());
      else
        ROS_ERROR("Selector '%s' does not exist", selectors[i].toXml().c_str());
    }
  else
    ROS_ERROR("No selectors are defined (namespace %s)", nh.getNamespace().c_str());

  ros::NodeHandle kf_nh = ros::NodeHandle(nh, "linear_kf");
  predictor_.init(kf_nh);
  apriltag_receiver_ = std::make_shared<AprilTagReceiver>(nh, buffer_, tf_buffer_, update_flag_, "/tag_detections");
  rm_detection_receiver_ = std::make_shared<RmDetectionReceiver>(nh, buffer_, tf_buffer_, update_flag_, "/detection");
  track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);
}

void RmTrack::run()
{
  Buffer buffer = buffer_;
  for (auto& filter : logic_filters_)
    filter->input(buffer);
  buffer.eraseUselessData();

  double x[6]{};
  ros::Time now = ros::Time::now();
  if (!predictor_.inited_)
  {
    Armor target_armor{};
    if (buffer.id2caches_.empty())
      track_target_id_ = 0;
    else
    {
      if (buffer.id2caches_.size() == 1 && buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.size() == 1)
        target_armor =
            Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
                   .id = buffer.id2caches_.begin()->first,
                   .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
      else
        for (auto selector : logic_selectors_)
          if (selector.input(buffer))
          {
            target_armor = selector.output();
            break;
          }
      track_target_id_ = target_armor.id;
      double x_init[6] = { target_armor.transform.getOrigin().x(), 0, target_armor.transform.getOrigin().y(), 0,
                           target_armor.transform.getOrigin().z(), 0 };
      predictor_.reset(x_init);
      lost_count_ = 0;
      last_predict_time_ = target_armor.stamp;
    }
  }
  else
  {
    if (!update_flag_)
    {
      now = ros::Time::now();
      double dt = (now - last_predict_time_).toSec();
      predictor_.predict(x, dt);
    }
    else
    {
      update_flag_ = false;
      if (!buffer.id2caches_.count(track_target_id_))
      {
        predictor_.inited_ = false;
        track_target_id_ = 0;
      }
      else
      {
        double dt = (buffer.id2caches_[track_target_id_].storage_que_.front().stamp_ - last_predict_time_).toSec();
        predictor_.predict(dt);
        double predict_x[6];
        double match_z[3];
        predictor_.getState(predict_x);
        double predict_z[3]{ predict_x[0], predict_x[2], predict_x[4] };
        if (matchTarget(buffer, track_target_id_, predict_z, match_z))
        {
          predictor_.update(match_z);
          lost_count_ = 0;
        }
        else
        {
          lost_count_++;
          if (lost_count_ > lost_threshold_)
          {
            predictor_.inited_ = false;
            track_target_id_ = 0;
          }
        }
        last_predict_time_ = buffer.id2caches_[track_target_id_].storage_que_.front().stamp_;
        now = ros::Time::now();
        predictor_.predict(x, (now - last_predict_time_).toSec());
      }
    }
  }

  rm_msgs::TrackData track_data;
  track_data.header.frame_id = "odom";
  track_data.header.stamp = now;
  track_data.id = track_target_id_;
  track_data.target_pos.x = x[0];
  track_data.target_pos.y = x[2];
  track_data.target_pos.z = x[4];
  track_data.target_vel.x = x[1];
  track_data.target_vel.y = x[3];
  track_data.target_vel.z = x[5];

  track_pub_.publish(track_data);
}

bool RmTrack::matchTarget(Buffer buffer, int id, double predict_z[3], double match_z[3])
{
  DetectionStorage detections = buffer.id2caches_[id].storage_que_.front();
  if (detections.stamp_ <= last_predict_time_)
    return false;
  double min_position_diff = DBL_MAX;
  Target match_target;
  for (const auto& target : detections.targets_)
  {
    double position_diff =
        target.transform.getOrigin().distance(tf2::Vector3(predict_z[0], predict_z[1], predict_z[2]));
    if (position_diff < min_position_diff)
    {
      min_position_diff = position_diff;
      match_target = target;
    }
  }
  if (min_position_diff < max_match_distance_)
  {
    match_z[0] = match_target.transform.getOrigin().x();
    match_z[1] = match_target.transform.getOrigin().y();
    match_z[2] = match_target.transform.getOrigin().z();
    return true;
  }
  else
    return false;
}

}  // namespace rm_track
