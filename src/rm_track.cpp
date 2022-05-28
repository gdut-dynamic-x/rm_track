//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/rm_track.h"
#include <rm_msgs/TrackData.h>

namespace rm_track
{
RmTrack::RmTrack(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue filters;
  if (nh.getParam("filters", filters))
    for (int i = 0; i < filters.size(); ++i)
    {
      if (filters[i]["type"] == "height_filter")
        logic_filters_.push_back(HeightFilter(filters[i]));
      else if (filters[i]["type"] == "distance_filter")
        logic_filters_.push_back(DistanceFilter(filters[i]));
      else if (filters[i]["type"] == "confidence_filter")
        logic_filters_.push_back(ConfidenceFilter(filters[i]));
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

  predictor_.init(nh);
  apriltag_receiver_ = std::make_shared<AprilTagReceiver>(nh, buffer_, update_flag_, "/tag_detections");
  rm_detection_receiver_ = std::make_shared<RmDetectionReceiver>(nh, buffer_, update_flag_, "/detection");
  track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);
}

void RmTrack::run()
{
  if (!update_flag_)
    return;
  else
    update_flag_ = false;
  Buffer buffer = buffer_;
  buffer.eraseUselessData();
  for (auto filter : logic_filters_)
    filter.input(buffer);

  if (buffer.id2caches_.empty())
    return;

  if (buffer.id2caches_.size() == 1 && buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.size() == 1)
    target_armor_ =
        Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
               .id = buffer.id2caches_.begin()->first,
               .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
  else
    for (auto selector : logic_selectors_)
      if (selector.input(buffer))
      {
        target_armor_ = selector.output();
        break;
      }

  // TODO ekf
  if (!predictor_.inited_)
  {
    double x[6] = { target_armor_.transform.getOrigin().x(), 0, target_armor_.transform.getOrigin().y(), 0,
                    target_armor_.transform.getOrigin().z(), 0 };
    predictor_.reset(x);
  }
  else
  {
    auto it = buffer_.id2caches_.begin()->second.storage_que_.begin();
    double dt = (it->stamp_ - (it + 1)->stamp_).toSec();
    predictor_.updateQR();
    predictor_.predict(dt);
    double z[3] = { target_armor_.transform.getOrigin().x(), target_armor_.transform.getOrigin().y(),
                    target_armor_.transform.getOrigin().z() };
    predictor_.update(z, dt);
  }

  double x[6];
  predictor_.getState(x);
  rm_msgs::TrackData track_data;
  track_data.header.frame_id = "map";
  track_data.header.stamp = target_armor_.stamp;
  track_data.id = target_armor_.id;
  track_data.target_pos.x = x[0];
  track_data.target_pos.y = x[2];
  track_data.target_pos.z = x[4];
  track_data.target_vel.x = x[1];
  track_data.target_vel.y = x[3];
  track_data.target_vel.z = x[5];

  track_pub_.publish(track_data);
}

}  // namespace rm_track
