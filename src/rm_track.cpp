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

  apriltag_receiver_ = std::make_shared<AprilTagReceiver>(nh, buffer_);
  track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);
}

void RmTrack::run()
{
  Buffer buffer = buffer_;
  buffer.eraseUselessData();
  for (auto filter : logic_filters_)
    filter.input(buffer);

  if (buffer.id2caches_.empty())
    return;

  if (buffer.id2caches_.size() == 1 && buffer.id2caches_.begin()->second.storage_.size() == 1 &&
      buffer.id2caches_.begin()->second.storage_.front().targets_.size() == 1)
    target_armor_ = Armor{ .stamp = buffer.id2caches_.begin()->second.storage_.front().stamp_,
                           .id = buffer.id2caches_.begin()->first,
                           .transform = buffer.id2caches_.begin()->second.storage_.front().targets_.front().transform };
  else
    for (auto selector : logic_selectors_)
      if (selector.input(buffer))
      {
        target_armor_ = selector.output();
        break;
      }

  rm_msgs::TrackData track_data;
  track_data.stamp = target_armor_.stamp;
  track_data.id = target_armor_.id;
  track_data.camera2detection.x = target_armor_.transform.getOrigin().x();
  track_data.camera2detection.y = target_armor_.transform.getOrigin().y();
  track_data.camera2detection.z = target_armor_.transform.getOrigin().z();
  track_data.detection_vel.x = 0.;
  track_data.detection_vel.y = 0.;
  track_data.detection_vel.z = 0.;

  track_pub_.publish(track_data);
}

}  // namespace rm_track