//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/rm_track.h"

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
    }
  else
    ROS_ERROR("No filters are defined (namespace %s)", nh.getNamespace().c_str());

  apriltag_receiver_ = std::make_shared<AprilTagReceiver>(nh, buffer_);
}

void RmTrack::run()
{
  Buffer buffer = buffer_;
  for (auto filter : logic_filters_)
    filter.input(buffer);

  // TODO selectors(input: buffer, outout: target pose)
}

}  // namespace rm_track