//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/logic_filter.h"
#include <rm_common/ros_utilities.h>

namespace rm_track
{
HeightFilter::HeightFilter(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue rpc_value;
  if (!nh.getParam("height_filter", rpc_value))
    ROS_ERROR("height_filter no defined (namespace %s)", nh.getNamespace().c_str());
  if (rpc_value.hasMember("basic_range") && rpc_value.hasMember("double_check_range"))
  {
    basic_range_[0] = rpc_value["basic_range"][0];
    basic_range_[1] = rpc_value["basic_range"][1];
    double_check_range_[0] = rpc_value["double_check_range"][0];
    double_check_range_[1] = rpc_value["double_check_range"][1];
  }
  else
    ROS_ERROR("Some height_filter params doesn't given (namespace: %s)", nh.getNamespace().c_str());
}
void HeightFilter::input(Buffer& buffer)
{
}

}  // namespace rm_track