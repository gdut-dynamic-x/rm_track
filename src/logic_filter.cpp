//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/logic_filter.h"

namespace rm_track
{
LogicFilterBase::LogicFilterBase(XmlRpc::XmlRpcValue rpc_value)
{
  if (rpc_value.hasMember("basic_range") && rpc_value.hasMember("double_check_range"))
  {
    basic_range_[0] = rpc_value["basic_range"][0];
    basic_range_[1] = rpc_value["basic_range"][1];
    double_check_range_[0] = rpc_value["double_check_range"][0];
    double_check_range_[1] = rpc_value["double_check_range"][1];
  }
  else
    ROS_ERROR("Some filter params doesn't given");
}

HeightFilter::HeightFilter(XmlRpc::XmlRpcValue rpc_value) : LogicFilterBase(rpc_value)
{
}
void HeightFilter::input(std::vector<Armor>& armors)
{
}

DistanceFilter::DistanceFilter(XmlRpc::XmlRpcValue rpc_value) : LogicFilterBase(rpc_value)
{
}
void DistanceFilter::input(std::vector<Armor>& armors)
{
}

ConfidenceFilter::ConfidenceFilter(XmlRpc::XmlRpcValue rpc_value) : LogicFilterBase(rpc_value)
{
}
void ConfidenceFilter::input(std::vector<Armor>& armors)
{
}

}  // namespace rm_track