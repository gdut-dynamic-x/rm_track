//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool LastArmorSelector::input(const Buffer& buffer)
{
  return true;
}

bool SameIDArmorSelector::input(const Buffer& buffer)
{
  return true;
}

bool StaticArmorSelector::input(const Buffer& buffer)
{
  return true;
}

bool ClosestArmorSelector::input(const Buffer& buffer)
{
  return true;
}

}  // namespace rm_track