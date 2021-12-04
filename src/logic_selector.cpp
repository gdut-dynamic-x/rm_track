//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool SameIDArmorSelector::input(const std::vector<Armor>& armors)
{
  return true;
}

bool StaticArmorSelector::input(const std::vector<Armor>& armors)
{
  return true;
}

bool ClosestArmorSelector::input(const std::vector<Armor>& armors)
{
  return true;
}

}  // namespace rm_track