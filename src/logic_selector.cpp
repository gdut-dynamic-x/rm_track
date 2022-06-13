//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool LastArmorSelector::input(const Buffer& buffer)
{
  target_armor_ =
      Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
             .id = buffer.id2caches_.begin()->first,
             .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
  return true;
}

bool SameIDArmorSelector::input(const Buffer& buffer)
{
  target_armor_ =
      Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
             .id = buffer.id2caches_.begin()->first,
             .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
  return true;
}

bool StaticArmorSelector::input(const Buffer& buffer)
{
  target_armor_ =
      Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
             .id = buffer.id2caches_.begin()->first,
             .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
  return true;
}

bool ClosestArmorSelector::input(const Buffer& buffer)
{
  target_armor_ =
      Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
             .id = buffer.id2caches_.begin()->first,
             .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
  return true;
}

}  // namespace rm_track
