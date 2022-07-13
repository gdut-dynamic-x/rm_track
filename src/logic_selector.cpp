//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool RandomArmorSelector::input(const Buffer& buffer)
{
  target_armor_ =
      Armor{ .stamp = buffer.id2caches_.begin()->second.storage_que_.begin()->stamp_,
             .id = buffer.id2caches_.begin()->first,
             .transform = buffer.id2caches_.begin()->second.storage_que_.begin()->targets_.begin()->transform };
  ROS_INFO("SUCCESSFUL,%d", target_armor_.id);
  return true;
}

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
  if (buffer.id2caches_.count(last_armor_.id))
  {
    target_armor_ =
        Armor{ .stamp = buffer.id2caches_.at(last_armor_.id).storage_que_.front().stamp_,
               .id = last_armor_.id,
               .transform = buffer.id2caches_.at(last_armor_.id).storage_que_.front().targets_.front().transform };
    last_armor_ = target_armor_;
    return true;
  }
  else
    return false;
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

bool HeroArmorSelector::input(const Buffer& buffer)
{
  if (buffer.id2caches_.count(1))
  {
    target_armor_ = Armor{ .stamp = buffer.id2caches_.at(1).storage_que_.front().stamp_,
                           .id = 1,
                           .transform = buffer.id2caches_.at(1).storage_que_.front().targets_.front().transform };
    last_armor_ = target_armor_;
    return true;
  }
  else
    return false;
}

bool StandardArmorSelector::input(const Buffer& buffer)
{
  int armor_id;
  bool has_standard = true;
  if (buffer.id2caches_.count(3))
    armor_id = 3;
  else if (buffer.id2caches_.count(4))
    armor_id = 4;
  else if (buffer.id2caches_.count(5))
    armor_id = 5;
  else
    has_standard = false;
  if (has_standard)
  {
    target_armor_ =
        Armor{ .stamp = buffer.id2caches_.at(armor_id).storage_que_.front().stamp_,
               .id = armor_id,
               .transform = buffer.id2caches_.at(armor_id).storage_que_.front().targets_.front().transform };
    last_armor_ = target_armor_;
    ROS_INFO("SUCCESSFUL,%d", armor_id);
    return true;
  }
  else
    return false;
}

Armor LogicSelectorBase::target_armor_;
Armor LogicSelectorBase::last_armor_;
}  // namespace rm_track
