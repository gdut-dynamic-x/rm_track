//
// Created by qiayuan on 2021/9/29.
//

#pragma once
#include "tracker.h"
#include <rm_msgs/TargetDetectionArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <rm_track/ImpreciseAutoAimConfig.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_track
{
template <class MsgType>
class ReceiverBase
{
public:
  ReceiverBase(ros::NodeHandle& nh, std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers, std::mutex& mutex,
               double max_match_distance, std::string dynamic_reconfig_node_name, tf2_ros::Buffer* tf_buffer,
               std::string topic)
    : id2trackers_(id2trackers)
    , mutex_(mutex)
    , max_match_distance_(max_match_distance)
    , dynamic_reconfig_node_name_(dynamic_reconfig_node_name)
    , tf_buffer_(tf_buffer)
    , tf_filter_(msg_sub_, *tf_buffer_, "odom", 10, nullptr)
  {
    nh.param("max_lost_time", max_lost_time_, 0.1);
    nh.param("max_storage_time", max_storage_time_, 5.0);
    nh.param("max_new_armor_time", max_new_armor_time_, 0.04);
    nh.param("max_judge_period", max_judge_period_, 0.1);
    nh.param("fliter_deque_length", fliter_deque_length_, 100);
    nh.param("num_data", num_data_, 20);
    nh.param("max_new_armor_match_distance", max_new_armor_match_distance_, 0.01);
    nh.param("max_spinning_time", max_spinning_time_, 0.004);
    nh.param("max_filter_deque_storage_time", max_filter_deque_storage_time_, 1.);
    msg_sub_.subscribe(nh, topic, 10);
    tf_filter_.registerCallback(boost::bind(&ReceiverBase::msgCallback, this, _1));
    staticValueInit();
  }

protected:
  void staticValueInit()
  {
    ReceiverBase<MsgType>::max_new_armor_time_dynamic_ = max_new_armor_time_;
    ReceiverBase<MsgType>::max_judge_period_dynamic_ = max_judge_period_;
    ReceiverBase<MsgType>::fliter_deque_length_dynamic_ = fliter_deque_length_;
    ReceiverBase<MsgType>::reconf_server_ = new dynamic_reconfigure::Server<rm_track::ImpreciseAutoAimConfig>(
        ros::NodeHandle("~/" + dynamic_reconfig_node_name_));
    dynamic_reconfigure::Server<rm_track::ImpreciseAutoAimConfig>::CallbackType cb;
    cb = boost::bind(&ReceiverBase<MsgType>::reconfCallback, this, _1, _2);
    ReceiverBase<MsgType>::reconf_server_->setCallback(cb);
  }
  void reconfCallback(rm_track::ImpreciseAutoAimConfig& conf, uint32_t level)
  {
    if (!ReceiverBase<MsgType>::dynamic_reconfig_initialized_)
    {
      conf.max_judge_period = ReceiverBase<MsgType>::max_judge_period_dynamic_;
      conf.fliter_deque_length = ReceiverBase<MsgType>::fliter_deque_length_dynamic_;
      conf.max_new_armor_time = ReceiverBase<MsgType>::max_new_armor_time_dynamic_;
      ReceiverBase<MsgType>::dynamic_reconfig_initialized_ = true;
    }
    ReceiverBase<MsgType>::max_new_armor_time_dynamic_ = conf.max_new_armor_time;
    ReceiverBase<MsgType>::max_judge_period_dynamic_ = conf.max_judge_period;
    ReceiverBase<MsgType>::fliter_deque_length_dynamic_ = conf.fliter_deque_length;
  }

  std::shared_ptr<Trackers>& allocateTrackers(int id)
  {
    id2trackers_.insert(std::make_pair(
        id, std::make_shared<Trackers>(id, max_match_distance_, max_lost_time_, max_storage_time_, num_data_,
                                       max_new_armor_time_dynamic_, max_new_armor_match_distance_, max_spinning_time_,
                                       max_judge_period_dynamic_, fliter_deque_length_dynamic_,
                                       max_filter_deque_storage_time_)));
    return id2trackers_[id];
  }
  void addTracker(ros::Time stamp, Target& target)
  {
    (id2trackers_.find(target.id) == id2trackers_.end() ? allocateTrackers(target.id) : id2trackers_[target.id])
        ->addTracker(stamp, target);
  }
  void updateTracker(TargetsStamp targets_stamp)
  {
    for (auto it = id2trackers_.begin(); it != id2trackers_.end();)
    {
      if (it->second->trackers_.empty())
        it = id2trackers_.erase(it);
      else
      {
        it->second->updateTracker(targets_stamp);
        it++;
      }
    }
    if (!targets_stamp.targets.empty())
      for (auto& target : targets_stamp.targets)
        addTracker(targets_stamp.stamp, target);
  }

  tf2_ros::Buffer* tf_buffer_;
  std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers_;
  double max_storage_time_, max_lost_time_;
  double max_match_distance_;
  double max_new_armor_time_;
  double max_new_armor_match_distance_;
  double max_spinning_time_;
  double max_judge_period_;
  double max_filter_deque_storage_time_;
  int fliter_deque_length_;
  std::string dynamic_reconfig_node_name_;
  static double max_new_armor_time_dynamic_;
  static int fliter_deque_length_dynamic_;
  static double max_judge_period_dynamic_;
  static bool dynamic_reconfig_initialized_;
  static dynamic_reconfigure::Server<rm_track::ImpreciseAutoAimConfig>* reconf_server_;
  int num_data_;
  std::mutex& mutex_;

private:
  virtual void msgCallback(const boost::shared_ptr<const MsgType>& msg) = 0;
  message_filters::Subscriber<MsgType> msg_sub_;
  tf2_ros::MessageFilter<MsgType> tf_filter_;
};

class RmDetectionReceiver : public ReceiverBase<rm_msgs::TargetDetectionArray>
{
public:
  RmDetectionReceiver(ros::NodeHandle& nh, std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers,
                      std::mutex& mutex, double max_match_distance, std::string dynamic_reconfig_node_name,
                      tf2_ros::Buffer* tf_buffer, std::string topic)
    : ReceiverBase(nh, id2trackers, mutex, max_match_distance, dynamic_reconfig_node_name, tf_buffer, topic)
    , tf_listener(*tf_buffer_)
  {
  }

private:
  tf2_ros::TransformListener tf_listener;

  void msgCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    TargetsStamp targets_stamp;
    targets_stamp.stamp = msg->header.stamp;
    for (const auto& detection : msg->detections)
    {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = msg->header.frame_id;
      pose_stamped.header.stamp = msg->header.stamp;
      pose_stamped.pose = detection.pose;
      try
      {
        tf_buffer_->transform(pose_stamped, pose_stamped, "odom");
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("Failure %s\n", ex.what());
      }
      tf2::Transform transform;
      tf2::fromMsg(pose_stamped.pose, transform);
      targets_stamp.targets.push_back(Target{
          .id = detection.id,
          .transform = transform,
          .confidence = detection.confidence,
          .distance_to_image_center = detection.distance_to_image_center,
      });
    }
    updateTracker(targets_stamp);
  }
};

class AprilTagReceiver : public ReceiverBase<apriltag_ros::AprilTagDetectionArray>
{
public:
  using ReceiverBase<apriltag_ros::AprilTagDetectionArray>::ReceiverBase;

private:
  void msgCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) override
  {
    TargetsStamp targets_stamp;
    targets_stamp.stamp = msg->header.stamp;
    for (const auto& detection : msg->detections)
    {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = msg->header.frame_id;
      pose_stamped.header.stamp = msg->header.stamp;
      pose_stamped.pose = detection.pose.pose.pose;
      try
      {
        tf_buffer_->transform(pose_stamped, pose_stamped, "odom");
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("Failure %s\n", ex.what());
      }
      tf2::Transform transform;
      tf2::fromMsg(pose_stamped.pose, transform);
      targets_stamp.targets.push_back(Target{ .id = detection.id[0], .transform = transform, .confidence = 1.0 });
    }
    updateTracker(targets_stamp);
  }
};
template <class MsgType>
bool ReceiverBase<MsgType>::dynamic_reconfig_initialized_ = false;
template <class MsgType>
double ReceiverBase<MsgType>::max_new_armor_time_dynamic_;
template <class MsgType>
double ReceiverBase<MsgType>::max_judge_period_dynamic_;
template <class MsgType>
int ReceiverBase<MsgType>::fliter_deque_length_dynamic_;
template <class MsgType>
dynamic_reconfigure::Server<rm_track::ImpreciseAutoAimConfig>* ReceiverBase<MsgType>::reconf_server_;
}  // namespace rm_track
