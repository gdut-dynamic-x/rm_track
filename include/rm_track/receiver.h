//
// Created by qiayuan on 2021/9/29.
//

#pragma once
#include "tracker.h"
#include <rm_msgs/TargetDetectionArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

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
               double max_match_distance, tf2_ros::Buffer* tf_buffer, std::string topic)
    : id2trackers_(id2trackers)
    , mutex_(mutex)
    , max_match_distance_(max_match_distance)
    , tf_buffer_(tf_buffer)
    , tf_filter_(msg_sub_, *tf_buffer_, "odom", 10, nullptr)
  {
    nh.param("max_storage_time", max_storage_time_, 5.0);
    nh.param("max_lost_time", max_lost_time_, 0.1);
    nh.param("num_data", num_data_, 20);
    msg_sub_.subscribe(nh, topic, 10);
    tf_filter_.registerCallback(boost::bind(&ReceiverBase::msgCallback, this, _1));
  }

protected:
  std::shared_ptr<Trackers>& allocateTrackers(int id)
  {
    id2trackers_.insert(std::make_pair(id, std::make_shared<Trackers>(id, max_match_distance_, max_lost_time_,
                                                                      max_storage_time_, num_data_)));
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
                      std::mutex& mutex, double max_match_distance, tf2_ros::Buffer* tf_buffer, std::string topic)
    : ReceiverBase(nh, id2trackers, mutex, max_match_distance, tf_buffer, topic), tf_listener(*tf_buffer_)
  {
  }
  ros::Time lastReceiveTime()
  {
    return last_receive_time_;
  }

private:
  tf2_ros::TransformListener tf_listener;
  ros::Time last_receive_time_;
  void msgCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    last_receive_time_ = ros::Time::now();
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
      targets_stamp.targets.push_back(Target{ .id = detection.id,
                                              .transform = transform,
                                              .confidence = detection.confidence,
                                              .distance_to_image_center = detection.distance_to_image_center });
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

}  // namespace rm_track
