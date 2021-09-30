//
// Created by qiayuan on 2021/9/29.
//

#pragma once
#include "buffer.h"
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
  ReceiverBase(ros::NodeHandle& nh, Buffer& buffer)
    : buffer_(buffer), tf_listener(tf_buffer_), tf_filter_(msg_sub_, tf_buffer_, "map", 10, 0)
  {
    msg_sub_.subscribe(nh, "/detection", 10);
    tf_filter_.registerCallback(boost::bind(&ReceiverBase::msgCallback, this, _1));
  }

protected:
  std::unordered_map<int, DetectionStorage> id2storage_;

  void insertData(int id, const geometry_msgs::PoseStamped& pose, double confidence)
  {
    geometry_msgs::PoseStamped pose_out;
    try
    {
      tf_buffer_.transform(pose, pose_out, "map");
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Failure %s\n", ex.what());
    }
    if (id2storage_.find(id) != id2storage_.end())
      id2storage_.insert(std::make_pair(id, DetectionStorage(pose_out.header.stamp)));
    id2storage_[id].insertData(pose_out.pose, confidence);
  }

  void updateBuffer()
  {
    for (const auto& storage : id2storage_)
      buffer_.insertData(storage.first, storage.second);
    id2storage_.clear();
  }

private:
  virtual void msgCallback(const MsgType& msg) = 0;

  Buffer& buffer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener;
  message_filters::Subscriber<MsgType> msg_sub_;
  tf2_ros::MessageFilter<MsgType> tf_filter_;
};

class RmDetectionReceiver : public ReceiverBase<rm_msgs::TargetDetectionArray>
{
public:
  using ReceiverBase<rm_msgs::TargetDetectionArray>::ReceiverBase;

private:
  void msgCallback(const rm_msgs::TargetDetectionArray& msg) override
  {
    for (const auto& detection : msg.detections)
    {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = msg.header.frame_id;
      pose_stamped.header.stamp = msg.header.stamp;
      pose_stamped.pose = detection.pose;
      insertData(detection.id, pose_stamped, detection.confidence);
    }
    updateBuffer();
  }
};

class AprilTagReceiver : public ReceiverBase<apriltag_ros::AprilTagDetectionArray>
{
public:
  using ReceiverBase<apriltag_ros::AprilTagDetectionArray>::ReceiverBase;

private:
  void msgCallback(const apriltag_ros::AprilTagDetectionArray& msg) override
  {
    for (const auto& detection : msg.detections)
    {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = msg.header.frame_id;
      pose_stamped.header.stamp = msg.header.stamp;
      pose_stamped.pose = detection.pose.pose.pose;
      // Suppose not tag bundle and take tens digit as id, since apriltag_ros does not support tags with the same id
      // appear in one image
      insertData(detection.id[0] % 10, pose_stamped, 1.0);
    }
    updateBuffer();
  }
};

}  // namespace rm_track
