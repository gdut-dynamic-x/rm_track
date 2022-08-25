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
  ReceiverBase(ros::NodeHandle& nh, Buffer& buffer, tf2_ros::Buffer* tf_buffer, std::string topic)
    : buffer_(buffer), tf_buffer_(tf_buffer), tf_filter_(msg_sub_, *tf_buffer_, "odom", 10, nullptr)
  {
    msg_sub_.subscribe(nh, topic, 10);
    tf_filter_.registerCallback(boost::bind(&ReceiverBase::msgCallback, this, _1));
  }

protected:
  tf2_ros::Buffer* tf_buffer_;
  Buffer& buffer_;

private:
  virtual void msgCallback(const boost::shared_ptr<const MsgType>& msg) = 0;

  message_filters::Subscriber<MsgType> msg_sub_;
  tf2_ros::MessageFilter<MsgType> tf_filter_;
};

class RmDetectionReceiver : public ReceiverBase<rm_msgs::TargetDetectionArray>
{
public:
  RmDetectionReceiver(ros::NodeHandle& nh, Buffer& buffer, tf2_ros::Buffer* tf_buffer, std::string topic)
    : ReceiverBase(nh, buffer, tf_buffer, topic), tf_listener(*tf_buffer_)
  {
  }

private:
  tf2_ros::TransformListener tf_listener;
  void msgCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg) override
  {
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
      targets_stamp.targets.push_back(
          Target{ .id = detection.id, .transform = transform, .confidence = detection.confidence });
    }
    buffer_.updateBuffer(targets_stamp);
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
    buffer_.updateBuffer(targets_stamp);
  }
};

}  // namespace rm_track
