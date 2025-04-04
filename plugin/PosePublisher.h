#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>

namespace MujocoRosUtils
{

struct PosePublisherOptions
{
   std::string frame_id;
   std::string pose_topic_name;
   std::string vel_topic_name;
   mjtNum publish_rate;
   bool output_tf;
   std::string tf_child_frame_id;
   int sensor_id;
   int body_id;
   std::string body_name;
};

/** \brief Plugin to publish topics or broadcast TF of pose and velocity of the body. */
class PosePublisher
{
 public:
   /** \brief Register plugin. */
   static void RegisterPlugin();

   /** \brief Create an instance.
       \param m model
       \param d data
       \param plugin_id plugin ID
    */
   static PosePublisher* Create(const mjModel* m, mjData* d, int plugin_id);

 public:
   /** \brief Copy constructor. */
   PosePublisher(PosePublisher&&) = default;

   /** \brief Reset.
       \param m model
       \param plugin_id plugin ID
    */
   void reset(const mjModel* m, int plugin_id);

   /** \brief Compute.
       \param m model
       \param d data
       \param plugin_id plugin ID
    */
   void compute(const mjModel* m, mjData* d, int plugin_id);

 protected:
   /** \brief Constructor.
       \param m model
       \param d data
       \param sensor_id sensor ID
       \param frame_id frame ID of topics header or TF parent
       \param pose_topic_name topic name of pose
       \param vel_topic_name topic name of velocity
       \param publish_rate publish rate
       \param output_tf whether to broadcast TF
       \param tf_child_frame_id child frame ID for TF
   */
   PosePublisher(const mjModel* m, mjData* d, PosePublisherOptions options);

 protected:

   PosePublisherOptions options_;

   //! ROS node handle
   rclcpp::Node::SharedPtr nh_;

   //! ROS publisher for pose
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

   //! ROS publisher for velocity
   rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

   //! TF broadcaster
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

   //! Iteration interval to skip ROS publish
   int publish_skip_ = 0;

   //! Iteration count of simulation
   int sim_cnt_ = 0;
};

}  // namespace MujocoRosUtils
