#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <mujoco_ros_utils/msg/scalar_stamped.hpp>
#include <string>

namespace MujocoRosUtils
{

/** \brief Type of ROS message. */
typedef enum MessageType_
{
   //! Scalar
   MsgScalar = 0,

   //! Point
   MsgPoint,

   //! 3D vector
   MsgVector3,

   //! Quaternion
   MsgQuaternion
} MessageType;

struct SensorPublisherOptions
{
   int sensor_id;
   MessageType msg_type;
   std::string frame_id;
   std::string topic_name;
   std::string sensor_name;
   mjtNum publish_rate;
};

/** \brief Plugin to publish sensor data. */
class SensorPublisher
{
 public:
   /** \brief Register plugin. */
   static void RegisterPlugin();

   /** \brief Create an instance.
       \param m model
       \param d data
       \param plugin_id plugin ID
    */
   static SensorPublisher* Create(const mjModel* m, mjData* d, int plugin_id);

 public:
   /** \brief Copy constructor. */
   SensorPublisher(SensorPublisher&&) = default;

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
       \param msg_type type of ROS message
       \param frame_id frame ID of message header
       \param topic_name topic name
       \param publish_rate publish rate
    */
   SensorPublisher(const mjModel* m, mjData* d, SensorPublisherOptions options);

 protected:

   //! Options for the class
   SensorPublisherOptions options_;

   //! ROS node handle
   rclcpp::Node::SharedPtr nh_;

   //! ROS publisher
   rclcpp::PublisherBase::SharedPtr pub_;

   //! Iteration interval to skip ROS publish
   int publish_skip_ = 0;

   //! Iteration count of simulation
   int sim_cnt_ = 0;
};

}  // namespace MujocoRosUtils
