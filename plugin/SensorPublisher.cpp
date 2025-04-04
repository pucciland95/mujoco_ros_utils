#include "SensorPublisher.h"

#include <mujoco/mujoco.h>

#include <iostream>

namespace MujocoRosUtils
{

void SensorPublisher::RegisterPlugin()
{
   mjpPlugin plugin;
   mjp_defaultPlugin(&plugin);

   plugin.name = "MujocoRosUtils::SensorPublisher";
   plugin.capabilityflags |= mjPLUGIN_SENSOR;

   const char* attributes[] = { "sensor_name", "frame_id", "topic_name", "publish_rate" };

   plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
   plugin.attributes = attributes;

   plugin.nstate = +[](const mjModel*,  // m
                       int              // plugin_id
                    ) { return 0; };

   plugin.nsensordata = +[](const mjModel*,  // m
                            int,             // plugin_id
                            int              // sensor_id
                         ) { return 0; };

   // Can only run after forces have been computed
   plugin.needstage = mjSTAGE_ACC;

   plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
      auto* plugin_instance = SensorPublisher::Create(m, d, plugin_id);
      if (!plugin_instance)
      {
         return -1;
      }
      d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
      return 0;
   };

   plugin.destroy = +[](mjData* d, int plugin_id) {
      delete reinterpret_cast<SensorPublisher*>(d->plugin_data[plugin_id]);
      d->plugin_data[plugin_id] = 0;
   };

   plugin.reset = +[](const mjModel* m, double*,  // plugin_state
                      void* plugin_data, int plugin_id) {
      auto* plugin_instance = reinterpret_cast<class SensorPublisher*>(plugin_data);
      plugin_instance->reset(m, plugin_id);
   };

   plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id, int  // capability_bit
                     ) {
      auto* plugin_instance = reinterpret_cast<class SensorPublisher*>(d->plugin_data[plugin_id]);
      plugin_instance->compute(m, d, plugin_id);
   };

   mjp_registerPlugin(&plugin);
}

SensorPublisher* SensorPublisher::Create(const mjModel* m, mjData* d, int plugin_id)
{
   SensorPublisherOptions options;

   // sensor_name
   const char* sensor_name_char = mj_getPluginConfig(m, plugin_id, "sensor_name");
   if (strlen(sensor_name_char) == 0)
   {
      mju_error("[SensorPublisher] `sensor_name` is missing.");
      return nullptr;
   }
   options.sensor_name = std::string(sensor_name_char);

   int sensor_id = 0;
   for (; sensor_id < m->nsensor; sensor_id++)
   {
      if (strcmp(sensor_name_char, mj_id2name(m, mjOBJ_SENSOR, sensor_id)) == 0)
      {
         break;
      }
   }
   if (sensor_id == m->nsensor)
   {
      mju_error("[SensorCommand] The sensor with the specified name not found.");
      return nullptr;
   }
   options.sensor_id = sensor_id;

   // msg_type
   MessageType msg_type;
   int sensor_dim = m->sensor_dim[sensor_id];
   if (sensor_dim == 1)
   {
      msg_type = MsgScalar;
   }
   else if (sensor_dim == 3)
   {
      if (m->sensor_type[sensor_id] == mjSENS_FRAMEPOS)
      {
         msg_type = MsgPoint;
      }
      else
      {
         msg_type = MsgVector3;
      }
   }
   else if (sensor_dim == 4)
   {
      msg_type = MsgQuaternion;
   }
   else
   {
      mju_error("[SensorPublisher] Unsupported sensor data dimensions: %d.", sensor_dim);
      return nullptr;
   }
   options.msg_type = msg_type;

   // frame_id
   const char* frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
   std::string frame_id = "";
   if (strlen(frame_id_char) > 0)
   {
      frame_id = std::string(frame_id_char);
   }
   options.frame_id = frame_id;

   // topic_name
   const char* topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
   std::string topic_name = "";
   if (strlen(topic_name_char) > 0)
   {
      topic_name = std::string(topic_name_char);
   }
   options.topic_name = topic_name;

   // publish_rate
   const char* publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
   mjtNum publish_rate = 30.0;
   if (strlen(publish_rate_char) > 0)
   {
      publish_rate = strtod(publish_rate_char, nullptr);
   }
   if (publish_rate <= 0)
   {
      mju_error("[SensorPublisher] `publish_rate` must be positive.");
      return nullptr;
   }
   options.publish_rate = publish_rate;

   return new SensorPublisher(m, d, options);
}


SensorPublisher::SensorPublisher(const mjModel* m,
                                 mjData*,  // d
                                 SensorPublisherOptions options) : options_(options)
{
   publish_skip_ = std::max(static_cast<int>(1.0 / (options_.publish_rate * m->opt.timestep)), 1);

   if (options_.frame_id.empty())
   {
      options_.frame_id = "map";
   }
   if (options_.topic_name.empty())
   {
      std::string sensor_name = std::string(mj_id2name(m, mjOBJ_SENSOR, options_.sensor_id));
      options_.topic_name = "mujoco/" + sensor_name;
   }

   int argc = 0;
   char** argv = nullptr;
   if (!rclcpp::ok())
   {
      rclcpp::init(argc, argv);
   }
   rclcpp::NodeOptions node_options;

   nh_ = rclcpp::Node::make_shared("sensor_" + options_.sensor_name, "mujoco_sensor", node_options);
   if (options_.msg_type == MsgScalar)
   {
      pub_ = nh_->create_publisher<mujoco_ros_utils::msg::ScalarStamped>(options_.topic_name, 1);
   }
   else if (options_.msg_type == MsgPoint)
   {
      pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(options_.topic_name, 1);
   }
   else if (options_.msg_type == MsgVector3)
   {
      pub_ = nh_->create_publisher<geometry_msgs::msg::Vector3Stamped>(options_.topic_name, 1);
   }
   else  // if(options_.msg_type == MsgQuaternion)
   {
      pub_ = nh_->create_publisher<geometry_msgs::msg::QuaternionStamped>(options_.topic_name, 1);
   }

   RCLCPP_INFO(nh_->get_logger(), "[SensorPublisher] Associated to %s sensor has been created.", options_.sensor_name.c_str());
}

void SensorPublisher::reset(const mjModel*,  // m
                            int              // plugin_id
)
{
}

void SensorPublisher::compute(const mjModel* m, mjData* d, int  // plugin_id
)
{
   sim_cnt_++;
   if (sim_cnt_ % publish_skip_ != 0)
   {
      return;
   }

   std_msgs::msg::Header header;
   header.stamp = nh_->get_clock()->now();
   header.frame_id = options_.frame_id;

   int sensor_adr = m->sensor_adr[options_.sensor_id];
   if (options_.msg_type == MsgScalar)
   {
      mujoco_ros_utils::msg::ScalarStamped msg;
      msg.header = header;
      msg.value.data = d->sensordata[sensor_adr];
      std::dynamic_pointer_cast<rclcpp::Publisher<mujoco_ros_utils::msg::ScalarStamped>>(pub_)->publish(msg);
   }
   else if (options_.msg_type == MsgPoint)
   {
      geometry_msgs::msg::PointStamped msg;
      msg.header = header;
      msg.point.x = d->sensordata[sensor_adr + 0];
      msg.point.y = d->sensordata[sensor_adr + 1];
      msg.point.z = d->sensordata[sensor_adr + 2];
      std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::PointStamped>>(pub_)->publish(msg);
   }
   else if (options_.msg_type == MsgVector3)
   {
      geometry_msgs::msg::Vector3Stamped msg;
      msg.header = header;
      msg.vector.x = d->sensordata[sensor_adr + 0];
      msg.vector.y = d->sensordata[sensor_adr + 1];
      msg.vector.z = d->sensordata[sensor_adr + 2];
      std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>>(pub_)->publish(msg);
   }
   else  // if(options_.msg_type == MsgQuaternion)
   {
      geometry_msgs::msg::QuaternionStamped msg;
      msg.header = header;
      msg.quaternion.w = d->sensordata[sensor_adr + 0];
      msg.quaternion.x = d->sensordata[sensor_adr + 1];
      msg.quaternion.y = d->sensordata[sensor_adr + 2];
      msg.quaternion.z = d->sensordata[sensor_adr + 3];
      std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>>(pub_)->publish(msg);
   }
}

}  // namespace MujocoRosUtils
