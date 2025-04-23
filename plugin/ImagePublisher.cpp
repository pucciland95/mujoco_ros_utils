#include "ImagePublisher.h"

#include <mujoco/mujoco.h>

#include <iostream>

class ParallelPixelDepthOpencv : public cv::ParallelLoopBody
{
private:
   cv::Mat* p_ ;
   float near_;
   float far_;
public:
   ParallelPixelDepthOpencv(cv::Mat* ptr, float near, float far ) : p_(ptr), near_(near_), far_(far) {}

   virtual void operator()( const cv::Range &r ) const
   {
      for (int i = r.start; i < r.end; ++i)
      {
         int row = i / p_->cols;
         int col = i % p_->cols;
         p_->at<float>(row, col) = near_ / (1.0f - p_->at<float>(row, col) * (1.0f - near_ / far_));
      }
   }
};

namespace MujocoRosUtils
{

void ImagePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::ImagePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {"frame_id", "color_topic_name", "depth_topic_name", "info_topic_name", "height",
                               "width",    "publish_rate"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = ImagePublisher::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class ImagePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->free();
    delete reinterpret_cast<ImagePublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class ImagePublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class ImagePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

ImagePublisher * ImagePublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // frame_id
  const char * frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  std::string frame_id = "";
  if(strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
  }

  // color_topic_name
  const char * color_topic_name_char = mj_getPluginConfig(m, plugin_id, "color_topic_name");
  std::string color_topic_name = "";
  if(strlen(color_topic_name_char) > 0)
  {
    color_topic_name = std::string(color_topic_name_char);
  }

  // depth_topic_name
  const char * depth_topic_name_char = mj_getPluginConfig(m, plugin_id, "depth_topic_name");
  std::string depth_topic_name = "";
  if(strlen(depth_topic_name_char) > 0)
  {
    depth_topic_name = std::string(depth_topic_name_char);
  }

  // info_topic_name
  const char * info_topic_name_char = mj_getPluginConfig(m, plugin_id, "info_topic_name");
  std::string info_topic_name = "";
  if(strlen(info_topic_name_char) > 0)
  {
    info_topic_name = std::string(info_topic_name_char);
  }

  // height
  const char * height_char = mj_getPluginConfig(m, plugin_id, "height");
  int height = 240;
  if(strlen(height_char) > 0)
  {
    height = static_cast<int>(strtol(height_char, nullptr, 10));
  }
  if(height <= 0)
  {
    mju_error("[ImagePublisher] `height` must be positive.");
    return nullptr;
  }

  // width
  const char * width_char = mj_getPluginConfig(m, plugin_id, "width");
  int width = 320;
  if(strlen(width_char) > 0)
  {
    width = static_cast<int>(strtol(width_char, nullptr, 10));
  }
  if(width <= 0)
  {
    mju_error("[ImagePublisher] `width` must be positive.");
    return nullptr;
  }

  // publish_rate
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum publish_rate = 30.0;
  if(strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if(publish_rate <= 0)
  {
    mju_error("[ImagePublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // Set sensor_id
  int sensor_id = 0;
  for(; sensor_id < m->nsensor; sensor_id++)
  {
    if(m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if(sensor_id == m->nsensor)
  {
    mju_error("[ImagePublisher] Plugin not found in sensors.");
    return nullptr;
  }
  if(m->sensor_objtype[sensor_id] != mjOBJ_CAMERA)
  {
    mju_error("[ImagePublisher] Plugin must be attached to a camera.");
    return nullptr;
  }

  std::cout << "[ImagePublisher] Create." << std::endl;

  return new ImagePublisher(m, d, sensor_id, frame_id, color_topic_name, depth_topic_name, info_topic_name, height,
                            width, publish_rate);
}

ImagePublisher::ImagePublisher(const mjModel * m,
                               mjData *, // d
                               int sensor_id,
                               const std::string & frame_id,
                               std::string color_topic_name,
                               std::string depth_topic_name,
                               std::string info_topic_name,
                               int height,
                               int width,
                               mjtNum publish_rate)
: sensor_id_(sensor_id), camera_id_(m->sensor_objid[sensor_id]), frame_id_(frame_id),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1)), viewport_({0, 0, width, height})
{
  std::string camera_name = std::string(mj_id2name(m, mjOBJ_CAMERA, camera_id_));
  if(frame_id_.empty())
  {
    frame_id_ = camera_name;
  }
  if(color_topic_name.empty())
  {
    color_topic_name = "mujoco/" + camera_name + "/color";
  }
  if(depth_topic_name.empty())
  {
    depth_topic_name = "mujoco/" + camera_name + "/depth";
  }
  if(info_topic_name.empty())
  {
    info_topic_name = "mujoco/" + camera_name + "/camera_info";
  }

  // Init OpenGL
  if(!glfwInit())
  {
    mju_error("[ImagePublisher] Could not initialize GLFW.");
  }

  // Create invisible window, single-buffered
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  window_ = glfwCreateWindow(viewport_.width, viewport_.height, "MujocoRosUtils::ImagePublisher", nullptr, nullptr);
  if(!window_)
  {
    mju_error("[ImagePublisher] Could not create GLFW window.");
  }

  // Make context current
  // \todo Is it OK to override the current context of OpenGL?
  glfwMakeContextCurrent(window_);

  // Init default data for visualization structures
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);

  // Create scene and context
  mjv_makeScene(m, &scene_, 1000);
  mjr_makeContext(m, &context_, mjFONTSCALE_100);

  // Init camera
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;

  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  if(context_.currentBuffer != mjFB_OFFSCREEN)
  {
    mju_error("[ImagePublisher] Offscreen rendering not supported, using default/window framebuffer.");
  }

  // Allocate buffer
  color_buffer_opencv_ptr_ = std::make_unique<cv::Mat> (viewport_.height, viewport_.width, CV_8UC3);
  depth_buffer_opencv_ptr_ = std::make_unique<cv::Mat> (viewport_.height, viewport_.width, CV_32FC1);

  // Init ROS
  int argc = 0;
  char ** argv = nullptr;
  if(!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;

  std::string ns = "mujoco_ros";

  nh_ = rclcpp::Node::make_shared("image_publisher", ns, node_options);
  color_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(color_topic_name, 1);
  depth_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(depth_topic_name, 1);
  info_pub_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name, 1);
}

void ImagePublisher::reset(const mjModel *, // m
                           int // plugin_id
)
{
}

void ImagePublisher::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  // Make context current
  // \todo Is it OK to override the current context of OpenGL?
//   glfwMakeContextCurrent(window_);

  // Update abstract scene
  mjv_updateScene(m, d, &option_, nullptr, &camera_, mjCAT_STATIC | mjCAT_DYNAMIC, &scene_);

  // Render scene in offscreen buffer
  mjr_render(viewport_, &scene_, &context_);

  // Read rgb and depth pixels
  mjr_readPixels(color_buffer_opencv_ptr_->data, NULL, viewport_, &context_);

  // Convert raw depth to distance and flip images
  float near = static_cast<float>(m->vis.map.znear * m->stat.extent);
  float far = static_cast<float>(m->vis.map.zfar * m->stat.extent);

  // https://github.com/deepmind/mujoco/blob/631b16e7ad192df936195658fe79f2ada85f755c/python/mujoco/renderer.py#L175-L178
  // ParallelPixelDepthOpencv obj(depth_buffer_opencv_ptr_.get(), near, far);
  // cv::parallel_for_( cv::Range(0, depth_buffer_opencv_ptr_->rows * depth_buffer_opencv_ptr_->cols), obj) ;

  // Publish topic
  rclcpp::Time stamp_now = nh_->get_clock()->now();

  // Common header
  std_msgs::msg::Header header;
  header.stamp = stamp_now;
  header.frame_id = frame_id_;

  cv::flip(*color_buffer_opencv_ptr_, *color_buffer_opencv_ptr_, 0);
  cv_bridge::CvImage cv_bridge_color_image = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, *color_buffer_opencv_ptr_);
  std::shared_ptr<sensor_msgs::msg::Image> color_msg = cv_bridge_color_image.toImageMsg();
  color_pub_->publish(*color_msg);

  cv::flip(*depth_buffer_opencv_ptr_, *depth_buffer_opencv_ptr_, 0);
  cv_bridge::CvImage cv_bridge_depth_image = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, *depth_buffer_opencv_ptr_);
  std::shared_ptr<sensor_msgs::msg::Image> depth_msg = cv_bridge_depth_image.toImageMsg();
  depth_pub_->publish(*depth_msg);

  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header.stamp = stamp_now;
  info_msg.header.frame_id = frame_id_;
  info_msg.height = viewport_.height;
  info_msg.width = viewport_.width;
  info_msg.distortion_model = "plumb_bob";
  info_msg.d.resize(5, 0.0);
  info_msg.k.fill(0.0);
  info_msg.r.fill(0.0);
  info_msg.p.fill(0.0);
  double focal_scaling = (1.0 / std::tan((m->cam_fovy[camera_id_] * M_PI / 180.0) / 2.0)) * viewport_.height / 2.0;
  info_msg.k[0] = info_msg.p[0] = focal_scaling;
  info_msg.k[2] = info_msg.p[2] = static_cast<double>(viewport_.width) / 2.0;
  info_msg.k[4] = info_msg.p[5] = focal_scaling;
  info_msg.k[5] = info_msg.p[6] = static_cast<double>(viewport_.height) / 2.0;
  info_msg.k[8] = info_msg.p[10] = 1.0;
  info_pub_->publish(info_msg);
}

void ImagePublisher::free()
{
  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);

  glfwDestroyWindow(window_);
}

} // namespace MujocoRosUtils
