/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

//}

namespace mrs_uav_dji_tello_api
{

/* class MrsUavDjiTelloApi //{ */

class MrsUavDjiTelloApi : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~MrsUavDjiTelloApi(){};

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiStatus       getStatus();
  mrs_msgs::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
  bool callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg);

  void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<std_msgs::Bool> sh_armed_;

  void callbackArmed(const std_msgs::Bool::ConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sh_pose_;
  void                                                  callbackPose(const geometry_msgs::PoseStamped::ConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::TwistStamped> sh_twist_;
  void                                                   callbackTwist(const geometry_msgs::TwistStamped::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_battery_;
  void                                                 callbackBattery(const sensor_msgs::BatteryState::ConstPtr msg);

  mrs_lib::SubscribeHandler<std_msgs::Float64> sh_height_;
  void                                         callbackHeight(const std_msgs::Float64::ConstPtr msg);


  void publishOdom(void);

  // | --------------------- integrated pose -------------------- |

  Eigen::Vector3d pos_;

  ros::Time last_update_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd> ph_cmd_;

  // | ------------------------ services ------------------------ |

  mrs_lib::ServiceClientHandler<std_srvs::SetBool>    sch_arm_;
  mrs_lib::ServiceClientHandler<mavros_msgs::SetMode> sch_set_mode_;

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;

  geometry_msgs::PoseStamped pose_;
  std::mutex                 mutex_pose_;

  geometry_msgs::TwistStamped twist_;
  std::mutex                  mutex_twist_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUavDjiTelloApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  pos_ << 0, 0, 0;
  last_update_ = ros::Time(0);

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavDjiTelloApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MrsHwTelloApi";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_armed_ = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "armed_in", &MrsUavDjiTelloApi::callbackArmed, this);

  sh_pose_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "pose_in", &MrsUavDjiTelloApi::callbackPose, this);

  sh_twist_ = mrs_lib::SubscribeHandler<geometry_msgs::TwistStamped>(shopts, "twist_in", &MrsUavDjiTelloApi::callbackTwist, this);

  sh_battery_ = mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "battery_in", &MrsUavDjiTelloApi::callbackBattery, this);

  sh_height_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "height_in", &MrsUavDjiTelloApi::callbackHeight, this);

  // | --------------------- service clients -------------------- |

  sch_arm_      = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "arm_out");
  sch_set_mode_ = mrs_lib::ServiceClientHandler<mavros_msgs::SetMode>(nh_, "set_mode_out");

  // | ----------------------- publishers ----------------------- |

  ph_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>(nh_, "cmd_out", 1);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavDjiTelloApi]: initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

mrs_msgs::HwApiStatus MrsUavDjiTelloApi::getStatus() {

  mrs_msgs::HwApiStatus status;

  status.stamp = ros::Time::now();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

mrs_msgs::HwApiCapabilities MrsUavDjiTelloApi::getCapabilities() {

  mrs_msgs::HwApiCapabilities capabilities;

  capabilities.api_name = "TelloApi";
  capabilities.stamp    = ros::Time::now();

  capabilities.accepts_velocity_hdg_rate_cmd = true;

  capabilities.produces_odometry        = true;
  capabilities.produces_battery_state   = true;
  capabilities.produces_orientation     = true;
  capabilities.produces_distance_sensor = true;

  return capabilities;
}

//}

/* callbackControlActuatorCmd() //{ */

bool MrsUavDjiTelloApi::callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting actuator cmd");

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUavDjiTelloApi::callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting control group cmd");

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool MrsUavDjiTelloApi::callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting attitude rate cmd");

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool MrsUavDjiTelloApi::callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting attitude cmd");

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool MrsUavDjiTelloApi::callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool MrsUavDjiTelloApi::callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool MrsUavDjiTelloApi::callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting velocity+hdg rate cmd");

  auto pose = sh_pose_.getMsg();

  Eigen::Vector3d vel_world(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(pose->pose.orientation);

  Eigen::Vector3d vel_body = R.transpose() * vel_world;

  mrs_msgs::HwApiVelocityHdgRateCmd cmd_out;

  cmd_out            = *msg;
  cmd_out.velocity.x = vel_body[0];
  cmd_out.velocity.y = vel_body[1];
  cmd_out.velocity.z = vel_body[2];

  ph_cmd_.publish(cmd_out);

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool MrsUavDjiTelloApi::callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting velocity+hdg cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool MrsUavDjiTelloApi::callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting position cmd");

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void MrsUavDjiTelloApi::callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg) {

  ROS_INFO_ONCE("[Api]: getting tracker cmd");
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUavDjiTelloApi::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  std_srvs::SetBool srv_out;

  srv_out.request.data = request;

  if (!request) {
    offboard_ = false;
  }

  ROS_INFO("[TelloApi]: calling for %s", request ? "arming" : "disarming");

  if (sch_arm_.call(srv_out)) {

    if (srv_out.response.success) {
      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      ROS_INFO_STREAM_THROTTLE(1.0, "[TelloApi]: " << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[TelloApi]: " << ss.str());
    }

  } else {
    ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.message << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TelloApi]: " << ss.str());
  }

  return {srv_out.response.success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUavDjiTelloApi::callbackOffboard(void) {

  std::stringstream ss;

  if (!armed_) {

    offboard_ = false;
    ss << "not armed";
    ROS_ERROR_THROTTLE(1.0, "[TelloApi]: %s", ss.str().c_str());
    return {false, ss.str()};

  } else {

    offboard_ = true;
    ss << "success";
    ROS_ERROR_THROTTLE(1.0, "[TelloApi]: %s", ss.str().c_str());
    return {true, ss.str()};
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackTelloStatus() */

void MrsUavDjiTelloApi::callbackArmed(const std_msgs::Bool::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting armed state");

  auto state = msg;

  {
    std::scoped_lock lock(mutex_status_);

    armed_     = state->data;
    connected_ = true;
  }

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = ros::Time::now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);
}

//}

/* calbackPose() //{ */

void MrsUavDjiTelloApi::callbackPose(const geometry_msgs::PoseStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting local odometry");

  // | --------------- publish the local odometry --------------- |

  mrs_lib::set_mutexed(mutex_pose_, *msg, pose_);
}

//}

/* calbackHeight() //{ */

void MrsUavDjiTelloApi::callbackHeight(const std_msgs::Float64::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting height");

  sensor_msgs::Range range_out;
  range_out.min_range = 0.1;
  range_out.max_range = 10.0;
  range_out.range     = msg->data;

  common_handlers_->publishers.publishDistanceSensor(range_out);
}

//}

/* calbackTwist() //{ */

void MrsUavDjiTelloApi::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting local odometry");

  if (!sh_pose_.hasMsg()) {
    return;
  }

  // | ----------------- integrate the velocity ----------------- |

  if (last_update_ == ros::Time(0)) {
    last_update_ = ros::Time::now();
    return;
  }

  double dt    = (ros::Time::now() - last_update_).toSec();
  last_update_ = ros::Time::now();

  auto pose  = sh_pose_.getMsg();
  auto twist = msg;

  Eigen::Vector3d vel_world(twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z);
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(pose->pose.orientation);

  /* Eigen::Vector3d vel_world = R * vel_body; */

  pos_ += vel_world * dt;

  // | --------------- publish the local odometry --------------- |

  mrs_lib::set_mutexed(mutex_twist_, *twist, twist_);

  publishOdom();
}

//}

/* callbackBattery() //{ */

void MrsUavDjiTelloApi::callbackBattery(const sensor_msgs::BatteryState::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavDjiTelloApi]: getting battery");

  common_handlers_->publishers.publishBatteryState(*msg);
}

//}

// | ------------------------ routines ------------------------ |

/* publishOdom() //{ */

void MrsUavDjiTelloApi::publishOdom(void) {

  // | ---------------- fill in the odom message ---------------- |

  auto twist = mrs_lib::get_mutexed(mutex_twist_, twist_);
  auto pose  = mrs_lib::get_mutexed(mutex_pose_, pose_);

  nav_msgs::Odometry odom;

  odom.header          = pose.header;
  odom.header.frame_id = common_handlers_->getWorldFrameName();
  /* odom.pose.pose.position.x = pos_[0]; */
  /* odom.pose.pose.position.y = pos_[1]; */
  /* odom.pose.pose.position.z = pos_[2]; */

  odom.pose.pose = pose.pose;

  odom.child_frame_id = common_handlers_->getUavName() + "/" + common_handlers_->getBodyFrameName();
  odom.twist.twist    = twist.twist;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  common_handlers_->publishers.publishOdometry(odom);

  // | ----------------- publish the orientation ---------------- |

  geometry_msgs::QuaternionStamped quat;

  quat.header     = pose.header;
  quat.quaternion = pose.pose.orientation;

  common_handlers_->publishers.publishOrientation(quat);

  // | ---------------- publish angular velocity ---------------- |

  /* geometry_msgs::Vector3Stamped angular_velocity; */

  /* angular_velocity.header.stamp    = pose.header.stamp; */
  /* angular_velocity.header.frame_id = common_handlers_->getUavName() + "/" + common_handlers_->getBodyFrameName(); */

  /* angular_velocity.vector.x = twist.twist.angular.x; */
  /* angular_velocity.vector.y = twist.twist.angular.y; */
  /* angular_velocity.vector.z = twist.twist.angular.z; */

  /* common_handlers_->publishers.publishAngularVelocity(angular_velocity); */

  // | ------------------ publish emulated gnss ----------------- |

  /* { */
  /*   double lat; */
  /*   double lon; */

  /*   mrs_lib::UTMtoLL(pose.pose.position.y + _utm_y_, pose.pose.position.x + _utm_x_, "32T", lat, lon); */

  /*   sensor_msgs::NavSatFix gnss; */

  /*   gnss.header.stamp = pose.header.stamp; */

  /*   gnss.latitude  = lat; */
  /*   gnss.longitude = lon; */
  /*   gnss.altitude  = pose.pose.position.z + _amsl_; */

  /*   common_handlers_->publishers.publishGNSS(gnss); */
  /* } */

  // | -------------------- publish altitude -------------------- |


  /* mrs_msgs::HwApiAltitude altitude; */

  /* altitude.stamp = pose.header.stamp; */

  /* altitude.amsl = pose.pose.position.z + _amsl_; */

  /* common_handlers_->publishers.publishAltitude(altitude); */

  // | --------------------- publish heading -------------------- |

  /* double heading = mrs_lib::AttitudeConverter(pose.pose.orientation).getHeading(); */

  /* mrs_msgs::Float64Stamped hdg; */

  /* hdg.header.stamp = ros::Time::now(); */
  /* hdg.value        = heading; */

  /* common_handlers_->publishers.publishMagnetometerHeading(hdg); */
}

//}

}  // namespace mrs_uav_dji_tello_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_dji_tello_api::MrsUavDjiTelloApi, mrs_uav_hw_api::MrsUavHwApi)
