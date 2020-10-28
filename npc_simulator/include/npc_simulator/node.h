/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <dummy_perception_publisher/Object.h>
#include <npc_simulator/GetObject.h>
#include <npc_simulator/Object.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <memory>
#include <random>
#include <tuple>

// lanelet
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

class NPCSimulatorNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher dummy_perception_object_pub_;
  ros::Publisher debug_object_pub_;  // for visualization
  ros::Subscriber engage_sub_;
  ros::Subscriber object_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer getobject_srv_;
  ros::Timer timer_main_;
  ros::Timer timer_pub_info_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<npc_simulator::Object> objects_;

  //simulation state
  bool engage_state_;

  //vehicle info
  geometry_msgs::PoseStamped ego_pose_;
  double vehicle_width_;
  double vehicle_length_;
  double vehicle_rear_overhang_;
  double vehicle_base2center_;

  // lanelet
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  void mainTimerCallback(const ros::TimerEvent &);
  void pubInfoTimerCallback(const ros::TimerEvent &);
  void updateVelocity(npc_simulator::Object * obj, double dt);
  void updateObjectPosition(
    npc_simulator::Object * obj, const double move_distance,
    const geometry_msgs::Quaternion diff_quat);
  double addCostByLaneTag(
    const int lane_follow_dir, const std::string & lanetag, const double base_cost = 0.2);
  double addCostByBesidesLane(const bool is_in_besides_lane, const double base_cost = 0.5);
  int getCurrentLaneletID(
    const npc_simulator::Object & obj, const bool with_target_lane = true,
    const double max_dist = 20.0,
    const double max_delta_yaw = boost::math::constants::pi<double>() * 3.0 / 4.0);
  bool checkValidLaneChange(
    const int current_lane_id, const int lane_change_id, int & result_lane_id);
  bool checkValidLaneChange(
    const int current_lane_id, const std::string & lane_change_dir, int & result_lane_id);
  bool checkValidUTurn(
    const geometry_msgs::Pose & obj_pose, const int current_lane_id, int & result_lane_id);
  bool checkToFinishLaneChange(const npc_simulator::Object & obj, const int lane_id);
  int DecideLaneIdWithLaneChangeMode(npc_simulator::Object * obj, const int current_lane_id);
  geometry_msgs::Quaternion calcQuatForMove(
    npc_simulator::Object & obj, const int current_lane_id, const double dt);
  double getRemainingLaneDistance(const geometry_msgs::Pose pose, const int lane_id);
  double getCurrentLaneYaw(const geometry_msgs::Pose & pose, const int lane_id);
  double getCurrentDiffYaw(const geometry_msgs::Pose & pose, const double lane_yaw);
  double getFootOfPerpendicularLineLength(
    const double lx1, const double ly1, const double lx2, const double ly2,
    const geometry_msgs::Pose & pose);
  double getCurrentLaneDist(
    const geometry_msgs::Pose & pose, const double offset_rate_from_center, const int lane_id);
  double calcMaxYawRate(const npc_simulator::Object & obj);
  double calcMaxSpeed(const npc_simulator::Object & obj, int obj_lane_id);
  double getFollowLaneDiffYaw(
    const double diff_yaw, const double signed_lane_dist, const double current_vel, const double dt,
    const double max_yaw_rate = boost::math::constants::pi<double>() * 1.0);
  double getNearestZPos(const geometry_msgs::Pose & pose);
  double calcSmoothZPos(
    geometry_msgs::Point current_point, geometry_msgs::Point p1, geometry_msgs::Point p2);
  bool calcCollisionDistance(const npc_simulator::Object & obj, double * col_dist);
  geometry_msgs::Pose getRelativePose(
    const geometry_msgs::Pose & source, const geometry_msgs::Pose & target);
  double calcSpeedToAvoidCollision(const double col_dist);
  void inputImuInfo(
    npc_simulator::Object * obj, const double prev_vel, const double prev_yaw,
    const double delta_time);
  void inputVelocityZ(
    npc_simulator::Object * obj, const double prev_z_pos, const double delta_time);

  bool getObject(npc_simulator::GetObject::Request & req, npc_simulator::GetObject::Response & res);
  void engageCallback(const std_msgs::Bool::ConstPtr & engage);
  void objectCallback(const npc_simulator::Object::ConstPtr & msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin & msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
  dummy_perception_publisher::Object convertObjectMsgToDummyPerception(npc_simulator::Object * obj);
  autoware_perception_msgs::DynamicObjectArray convertObjectMsgToAutowarePerception(
    const std::vector<npc_simulator::Object> & obj_vec, const bool prediction);

  // parameter
  const double p_coef_diff_dist_ = 0.1;
  const double p_coef_diff_dist_vel_ = 0.01;
  const double max_yaw_diff_dist_ =
    boost::math::constants::pi<double>() / 8.0;  // max value of additional yaw by distance to lane
  const double base_cost_by_lane_tag_ = 3.5;

  const double max_yaw_rate_coef_ =
    boost::math::constants::pi<double>() / 15.0;  // coef * current_velocity = max_yaw
  const double max_yaw_rate_uturn_ = boost::math::constants::pi<double>() / 6.0;

  const double max_speed_ = 100.0;             //[m/s]
  const double max_speed_z_ = 0.5;             //[m/s]
  const double max_speed_lane_change_ = 10.0;  //[m/s]
  const double max_speed_uturn_ = 2.0;         //[m/s]
  const double max_speed_curve_ = 5.0;         //[m/s]

  const double thr_dist_lane_change_ = 1.0;
  const double thr_yaw_lane_change_ = boost::math::constants::pi<double>() / 10.0;

  const double future_consideration_time_ = 0.05;
  const double max_consideration_dist_ = 0.5;

  /* npc stop by vehicle*/
  const double margin_dist_to_avoid_collision_ = 1.0;
  const double margin_time_to_avoid_collision_ = 2.2;
  const double accel_to_avoid_collision_ = 5.0;
  const double max_stop_distance_thresh_ = 100.0;
  const double collision_width_margin_ = 1.0;

  /* search nearest lane*/
  const double max_dist_without_target_ = 10.0;
  const double max_delta_yaw_without_target_ = boost::math::constants::pi<double>() / 3.0;
  const double max_dist_uturn_ = 10.0;
  const double max_delta_yaw_uturn_ = boost::math::constants::pi<double>() / 3.0;

public:
  NPCSimulatorNode();
  ~NPCSimulatorNode(){};
};

constexpr double normalizeRadian(
  const double rad, const double min_rad = -boost::math::constants::pi<double>(),
  const double max_rad = boost::math::constants::pi<double>())
{
  const auto value = std::fmod(rad, 2 * boost::math::constants::pi<double>());
  if (min_rad < value && value <= max_rad)
    return value;
  else
    return value - std::copysign(2 * boost::math::constants::pi<double>(), value);
}

inline geometry_msgs::Quaternion getQuatFromYaw(const double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quat);
}

inline geometry_msgs::Point toMsg(const lanelet::ConstPoint3d & ll_point)
{
  geometry_msgs::Point point;
  point.x = ll_point.x();
  point.y = ll_point.y();
  point.z = ll_point.z();
  return point;
}

inline double calcDist2D(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::hypot(dx, dy);
}
