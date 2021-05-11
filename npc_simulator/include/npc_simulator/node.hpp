// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NPC_SIMULATOR__NODE_HPP_
#define NPC_SIMULATOR__NODE_HPP_

#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "npc_simulator/msg/object.hpp"
#include "npc_simulator/srv/get_object.hpp"

#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "dummy_perception_publisher/msg/object.hpp"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_extension/utility/utilities.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


class NPCSimulator
{
private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<dummy_perception_publisher::msg::Object>::SharedPtr
    dummy_perception_object_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    debug_object_pub_;  // for visualization
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Subscription<npc_simulator::msg::Object>::SharedPtr object_sub_;
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<npc_simulator::srv::GetObject>::SharedPtr getobject_srv_;
  rclcpp::TimerBase::SharedPtr timer_main_;
  rclcpp::TimerBase::SharedPtr timer_pub_info_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<npc_simulator::msg::Object> objects_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  // simulation state
  bool engage_state_;

  // vehicle info
  geometry_msgs::msg::PoseStamped ego_pose_;
  double vehicle_width_;
  double vehicle_length_;
  double vehicle_base2center_;

  // lanelet
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  void mainTimerCallback();
  void pubInfoTimerCallback();
  void updateVelocity(npc_simulator::msg::Object * obj, double dt);
  void updateObjectPosition(
    npc_simulator::msg::Object * obj, const double move_distance,
    const geometry_msgs::msg::Quaternion diff_quat);
  double addCostByLaneTag(
    const int lane_follow_dir, const std::string & lanetag, const double base_cost = 0.2);
  double addCostByBesidesLane(const bool is_in_besides_lane, const double base_cost = 0.5);
  int getCurrentLaneletID(
    const npc_simulator::msg::Object & obj, const bool with_target_lane = true,
    const double max_dist = 20.0,
    const double max_delta_yaw = boost::math::constants::pi<double>() * 3.0 / 4.0);
  bool checkValidLaneChange(
    const int current_lane_id, const int lane_change_id, int & result_lane_id);
  bool checkValidLaneChange(
    const int current_lane_id, const std::string & lane_change_dir, int & result_lane_id);
  bool checkValidUTurn(
    const geometry_msgs::msg::Pose & obj_pose, const int current_lane_id, int & result_lane_id);
  bool checkToFinishLaneChange(const npc_simulator::msg::Object & obj, const int lane_id);
  int DecideLaneIdWithLaneChangeMode(npc_simulator::msg::Object * obj, const int current_lane_id);
  geometry_msgs::msg::Quaternion calcQuatForMove(
    npc_simulator::msg::Object & obj, const int current_lane_id, const double dt);
  double getRemainingLaneDistance(const geometry_msgs::msg::Pose pose, const int lane_id);
  double getCurrentLaneYaw(const geometry_msgs::msg::Pose & pose, const int lane_id);
  double getCurrentDiffYaw(const geometry_msgs::msg::Pose & pose, const double lane_yaw);
  double getFootOfPerpendicularLineLength(
    const double lx1, const double ly1, const double lx2, const double ly2,
    const geometry_msgs::msg::Pose & pose);
  double getCurrentLaneDist(
    const geometry_msgs::msg::Pose & pose, const double offset_rate_from_center, const int lane_id);
  double calcMaxYawRate(const npc_simulator::msg::Object & obj);
  double calcMaxSpeed(const npc_simulator::msg::Object & obj, int obj_lane_id);
  double getFollowLaneDiffYaw(
    const double diff_yaw, const double signed_lane_dist, const double current_vel, const double dt,
    const double max_yaw_rate = boost::math::constants::pi<double>() * 1.0);
  double getNearestZPos(const geometry_msgs::msg::Pose & pose);
  double calcSmoothZPos(
    geometry_msgs::msg::Point current_point, geometry_msgs::msg::Point p1,
    geometry_msgs::msg::Point p2);
  bool calcCollisionDistance(const npc_simulator::msg::Object & obj, double * col_dist);
  geometry_msgs::msg::Pose getRelativePose(
    const geometry_msgs::msg::Pose & source, const geometry_msgs::msg::Pose & target);
  double calcSpeedToAvoidCollision(const double col_dist);
  void inputImuInfo(
    npc_simulator::msg::Object * obj, const double prev_vel, const double prev_yaw,
    const double delta_time);
  void inputVelocityZ(
    npc_simulator::msg::Object * obj, const double prev_z_pos, const double delta_time);

  void engageCallback(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr engage);
  void objectCallback(const npc_simulator::msg::Object::ConstSharedPtr msg);
  void mapCallback(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  dummy_perception_publisher::msg::Object convertObjectMsgToDummyPerception(
    npc_simulator::msg::Object * obj);
  autoware_perception_msgs::msg::DynamicObjectArray convertObjectMsgToAutowarePerception(
    const std::vector<npc_simulator::msg::Object> & obj_vec, const bool prediction);

  // parameter
  const double p_coef_diff_dist_ = 0.1;
  const double p_coef_diff_dist_vel_ = 0.01;
  const double max_yaw_diff_dist_ =
    boost::math::constants::pi<double>() / 8.0;  // max value of additional yaw by distance to lane
  const double base_cost_by_lane_tag_ = 3.5;

  const double max_yaw_rate_coef_ =
    boost::math::constants::pi<double>() / 15.0;      // coef * current_velocity = max_yaw
  const double max_yaw_rate_uturn_ = boost::math::constants::pi<double>() / 6.0;

  const double max_speed_ = 100.0;             // [m/s]
  const double max_speed_z_ = 0.5;             // [m/s]
  const double max_speed_lane_change_ = 10.0;  // [m/s]
  const double max_speed_uturn_ = 2.0;         // [m/s]
  const double max_speed_curve_ = 5.0;         // [m/s]

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

  /**
   * \brief Initialize a timer and register it with this class
   *
   * @param duration Time to wait before timer is triggered
   * @param ptr_to_member_fn The timer callback, required to be a pointer to a member function
   * of NPCSimulator
   *
   * @return the timer
   */
  rclcpp::TimerBase::SharedPtr initTimer(
    rclcpp::Node & node, const rclcpp::Duration & duration, void (NPCSimulator::* ptr_to_member_fn)(
      void));

public:
  explicit NPCSimulator(rclcpp::Node & node);
  ~NPCSimulator()
  {
  }

  bool getObject(
    const npc_simulator::srv::GetObject::Request::SharedPtr req,
    const npc_simulator::srv::GetObject::Response::SharedPtr res);
};

#endif  // NPC_SIMULATOR__NODE_HPP_
