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
#include "npc_simulator/node.h"

#include <lanelet2_extension/utility/message_conversion.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::linestring<Point> Line;
typedef bg::model::polygon<Point> Polygon;

NPCSimulatorNode::NPCSimulatorNode()
: rclcpp::Node("npc_simulator"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  vehicle_info_(vehicle_info_util::VehicleInfo::create(*this))
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // get parameter
  engage_state_ = declare_parameter<bool>("initial_engage_state", true);

  // get vehicle parameter
  double wheel_tread{}, left_overhang{}, right_overhang{};
  assert(get_parameter("wheel_tread", wheel_tread));
  assert(get_parameter("left_overhang", left_overhang));
  assert(get_parameter("right_overhang", right_overhang));
  vehicle_width_ = wheel_tread + left_overhang + right_overhang;

  double wheel_base{}, front_overhang{}, rear_overhang{};
  assert(get_parameter("wheel_base", wheel_base));
  assert(get_parameter("front_overhang", front_overhang));
  assert(get_parameter("rear_overhang", rear_overhang));
  vehicle_length_ = wheel_base + front_overhang + rear_overhang;

  vehicle_base2center_ = vehicle_length_ / 2.0 - rear_overhang;

  static constexpr std::size_t single_element_in_queue = 1;
  static constexpr std::size_t small_queue_size = 10;
  static constexpr std::size_t large_queue_size = 100;
  rclcpp::QoS durable_qos(small_queue_size);
  durable_qos.transient_local();

  dummy_perception_object_pub_ = create_publisher<dummy_perception_publisher::msg::Object>(
    "output/dynamic_object_info", durable_qos);
  debug_object_pub_ = create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(
    "output/debug_object_info", durable_qos);

  // register callback
  engage_sub_ = create_subscription<std_msgs::msg::Bool>(
    "input/engage", large_queue_size, std::bind(&NPCSimulatorNode::engageCallback, this, _1));
  object_sub_ = create_subscription<npc_simulator::msg::Object>(
    "input/object", large_queue_size, std::bind(&NPCSimulatorNode::objectCallback, this, _1));
  map_sub_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/vector_map", single_element_in_queue,
    std::bind(&NPCSimulatorNode::mapCallback, this, _1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/ego_vehicle_pose", single_element_in_queue,
    std::bind(&NPCSimulatorNode::poseCallback, this, _1));
  getobject_srv_ = create_service<npc_simulator::srv::GetObject>(
    "get_object", std::bind(&NPCSimulatorNode::getObject, this, _1, _2));

  timer_main_ = initTimer(rclcpp::Duration(0.1), &NPCSimulatorNode::mainTimerCallback);
  timer_pub_info_ = initTimer(rclcpp::Duration(0.02), &NPCSimulatorNode::pubInfoTimerCallback);
}

bool NPCSimulatorNode::getObject(
  const npc_simulator::srv::GetObject::Request::SharedPtr req,
  const npc_simulator::srv::GetObject::Response::SharedPtr res)
{
  unique_identifier_msgs::msg::UUID id = req->object_id;
  for (const auto & obj : objects_) {
    if (boost::equal(obj.id.uuid, id.uuid)) {
      res->object = obj;
      res->success = true;
      return true;
    }
  }
  // no corresponding object
  res->success = false;
  return false;
}

void NPCSimulatorNode::mainTimerCallback()
{
  rclcpp::Time current_time = this->now();

  // get transform
  tf2::Transform tf_base_link2map;
  try {
    geometry_msgs::msg::TransformStamped ros_base_link2map;
    ros_base_link2map = tf_buffer_.lookupTransform(
      /*target*/ "base_link", /*src*/ "map", current_time, rclcpp::Duration(0.5));
    tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  // update npc information
  for (auto & obj : objects_) {
    if (engage_state_) {
      //memory original v, yaw for calculate Imu info(acceleration and yaw rate)
      const double prev_velocity = obj.initial_state.twist_covariance.twist.linear.x;
      const double prev_z_pos = obj.initial_state.pose_covariance.pose.position.z;
      const double prev_yaw = tf2::getYaw(obj.initial_state.pose_covariance.pose.orientation);

      // calc distance
      const double delta_time = (current_time.seconds() - rclcpp::Time(obj.header.stamp).seconds());
      const double move_distance = obj.initial_state.twist_covariance.twist.linear.x * delta_time;

      int current_lane_id = -1;
      if (lanelet_map_ptr_) {
        current_lane_id = getCurrentLaneletID(obj);
      }

      // update velocity x
      updateVelocity(&obj, delta_time);

      //calculate future position
      npc_simulator::msg::Object * future_obj = new npc_simulator::msg::Object(obj);  //deep copy
      double predicted_distance =
        future_obj->initial_state.twist_covariance.twist.linear.x * future_consideration_time_;
      predicted_distance = std::min(predicted_distance, max_consideration_dist_);
      updateObjectPosition(future_obj, predicted_distance, getQuatFromYaw(0));

      //calculate posture to follow lane (by predicted position)
      geometry_msgs::msg::Quaternion quat =
        calcQuatForMove(*future_obj, current_lane_id, delta_time);

      //input lane change info
      obj.lane_change_id = future_obj->lane_change_id;
      obj.lane_change_dir.dir = future_obj->lane_change_dir.dir;

      // update object position, velocity, imu
      updateObjectPosition(&obj, move_distance, quat);
      inputImuInfo(&obj, prev_velocity, prev_yaw, delta_time);
      inputVelocityZ(&obj, prev_z_pos, delta_time);  //for visualization
    }

    // update object time
    obj.header.stamp = current_time;

    // publish
    const auto dummy_perception_obj_msg = convertObjectMsgToDummyPerception(&obj);
    dummy_perception_object_pub_->publish(dummy_perception_obj_msg);
  }
}

void NPCSimulatorNode::pubInfoTimerCallback()
{
  //publish npc info for visulaization
  const auto autoware_perception_msg = convertObjectMsgToAutowarePerception(objects_, true);
  debug_object_pub_->publish(autoware_perception_msg);
}

void NPCSimulatorNode::updateObjectPosition(
  npc_simulator::msg::Object * obj, const double move_distance,
  const geometry_msgs::msg::Quaternion diff_quat)
{
  //update object position 2D by move_distance
  tf2::Transform tf_object_origin2moved_object;
  tf2::Transform tf_map2object_origin;
  tf2::Transform tf_map2moved_object;
  geometry_msgs::msg::PoseStamped output_moved_object_pose;
  geometry_msgs::msg::Transform ros_object_origin2moved_object;
  ros_object_origin2moved_object.translation.x = move_distance;
  ros_object_origin2moved_object.rotation = diff_quat;
  tf2::fromMsg(ros_object_origin2moved_object, tf_object_origin2moved_object);
  tf2::fromMsg(obj->initial_state.pose_covariance.pose, tf_map2object_origin);
  tf_map2moved_object = tf_map2object_origin * tf_object_origin2moved_object;
  tf2::toMsg(tf_map2moved_object, output_moved_object_pose.pose);
  obj->initial_state.pose_covariance.pose = output_moved_object_pose.pose;

  //update object position z
  obj->initial_state.pose_covariance.pose.position.z =
    getNearestZPos(obj->initial_state.pose_covariance.pose) + obj->shape.dimensions.z / 2.0;
}

void NPCSimulatorNode::engageCallback(const std_msgs::msg::Bool::ConstSharedPtr engage)
{
  engage_state_ = engage->data;
}

void NPCSimulatorNode::inputImuInfo(
  npc_simulator::msg::Object * obj, const double prev_vel, const double prev_yaw,
  const double delta_time)
{
  const double current_vel = obj->initial_state.twist_covariance.twist.linear.x;
  const double current_yaw = tf2::getYaw(obj->initial_state.pose_covariance.pose.orientation);
  const double current_accelration = (current_vel - prev_vel) / delta_time;
  const double current_yaw_rate = (current_yaw - prev_yaw) / delta_time;
  obj->imu.linear_acceleration.x = current_accelration;
  obj->imu.angular_velocity.z = current_yaw_rate;
}

void NPCSimulatorNode::inputVelocityZ(
  npc_simulator::msg::Object * obj, const double prev_z_pos, const double delta_time)
{
  // calculate velocity_z by current z-pos and previous z-pos
  const double current_z_pos = obj->initial_state.pose_covariance.pose.position.z;
  double velocity_z = (current_z_pos - prev_z_pos) / delta_time;
  velocity_z = boost::algorithm::clamp(velocity_z, -max_speed_z_, max_speed_z_);
  obj->initial_state.twist_covariance.twist.linear.z = velocity_z;
}

bool NPCSimulatorNode::checkValidLaneChange(
  int current_lane_id, const int lane_change_id, int & result_lane_id)
{
  auto current_lane = lanelet_map_ptr_->laneletLayer.get(current_lane_id);
  auto beside_lanes = routing_graph_ptr_->besides(current_lane);
  if (current_lane.id() == lane_change_id) {
    // use lane change id
    result_lane_id = lane_change_id;
    return true;
  }
  for (auto beside_lane : beside_lanes) {
    if (beside_lane.id() == lane_change_id) {
      // use lane change id
      result_lane_id = lane_change_id;
      return true;
    }
  }
  // wait lane change or finish lane change
  return false;
}

bool NPCSimulatorNode::checkValidLaneChange(
  const int current_lane_id, const std::string & lane_change_dir, int & result_lane_id)
{
  auto current_lane = lanelet_map_ptr_->laneletLayer.get(current_lane_id);
  lanelet::Optional<lanelet::ConstLanelet> change_lane;
  if (lane_change_dir == std::string("right")) {
    change_lane = routing_graph_ptr_->right(current_lane);
  } else if (lane_change_dir == std::string("left")) {
    change_lane = routing_graph_ptr_->left(current_lane);
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "lane change direction is only left or right");
    return false;
  }

  if (!change_lane) {
    RCLCPP_WARN_STREAM(
      get_logger(), "no right lane, current lane id=" << current_lane_id
                                                      << ",lane change num= " << lane_change_dir);
    return false;
  }

  RCLCPP_INFO_STREAM(
    get_logger(), "start lane change , current_lane_id =" << current_lane_id << ", target lane_id="
                                                          << change_lane->id());
  result_lane_id = change_lane->id();
  return true;
}

bool NPCSimulatorNode::checkValidUTurn(
  const geometry_msgs::msg::Pose & obj_pose, const int current_lane_id, int & result_lane_id)
{
  // TODO: Is there API to get opposite adjacent lane???
  // get opposite pose
  geometry_msgs::msg::Pose * turn_pose = new geometry_msgs::msg::Pose(obj_pose);
  tf2::Quaternion quat;
  tf2::fromMsg(turn_pose->orientation, quat);
  tf2::Quaternion rotate_yaw_pi(0.0, 0.0, 1.0, 0.0);
  tf2::Quaternion rotated_quat = quat * rotate_yaw_pi;
  turn_pose->orientation = tf2::toMsg(rotated_quat);

  // get opposite lane
  npc_simulator::msg::Object obj;
  obj.initial_state.pose_covariance.pose = *turn_pose;
  obj.lane_follow_mode.mode = npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT;
  int ops_lane_id = getCurrentLaneletID(obj, false, max_dist_uturn_, max_delta_yaw_uturn_);

  if (ops_lane_id < 0) {
    RCLCPP_WARN_STREAM(
      get_logger(), "no opposite lane for u-turn, current lane id=" << current_lane_id);
    return false;
  }

  RCLCPP_INFO_STREAM(
    get_logger(),
    "start u-turn, current_lane_id =" << current_lane_id << ", lane_id=" << ops_lane_id);
  result_lane_id = ops_lane_id;
  return true;
}

bool NPCSimulatorNode::checkToFinishLaneChange(
  const npc_simulator::msg::Object & obj, const int lane_id)
{
  const double current_dist = getCurrentLaneDist(
    obj.initial_state.pose_covariance.pose, obj.offset_rate_from_center, lane_id);
  const double current_lane_yaw =
    getCurrentLaneYaw(obj.initial_state.pose_covariance.pose, lane_id);
  const double diff_yaw =
    getCurrentDiffYaw(obj.initial_state.pose_covariance.pose, current_lane_yaw);
  if (
    std::fabs(current_dist) < thr_dist_lane_change_ && std::fabs(diff_yaw) < thr_yaw_lane_change_) {
    return true;
  }
  return false;
}

int NPCSimulatorNode::DecideLaneIdWithLaneChangeMode(
  npc_simulator::msg::Object * obj, const int current_lane_id)
{
  // decide current lane

  int lane_id;
  if (obj->lane_change_id != 0) {  //lane change
    //validation check of lane change target
    if (obj->lane_change_dir.dir != npc_simulator::msg::LaneChangeDir::LANE_CHANGE_UTURN) {
      //In U-turn, no valid check
      if (!checkValidLaneChange(current_lane_id, obj->lane_change_id, lane_id)) {
        //wait lane change until to be possible
        return current_lane_id;
      }
    }
    lane_id = obj->lane_change_id;

  } else if (obj->lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::NO_LANE_CHANGE) {
    // use id of current(nearest) lane
    return current_lane_id;
  } else if (obj->lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::LEFT_LANE_CHANGE) {
    if (!checkValidLaneChange(current_lane_id, "left", lane_id)) {
      //wait lane change until to be possible
      return current_lane_id;
    }

  } else if (obj->lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::RIGHT_LANE_CHANGE) {
    if (!checkValidLaneChange(current_lane_id, "right", lane_id)) {
      //wait lane change until to be possible
      return current_lane_id;
    }
  } else if (obj->lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::LANE_CHANGE_UTURN) {
    if (!checkValidUTurn(obj->initial_state.pose_covariance.pose, current_lane_id, lane_id)) {
      //wait u-turn change until to be possible
      return current_lane_id;
    }
  }

  // check existance of lane with target id
  if (!lanelet_map_ptr_->laneletLayer.exists(lane_id)) {
    RCLCPP_WARN_STREAM(get_logger(), "target lane:" << current_lane_id << "does not exist.");
    //return nearest lane
    return getCurrentLaneletID(*obj);
  }

  // check to finish lane-change
  if (checkToFinishLaneChange(*obj, lane_id)) {
    //end lane change
    RCLCPP_INFO_STREAM(get_logger(), "lane change/u-turn end, lane id=" << lane_id);
    obj->lane_change_id = 0;
    obj->lane_change_dir.dir = npc_simulator::msg::LaneChangeDir::NO_LANE_CHANGE;
  } else {
    //input target lane id
    obj->lane_change_id = lane_id;
  }

  return lane_id;
}

geometry_msgs::msg::Quaternion NPCSimulatorNode::calcQuatForMove(
  npc_simulator::msg::Object & obj, int current_lane_id, double dt)
{
  geometry_msgs::msg::Quaternion q;

  if (obj.semantic.type == autoware_perception_msgs::msg::Semantic::PEDESTRIAN) {
    // go straight(pedestrian)
    q.w = 1.0;
    return q;
  } else if (std::abs(obj.initial_state.twist_covariance.twist.linear.x) < 10e-02) {
    // do not move
    q.w = 1.0;
    return q;
  } else if (current_lane_id < 0 && obj.lane_change_id == 0) {
    // go straight(cannot find current lane)
    q.w = 1.0;
    return q;
  }

  // calculate quaternion for moving

  if (obj.lane_follow_mode.mode == npc_simulator::msg::LaneFollowMode::MOVE_STRAIGHT) {
    // does not lane follow
    q.x = q.y = q.z = 0;
    q.w = 1;
    return q;
  }

  int target_lane_id = DecideLaneIdWithLaneChangeMode(&obj, current_lane_id);
  double current_lane_yaw =
    getCurrentLaneYaw(obj.initial_state.pose_covariance.pose, target_lane_id);
  double diff_yaw = getCurrentDiffYaw(obj.initial_state.pose_covariance.pose, current_lane_yaw);
  double current_lane_dist = getCurrentLaneDist(
    obj.initial_state.pose_covariance.pose, obj.offset_rate_from_center, target_lane_id);
  double max_yaw_rate = calcMaxYawRate(obj);
  double diff_yaw_for_follow = getFollowLaneDiffYaw(
    diff_yaw, current_lane_dist, obj.initial_state.twist_covariance.twist.linear.x, dt,
    max_yaw_rate);
  q = getQuatFromYaw(diff_yaw_for_follow);
  return q;
}

void NPCSimulatorNode::updateVelocity(npc_simulator::msg::Object * obj, double dt)
{
  const double current_vel = obj->initial_state.twist_covariance.twist.linear.x;
  const double current_vel_sign = boost::math::sign(current_vel);
  double current_vel_abs = std::fabs(current_vel);

  //avoid collision/emergency stop (only object has plus velocity)
  double col_dist;
  double speed_avoid_col = max_speed_;
  if (calcCollisionDistance(*obj, &col_dist)) {
    speed_avoid_col = calcSpeedToAvoidCollision(col_dist);
  }
  if (speed_avoid_col < current_vel_abs) {
    double dif_vel = speed_avoid_col - current_vel_abs;
    if (std::fabs(dif_vel) > std::fabs(accel_to_avoid_collision_ * dt)) {
      // clipping by accel
      dif_vel = std::copysign(accel_to_avoid_collision_ * dt, dif_vel);
    }
    current_vel_abs = current_vel_abs + dif_vel;
    obj->initial_state.twist_covariance.twist.linear.x = current_vel_abs * current_vel_sign;
    //do not update velocity naturally
    return;
  }

  //update velocity naturally
  if (obj->accel < 1e-03) return;

  if (std::fabs(obj->target_vel) > speed_avoid_col) return;

  double dif_vel = obj->target_vel - current_vel;
  if (std::fabs(dif_vel) < 1e-03) return;
  if (std::fabs(dif_vel) > std::fabs(obj->accel * dt)) {
    // clipping by accel
    dif_vel = dif_vel * std::fabs(obj->accel * dt / dif_vel);
  }
  double update_vel = current_vel + dif_vel;

  obj->initial_state.twist_covariance.twist.linear.x = update_vel;
}

double NPCSimulatorNode::addCostByLaneTag(
  const int lane_follow_dir, const std::string lanetag, const double base_cost)
{
  double cost = 0;
  // introduce cost
  switch (lane_follow_dir) {
    case npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT: {
      if (lanetag == "left") cost = base_cost;
      if (lanetag == "right") cost = base_cost;
      break;
    }
    case npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_LEFT: {
      if (lanetag == "straight" || lanetag == "else") cost = base_cost;
      if (lanetag == "right") cost = base_cost * 2;
      break;
    }
    case npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_RIGHT: {
      if (lanetag == "straight" || lanetag == "else") cost += base_cost;
      if (lanetag == "left") cost += base_cost * 2;
      break;
    }
  }

  return cost;
}

int NPCSimulatorNode::getCurrentLaneletID(
  const npc_simulator::msg::Object & obj, const bool with_target_lane, const double max_dist,
  const double max_delta_yaw)
{
  const auto obj_pose = obj.initial_state.pose_covariance.pose;
  const auto obj_route = obj.target_route;
  const int lane_follow_dir = obj.lane_follow_mode.mode;

  if (obj.target_route.data.size() == 0 && with_target_lane) {
    //no target route
    //search current lanelet in entire lanelet
    return getCurrentLaneletID(obj, false, max_dist_without_target_, max_delta_yaw_without_target_);
  }

  lanelet::BasicPoint2d search_point(obj_pose.position.x, obj_pose.position.y);
  const auto nearest_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr_->laneletLayer, search_point, 20);  // distance, lanelet
  lanelet::Lanelet target_closest_lanelet;
  bool is_found_target_closest_lanelet = false;
  double min_dist = max_dist;
  for (const auto & lanelet : nearest_lanelets) {
    //check lenalet is involved in routes or not
    bool is_lane_in_route = false;
    for (const auto & target_lane_id : obj_route.data) {
      if (lanelet.second.id() == target_lane_id) {
        is_lane_in_route = true;
      }
    }

    if (!is_lane_in_route && with_target_lane) {
      //when "with_target_lane" option is false, search current lanelet in entire lane.
      continue;
    }

    double current_yaw = tf2::getYaw(obj_pose.orientation);
    double lane_yaw = lanelet::utils::getLaneletAngle(lanelet.second, obj_pose.position);
    double delta_yaw = std::abs(normalizeRadian(current_yaw - lane_yaw));
    auto lanetag = lanelet.second.attributeOr("turn_direction", "else");
    double current_dist =
      lanelet.first + addCostByLaneTag(lane_follow_dir, lanetag, base_cost_by_lane_tag_);

    if (current_dist < max_dist && delta_yaw < max_delta_yaw and current_dist < min_dist) {
      min_dist = current_dist;
      target_closest_lanelet = lanelet.second;
      is_found_target_closest_lanelet = true;
    }
  }

  if (is_found_target_closest_lanelet) {
    return (int)target_closest_lanelet.id();
  } else {
    if (with_target_lane) {
      //nearest lane is not found in target route.
      //search current lanelet in entire lanelet
      return getCurrentLaneletID(
        obj, false, max_dist_without_target_, max_delta_yaw_without_target_);
    } else {
      return -1;
    }
  }
}

double NPCSimulatorNode::getRemainingLaneDistance(const geometry_msgs::msg::Pose pose, int lane_id)
{
  lanelet::Lanelet current_lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
  const auto & centerline2d = lanelet::utils::to2D(current_lanelet.centerline());
  lanelet::BasicPoint2d point2d(pose.position.x, pose.position.y);
  lanelet::ArcCoordinates arc_coordinates =
    lanelet::geometry::toArcCoordinates(centerline2d, point2d);
  double remain_distance =
    lanelet::utils::getLaneletLength2d(current_lanelet) - arc_coordinates.length;
  return remain_distance;
}

double NPCSimulatorNode::getCurrentLaneYaw(const geometry_msgs::msg::Pose & pose, const int lane_id)
{
  lanelet::Lanelet current_lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
  double lane_yaw = lanelet::utils::getLaneletAngle(current_lanelet, pose.position);
  return lane_yaw;
}

double NPCSimulatorNode::getCurrentDiffYaw(
  const geometry_msgs::msg::Pose & pose, const double lane_yaw)
{
  double current_yaw = tf2::getYaw(pose.orientation);
  return normalizeRadian(lane_yaw - current_yaw);
}

double NPCSimulatorNode::getFootOfPerpendicularLineLength(
  const double lx1, const double ly1, const double lx2, const double ly2,
  const geometry_msgs::msg::Pose & pose)
{
  //calc line((lx1,ly1), (lx2,ly2))
  //ax + by + c = 0
  const double a = ly2 - ly1;
  const double b = lx1 - lx2;
  const double c = lx2 * ly1 - lx1 * ly2;

  const double p_x = pose.position.x;
  const double p_y = pose.position.y;

  //calc length of foot of perperndicular line
  double pl_length = std::fabs(a * p_x + b * p_y + c) / std::sqrt(a * a + b * b);
  return pl_length;
}

double NPCSimulatorNode::getCurrentLaneDist(
  const geometry_msgs::msg::Pose & pose, const double offset_rate_from_center, const int lane_id)
{
  lanelet::Lanelet current_lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
  const auto centerline = current_lanelet.centerline2d();
  if (centerline.empty()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *this->get_clock(), 5.0,
      "cannot get distance from centerline (invalid centerline)");
    return 0.0;
  }

  double dist_from_center_line = 1e5;
  double pl_dist_from_center_line = 0;
  Line nearest_line;
  bool exist_nearest = false;
  for (size_t i = 0; i < centerline.size() - 1; i++) {
    const Line laneline = {{centerline[i].x(), centerline[i].y()},
                           {centerline[i + 1].x(), centerline[i + 1].y()}};
    const Point point2d(pose.position.x, pose.position.y);
    double dist = bg::distance(laneline, point2d);
    if (dist <= dist_from_center_line) {
      dist_from_center_line = dist;
      pl_dist_from_center_line = getFootOfPerpendicularLineLength(
        centerline[i].x(), centerline[i].y(), centerline[i + 1].x(), centerline[i + 1].y(), pose);
      nearest_line = laneline;
      exist_nearest = true;
    }
  }

  if (!exist_nearest) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *this->get_clock(), 5.0,
      "cannot get distance from centerline (no nearest line)");
    return 0.0;
  }

  //calculate distance from center line
  double nearest_x = (nearest_line.at(0).x() + nearest_line.at(1).x()) / 2.0;
  double nearest_y = (nearest_line.at(0).y() + nearest_line.at(1).y()) / 2.0;
  double diff_x = nearest_x - pose.position.x;
  double diff_y = nearest_y - pose.position.y;
  double current_yaw = tf2::getYaw(pose.orientation);
  double nearest_yaw = atan2(diff_y, diff_x);
  double diff_yaw = normalizeRadian(nearest_yaw - current_yaw);
  if (std::abs(diff_yaw) < 10e-04) return 0.0;
  double base_dist = pl_dist_from_center_line * (diff_yaw / std::abs(diff_yaw));

  //calculate offset distance
  const auto rightline = current_lanelet.rightBound2d();
  double path_width = 1e5;  //half width
  for (size_t i = 0; i < rightline.size() - 1; i++) {
    const Line edgeline = {{rightline[i].x(), rightline[i].y()},
                           {rightline[i + 1].x(), rightline[i + 1].y()}};
    double dist_to_edge = bg::distance(edgeline, nearest_line);
    if (path_width > dist_to_edge) {
      path_width = dist_to_edge;
    }
  }
  double offset_dist = path_width * offset_rate_from_center;

  return base_dist + offset_dist;
}

double NPCSimulatorNode::calcMaxYawRate(const npc_simulator::msg::Object & obj)
{
  if (obj.lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::LANE_CHANGE_UTURN) {
    // when to do u-turn, restrict max_yaw_rate
    return max_yaw_rate_uturn_;
  }
  return max_yaw_rate_coef_ * std::fabs(obj.initial_state.twist_covariance.twist.linear.x);
}

double NPCSimulatorNode::calcMaxSpeed(const npc_simulator::msg::Object & obj, const int obj_lane_id)
{
  if (obj.lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::LANE_CHANGE_UTURN) {
    // when to do u-turn, restrict max_velocity
    return max_speed_uturn_;
  } else if (
    obj.lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::LEFT_LANE_CHANGE ||
    obj.lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::RIGHT_LANE_CHANGE) {
    // when to change lane, restrict max_velocity
    return max_speed_lane_change_;
  }

  if (lanelet_map_ptr_->laneletLayer.exists(obj_lane_id)) {
    auto lanelet = lanelet_map_ptr_->laneletLayer.get(obj_lane_id);
    std::string lanetag = lanelet.attributeOr("turn_direction", "else");
    if (lanetag == std::string("left") or lanetag == std::string("right")) {
      return max_speed_curve_;
    }
  }

  return max_speed_;
}

bool NPCSimulatorNode::calcCollisionDistance(
  const npc_simulator::msg::Object & obj, double * col_dist)
{
  if (!obj.stop_by_vehicle) {
    return false;
  }

  // calc center position of ego-polygon
  // (ego_pose_.pose is base_link position of ego. not center.)
  auto ego_center = ego_pose_.pose;
  const double ego_yaw = tf2::getYaw(ego_center.orientation);
  ego_center.position.x += std::cos(ego_yaw) * vehicle_base2center_;
  ego_center.position.y += std::sin(ego_yaw) * vehicle_base2center_;

  // get object center position
  auto obj_center = obj.initial_state.pose_covariance.pose;
  // when npc runs backward, flip orientation
  if (obj.initial_state.twist_covariance.twist.linear.x < 0.0) {
    const double obj_yaw = tf2::getYaw(obj_center.orientation);
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, obj_yaw + boost::math::constants::pi<double>());
    obj_center.orientation = tf2::toMsg(quat);
  }

  // calculate relative pose/yaw of ego car
  auto relative_pose = getRelativePose(obj_center, ego_center);
  auto rel_yaw = tf2::getYaw(relative_pose.orientation);

  auto rel_vehicle_width =
    std::fabs(std::cos(rel_yaw)) * vehicle_width_ + std::fabs(std::sin(rel_yaw)) * vehicle_length_;
  if (
    std::fabs(relative_pose.position.y) >
    (obj.shape.dimensions.y + rel_vehicle_width) / 2.0 + collsion_width_margin_) {
    // ego vehicle does not exists in front of npc
    return false;
  }

  // calculate distance to forward ego vehicle
  auto rel_vehicle_length =
    std::fabs(std::sin(rel_yaw)) * vehicle_width_ + std::fabs(std::cos(rel_yaw)) * vehicle_length_;
  *col_dist = relative_pose.position.x - (obj.shape.dimensions.x + rel_vehicle_length) / 2.0;

  // ego vehicle is too far from npc or in back of npc
  if (*col_dist < 0 || *col_dist > max_stop_distance_thresh_) {
    return false;
  }

  // no check about relation between npc and ego lane
  return true;

  /*
  //get lane id of npc and ego vehicle
  auto npc_lane_id = getCurrentLaneletID(obj);
  npc_simulator::msg::Object ego;
  ego.initial_state.pose_covariance.pose = ego_pose_.pose;
  ego.lane_follow_mode.mode = npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT;
  auto ego_lane_id = getCurrentLaneletID(ego);

  //check lane id is valid or not
  if (npc_lane_id < 0 || ego_lane_id < 0) {
    return false;
  }

  //check ego car is in same lane with npc
  if (npc_lane_id == ego_lane_id) {
    return true;
  }

  //check ego car is in next lane of npc' one
  auto lanelet = lanelet_map_ptr_->laneletLayer.get(npc_lane_id);
  auto next_lanelets = routing_graph_ptr_->following(lanelet, true);
  for (const auto next_lanelet : next_lanelets) {
    auto npc_next_lane_id = next_lanelet.id();
    if (npc_next_lane_id == ego_lane_id) {
      return true;
    }
  }

  //ego car is not in relative lane with npc
  return false;
  */
}

geometry_msgs::msg::Pose NPCSimulatorNode::getRelativePose(
  const geometry_msgs::msg::Pose & source, const geometry_msgs::msg::Pose & target)
{
  tf2::Transform transform_src, transform_trg;

  transform_src.setOrigin(tf2::Vector3(source.position.x, source.position.y, source.position.z));
  transform_src.setRotation(tf2::Quaternion(
    source.orientation.x, source.orientation.y, source.orientation.z, source.orientation.w));

  transform_trg.setOrigin(tf2::Vector3(target.position.x, target.position.y, target.position.z));
  transform_trg.setRotation(tf2::Quaternion(
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w));

  tf2::Transform transform_s2t = transform_src.inverse() * transform_trg;
  geometry_msgs::msg::Pose source2target;
  tf2::toMsg(transform_s2t, source2target);
  return source2target;
}

double NPCSimulatorNode::calcSpeedToAvoidCollision(const double col_dist)
{
  if (col_dist <= margin_dist_to_avoid_collision_) {
    return 0.0;
  }

  return (col_dist - margin_dist_to_avoid_collision_) / margin_time_to_avoid_collision_;
}

double NPCSimulatorNode::getFollowLaneDiffYaw(
  const double diff_yaw, const double signed_lane_dist, const double current_vel, const double dt,
  const double max_yaw_rate)
{
  double lane_follow_yaw = 0.0;
  double restricted_dif_yaw = 0.0;

  // when to go to opposite lane, force the turn-direction
  if (std::fabs(
        diff_yaw > boost::math::constants::pi<double>() / 1.2)) {  // |diff_yaw| is nearly to M_PI
    if (signed_lane_dist > 0) {
      if (diff_yaw < 0) {
        restricted_dif_yaw = diff_yaw + boost::math::constants::pi<double>() * 2.0;
      }
    } else {
      if (diff_yaw > 0) {
        restricted_dif_yaw = diff_yaw - boost::math::constants::pi<double>() * 2.0;
      }
    }
  } else {
    restricted_dif_yaw = diff_yaw;
  }

  // P control
  lane_follow_yaw += restricted_dif_yaw;
  double lane_follow_yaw_diff_dist =
    signed_lane_dist * (p_coef_diff_dist_ + p_coef_diff_dist_vel_ * std::fabs(current_vel));

  if (current_vel < 0) {
    //if obj has minus velocity, lane_follow_yaw_diff_dist flips.
    lane_follow_yaw_diff_dist *= -1;
  }

  lane_follow_yaw +=
    boost::algorithm::clamp(lane_follow_yaw_diff_dist, -max_yaw_diff_dist_, max_yaw_diff_dist_);
  lane_follow_yaw = boost::algorithm::clamp(lane_follow_yaw, -max_yaw_rate * dt, max_yaw_rate * dt);
  return lane_follow_yaw;
}

double NPCSimulatorNode::getNearestZPos(const geometry_msgs::msg::Pose & pose)
{
  //get current lanelet id
  lanelet::BasicPoint2d search_point(pose.position.x, pose.position.y);
  const auto nearest_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr_->laneletLayer, search_point, 1);  // distance, lanelet

  if (nearest_lanelets.size() == 0) {
    //no nearest_lanelets
    return 0.0;
  }

  //get centerline from current lanelet
  const auto current_lanelet = nearest_lanelets.at(0).second;
  const auto centerline = current_lanelet.centerline3d();
  if (centerline.empty()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *this->get_clock(), 5.0,
      "cannot get distance from centerline (invalid centerline)");
    return 0.0;
  }

  //search nearest centerline point from current_lanelet

  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx;
  size_t second_nearest_idx;
  for (size_t i = 0; i < centerline.size() - 1; i++) {
    const double dist = calcDist2D(pose.position, toMsg(centerline[i]));
    if (dist < min_dist) {
      //get z position from nearest centerline point
      nearest_idx = i;
      min_dist = dist;
    }
  }

  //search second nearest centerline point
  if (nearest_idx == 0) {
    second_nearest_idx = nearest_idx + 1;
  } else if (nearest_idx == centerline.size() - 1) {
    second_nearest_idx = nearest_idx - 1;
  } else {
    const double dist_minus = calcDist2D(pose.position, toMsg(centerline[nearest_idx - 1]));
    const double dist_plus = calcDist2D(pose.position, toMsg(centerline[nearest_idx + 1]));
    if (dist_minus < dist_plus) {
      second_nearest_idx = nearest_idx - 1;
    } else {
      second_nearest_idx = nearest_idx + 1;
    }
  }

  //calc smooth z from two nearest point
  const double nearest_z = calcSmoothZPos(
    pose.position, toMsg(centerline[nearest_idx]), toMsg(centerline[second_nearest_idx]));
  return nearest_z;
}

double NPCSimulatorNode::calcSmoothZPos(
  geometry_msgs::msg::Point current_point, geometry_msgs::msg::Point p1,
  geometry_msgs::msg::Point p2)
{
  const double x_p1_to_cp = current_point.x - p1.x;
  const double y_p1_to_cp = current_point.y - p1.y;
  const double x_p1_to_p2 = p2.x - p1.x;
  const double y_p1_to_p2 = p2.y - p1.y;

  // project vec(p1_to_cp) to vec(p1_to_p2), and calc coefficient k:
  // proj(vec(p1_to_cp)) = k * vec(p1_to_p2)

  const double coef_k = (x_p1_to_cp * x_p1_to_p2 + y_p1_to_cp * y_p1_to_p2) /
                        (x_p1_to_p2 * x_p1_to_p2 + y_p1_to_p2 * y_p1_to_p2);

  if (coef_k < 0) {
    // current point is outer point of p1_to_p2 (p1 side)
    return p1.z;
  }
  if (coef_k > 1) {
    // current point is outer point of p1_to_p2 (p2 side)
    return p2.z;
  }

  // current point is inner point of p1_to_p2
  return p1.z * (1 - coef_k) + p2.z * coef_k;
}

void NPCSimulatorNode::objectCallback(const npc_simulator::msg::Object::ConstSharedPtr msg)
{
  switch (msg->action) {
    case npc_simulator::msg::Object::ADD: {
      for (const auto & each : objects_) {
        if (each.id == msg->id) {
          return;
        }
      }

      tf2::Transform tf_input2map;

      try {
        geometry_msgs::msg::TransformStamped ros_input2map = tf_buffer_.lookupTransform(
          msg->header.frame_id, "map", msg->header.stamp, rclcpp::Duration(0.5));
        tf2::fromMsg(ros_input2map.transform, tf_input2map);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }

      tf2::Transform tf_input2object_origin;
      tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);

      npc_simulator::msg::Object object = *msg;
      object.header.frame_id = "map";

      tf2::Transform tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
      tf2::toMsg(tf_map2object_origin, object.initial_state.pose_covariance.pose);
      // publish
      dummy_perception_object_pub_->publish(convertObjectMsgToDummyPerception(&object));
      objects_.push_back(object);
      break;
    }
    case npc_simulator::msg::Object::DELETE: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          // publish
          objects_.at(i).action = npc_simulator::msg::Object::DELETE;
          const auto dummy_perception_obj_msg = convertObjectMsgToDummyPerception(&objects_.at(i));
          dummy_perception_object_pub_->publish(dummy_perception_obj_msg);
          objects_.erase(objects_.begin() + i);
          break;
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFY: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          tf2::Transform tf_input2map;
          tf2::Transform tf_input2object_origin;
          tf2::Transform tf_map2object_origin;
          try {
            geometry_msgs::msg::TransformStamped ros_input2map;
            ros_input2map = tf_buffer_.lookupTransform(
              /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
              rclcpp::Duration(0.5));
            tf2::fromMsg(ros_input2map.transform, tf_input2map);
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(get_logger(), "%s", ex.what());
            return;
          }
          tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
          tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
          obj = *msg;
          obj.header.frame_id = "map";
          tf2::toMsg(tf_map2object_origin, obj.initial_state.pose_covariance.pose);
          // publish
          const auto dummy_perception_obj_msg = convertObjectMsgToDummyPerception(&obj);
          dummy_perception_object_pub_->publish(dummy_perception_obj_msg);
          break;
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYTURNDIRECTION: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.action = msg->action;
          obj.lane_follow_mode.mode = msg->lane_follow_mode.mode;
          // no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYTWIST: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.action = msg->action;
          obj.initial_state.twist_covariance = msg->initial_state.twist_covariance;
          obj.target_vel = msg->initial_state.twist_covariance.twist.linear.x;
          // no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYLANECHANGE: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.action = msg->action;
          obj.lane_change_dir.dir = msg->lane_change_dir.dir;
          //no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYTARGETLANE: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.action = msg->action;
          obj.lane_change_id = msg->lane_change_id;
          //no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYACCEL: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.action = msg->action;
          obj.accel = std::fabs(msg->accel);
          obj.target_vel = msg->target_vel;
          //no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYROUTE: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.target_route = msg->target_route;
          //no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::MODIFYCONSIDERVEHICLE: {
      for (auto & obj : objects_) {
        if (obj.id.uuid == msg->id.uuid) {
          obj.stop_by_vehicle = msg->stop_by_vehicle;
          //no publish
        }
      }
      break;
    }
    case npc_simulator::msg::Object::DELETEALL: {
      objects_.clear();
      // publish
      dummy_perception_publisher::msg::Object dummy_perception_obj_msg;
      dummy_perception_obj_msg.action = dummy_perception_publisher::msg::Object::DELETEALL;
      dummy_perception_object_pub_->publish(dummy_perception_obj_msg);
      break;
    }
  }
}

void NPCSimulatorNode::mapCallback(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "Map is loaded");
}

void NPCSimulatorNode::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  ego_pose_ = *msg;
}

dummy_perception_publisher::msg::Object NPCSimulatorNode::convertObjectMsgToDummyPerception(
  npc_simulator::msg::Object * obj)
{
  dummy_perception_publisher::msg::Object output_obj;
  output_obj.header = obj->header;
  output_obj.id = obj->id;
  output_obj.initial_state = obj->initial_state;
  output_obj.semantic = obj->semantic;
  output_obj.shape = obj->shape;
  if (obj->action == npc_simulator::msg::Object::ADD) {
    output_obj.action = dummy_perception_publisher::msg::Object::ADD;
  } else if (obj->action == npc_simulator::msg::Object::DELETE) {
    output_obj.action = dummy_perception_publisher::msg::Object::DELETE;
  } else if (obj->action == npc_simulator::msg::Object::DELETEALL) {
    output_obj.action = dummy_perception_publisher::msg::Object::DELETEALL;
  } else {
    output_obj.action = dummy_perception_publisher::msg::Object::MODIFY;
  }
  //generate dummy_perception object as static obstacle
  output_obj.initial_state.twist_covariance.twist.linear.x = 0;
  output_obj.initial_state.twist_covariance.twist.linear.y = 0;
  output_obj.initial_state.twist_covariance.twist.linear.z = 0;
  output_obj.initial_state.twist_covariance.twist.angular.x = 0;
  output_obj.initial_state.twist_covariance.twist.angular.y = 0;
  output_obj.initial_state.twist_covariance.twist.angular.z = 0;

  if (obj->action == npc_simulator::msg::Object::ADD) {
    //from second time, obj.action must change to MODIFY
    obj->action = npc_simulator::msg::Object::MODIFY;
    sleep(0.01);  //avoid missing msg
  }
  return output_obj;
}

autoware_perception_msgs::msg::DynamicObjectArray
NPCSimulatorNode::convertObjectMsgToAutowarePerception(
  const std::vector<npc_simulator::msg::Object> & obj_vec, bool prediction)
{
  autoware_perception_msgs::msg::DynamicObjectArray output_msg;
  output_msg.header.frame_id = "map";
  output_msg.header.stamp = this->now();

  for (const auto obj : obj_vec) {
    autoware_perception_msgs::msg::DynamicObject autoware_obj;
    //convert obj type from npc_simulator_object to autoware_dynamic_object
    autoware_obj.id = obj.id;
    autoware_obj.semantic = obj.semantic;
    autoware_obj.shape = obj.shape;
    autoware_obj.state.pose_covariance = obj.initial_state.pose_covariance;
    autoware_obj.state.twist_covariance = obj.initial_state.twist_covariance;

    //position prediction for smooth visualization
    if (prediction && engage_state_) {
      const double dt =
        (rclcpp::Time(output_msg.header.stamp) - rclcpp::Time(obj.header.stamp)).seconds();
      const double obj_yaw = tf2::getYaw(obj.initial_state.pose_covariance.pose.orientation);
      const double vel_hor = autoware_obj.state.twist_covariance.twist.linear.x;
      const double vel_ver = autoware_obj.state.twist_covariance.twist.linear.z;
      const double dx = std::cos(obj_yaw) * vel_hor * dt;
      const double dy = std::sin(obj_yaw) * vel_hor * dt;
      const double dz = vel_ver * dt;
      autoware_obj.state.pose_covariance.pose.position.x += dx;
      autoware_obj.state.pose_covariance.pose.position.y += dy;
      autoware_obj.state.pose_covariance.pose.position.z += dz;
    }

    output_msg.objects.emplace_back(autoware_obj);
  }
  return output_msg;
}

rclcpp::TimerBase::SharedPtr NPCSimulatorNode::initTimer(
  const rclcpp::Duration & duration, void (NPCSimulatorNode::*ptr_to_member_fn)(void))
{
  auto timer_callback = std::bind(ptr_to_member_fn, this);
  rclcpp::TimerBase::SharedPtr timer =
    std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), std::chrono::nanoseconds{duration.nanoseconds()},
      std::move(timer_callback), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer, nullptr);
  return timer;
}
