/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <scenario_api_simulator/npc_route_manager.h>

#include <scenario_api_utils/scenario_api_utils.h>
#include <npc_simulator/msg/lane_follow_mode.hpp>
#include <lanelet2_extension/utility/message_conversion.h>

#include <functional>

NPCRouteManager::NPCRouteManager()
: rclcpp::Node("npcl_route_manager")
{
  sub_map_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/vectormap", 1, std::bind(
      &NPCRouteManager::callbackMap, this,
      std::placeholders::_1));
}

NPCRouteManager::~NPCRouteManager() {}

bool NPCRouteManager::isAPIReady()
{
  if (lanelet_map_ptr_ == nullptr) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(),
      *get_clock(), 5000 /* ms */, "lanelet_map is nullptr");
    return false;
  }
  return true;
}

bool NPCRouteManager::planRoute(
  const std::string & name, const geometry_msgs::msg::Pose initial_pose,
  const geometry_msgs::msg::Pose goal_pose, std::vector<int> * const route)
{
  //create check point list
  std::vector<geometry_msgs::msg::Pose> checkpoints;
  checkpoints.emplace_back(initial_pose);

  if (npc_checkpoints_map_.find(name) != npc_checkpoints_map_.end()) {
    auto point_list = npc_checkpoints_map_[name];
    for (const auto point : point_list) {
      checkpoints.emplace_back(point);
    }
  }
  checkpoints.emplace_back(goal_pose);

  // get all possible lanes that can be used to reach goal (including all possible lane change)
  lanelet::ConstLanelets path_lanelets_ptr;

  for (std::size_t i = 1; i < checkpoints.size(); i++) {
    const auto start_checkpoint = checkpoints.at(i - 1);
    const auto goal_checkpoint = checkpoints.at(i);
    if (!planPathBetweenCheckpoints(start_checkpoint, goal_checkpoint, &path_lanelets_ptr)) {
      return false;
    }
  }

  //register npc info
  npc_goal_map_[name] = goal_pose;
  npc_lane_map_[name] = path_lanelets_ptr;

  //output route
  route->clear();
  for (const auto & lanelet : path_lanelets_ptr) {
    route->emplace_back(lanelet.id());
  }

  return true;
}

bool NPCRouteManager::planPathBetweenCheckpoints(
  const geometry_msgs::msg::Pose & start_checkpoint,
  const geometry_msgs::msg::Pose & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets_ptr)
{
  lanelet::Lanelet start_lanelet;
  if (!getClosestLanelet(start_checkpoint, lanelet_map_ptr_, &start_lanelet)) {
    return false;
  }
  lanelet::Lanelet goal_lanelet;
  if (!getClosestLanelet(goal_checkpoint, lanelet_map_ptr_, &goal_lanelet)) {
    return false;
  }

  // get all possible lanes that can be used to reach goal (including all possible lane change)
  lanelet::Optional<lanelet::routing::Route> optional_route =
    routing_graph_ptr_->getRoute(start_lanelet, goal_lanelet, 0);
  if (!optional_route) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to find a proper path");
    return false;
  }

  const auto shortest_path = optional_route->shortestPath();
  for (const auto & llt : shortest_path) {
    path_lanelets_ptr->push_back(llt);
  }
  return true;
}

bool NPCRouteManager::setCheckPoint(
  const std::string & name, const geometry_msgs::msg::Pose checkpoint_pose)
{
  if (npc_checkpoints_map_.find(name) != npc_checkpoints_map_.end()) {
    npc_checkpoints_map_[name].emplace_back(checkpoint_pose);
  } else {
    std::vector<geometry_msgs::msg::Pose> checkpoint_list;
    checkpoint_list.emplace_back(checkpoint_pose);
    npc_checkpoints_map_[name] = checkpoint_list;
  }
  return true;
}

bool NPCRouteManager::getNPCGoal(const std::string & name, geometry_msgs::msg::Pose * pose)
{
  if (npc_goal_map_.find(name) == npc_goal_map_.end()) {
    return false;
  }

  *pose = npc_goal_map_[name];
  return true;
}

void NPCRouteManager::callbackMap(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "Map is loaded");
}

std::unordered_map<std::string, uint8_t> NPCRouteManager::updateNPCLaneFollowState(
  std::unordered_map<std::string, npc_simulator::msg::Object> npc_infos)
{
  std::unordered_map<std::string, uint8_t> lane_follow_states;

  for (const auto & npc_info_pair : npc_infos) {
    std::string npc_name = npc_info_pair.first;
    auto npc_info = npc_info_pair.second;
    geometry_msgs::msg::Pose npc_curr_pose = npc_info.initial_state.pose_covariance.pose;

    if (npc_lane_map_.find(npc_name) == npc_lane_map_.end()) {
      //"name" NPC is not in npc-lane list.
      continue;
    }
    auto route = npc_lane_map_[npc_name];
    uint8_t lane_follow_dir = decideNPCLaneFollowDir(route, npc_name, npc_curr_pose);
    lane_follow_states[npc_name] = lane_follow_dir;
  }
  return lane_follow_states;
}

std::unordered_map<std::string, bool> NPCRouteManager::updateNPCStopState(
  std::unordered_map<std::string, npc_simulator::msg::Object> npc_infos)
{
  //check goal
  for (const auto & npc_goal : npc_goal_map_) {
    std::string name = npc_goal.first;
    geometry_msgs::msg::Pose goal_pose = npc_goal.second;

    if (npc_infos.find(name) == npc_infos.end()) {
      //"name" NPC is not in npc list.
      continue;
    }
    if (npc_stop_state_.find(name) != npc_stop_state_.end()) {
      if (npc_stop_state_[name]) {
        //already stop state
        continue;
      }
    }
    const auto npc_info = npc_infos[name];
    const auto npc_pose = npc_info.initial_state.pose_covariance.pose;
    const auto npc_vel = npc_info.initial_state.twist_covariance.twist.linear.x;
    if (isGoal(goal_pose, npc_pose, npc_vel)) {
      npc_stop_state_[name] = true;
    }
  }

  return npc_stop_state_;
}

uint8_t NPCRouteManager::decideNPCLaneFollowDir(
  lanelet::ConstLanelets routes, std::string npc_name, geometry_msgs::msg::Pose npc_pose)
{
  lanelet::Lanelet closest_lanelet;
  if (!getClosestLaneletWithRoutes(npc_pose, lanelet_map_ptr_, &closest_lanelet, routes)) {
    return npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT;
  }
  int lane_id = closest_lanelet.id();

  const std::string left = "left";
  const std::string right = "right";

  for (size_t i = 0; i < routes.size(); i++) {
    if (routes.at(i).id() != lane_id) {
      continue;
    }

    //check current lane tag
    if (routes.at(i).attributeOr("turn_direction", std::string("else")) == left) {
      return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_LEFT);
    } else if (routes.at(i).attributeOr("turn_direction", std::string("else")) == right) {
      return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_RIGHT);
    }

    //next lane id is null
    if (i == routes.size() - 1) {
      return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT);
    }

    //check next lane tag
    if (routes.at(i + 1).attributeOr("turn_direction", std::string("else")) == left) {
      return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_LEFT);
    } else if (routes.at(i + 1).attributeOr("turn_direction", std::string("else")) == right) {
      return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_RIGHT);
    }

    //no tag
    return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT);
  }

  //no lanelet id to correspond to closest lanelet id
  for (const auto & route : routes) {
    //serach adjacent lane
    auto beside_lanes = routing_graph_ptr_->besides(route);
    for (const auto & lane : beside_lanes) {
      if (lane.id() != lane_id) {
        continue;
      }

      //check current lane tag
      if (lane.attributeOr("turn_direction", "else") == left) {
        return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_LEFT);
      } else if (lane.attributeOr("turn_direction", "else") == right) {
        return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_RIGHT);
      }

      //check next lane tag
      if (route.attributeOr("turn_direction", "else") == left) {
        return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_LEFT);
      } else if (route.attributeOr("turn_direction", "else") == right) {
        return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_RIGHT);
      }
      //no tag
      return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT);
    }
  }

  //out of route. npc stops now.
  npc_stop_state_[npc_name] = true;
  return static_cast<uint8_t>(npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT);
}

bool NPCRouteManager::isGoal(
  const geometry_msgs::msg::Pose goal_pose, const geometry_msgs::msg::Pose npc_pose,
  const double npc_vel,
  const double thresh_dist, const double thresh_delta_yaw)
{
  const double dx = goal_pose.position.x - npc_pose.position.x;
  const double dy = goal_pose.position.y - npc_pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double obj_yaw = tf2::getYaw(npc_pose.orientation);
  const double relative_goal_yaw = std::atan2(dy, dx);
  const double lateral_dist = std::fabs(dist * std::cos(relative_goal_yaw - obj_yaw));
  const double longitudinal_dist = std::fabs(dist * std::sin(relative_goal_yaw - obj_yaw));

  const double delta_yaw = std::fabs(
    normalizeRadian(yawFromQuat(goal_pose.orientation) - yawFromQuat(npc_pose.orientation)));

  double stop_dist =
    (1.0 / 2.0) * npc_vel * npc_vel / npc_stop_accel_ + npc_vel * npc_stop_margin_time_;
  stop_dist = std::min(std::max(stop_dist, thresh_dist), npc_max_stop_dist_);

  if (
    longitudinal_dist <= stop_dist && lateral_dist <= npc_min_lateral_stop_dist_ &&
    delta_yaw <= thresh_delta_yaw)
  {
    return true;
  }

  return false;
}

bool NPCRouteManager::getClosestLanelet(
  const geometry_msgs::msg::Pose current_pose, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  lanelet::Lanelet * closest_lanelet, double max_dist, double max_delta_yaw)
{
  lanelet::BasicPoint2d search_point(current_pose.position.x, current_pose.position.y);
  const auto nearest_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr->laneletLayer, search_point, 20);  // distance, lanelet
  lanelet::Lanelet target_closest_lanelet;
  bool is_found_target_closest_lanelet = false;
  double min_dist = max_dist;
  for (const auto & lanelet : nearest_lanelets) {
    double current_yaw = tf2::getYaw(current_pose.orientation);
    double lane_yaw = lanelet::utils::getLaneletAngle(lanelet.second, current_pose.position);
    double delta_yaw = std::abs(normalizeRadian(current_yaw - lane_yaw));
    if (lanelet.first < max_dist && delta_yaw < max_delta_yaw and lanelet.first < min_dist) {
      min_dist = lanelet.first;
      target_closest_lanelet = lanelet.second;
      is_found_target_closest_lanelet = true;
    }
  }

  if (is_found_target_closest_lanelet) {
    *closest_lanelet = target_closest_lanelet;
    return true;
  } else {
    return false;
  }
}

bool NPCRouteManager::getClosestLaneletWithRoutes(
  const geometry_msgs::msg::Pose current_pose, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  lanelet::Lanelet * closest_lanelet, lanelet::ConstLanelets routes, double max_dist,
  double max_delta_yaw)
{
  lanelet::BasicPoint2d search_point(current_pose.position.x, current_pose.position.y);
  const auto nearest_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr->laneletLayer, search_point, 10);  // <distance, lanelet>
  lanelet::Lanelet target_closest_lanelet;
  bool is_found_target_closest_lanelet = false;
  double min_dist = max_dist;
  for (const auto & lanelet : nearest_lanelets) {
    //check lenalet is involved in routes or not
    bool is_lane_in_route = false;
    bool is_lane_in_besides = false;
    for (const auto & route : routes) {
      if (lanelet.second.id() == route.id()) {
        is_lane_in_route = true;
        break;
      }
    }

    for (const auto & route : routes) {
      if (is_lane_in_route) {
        //if already in lane, skip here
        break;
      }
      //check lenalet is involved in besides of routes or not (for lane change)
      auto current_lanelet = lanelet_map_ptr_->laneletLayer.get(route.id());
      auto besides_lanelets = routing_graph_ptr_->besides(current_lanelet);
      for (const auto & beside_lane : besides_lanelets) {
        if (lanelet.second.id() == beside_lane.id()) {
          is_lane_in_route = true;
          is_lane_in_besides = true;
          break;
        }
      }
    }

    if (!is_lane_in_route) {
      continue;
    }

    double current_yaw = tf2::getYaw(current_pose.orientation);
    double lane_yaw = lanelet::utils::getLaneletAngle(lanelet.second, current_pose.position);
    double delta_yaw = std::abs(normalizeRadian(current_yaw - lane_yaw));
    double lane_dist = lanelet.first;
    std::string lanetag = lanelet.second.attributeOr("turn_direction", "else");
    if (lanetag != std::string("left") && lanetag != std::string("right")) {
      //prioritize left, right tag
      lane_dist += base_cost_no_curve_;
      if (is_lane_in_besides) {
        lane_dist += base_cost_besides_lane_;
      }
    }
    if (lane_dist < max_dist && delta_yaw < max_delta_yaw && lanelet.first < min_dist) {
      min_dist = lanelet.first;
      target_closest_lanelet = lanelet.second;
      is_found_target_closest_lanelet = true;
    }
  }

  if (is_found_target_closest_lanelet) {
    *closest_lanelet = target_closest_lanelet;
    return true;
  } else {
    //serach closest lanelet without routes
    return getClosestLanelet(current_pose, lanelet_map_ptr, closest_lanelet);
  }
}
