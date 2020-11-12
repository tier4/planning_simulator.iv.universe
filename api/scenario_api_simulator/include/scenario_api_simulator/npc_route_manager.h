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

#ifndef NPC_ROUTE_MANAGER_H_INCLUDED
#define NPC_ROUTE_MANAGER_H_INCLUDED

#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/utility/utilities.h>
#include <npc_simulator/msg/object.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <vector>


namespace lanelet
{
  class Lanelet;
  class LaneletMap;
  using LaneletMapPtr = std::shared_ptr < LaneletMap >;
  namespace routing
  {
    class RoutingGraph;
  }
  namespace traffic_rules
  {
    class TrafficRules;
  }
}  // namespace lanelet

class NPCRouteManager: public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  NPCRouteManager();

  /**
   * @brief destructor
   */
  ~NPCRouteManager();

  bool isAPIReady();

  /**
   * @brief search route from initial_pose to goal_pose with checkpoint
   */
  bool planRoute(
    const std::string & name, const geometry_msgs::msg::Pose initial_pose,
    const geometry_msgs::msg::Pose goal_pose, std::vector < int > * const route);

  /**
   * @brief search route from initial_pose to goal_pose
   */
  bool planPathBetweenCheckpoints(
    const geometry_msgs::msg::Pose & start_checkpoint,
    const geometry_msgs::msg::Pose & goal_checkpoint,
    lanelet::ConstLanelets * path_lanelets_ptr);

  /**
   * @brief set npc check point.
   */

  bool setCheckPoint(const std::string & name, const geometry_msgs::msg::Pose checkpoint_pose);

  /**
   * @brief decide npc lane follow state(go straight, turn left, turn light)
   */
  std::unordered_map < std::string, uint8_t > updateNPCLaneFollowState(
    std::unordered_map < std::string, npc_simulator::msg::Object > npc_infos);

  /**
   * @brief decide npc lane stop state
   */
  std::unordered_map < std::string, bool > updateNPCStopState(
    std::unordered_map < std::string, npc_simulator::msg::Object > npc_infos);

  /**
   * @brief get npc goal position
   */
  bool getNPCGoal(const std::string & name, geometry_msgs::msg::Pose * pose);

private:
  rclcpp::Subscription < autoware_lanelet2_msgs::msg::MapBin > ::SharedPtr sub_map_;  //!< @brief topic subscriber for map

  // lanelet
  std::shared_ptr < lanelet::LaneletMap > lanelet_map_ptr_;
  std::shared_ptr < lanelet::routing::RoutingGraph > routing_graph_ptr_;
  std::shared_ptr < lanelet::traffic_rules::TrafficRules > traffic_rules_ptr_;
  std::shared_ptr < lanelet::Lanelet > closest_lanelet_ptr_;

  std::unordered_map < std::string, bool > npc_stop_state_;
  std::unordered_map < std::string, geometry_msgs::msg::Pose > npc_goal_map_;
  std::unordered_map < std::string, lanelet::ConstLanelets > npc_lane_map_;
  std::unordered_map < std::string, std::vector < geometry_msgs::msg::Pose >> npc_checkpoints_map_;

  void callbackMap(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  bool getClosestLanelet(
    const geometry_msgs::msg::Pose current_pose, const lanelet::LaneletMapPtr & lanelet_map_ptr,
    lanelet::Lanelet * closest_lanelet, double max_dist = 10.0,
    double max_delta_yaw = M_PI / 4.0);
  bool getClosestLaneletWithRoutes(
    const geometry_msgs::msg::Pose current_pose, const lanelet::LaneletMapPtr & lanelet_map_ptr,
    lanelet::Lanelet * closest_lanelet, lanelet::ConstLanelets routes, double max_dist = 20.0,
    double max_delta_yaw = M_PI);
  uint8_t decideNPCLaneFollowDir(
    lanelet::ConstLanelets routes, std::string npc_name, geometry_msgs::msg::Pose npc_pose);
  bool isGoal(
    const geometry_msgs::msg::Pose goal_pose, const geometry_msgs::msg::Pose npc_pose,
    const double npc_vel,
    const double thresh_dist = 0.5,
    const double thresh_delta_yaw = M_PI);

  //parameter
  const double npc_stop_accel_ = 3.0;
  const double npc_stop_margin_time_ = 0.1;
  const double npc_max_stop_dist_ = 40.0;
  const double npc_min_lateral_stop_dist_ = 3.0;
  const double base_cost_no_curve_ = 10.0;
  const double base_cost_besides_lane_ = 3.0;
};

#endif  // NPC_ROUTE_MANAGER_H_INCLUDED
