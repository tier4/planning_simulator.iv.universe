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

#include <scenario_api_simulator/scenario_api_simulator.h>

#include <autoware_perception_msgs/msg/semantic.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <dummy_perception_publisher/msg/initial_state.hpp>
#include <scenario_api_utils/scenario_api_utils.h>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <functional>
#include <random>

ScenarioAPISimulator::ScenarioAPISimulator()
: rclcpp::Node("scenario_api_simulator")
{
  /* initializer*/
  npc_route_manager_ = std::make_shared<NPCRouteManager>();

  /* register service client*/
  client_ = this->create_client<npc_simulator::srv::GetObject>(
    "/simulation/npc_simulator/srv/get_object");

  /* register publisher */
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_object_info_ = this->create_publisher<npc_simulator::msg::Object>(
    "output/object_info",
    durable_qos);
  pub_simulator_engage_ = this->create_publisher<std_msgs::msg::Bool>(
    "output/simulator_engage",
    durable_qos);
  pub_npc_engage_ = this->create_publisher<std_msgs::msg::Bool>(
    "output/npc_simulator_engage",
    durable_qos);

  auto timer_callback = std::bind(&ScenarioAPISimulator::timerCallback, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(0.02));
  timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_control_, nullptr);
}

ScenarioAPISimulator::~ScenarioAPISimulator() {}

void ScenarioAPISimulator::timerCallback() {updateState();}

// basic API

bool ScenarioAPISimulator::isAPIReady()
{
  if (!npc_route_manager_->isAPIReady()) {
    return false;
  }
  return true;
}

bool ScenarioAPISimulator::updateState()
{
  updateNPC();
  return true;
}

// start API
bool ScenarioAPISimulator::spawnStartPoint(const geometry_msgs::msg::Pose pose)
{
  // spawn vehicle (planning simulator needs nothing)
  return true;
}

bool ScenarioAPISimulator::sendEngage(const bool engage)
{
  return sendSimulatorEngage(engage) && sendNPCEngage(engage);
}

bool ScenarioAPISimulator::sendSimulatorEngage(const bool engage)
{
  std_msgs::msg::Bool boolmsg;
  boolmsg.data = engage;
  pub_simulator_engage_->publish(boolmsg);
  return true;  // TODO check success
}

bool ScenarioAPISimulator::sendNPCEngage(const bool engage)
{
  std_msgs::msg::Bool boolmsg;
  boolmsg.data = engage;
  pub_npc_engage_->publish(boolmsg);
  return true;  // TODO check success
}

// NPC API
bool ScenarioAPISimulator::addNPC(
  const std::string & npc_type, const std::string & name, geometry_msgs::msg::Pose pose,
  const double velocity, const bool stop_by_vehicle, const std::string & frame_type)
{
  // uuid map check
  if (uuid_map_.find(name) != uuid_map_.end()) {
    RCLCPP_WARN_STREAM(get_logger(), "NPC name :" << name << " already exsists");
    return true;
  } else {
    // generate random UUID
    unique_identifier_msgs::msg::UUID id;
    std::mt19937 gen(std::random_device{} ());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(id.uuid.begin(), id.uuid.end(), bit_eng);
    uuid_map_[name] = id;
  }

  static const std::unordered_map<std::string, npc_simulator::msg::Object> objects
  {
    #define MAKE_OBJECT(TYPE, ...) \
  getObjectMsg( \
    autoware_perception_msgs::msg::Semantic::TYPE, 1.0, \
    autoware_perception_msgs::msg::Shape::BOUNDING_BOX, __VA_ARGS__)

    {"car", MAKE_OBJECT(CAR, 4.0, 1.8, 2.5)},
    {"pedestrian", MAKE_OBJECT(PEDESTRIAN, 0.8, 0.8, 2.0)},
    {"bicycle", MAKE_OBJECT(BICYCLE, 2.0, 0.8, 2.5)},
    {"motorbike", MAKE_OBJECT(MOTORBIKE, 2.5, 1.5, 2.5)},
    {"bus", MAKE_OBJECT(BUS, 10.0, 2.5, 3.5)},
    {"truck", MAKE_OBJECT(TRUCK, 7.5, 2.5, 3.0)},
    {"unknown", MAKE_OBJECT(UNKNOWN, 1.0, 1.0, 1.0)}

    #undef MAKE_OBJECT
  };

  npc_simulator::msg::Object object;

  try {
    object = objects.at(npc_type);
    object.offset_rate_from_center = (npc_type == "bicycle" ? 0.95 : 0); // Bicycle runs on left edge of lane
  } catch (std::out_of_range &) {
    RCLCPP_WARN(get_logger(), "NPC type is invalid. Publish NPC object as unknown type");
    object = objects.at("unknown");
  }

  //get pose with frame_type
  geometry_msgs::msg::Pose original_pose;
  original_pose.position = pose.position;
  original_pose.orientation = quatFromYaw(yawFromQuat(pose.orientation)); // ignore roll/pitch information

  dummy_perception_publisher::msg::InitialState init_state;
  if (!shiftNPCPose(original_pose, frame_type, object, &init_state.pose_covariance.pose)) {
    return false;
  }

  object.header.stamp = this->now();
  object.header.frame_id = "map";
  object.initial_state = init_state;
  object.initial_state.twist_covariance.twist.linear.x = velocity;
  object.target_vel = velocity;
  object.accel = npc_default_accel_;
  object.id = uuid_map_[name];
  object.action = npc_simulator::msg::Object::ADD;
  object.stop_by_vehicle = stop_by_vehicle;

  for (npc_simulator::msg::Object result {}; not getNPC(name, result); sleep(0.1)) {
    pub_object_info_->publish(object);
  }

  return true;  // TODO check successs
}

bool ScenarioAPISimulator::checkValidNPC(const std::string & name)
{
  if (uuid_map_.find(name) == uuid_map_.end()) {
    RCLCPP_WARN_STREAM(get_logger(), "NPC name :" << name << " does not exist");
    return false;
  } else {
    return true;
  }
}

bool ScenarioAPISimulator::sendNPCToCheckPoint(
  const std::string & name, const geometry_msgs::msg::Pose checkpoint_pose, const bool wait_ready,
  const std::string frame_type)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  if (obj.initial_state.twist_covariance.twist.linear.x < 0) {
    //if obj has minus velocity, do nothing
    return true;
  }
  geometry_msgs::msg::Pose initial_pose = obj.initial_state.pose_covariance.pose;

  //get pose with frame_type
  geometry_msgs::msg::Pose shift_checkpoint_pose;
  if (!shiftNPCPose(checkpoint_pose, frame_type, obj, &shift_checkpoint_pose)) {
    return false;
  }

  npc_route_manager_->setCheckPoint(name, shift_checkpoint_pose);

  //if goal pose is already given, replan route.
  geometry_msgs::msg::Pose goal_pose;
  if (getNPCGoal(name, &goal_pose)) {
    std::vector<int> route;
    if (!npc_route_manager_->planRoute(name, initial_pose, goal_pose, &route)) {
      return false;
    }
    changeNPCRoute(name, route);
  }
  return true;
}

bool ScenarioAPISimulator::sendNPCToGoalPoint(
  const std::string & name, const geometry_msgs::msg::Pose goal_pose, const bool wait_ready,
  const std::string frame_type)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  if (obj.initial_state.twist_covariance.twist.linear.x < 0) {
    //if obj has minus velocity, do nothing
    return true;
  }
  geometry_msgs::msg::Pose initial_pose = obj.initial_state.pose_covariance.pose;
  if (!getNPCPosition(name, &initial_pose)) {
    return false;
  }

  //get pose with frame_type
  geometry_msgs::msg::Pose shift_goal_pose;
  if (!shiftNPCPose(goal_pose, frame_type, obj, &shift_goal_pose)) {
    return false;
  }

  std::vector<int> route;
  if (!npc_route_manager_->planRoute(name, initial_pose, shift_goal_pose, &route)) {
    return false;
  }
  changeNPCRoute(name, route);

  if (wait_ready) {
    sleep(0.01);  // TODO remove this(sleep for avoiding message loss)
  }
  return true;
}

bool ScenarioAPISimulator::changeNPCVelocity(const std::string & name, const double velocity)
{
  //get object current velocity
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    //case: cannot get object info
    //sudden velocity change
    return changeNPCVelocityWithoutAccel(name, velocity);
  }

  double obj_vel = obj.initial_state.twist_covariance.twist.linear.x;

  //acceleration
  if (velocity > obj_vel && maxacc_map_.find(name) != maxacc_map_.end()) {
    double accel = std::fabs(maxacc_map_[name]);
    return changeNPCVelocityWithAccel(name, velocity, accel);
  }

  //deceleration
  if (velocity < obj_vel and minacc_map_.find(name) != minacc_map_.end()) {
    double accel = std::fabs(minacc_map_[name]);
    return changeNPCVelocityWithAccel(name, velocity, accel);
  }

  //case: cannot get accel info
  //sudden velocity change
  return changeNPCVelocityWithoutAccel(name, velocity);
}

bool ScenarioAPISimulator::changeNPCVelocityWithoutAccel(
  const std::string & name, const double velocity)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYTWIST;
  object.initial_state.twist_covariance.twist.linear.x = velocity;
  object.target_vel = velocity;
  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)
  return true;
}

bool ScenarioAPISimulator::changeNPCAccelMin(const std::string & name, const double accel)
{
  minacc_map_[name] = accel;
  return true;
}

bool ScenarioAPISimulator::changeNPCAccelMax(const std::string & name, const double accel)
{
  maxacc_map_[name] = accel;
  return true;
}

bool ScenarioAPISimulator::changeNPCVelocityWithAccel(
  const std::string & name, const double velocity, const double accel)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYACCEL;
  object.target_vel = velocity;
  object.accel = accel;
  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)

  return true;
}

bool ScenarioAPISimulator::changeNPCConsiderVehicle(
  const std::string & name, const bool consider_ego_vehicle)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYCONSIDERVEHICLE;
  object.stop_by_vehicle = consider_ego_vehicle;
  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)

  return true;
}

bool ScenarioAPISimulator::changeNPCRoute(const std::string & name, const std::vector<int> route)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYROUTE;
  object.target_route.data.clear();
  for (auto lane : route) {
    //input lane id list
    object.target_route.data.push_back(lane);
  }

  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)
  return true;
}

bool ScenarioAPISimulator::targetLaneChangeNPC(const std::string & name, const int target_lane_id)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYTARGETLANE;
  object.lane_change_id = target_lane_id;
  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)

  return true;
}

bool ScenarioAPISimulator::laneChangeNPC(const std::string & name, const uint8_t lane_change_dir)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYLANECHANGE;
  object.lane_change_dir.dir = lane_change_dir;
  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)

  return true;
}

bool ScenarioAPISimulator::changeNPCLaneChangeLeft(const std::string & name)
{
  return laneChangeNPC(name, npc_simulator::msg::LaneChangeDir::LEFT_LANE_CHANGE);
}

bool ScenarioAPISimulator::changeNPCLaneChangeRight(const std::string & name)
{
  return laneChangeNPC(name, npc_simulator::msg::LaneChangeDir::RIGHT_LANE_CHANGE);
}

bool ScenarioAPISimulator::changeNPCLaneChange(const std::string & name, const int target_lane_id)
{
  return targetLaneChangeNPC(name, target_lane_id);
}

bool ScenarioAPISimulator::changeNPCUturn(const std::string & name)
{
  return laneChangeNPC(name, npc_simulator::msg::LaneChangeDir::LANE_CHANGE_UTURN);
}

bool ScenarioAPISimulator::changeNPCBehavior(const std::string & name, const uint8_t behavior_mode)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  // generate object info
  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::MODIFYTURNDIRECTION;
  object.lane_follow_mode.mode = behavior_mode;
  pub_object_info_->publish(object);
  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)

  return true;
}

bool ScenarioAPISimulator::changeNPCTurnLeft(const std::string & name)
{
  return changeNPCBehavior(name, npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_LEFT);
}

bool ScenarioAPISimulator::changeNPCTurnRight(const std::string & name)
{
  return changeNPCBehavior(name, npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_RIGHT);
}

bool ScenarioAPISimulator::changeNPCNoTurn(const std::string & name)
{
  return changeNPCBehavior(name, npc_simulator::msg::LaneFollowMode::MOVE_LANE_FOLLOW_STRAIGHT);
}

bool ScenarioAPISimulator::changeNPCIgnoreLane(const std::string & name)
{
  return changeNPCBehavior(name, npc_simulator::msg::LaneFollowMode::MOVE_STRAIGHT);
}

bool ScenarioAPISimulator::checkNPCFinishLaneChange(
  const std::string & name, bool & end_lane_change)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  if (obj.lane_change_dir.dir == npc_simulator::msg::LaneChangeDir::NO_LANE_CHANGE) {
    end_lane_change = true;
  } else {
    end_lane_change = false;
  }

  return true;
}

bool ScenarioAPISimulator::checkNPCFinishVelocityChange(
  const std::string & name, bool & end_velocity_change)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  if (obj.accel < 1e-03) {
    //no request for chaning velocity
    end_velocity_change = true;
    return true;
  }

  if (std::fabs(obj.initial_state.twist_covariance.twist.linear.x - obj.target_vel) < 1e-02) {
    //already finished chaning velocity
    end_velocity_change = true;
    return true;
  }

  //change velocity now
  end_velocity_change = false;
  return true;
}

bool ScenarioAPISimulator::deleteNPC(const std::string & name)
{
  if (!checkValidNPC(name)) {
    return false;
  }

  npc_simulator::msg::Object object = getObjectMsg(name);
  object.action = npc_simulator::msg::Object::DELETE;
  pub_object_info_->publish(object);

  // delete uuid map
  uuid_map_.erase(name);

  sleep(0.01);  // TODO remove this(sleep for avoiding message loss)
  return true;
}

std::vector<std::string> ScenarioAPISimulator::getNpcList()
{
  std::vector<std::string> ret;
  for (auto itr = uuid_map_.begin(); itr != uuid_map_.end(); itr++) {
    ret.push_back(itr->first);
  }
  return ret;
}

void ScenarioAPISimulator::updateNPC()
{
  std::unordered_map<std::string, npc_simulator::msg::Object> npc_infos = getNPCInfo();
  std::unordered_map<std::string, uint8_t> npc_follow_states =
    npc_route_manager_->updateNPCLaneFollowState(npc_infos);
  inputNPCLaneFollowState(npc_follow_states);
  std::unordered_map<std::string, bool> npc_stop_states =
    npc_route_manager_->updateNPCStopState(npc_infos);
  inputNPCStopState(npc_stop_states);
}

bool ScenarioAPISimulator::getNPC(const std::string & name, npc_simulator::msg::Object & obj)
{
  if (!checkValidNPC(name)) {
    RCLCPP_WARN_STREAM(get_logger(), "Invalid NPC name '" << name << "' requested.");
    return false;
  } else {
    auto req = std::make_shared<npc_simulator::srv::GetObject::Request>();
    req->object_id = uuid_map_.at(name);

    auto res_future = client_->async_send_request(req);
    auto status = res_future.wait_for(std::chrono::microseconds(1000));
    if (status == std::future_status::ready) {
      obj = res_future.get()->object;
      return true;
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Failed to get NPC");
      return false;
    }
  }
}

bool ScenarioAPISimulator::getNPC(
  const std::string & name, geometry_msgs::msg::Pose & object_pose,
  geometry_msgs::msg::Twist & object_twist,
  geometry_msgs::msg::Vector3 & object_size, std::string & object_name)
{
  npc_simulator::msg::Object obj;

  if (!getNPC(name, obj)) {
    return false;
  } else {
    object_pose = obj.initial_state.pose_covariance.pose;
    object_twist = obj.initial_state.twist_covariance.twist;
    object_size = obj.shape.dimensions;

    switch (obj.semantic.type) {
      case autoware_perception_msgs::msg::Semantic::BICYCLE:    {object_name = "bicycle";    break;}
      case autoware_perception_msgs::msg::Semantic::BUS:        {object_name = "bus";        break;}
      case autoware_perception_msgs::msg::Semantic::CAR:        {object_name = "car";        break;}
      case autoware_perception_msgs::msg::Semantic::MOTORBIKE:  {object_name = "motorbike";  break;}
      case autoware_perception_msgs::msg::Semantic::PEDESTRIAN: {object_name = "pedestrian"; break;}
      case autoware_perception_msgs::msg::Semantic::TRUCK:      {object_name = "truck";      break;}
      case autoware_perception_msgs::msg::Semantic::UNKNOWN:    {object_name = "unknown";    break;}

      default:
        object_name = "else";
    }

    return true;
  }
}

bool ScenarioAPISimulator::getNPCPosition(const std::string & name, geometry_msgs::msg::Pose * pose)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  *pose = obj.initial_state.pose_covariance.pose;
  return true;
}

bool ScenarioAPISimulator::getNPCVelocity(const std::string & name, double * velocity)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  *velocity = obj.initial_state.twist_covariance.twist.linear.x;
  return true;
}

bool ScenarioAPISimulator::getNPCAccel(const std::string & name, double * accel)
{
  npc_simulator::msg::Object obj;
  if (!getNPC(name, obj)) {
    return false;
  }

  *accel = obj.imu.linear_acceleration.x;
  return true;
}

bool ScenarioAPISimulator::getNPCGoal(const std::string & name, geometry_msgs::msg::Pose * pose)
{
  return npc_route_manager_->getNPCGoal(name, pose);
}

std::unordered_map<std::string, npc_simulator::msg::Object> ScenarioAPISimulator::getNPCInfo()
{
  std::unordered_map<std::string, npc_simulator::msg::Object> npc_infos;
  for (auto npc : uuid_map_) {
    std::string name = npc.first;
    npc_simulator::msg::Object obj;
    if (getNPC(name, obj)) {
      npc_infos[name] = obj;
    }
  }

  return npc_infos;
}

bool ScenarioAPISimulator::shiftNPCPose(
  const geometry_msgs::msg::Pose & pose, const std::string frame_type,
  const npc_simulator::msg::Object & obj,
  geometry_msgs::msg::Pose * shift_pose)
{
  // shift pose from farame_type to "Center"

  if (obj.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    RCLCPP_ERROR_STREAM(get_logger(), "Now, npc with polygon type is not supported");
    return false;
  }

  const double obj_length = obj.shape.dimensions.x;

  if (frame_type == "Center") {
    //no shift
    *shift_pose = pose;
    return true;
  }

  if (frame_type == "Front") {
    //shift to back
    *shift_pose = movePose(pose, -obj_length / 2.0);
    return true;
  }

  if (frame_type == "Rear") {
    //shift to front
    *shift_pose = movePose(pose, obj_length / 2.0);
    return true;
  }

  RCLCPP_ERROR_STREAM(
    get_logger(),
    "shiftEGoPose supports only Center, Front, and Rear as frame_type. " <<
      "Now, frame_type is " << frame_type << ".");
  return false;
}

npc_simulator::msg::Object ScenarioAPISimulator::getObjectMsg(
  uint32_t semantic_type, double confidence, uint8_t shape_type, double size_x, double size_y,
  double size_z)
{
  npc_simulator::msg::Object obj;
  obj.semantic.type = semantic_type;
  obj.semantic.confidence = confidence;
  obj.shape.type = shape_type;
  obj.shape.dimensions.x = size_x;
  obj.shape.dimensions.y = size_y;
  obj.shape.dimensions.z = size_z;
  return obj;
}

npc_simulator::msg::Object ScenarioAPISimulator::getObjectMsg(
  std::string npc_name,
  std::string frame_id)
{
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = frame_id;

  npc_simulator::msg::Object object;
  object.header = header;
  object.id = uuid_map_[npc_name];
  return object;
}

bool ScenarioAPISimulator::inputNPCLaneFollowState(std::unordered_map<std::string, uint8_t> states)
{
  //change lane follow state to states[npc_name]
  for (auto state : states) {
    std::string name = state.first;
    uint8_t lf_state = state.second;  //lane_follow_state
    changeNPCBehavior(name, lf_state);
  }
  return true;
}

bool ScenarioAPISimulator::inputNPCStopState(std::unordered_map<std::string, bool> states)
{
  //change npc velocity to 0, if states[npc_name] is true
  for (auto state : states) {
    std::string name = state.first;
    bool stop_state = state.second;  //npc_stop_state
    if (stop_state) {
      changeNPCVelocityWithAccel(name, 0.0, npc_stop_accel_);
    }
  }
  return true;
}

// traffic light API
bool ScenarioAPISimulator::setTrafficLight(int traffic_id, std::string traffic_color)
{
  RCLCPP_WARN(get_logger(), "setTrafficLight is not implemented yet.");
  // TODO
  return false;
}
