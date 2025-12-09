
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit/kinematic_constraints/utils.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_script");
const std::string PLANNING_GROUP = "arm";
const std::string PLANNER_ID = "PTP";
const double VELOCITY_SCALING_FACTOR = 0.1;
const double ACCELERATION_SCALING_FACTOR = 0.1;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("test_script", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto service_client = move_group_node->create_client<moveit_msgs::srv::GetMotionSequence>("/plan_sequence_path");

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId(PLANNER_ID);
  RCLCPP_INFO(LOGGER, "Planning pipeline: %s", move_group.getPlanningPipelineId().c_str());
  RCLCPP_INFO(LOGGER, "Planner ID: %s", move_group.getPlannerId().c_str());
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  rclcpp::Rate(0.25).sleep();

  if (!service_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "/plan_sequence_path server not available after waiting");
    rclcpp::shutdown();
  }

  rclcpp::Rate(1).sleep();

  // Move to ready position
  move_group.setNamedTarget("ready");
  move_group.move();

  rclcpp::Rate(1).sleep();

  // Define waypoints
  geometry_msgs::msg::Pose pick_waypoint1;
  pick_waypoint1.position.x = 0.697;
  pick_waypoint1.position.y = 0.005;
  pick_waypoint1.position.z = 0.387;
  pick_waypoint1.orientation.x = 0.0;
  pick_waypoint1.orientation.y = 1.0;
  pick_waypoint1.orientation.z = 0.0;
  pick_waypoint1.orientation.w = 0.0;

  geometry_msgs::msg::Pose pick_waypoint2;
  pick_waypoint2.position.x = 0.67;
  pick_waypoint2.position.y = -0.24;
  pick_waypoint2.position.z = 0.44;
  pick_waypoint2.orientation.x = 0.0;
  pick_waypoint2.orientation.y = 1.0;
  pick_waypoint2.orientation.z = 0.0;
  pick_waypoint2.orientation.w = 0.0;

  geometry_msgs::msg::Pose pick_waypoint3;
  pick_waypoint3.position.x = 0.67;
  pick_waypoint3.position.y = -0.24;
  pick_waypoint3.position.z = 0.35;
  pick_waypoint3.orientation.x = 0.0;
  pick_waypoint3.orientation.y = 1.0;
  pick_waypoint3.orientation.z = 0.0;
  pick_waypoint3.orientation.w = 0.0;

  geometry_msgs::msg::Pose place_waypoint1;
  place_waypoint1.position.x = 0.7249;
  place_waypoint1.position.y = 0.125;
  place_waypoint1.position.z = 0.487;
  place_waypoint1.orientation.x = 0.707;
  place_waypoint1.orientation.y = -0.707;
  place_waypoint1.orientation.z = 0.0;
  place_waypoint1.orientation.w = 0.0;

  geometry_msgs::msg::Pose place_waypoint2;
  place_waypoint2.position.x = 0.84;
  place_waypoint2.position.y = 0.2;
  place_waypoint2.position.z = 0.12;
  place_waypoint2.orientation.x = 0.707;
  place_waypoint2.orientation.y = -0.707;
  place_waypoint2.orientation.z = 0.0;
  place_waypoint2.orientation.w = 0.0;

  // Plan motion sequence
  moveit_msgs::msg::MotionSequenceItem pick_item1;
  pick_item1.blend_radius = 0.0;

  pick_item1.req.group_name = PLANNING_GROUP;
  pick_item1.req.planner_id = PLANNER_ID;
  pick_item1.req.allowed_planning_time = 5.0;
  pick_item1.req.max_velocity_scaling_factor = VELOCITY_SCALING_FACTOR;
  pick_item1.req.max_acceleration_scaling_factor = ACCELERATION_SCALING_FACTOR;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
  pose_stamped.pose = pick_waypoint1;

  pick_item1.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("wrist3_Link"), pose_stamped));

  auto pick_item2 = pick_item1;
  pick_item2.req.goal_constraints.clear();
  pose_stamped.pose = pick_waypoint2;
  pick_item2.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("wrist3_Link"), pose_stamped));

  auto pick_item3 = pick_item1;
  pick_item3.req.goal_constraints.clear();
  pose_stamped.pose = pick_waypoint3;
  pick_item3.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("wrist3_Link"), pose_stamped));

  auto pick_service_request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
  pick_service_request->request.items.push_back(pick_item1);
  pick_service_request->request.items.push_back(pick_item2);
  pick_service_request->request.items.push_back(pick_item3);

  auto place_item1 = pick_item1;
  place_item1.req.goal_constraints.clear();
  pose_stamped.pose = place_waypoint1;
  place_item1.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("wrist3_Link"), pose_stamped));

  auto place_item2 = place_item1;
  place_item2.req.goal_constraints.clear();
  pose_stamped.pose = place_waypoint2;
  place_item2.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("wrist3_Link"), pose_stamped));

  auto place_service_request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
  place_service_request->request.items.push_back(place_item1);
  place_service_request->request.items.push_back(place_item2);

  // Cycle pick and place
  while (rclcpp::ok())
  {
    RCLCPP_INFO(LOGGER, "Planning pick sequence");
    auto pick_future = service_client->async_send_request(pick_service_request);
    auto pick_response = pick_future.get();

    moveit::planning_interface::MoveGroupInterface::Plan pick_plan;

    if (pick_response->response.error_code.val == moveit::core::MoveItErrorCode::SUCCESS)
    {
      pick_plan.trajectory = pick_response.get()->response.planned_trajectories[0];
    }

    move_group.execute(pick_plan);

    rclcpp::Rate(10).sleep();

    RCLCPP_INFO(LOGGER, "Planning place sequence");
    auto place_future = service_client->async_send_request(place_service_request);
    auto place_response = place_future.get();

    moveit::planning_interface::MoveGroupInterface::Plan place_plan;

    if (place_response->response.error_code.val == moveit::core::MoveItErrorCode::SUCCESS)
    {
      place_plan.trajectory = place_response.get()->response.planned_trajectories[0];
    }

    move_group.execute(place_plan);

    rclcpp::Rate(10).sleep();
  }

  rclcpp::shutdown();
  return 0;
}