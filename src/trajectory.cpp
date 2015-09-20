#include "trajectory.h"
#include "mybag.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);
  moveit::planning_interface::MoveGroup group("right_arm");

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // some basic info
  std::cout << "Reference frame: " << group.getPlanningFrame() << std::endl;
  // by default - Reference frame: /odom_combined

  // We can also print the name of the end-effector link for this group.
  std::cout << "Reference frame: " << group.getEndEffectorLink() << std::endl;
  // by default - Reference frame: r_wrist_roll_link

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);
   moveit::planning_interface::MoveGroup::Plan my_plan;
  // bool success = group.plan(my_plan);
  // sleep(5.0);

  // for setting start state. (a bit lower than the default pose)
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  // ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(10.0);

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.
  std::vector<geometry_msgs::Pose> way_points;

  MyBag mybag;
  mybag.getWayPoints(way_points);


  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  // moveit_msgs::RobotTrajectory trajectory;
  // double fraction = group.computeCartesianPath(way_points,
  //                   0.01,  // eef_step
  //                   0.0,   // jump_threshold
  //                   trajectory);

  // ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
  //          fraction * 100.0);
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(15.0);

  ros::shutdown();
  return 0;
}
