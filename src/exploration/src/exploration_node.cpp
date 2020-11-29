// Main entry point for exploration node.

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

#include "director.h"
#include "goal_finder.h"
#include "map_manager.h"

using geometry_msgs::PoseStamped;
using nav_msgs::OccupancyGrid;
using nav_msgs::OccupancyGridConstPtr;

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle node;

  ROS_INFO_STREAM("Starting exploration node...");

  // Initialize an action client to send movement goals.
  exploration::Director::MoveBaseClient client("move_base", true);
  while (!client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up.");
  }

  exploration::MapManager manager;
  exploration::GoalFinder goal_finder(&manager);
  exploration::Director director(&goal_finder, &client);

  // Subscribe to map updates from the navigation system.
  ros::Subscriber map_subscriber =
      node.subscribe("map", 1, &exploration::Director::UpdateMap, &director);
  // Subscribe to pose updates from the navigation system.
  ros::Subscriber pose_subscriber =
      node.subscribe("rtabmap/localization_pose", 1,
                     &exploration::Director::UpdatePose, &director);

  ros::spin();

  return 0;
}
