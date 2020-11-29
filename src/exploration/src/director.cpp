#include "director.h"

#include "std_msgs/Header.h"

namespace exploration {

using geometry_msgs::PoseStamped;
using move_base_msgs::MoveBaseGoal;

Director::Director(GoalFinder *goal_finder, MoveBaseClient *client)
    : goal_finder_(goal_finder), client_(client) {}

void Director::UpdateMap(const OccupancyGridConstPtr &map) {
  ROS_ERROR_STREAM("Updating map.");

  // Determine a new goal.
  const auto kNextGoal = goal_finder_->FindNewGoal(map, current_pose_);

  // Publish the goal.
  MoveBaseGoal goal;
  // Make sure the proper frame is set.
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose = kNextGoal;
  goal.target_pose.pose.orientation.w = 1.0;

  client_->sendGoal(goal);
}

void Director::UpdatePose(const PoseWithCovarianceStampedConstPtr &pose) {
  ROS_ERROR_STREAM("Updating robot pose.");
  current_pose_ = pose->pose.pose;
}

} // namespace exploration
