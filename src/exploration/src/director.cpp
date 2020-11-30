#include "director.h"

#include "std_msgs/Header.h"

namespace exploration {

using geometry_msgs::PoseStamped;
using move_base_msgs::MoveBaseGoal;

Director::Director(GoalFinder *goal_finder, MoveBaseClient *client)
    : goal_finder_(goal_finder), client_(client) {}

void Director::UpdateMap(const OccupancyGridConstPtr &map) {
  ROS_ERROR_STREAM("Updating map.");

  if (has_a_goal_) {
    // Check up on the status of the current goal.
    const auto action_state = client_->getState();
    if (action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE) {
      ROS_INFO_STREAM("Previous goal is still active; not updating.");
      return;
    } else if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO_STREAM("Reached exploration goal.");
      has_a_goal_ = false;
    } else {
      ROS_ERROR_STREAM("Exploration goal appears to be unreachable.");
      // Mark this cell as unreachable so we don't try to go there again.
      goal_finder_->MarkGoalUnreachable(current_goal_);
      has_a_goal_ = false;
    }
  }

  // Determine a new goal.
  current_goal_ = goal_finder_->FindNewGoal(map, current_pose_);
  has_a_goal_ = true;

  // Publish the goal.
  MoveBaseGoal goal;
  // Make sure the proper frame is set.
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose = current_goal_;
  goal.target_pose.pose.orientation.w = 1.0;

  client_->sendGoal(goal);
}

void Director::UpdatePose(const PoseWithCovarianceStampedConstPtr &pose) {
  ROS_ERROR_STREAM("Updating robot pose.");
  current_pose_ = pose->pose.pose;
}

} // namespace exploration
