#include "director.h"

#include "std_msgs/Header.h"

#include "utils.h"

namespace exploration {
namespace {

/**
 * @brief The maximum distance we allow our goal to drift, in meters, before
 * we cancel it and set a new one.
 */
constexpr double kMaxGoalDrift = 2.0;
/**
 * @brief How many cycles we'll stick with a goal before trying to recompute it.
 */
constexpr uint8_t kMinCyclesBeforeGoalRecompute = 30;

} // namespace

using geometry_msgs::PoseStamped;
using move_base_msgs::MoveBaseGoal;

Director::Director(GoalFinder *goal_finder, MoveBaseClient *client)
    : goal_finder_(goal_finder), client_(client) {}

void Director::UpdateMap(const OccupancyGridConstPtr &map) {
  ROS_INFO_STREAM("Updating map.");
  current_map_ = map;

  if (!has_a_goal_) {
    // We don't have to check the current goal drift, because there is no
    // current goal.
    return;
  }

  if (++cycles_since_goal_compute_ < kMinCyclesBeforeGoalRecompute) {
    // We haven't waited long enough before recomputing this goal.
    return;
  }

  // Compute a new goal for this map.
  const auto &kNewGoal = goal_finder_->FindNewGoal(current_map_, current_pose_);
  cycles_since_goal_compute_ = 0;
  if (EuclideanDistance(kNewGoal.position, current_goal_.position) >
      kMaxGoalDrift) {
    // Our goal has moved to far. Cancel the current goal.
    ROS_INFO_STREAM("Goal has drifted by more than " << kMaxGoalDrift
                                                     << "m; cancelling.");
    client_->cancelGoal();
    has_a_goal_ = false;
  }
}

void Director::UpdatePose(const MoveBaseActionFeedbackConstPtr &pose) {
  ROS_INFO_STREAM("Updating robot pose.");
  current_pose_ = pose->feedback.base_position.pose;
}

void Director::UpdateStatus() {
  ROS_DEBUG_STREAM("Checking goal status.");

  if (current_map_ == nullptr) {
    // Can't do much without a map.
    ROS_INFO_STREAM("Waiting for map...");
    return;
  }

  if (!has_a_goal_) {
    ROS_INFO_STREAM("Sending a new goal.");

    // Update the goal and send a new one if we don't have one.
    current_goal_ = goal_finder_->FindNewGoal(current_map_, current_pose_);
    has_a_goal_ = true;

    // Publish the goal.
    MoveBaseGoal goal;
    // Make sure the proper frame is set.
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = current_goal_;
    goal.target_pose.pose.orientation.w = 1.0;

    client_->sendGoal(goal);
  }

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

} // namespace exploration
