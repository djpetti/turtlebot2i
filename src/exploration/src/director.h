#ifndef TURTLEBOT_DIRECTOR_H
#define TURTLEBOT_DIRECTOR_H

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include "goal_finder.h"

using geometry_msgs::Pose;
using move_base_msgs::MoveBaseAction;
using move_base_msgs::MoveBaseActionFeedbackConstPtr;
using nav_msgs::OccupancyGridConstPtr;

namespace exploration {

/**
 * @brief Provides high-level control of the exploration process.
 */
class Director {
public:
  using MoveBaseClient = actionlib::SimpleActionClient<MoveBaseAction>;

  /**
   * @param goal_finder The GoalFinder instance to use for finding
   *    exploration goals.
   * @param client The ActionClient to use for sending new movement goals.
   */
  Director(GoalFinder *goal_finder, MoveBaseClient *client);

  /**
   * @brief Updates the current map, and recomputes the exploration targets.
   * @param map The new map data.
   */
  void UpdateMap(const OccupancyGridConstPtr &map);

  /**
   * @brief Updates the current robot pose.
   * @param pose The new pose data.
   */
  void UpdatePose(const MoveBaseActionFeedbackConstPtr &pose);

  /**
   * @brief Checks the status of the current goal and responds accordingly.
   * This should be called periodically during operation.
   */
  void UpdateStatus();

private:
  /// The internal GoalFinder that we use.
  GoalFinder *goal_finder_;
  /// The internal ActionClient we use to send movement goals.
  MoveBaseClient *client_;

  /// The most recent robot pose.
  Pose current_pose_{};
  /// The most recent map.
  OccupancyGridConstPtr current_map_ = nullptr;
  /// The current goal that we're moving towards.
  Pose current_goal_{};

  /// Whether we are currently moving towards a goal.
  bool has_a_goal_ = false;
  /// Number of cycles since we last recomputed the goal.
  uint8_t cycles_since_goal_compute_ = 0;
};

} // namespace exploration

#endif // TURTLEBOT_DIRECTOR_H
