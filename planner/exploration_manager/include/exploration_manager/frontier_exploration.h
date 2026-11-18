#ifndef _FRONTIER_EXPLORATION_H_
#define _FRONTIER_EXPLORATION_H_

#include <Eigen/Eigen>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "exploration_manager/frontier_map.h"

namespace exploration_manager {

// Frontier exploration of a rectangular area at a fixed height.
//
// The planner's map publishes a horizontal slice at flight height
// (grid_map/slice); FrontierMap keeps it and finds the frontiers reachable
// through observed free space. The FSM sends the best frontier's goal to the
// planner, switches when that frontier has been observed or a much better one
// appears, blocks goals it makes no progress towards, and returns to the start
// when no reachable frontier is left (or the time limit is reached).
class FrontierExploration {
 public:
  void init(ros::NodeHandle& nh);

 private:
  enum State { INIT, WAIT_TRIGGER, EXPLORE, RETURN_HOME, FINISH };

  void fsmCallback(const ros::TimerEvent& e);
  void exploreState();
  void returnHomeState();
  void finishState();
  void transit(State s, const std::string& why);

  // Picks the frontier to fly to; returns false if there is none.
  bool selectGoal();
  void sendGoal(const Eigen::Vector2d& goal);
  // Tracks progress towards the goal; false once stuck for stuck_time_.
  bool makingProgress(double distance);

  void sliceCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void finishCallback(const std_msgs::EmptyConstPtr& msg);

  void publishState();
  void publishVisualisation();

  // Parameters
  Eigen::Vector2d area_min_, area_max_;
  double altitude_;
  double max_exploration_time_;
  double goal_tolerance_;     // m: goal reached
  double reach_radius_;       // m: a frontier goal (or the start) is reached within this
  double min_goal_distance_;  // m: nearer frontiers are not flown to (a forward camera cannot see
                              // them from here, and the planner handles very short hops poorly)
  double back_off_;           // m to move away when only near frontiers are left
  double min_goal_interval_;  // s between goal changes (unless the goal is reached or blocked)
  double goal_keep_radius_;   // m: the goal stays while a frontier is this close
  double switch_margin_;      // m of cost a new frontier must save to replace the goal
  double turn_weight_;        // m of cost per radian of heading change
  double size_weight_;        // m of cost saved per m of frontier (up to 2 m)
  double stuck_time_;         // s without progress before a goal is blocked
  double blocked_radius_;     // m around blocked goals
  int finish_confirm_;        // cycles without frontiers before finishing

  // State
  State state_ = INIT;
  std::vector<std::string> state_names_;
  FrontierMap map_;
  bool have_odom_ = false, have_slice_ = false, triggered_ = false;
  Eigen::Vector3d odom_pos_, odom_vel_;
  Eigen::Vector2d home_;
  bool have_goal_ = false;
  bool backing_off_ = false;  // not a frontier goal (backing off or heading past near frontiers)
  Eigen::Vector2d goal_;
  double best_distance_;
  ros::Time progress_time_;
  ros::Time last_goal_time_;
  ros::Time start_time_;
  std::vector<Eigen::Vector2d> blocked_;
  int no_frontier_cycles_ = 0;
  int goals_sent_ = 0;

  ros::Timer fsm_timer_;
  ros::Subscriber slice_sub_, odom_sub_, trigger_sub_, finish_sub_;
  ros::Publisher goal_pub_, state_pub_, coverage_pub_, map_pub_, frontier_pub_;
};

}  // namespace exploration_manager

#endif
