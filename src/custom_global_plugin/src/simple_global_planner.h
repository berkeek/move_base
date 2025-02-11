#ifndef SIMPLE_GLOBAL_PLANNER_H_
#define SIMPLE_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace simple_global_planner
{

class SimpleGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  // Default constructor (usually required for pluginlib)
  SimpleGlobalPlanner();
  
  // Overloaded constructor
  SimpleGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  // Initialization function
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  
  /**
   * @brief Given a start and goal in the world, compute a straight-line plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by this function
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
};

} // end namespace

#endif  // SIMPLE_GLOBAL_PLANNER_H_
