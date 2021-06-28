﻿/**
 * @file rrt_star.hpp
 * @author ShieldQiQi
 * @brief Contains the RRTStar class
 */

#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <limits>

#include "utils/utils.hpp"

/**
 * @brief Class for RRT Star objects
 */
class RRTStar {
 public:
  /**
   * @brief Find the nearest Node that has been seen by the algorithm. This does
   * not consider cost to reach the node.
   * @param new_node Node to which the nearest node must be found
   * @return nearest node
   */
  Node FindNearestPoint(Node& new_node);

  /**
   * @brief Check if there is any obstacle between the 2 nodes. As this planner
   * is for grid maps, the obstacles are square.
   * @param n_1 Node 1
   * @param n_2 Node 2
   * @return bool value of whether obstacle exists between nodes
   */
  bool CheckObstacle(const Node& n_1, const Node& n_2) const;

  /**
   * @brief Generates a random node
   * @return Generated node
   */
  Node GenerateRandomNode() const;

  /**
   * @brief Rewire the tree
   * @param new_node Node to which other nodes will be connected if their cost
   * decreases
   * @return void
   */
  void Rewire(const Node& new_node);

  /**
   * @brief Main algorithm of RRT*
   * @param grid Main grid
   * @param start_in starting node
   * @param goal_in goal node
   * @param max_iter_x_factor Maximum number of allowable iterations before
   * returning no path
   * @param threshold_in Maximum distance per move
   * @return path vector of nodes
   */
  std::vector<Node> rrt_star(
      std::vector<std::vector<int>>& grid, const Node& start_in,
      const Node& goal_in, const int max_iter_x_factor = 500,
      const double threshold_in = std::numeric_limits<double>::infinity());

  /**
   * @brief Check if goal is reachable from current node
   * @param new_node Current node
   * @return bool value of whether goal is reachable from current node
   */
  bool CheckGoalVisible(const Node& new_node);

  /**
   * @brief Create the obstacle list from the input grid
   * @param grid Input grid for algorithm
   * @return void
   */
  void CreateObstacleList(std::vector<std::vector<int>>& grid);

 private:
  std::vector<Node> point_list_;
  std::vector<Node> obstacle_list_;
  std::vector<Node> near_nodes_;
  std::vector<double> near_nodes_dist_;
  Node start_, goal_;
  double threshold_ = 1;
  bool found_goal_ = false;
  int n = 0;
};

#endif  // RRT_STAR_H
