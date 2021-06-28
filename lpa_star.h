﻿/**
 * @file lpa_star.hpp
 * @author ShieldQiQi
 * @brief Contains the LPAStar class
 */

#ifndef LPA_STAR_H
#define LPA_STAR_H

#include "utils/utils.hpp"

/**
 * @brief Class for LPAStar objects
 */
class LPAStar {
 public:
  /**
   * @brief Using insertion sort to sort the vector list.
   * @param v The vector to be sorted
   * @return void
   */
  static void VectorInsertionSort(std::vector<Node>& v);

  /**
   * @brief Calculate and return the heuristic distance between 2 nodes
   * @param s1 Node 1
   * @param s2 Node 2
   * @return Heuritic distance between the 2 nodes
   */
  static double GetHeuristic(const Node& s1, const Node& s2);

  /**
   * @brief Displays the G and RHS values for the entire grid.
   * @return void
   */
  void PrintGRHS() const;

  /**
   * @brief Returns the key (pair) values for a given node.
   * @param s Node whose key values are to be calcualted
   * @return Key for given input node
   */
  std::pair<double, double> CalculateKey(const Node& s) const;

  /**
   * @brief Returns the possible predecessors of a given node, based on allowed
   * motion primatives
   * @param u Node
   * @return Vector of nodes that are possible predecessors to input node
   */
  std::vector<Node> GetPred(const Node& u) const;

  /**
   * @brief Returns the possible successors of a given node, based on allowed
   * motion primatives
   * @param u Node
   * @return Vector of nodes that are possible successors to input node
   */
  std::vector<Node> GetSucc(const Node& u) const;

  /**
   * @brief Using insertion sort to sort the vector list that maintains the
   * priority queue. Good for a mostly sorted queue. Sort called after every
   * insertion to maintain queue. Not using standard queue as iterating over is
   * not allowed.
   * @return void
   */
  void InsertionSort();

  /**
   * @brief Returns the cost of motion moving from one node to another, based on
   * allowed motion primatives. Currently set to 1 for speed up given only 4
   * motions permitted, at constant cost. Change as required, based on permitted
   * motions.
   * @param s1 Node
   * @param s2 Node
   * @return cost of motion moving from first node to second
   */
  double C(const Node& s1, const Node& s2) const;

  /**
   * @brief Initialisation function of LPA*. Initialises G and RHS values for
   * all nodes, store motion allowable primatives, km value and the first value
   * of the priority queue.
   * @return void
   */
  void Init();

  /**
   * @brief Update vertex procedure as per LPA* algorithm, Figure 3.
   * @param u Node on which UpdateVertex has to be called
   * @return void
   */
  void UpdateVertex(const Node& u);

  /**
   * @brief Compare keys function for LPA*. Compares the key given as input with
   * the key values of the node given as input. Calls CalculateKey on the node.
   * @param pair_in Key pair
   * @param u Node whose key values will be calculated and compared to above key
   * pair
   * @return bool value based on comparison of key values
   */
  bool CompareKey(const std::pair<double, double>& pair_in,
                  const Node& u) const;

  /**
   * @brief ComputeShortestPath procedure as per LPA* algorithm, Figure 3.
   * @return void
   */
  int ComputeShortestPath();

  /**
   * @brief Main algorithm of LPA*
   * @param grid_in Main grid
   * @param start_in starting node
   * @param goal_in goal node
   * @param max_iter_in number of oterations to run of LPA*
   * @return path vector of nodes
   */
  std::vector<Node> lpa_star(std::vector<std::vector<int>>& grid_in,
                             const Node& start_in, const Node& goal_in,
                             const int max_iter_in,
                             const bool obs_creation = true);

  /**
  * @brief Replan route, called whenever a previously unknown obstacle is
  detected. Equivalent of the effects of the code after an edge change is
  detectedd in the while loop within the main procedure of LPA*.
  * @param u Node at which the change was detected
  * @param obs_creation Allow creation of obstacles
  * @return path vector of nodes
  */
  std::vector<Node> Replan(const Node& u);

  /**
   * @brief Create an obstacle on input node. Does not allow start or goal to be
   * declared an obstacle. Prints out the obstacle if created and displays the
   * grid. Calls Replan function.
   * @param u Node at which obstacle is to be created
   * @return path vector of nodes
   */
  void SetObs(const Node& u);

  /**
   * @brief Generate the path vector and set the appropriate grid values
   * @return void
   */
  void GeneratePathVector();

  /**
   * @brief Update the starting point of the algorithm. Created to be
   * independant, used by RunDStarLite function to update the position of the
   * bot. If using independantly, uncomment the commented section within the
   * function. Letting it return path_vector_ as that is required for
   * independent run.
   * @param start_in new starting position
   * @return Path vector of nodes. Can be made to void, but left as path vector
   * to allow independent call.
   */
  std::vector<Node> UpdateStart(const Node& start_in) const;

  /**
   * @brief Displays the grid stored by the LPAStar object.
   * @return void
   */
  void DisplayGrid() const;

  /**
   * @brief Removes path from grid
   * @return void
   */
  void RemovePathFromGrid(std::vector<std::vector<int>>& grid_in) const;

  std::vector<Node> path_vector_;

 private:
  Node start_, goal_;
  std::vector<std::vector<std::pair<double, double>>> S_;
  std::vector<std::pair<Node, std::pair<double, double>>> U_;
  std::vector<Node> motions;
  int n, iter_ = 0, max_iter_;

  std::pair<double, double> large_num;
  std::vector<std::vector<int>> grid;
};

#endif  // LPA_STAR_H
