/**
 * @file dijkstra.h
 * @author ShieldQiQi
 * @brief Contains the Dijkstra class
 */
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <queue>

// self-defined universal lib
#include "utils/utils.hpp"
#include "mainwindow.h"
#include <Windows.h>

/**
 * @brief Class for Dijkstra objects
 */
class Dijkstra
{

 public:
  /**
   * @brief Main algorithm of Dijstra.
   */
  std::vector<Node> doDijkstra(std::vector<std::vector<int>>& grid,
                             const Node& start_in, const Node& goal_in);

  friend void CALLBACK TimerProc(HWND hWnd, UINT nMsg, UINT nTimerid, DWORD dwTime);

  std::vector<Node> closed_list_;

  bool doneFlag = false;

 private:

  // define priority_queue to speed up the dijkstra algorithm
  // the Node class represent the vertex, and it contain id, parent id, position(x,y), cost, h_cost
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;

//  std::vector<Node> closed_list_;
  Node start_, goal_;
  int n;

};

#endif // DIJKSTRA_H
