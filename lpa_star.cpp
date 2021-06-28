﻿/**
 * @file lpa_star.cpp
 * @author ShieldQiQi
 * @brief Contains the LPAStar class
 */
#include "lpa_star.h"

#include <algorithm>
#include <chrono>
#include <iomanip>  // TODO(vss): replace setw
#include <iostream>
#include <random>
#include <thread>

// constants
constexpr int obs_found_pause_time = 500;  // ms

void LPAStar::VectorInsertionSort(std::vector<Node>& v) {
  for (auto it = v.begin(); it != v.end(); ++it) {
    auto const insertion_point = std::upper_bound(
        v.begin(), it, *it, [&](const Node& lhs, const Node& rhs) {
          return lhs.cost_ + lhs.h_cost_ < rhs.cost_ + rhs.h_cost_;
        });
    std::rotate(insertion_point, it, it + 1);
  }
}

double LPAStar::GetHeuristic(const Node& s1, const Node& s2) {
  return abs(s1.x_ - s2.x_) + abs(s1.y_ - s2.y_);
}
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void LPAStar::PrintGRHS() const {
  std::cout << "G values:" << '\n';
  for (const auto& row : S_) {
    for (const auto& ele : row) {
      std::cout << std::setw(5) << ele.first << ",";
    }
    std::cout << '\n';
  }
  std::cout << "RHS values:" << '\n';
  for (const auto& row : S_) {
    for (const auto& ele : row) {
      std::cout << std::setw(5) << ele.second << ",";
    }
    std::cout << '\n';
  }
}
#endif

std::pair<double, double> LPAStar::CalculateKey(const Node& s) const {
  return std::make_pair(std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second) +
                            GetHeuristic(goal_, s),
                        std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second));
}

std::vector<Node> LPAStar::GetPred(const Node& u) const {
  std::vector<Node> pred;
  for (const auto& m : motions) {
    // Modify to prevent points already in the queue fro  being added?
    Node new_node = u + m;
    if (!checkOutsideBoundary(new_node, n) &&
        grid[new_node.x_][new_node.y_] != 1) {
      pred.push_back(new_node);
    }
  }
  return pred;
}

std::vector<Node> LPAStar::GetSucc(const Node& u) const {
  std::vector<Node> succ;
  for (const auto& m : motions) {
    Node new_node = u + m;
    if (!checkOutsideBoundary(new_node, n) &&
        grid[new_node.x_][new_node.y_] != 1) {
      succ.push_back(new_node);
    }
  }
  return succ;
}

void LPAStar::InsertionSort() {
  typedef std::pair<Node, std::pair<double, double>> lazy_type;
  for (auto it = U_.begin(); it != U_.end(); ++it) {
    auto upper_bound = std::upper_bound(
        U_.begin(), it, *it, [&](const lazy_type& lhs, const lazy_type& rhs)
        {
          // Entries are ordered by k1 (which corresponds directly to the f-values used in A*), then by k2.
          return lhs.second.first < rhs.second.first ||
                 (lhs.second.first == rhs.second.first &&
                  lhs.second.second < rhs.second.second);
        }
    );
    std::rotate(upper_bound, it, it + 1);
  }
}

double LPAStar::C(const Node& s1, const Node& s2) const {
  if (!checkOutsideBoundary(s1, n) && !checkOutsideBoundary(s2, n) &&
      grid[s1.x_][s1.y_] != 1 && grid[s2.x_][s2.y_] != 1) {
    // Node diff = s2-s1;
    // for(auto it = motions.begin(); it!=motions.end(); ++it){
    //   if(diff == *it){
    //     return (*it).cost_;
    //   }
    // }
    return 1;
  }
  return n * n;
}

void LPAStar::Init() {
  U_.clear();
  motions = GetMotion();

  double n2 = n * n;
  // regard the n^2 as infinity
  large_num = std::make_pair(n2, n2);

  std::vector<std::pair<double, double>> tmp(n, large_num);
  S_ = std::vector<std::vector<std::pair<double, double>>>(n, tmp);

  // set the rhs(start) as 0
  S_[start_.x_][start_.y_].second = 0;
  std::pair<Node, std::pair<double, double>> u_pair =
      std::make_pair(start_, CalculateKey(start_));
  InsertionSort();

  // initially, only the start node pushed in the priority queue
  U_.push_back(u_pair);
}

void LPAStar::UpdateVertex(const Node& u) {
  if (!CompareCoordinates(u, start_)) {
    std::vector<Node> pred = GetPred(u);
    double init_min = n * n;

    // update: find min in all the {g(n')+c(n',n)}
    for (const auto& p : pred) {
      init_min = std::min(init_min, S_[p.x_][p.y_].first + C(u, p));
    }
    // uodate the rhs(node)
    S_[u.x_][u.y_].second = init_min;
  }
  // can optimise following by using hash
  // if there exists same node in queue, we should erase it then make a sort
  for (auto it = U_.begin(); it != U_.end(); ++it) {
    if (CompareCoordinates((*it).first, u)) {
      U_.erase(it);
      break;
    }
  }
  // if g(n) != rhs(n), then push into priority queue
  if (S_[u.x_][u.y_].first != S_[u.x_][u.y_].second) {
    U_.emplace_back(std::make_pair(u, CalculateKey(u)));
    InsertionSort();
  }
}

bool LPAStar::CompareKey(const std::pair<double, double>& pair_in,
                         const Node& u) const {
  std::pair<double, double> node_key = CalculateKey(u);
  return pair_in.first < node_key.first ||
         (pair_in.first == node_key.first && pair_in.second < node_key.second);
}

// node expansion step
int LPAStar::ComputeShortestPath() {
    // find if the iteration is finished
    while ((!U_.empty() && CompareKey(U_[0].second, goal_)) || S_[goal_.x_][goal_.y_].first != S_[goal_.x_][goal_.y_].second)
    {
        // PrintGRHS();
        // if not, continue iteration
        Node u = U_[0].first;
        U_.erase(U_.begin());

        // case of over consistent
        if (S_[u.x_][u.y_].first > S_[u.x_][u.y_].second) {
          S_[u.x_][u.y_].first = S_[u.x_][u.y_].second;
          std::vector<Node> succ = GetSucc(u);
          for (const auto& s : succ) {
            UpdateVertex(s);
          }
        } else {
        // case of underconsitent
          S_[u.x_][u.y_].first = n * n;
          std::vector<Node> succ = GetSucc(u);
          for (const auto& s : succ) {
            UpdateVertex(s);
          }
          UpdateVertex(u);
        }
    }
    // no solution founded
    if (S_[goal_.x_][goal_.y_] == large_num) {
    return -1;
    }
    return 0;
}

std::vector<Node> LPAStar::lpa_star(std::vector<std::vector<int>>& grid_in,
                                    const Node& start_in, const Node& goal_in,
                                    const int max_iter_in,
                                    const bool obs_creation) {
  max_iter_ = max_iter_in;
  grid = grid_in;
  start_ = start_in;
  goal_ = goal_in;
  n = grid.size();

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range
  Init();

  int ans = ComputeShortestPath();

  // no solution founded
  if (ans < 0 || S_[start_.x_][start_.y_].first == large_num.first) {
    path_vector_.clear();
    Node no_path_node(-1, -1, -1, -1, -1);
    path_vector_.push_back(no_path_node);
    grid_in = grid;
    RemovePathFromGrid(grid_in);
    return path_vector_;
  }
  // generate path vector
  GeneratePathVector();

  // simulate if there is a cost-change and uodate the soluiton accordingly
  while (iter_ < max_iter_)
  {
    if (distr(eng) > n - 2) {
      int rand = static_cast<int>(
          distr(eng) *
          (path_vector_.size() / (n - 1)));  // Scaling along path so any point
                                             // on path could become an obstacle
      // generate a random obstacle in the pathVector thus a uopdate is needed
      Node new_obs = path_vector_[rand];
      std::vector<Node> succ = GetSucc(new_obs);
      if (obs_creation) {
        SetObs(new_obs);
      }
      // thus a update is needed
      for (const auto& s : succ) {
        UpdateVertex(s);
      }
      UpdateVertex(new_obs);
    }
    // recompuate the path_vector
    int ans = ComputeShortestPath();
    if (ans < 0 || S_[start_.x_][start_.y_].first == large_num.first) {
      path_vector_.clear();
      Node no_path_node(-1, -1, -1, -1, -1);
      path_vector_.push_back(no_path_node);
      grid_in = grid;
      RemovePathFromGrid(grid_in);
      return path_vector_;
    }
    GeneratePathVector();
    iter_++;
  }

  grid_in = grid;
  RemovePathFromGrid(grid_in);
  return path_vector_;
}

void LPAStar::RemovePathFromGrid(std::vector<std::vector<int>>& grid_in) const {
  for (auto& row : grid_in) {
    for (auto& ele : row) {
      if (ele == 2) {
        ele = 0;
      }
    }
  }
}

void LPAStar::SetObs(const Node& u) {
  // PrintGrid(grid,n); // Uncomment if you want to see old and new path
  if (CompareCoordinates(u, goal_) || CompareCoordinates(u, start_)) {
    std::cout << "Cannot set current start or goal as obstacle" << '\n';
  } else {
    std::cout << "Current grid and path: " << '\n';
    PrintGrid(grid);
    grid[u.x_][u.y_] = 1;
    std::cout << "Obstacle found at: " << '\n';
    u.PrintStatus();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(obs_found_pause_time));
  }
}

void LPAStar::GeneratePathVector() {
  path_vector_.clear();
  goal_.cost_ = S_[goal_.x_][goal_.y_].second;
  path_vector_.push_back(goal_);
  while (!CompareCoordinates(path_vector_[0], start_)) {
    Node u = path_vector_[0];
    grid[u.x_][u.y_] = 2;
    for (const auto& motion : motions) {
      Node new_node = u + motion;
      if (checkOutsideBoundary(new_node, n) ||
          grid[new_node.x_][new_node.y_] == 1) {
        continue;
      }
      if (new_node.x_ < n && new_node.x_ >= 0 && new_node.y_ < n &&
          new_node.y_ >= 0) {
        new_node.cost_ = S_[new_node.x_][new_node.y_].second;
        if (new_node.cost_ < u.cost_) {
          new_node.id_ = n * new_node.x_ + new_node.y_;
          path_vector_[0].pid_ = new_node.id_;
          path_vector_.push_back(new_node);
          VectorInsertionSort(path_vector_);
          break;  // So that only one of the predecessors is added to the queue
        }
      }
    }
  }
  if (CompareCoordinates(path_vector_[0], goal_)) {
    grid[goal_.x_][goal_.y_] = 2;
  }
}

#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void LPAStar::DisplayGrid() const {
  std::cout << "Grid: " << '\n'
            << "1. Points not considered ---> 0" << '\n'
            << "2. Obstacles             ---> 1" << '\n'
            << "3. Points considered     ---> 2" << '\n'
            << "4. Points in final path  ---> 3" << '\n'
            << "5. Current point         ---> 4" << '\n';
  for (int j = 0; j < n; j++) {
    std::cout << "---";
  }
  std::cout << '\n';
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (grid[i][j] == 3)
        std::cout << GREEN << grid[i][j] << RESET << " , ";
      else if (grid[i][j] == 1)
        std::cout << RED << grid[i][j] << RESET << " , ";
      else if (grid[i][j] == 2)
        std::cout << BLUE << grid[i][j] << RESET << " , ";
      else if (grid[i][j] == 4)
        std::cout << YELLOW << grid[i][j] << RESET << " , ";
      else
        std::cout << grid[i][j] << " , ";
    }
    std::cout << '\n' << '\n';
  }
  for (int j = 0; j < n; j++) std::cout << "---";
  std::cout << '\n';
}
#endif

#ifdef BUILD_INDIVIDUAL
/**
 * @brief Script main function. Generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main() {
  int n = 11;
  bool obs_creation = true;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n));
  MakeGrid(grid);
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Node start(distr(eng), distr(eng), 0, 0, 0, 0);
  Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);
  std::vector<Node> path_vector;
  LPAStar new_lpa_star;
  path_vector = new_lpa_star.lpa_star(grid, start, goal, n, obs_creation);
  PrintPath(path_vector, start, goal, grid);
}
#endif  // BUILD_INDIVIDUAL
