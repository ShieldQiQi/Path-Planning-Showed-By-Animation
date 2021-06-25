/**
 * @file a_star.cpp
 * @author ShieldQiQi
 * @brief Contains the AStar class
 */

#include <cmath>

#include "a_star.h"

extern AStar new_a_star;
using namespace std;
extern std::vector<std::vector<int>> grid;
extern std::vector<std::vector<int>> costGrid;
extern MainWindow *w;

void CALLBACK TimerProcAstar(HWND hWnd, UINT nMsg, UINT nTimerid, DWORD dwTime)
{
    Q_UNUSED(hWnd);Q_UNUSED(nMsg);Q_UNUSED(nTimerid);Q_UNUSED(dwTime);

    const std::vector<Node> motion = GetMotion();

    // Main loop
    if(!new_a_star.doneFlag)
    {
        Node current = new_a_star.open_list_.top();
        new_a_star.open_list_.pop();
        current.id_ = current.x_ * new_a_star.n + current.y_;

        // find if current is the goal node
        if (CompareCoordinates(current, new_a_star.goal_))
        {
            new_a_star.closed_list_.push_back(current);
            grid[current.x_][current.y_] = 5;
            //return new_a_star.closed_list_;
            new_a_star.doneFlag = true;
            KillTimer(NULL, 1);
        }
        grid[current.x_][current.y_] = 5;
        w->update();

        // find all the neighbours and update their f(n)=g(n)+h(n)
        for (const auto& m : motion)
        {
            Node new_point;
            new_point = current + m;

            // update id and parent id
            new_point.id_ = new_a_star.n * new_point.x_ + new_point.y_;
            new_point.pid_ = current.id_;

            // the difference between Dijkstar and A-star Algorithm
            // f(n)=g(n)+h(n) where the g(n)=cost while h(n)=Manhattan distance of current node and goal node
            new_point.h_cost_ = std::abs(new_point.x_ - new_a_star.goal_.x_) + std::abs(new_point.y_ - new_a_star.goal_.y_);

            if (CompareCoordinates(new_point, new_a_star.goal_)){
                new_a_star.open_list_.push(new_point);
                break;
            }

            // Check boundaries
            if (checkOutsideBoundary(new_point, new_a_star.n)) {
                continue;}
            // obstacle or visited
            if (grid[new_point.x_][new_point.y_] == 1 || grid[new_point.x_][new_point.y_] == 6) {
              continue;}

            if(costGrid[new_point.x_][new_point.y_]>new_point.cost_)
            {
                // check the new_point if is already in open_list_
                // if new_point in open_list then
                new_a_star.open_list_.push(new_point);
                grid[new_point.x_][new_point.y_] = 2;
                costGrid[new_point.x_][new_point.y_] = new_point.cost_;
                // otherwise, update the cost of the previus one
                // Todo, not a serious problem, but would be better if is done
            }
        }

        new_a_star.closed_list_.push_back(current);
        grid[current.x_][current.y_] = 6;
    }else{
        new_a_star.closed_list_.clear();
        Node no_path_node(-1, -1, -1, -1, -1, -1);
        new_a_star.closed_list_.push_back(no_path_node);
        new_a_star.doneFlag = true;
        KillTimer(NULL, 1);
    }
}

std::vector<Node> AStar::a_star(std::vector<std::vector<int>>& grid, const Node& start_in, const Node& goal_in)
{
  start_ = start_in;
  goal_ = goal_in;
  n = grid.size();
  // Get possible motions

  open_list_.push(start_);

  SetTimer(NULL, 1, 50, (TIMERPROC)TimerProcAstar);

  return closed_list_;
}
