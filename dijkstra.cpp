/**
 * @file dijkstra.cpp
 * @author ShieldQiQi
 * @brief Contains the Dijkstra class
 */

#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL

#include "dijkstra.h"
#include <QDebug>
#include <math.h>

using namespace std;
extern std::vector<std::vector<int>> grid;
extern std::vector<std::vector<int>> costGrid;
extern Dijkstra new_dijkstra;
extern MainWindow *w;

void CALLBACK TimerProc(HWND hWnd, UINT nMsg, UINT nTimerid, DWORD dwTime)
{
    Q_UNUSED(hWnd);Q_UNUSED(nMsg);Q_UNUSED(nTimerid);Q_UNUSED(dwTime);

    if(!new_dijkstra.doneFlag)
    {
        std::vector<Node> motion = GetMotion();
        if (!new_dijkstra.open_list_.empty())
        {
            // get the minist-cost node as a new one in each iteration
            Node current = new_dijkstra.open_list_.top();
            new_dijkstra.open_list_.pop();
            // assign the order as the id
            current.id_ = current.x_ * new_dijkstra.n + current.y_;

            // find if current is the goal node
            if (CompareCoordinates(current, new_dijkstra.goal_))
            {
                // if yes, than the work is done and exit
                new_dijkstra.closed_list_.push_back(current);
                grid[current.x_][current.y_] = 5;
                new_dijkstra.doneFlag = true;
                KillTimer(NULL, 0);
                //return closed_list_;
            }
            grid[current.x_][current.y_] = 5;  // Point opened
            w->update();

            // find all the neighbours and update their distance to the original node
            for (const auto& m : motion)
            {
                Node new_point;
                new_point = current + m;

                new_point.id_ = new_dijkstra.n * new_point.x_ + new_point.y_;
                new_point.pid_ = current.id_;

                if (CompareCoordinates(new_point, new_dijkstra.goal_)){
                    new_dijkstra.open_list_.push(new_point);
                    break;
                }
                // Check boundaries
                if (new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= new_dijkstra.n || new_point.y_ >= new_dijkstra.n)
                    continue;
                // obstacle or visited
                // there should be "grid[new_point.x_][new_point.y_] == 1 || grid[new_point.x_][new_point.y_] == 6" in the case that the cost between
                // every two nodes is not equal!!!
                qDebug()<<grid[new_point.x_][new_point.y_];
                if (grid[new_point.x_][new_point.y_] == 1 || grid[new_point.x_][new_point.y_] == 6)
                    continue;

                // push all the mearsured node into a priority queue which the minist-cost one will be in the top
                if(costGrid[new_point.x_][new_point.y_]>new_point.cost_)
                {
                    // check the new_point if is already in open_list_
                    // if new_point in open_list then
                    new_dijkstra.open_list_.push(new_point);
                    grid[new_point.x_][new_point.y_] = 2;
                    costGrid[new_point.x_][new_point.y_] = new_point.cost_;
                    // otherwise, update the cost of the previus one
                    // Todo, not a serious problem, but would be better if is done
                }
            }
            new_dijkstra.closed_list_.push_back(current);
            grid[current.x_][current.y_] = 6;
        }else{
            // no solution founded
            new_dijkstra.closed_list_.clear();
            Node no_path_node(-1, -1, -1, -1, -1, -1);
            new_dijkstra.closed_list_.push_back(no_path_node);
            //return new_dijkstra.closed_list_;
            new_dijkstra.doneFlag = true;
        }
    }

}


std::vector<Node> Dijkstra::doDijkstra(std::vector<std::vector<int>>& grid,
                                     const Node& start_in,
                                     const Node& goal_in)
{
    start_ = start_in;
    goal_ = goal_in;
    n = grid.size();

    // Get possible motions--> right, down and left, up
//    std::vector<Node> motion = GetMotion();
    // set the start_ node as the current vertex
    open_list_.push(start_);
    //qDebug()<<open_list_.top().cost_<<" "<<open_list_.top().h_cost_;

    // Main loop

    SetTimer(NULL, 0, 50, (TIMERPROC)TimerProc);

//    while (!open_list_.empty())
//    {
//        // get the minist-cost node as a new one in each iteration
//        Node current = open_list_.top();
//        open_list_.pop();
//        // assign the order as the id
//        current.id_ = current.x_ * n + current.y_;

//        // find if current is the goal node
//        if (CompareCoordinates(current, goal_))
//        {
//            // if yes, than the work is done and exit
//            closed_list_.push_back(current);
//            grid[current.x_][current.y_] = 2;
//            KillTimer(NULL, 0);
//            return closed_list_;
//        }
//        grid[current.x_][current.y_] = 2;  // Point opened

//        // find all the neighbours and update their distance to the original node
//        for (const auto& m : motion)
//        {
//            Node new_point;
//            //qDebug()<<new_point.cost_<<" "<<new_point.h_cost_;
//            //qDebug()<<m.cost_<<" "<<m.h_cost_;
//            new_point = current + m;
//            //qDebug()<<new_point.cost_<<" "<<new_point.h_cost_;

//            new_point.id_ = n * new_point.x_ + new_point.y_;
//            new_point.pid_ = current.id_;

//            if (CompareCoordinates(new_point, goal_))
//            {
//                open_list_.push(new_point);
//                break;
//            }
//            // Check boundaries
//            if (new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n)
//            {
//                continue;
//            }
//            // obstacle or visited
//            if (grid[new_point.x_][new_point.y_] != 0)
//            {
//                continue;
//            }
//            // push all the mearsured node into a priority queue which the minist-cost one will be in the top
//            open_list_.push(new_point);
//        }
//        closed_list_.push_back(current);
//        // update the grid to get a dynamic-view

//    }

    //KillTimer(NULL, 0);

//    // no solution founded
//    closed_list_.clear();
//    Node no_path_node(-1, -1, -1, -1, -1, -1);
//    closed_list_.push_back(no_path_node);
    return closed_list_;
}
