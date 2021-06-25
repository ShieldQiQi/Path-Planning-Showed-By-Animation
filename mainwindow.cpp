#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "dijkstra.h"
#include "a_star.h"
#include <iostream>
#include <random>
#include <QPainter>
#include <QVector>
#include <QPoint>
#include <QLine>

#include <QDebug>

#define A_STAR 1

constexpr int n = 21;
std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
std::vector<std::vector<int>> costGrid(n, std::vector<int>(n, 100000));
Node start(20, 0, 0, 0, 0, 0);
Node goal(7, 20, 0, 0, 0, 0);
std::vector<Node> path_vector;

Dijkstra new_dijkstra;
AStar new_a_star;
PaintWidget* graphWidget;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

//    Dijkstra new_dijkstra;

    graphWidget = new PaintWidget(nullptr);
    graphWidget->setGeometry(0,0,840,900);
    graphWidget->setStyleSheet("background-color: rgb(0, 0, 0);");


    // get a random grid graph
    MakeGrid(grid);

//    std::random_device rd;   // obtain a random number from hardware
//    std::mt19937 eng(rd());  // seed the generator
//    std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

//    Node start(distr(eng), distr(eng), 0, 0, 0, 0);
//    Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

    start.id_ = start.x_ * n + start.y_;
    start.pid_ = start.x_ * n + start.y_;
    goal.id_ = goal.x_ * n + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
    // Make sure start and goal are not obstacles and their ids are correctly
    // assigned.
    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;
    //PrintGrid(grid);

    // Store points after algorithm has run
    std::vector<std::vector<int>> main_grid = grid;

    //grid = main_grid;
#ifdef DIJKSTRA
    new_dijkstra.doDijkstra(grid, start, goal);
#elif defined (A_STAR)
    new_a_star.a_star(grid, start, goal);
#elif defined (D_STAR)

#elif defined (RRT)

#elif defined (PRM)

#endif

    // use QPainter draw graph on widget
//    path_vector = new_dijkstra.closed_list_;
//    PrintPath(path_vector, start, goal, grid);

    update();
}

void PaintWidget::paintEvent(QPaintEvent *event)
{
#ifdef DIJKSTRA
    path_vector = new_dijkstra.closed_list_;
#elif defined (A_STAR)
    path_vector = new_a_star.closed_list_;
#elif defined (D_STAR)

#elif defined (RRT)

#elif defined (PRM)

#endif

    PrintPath(path_vector, start, goal, grid);

    Q_UNUSED(event);
    QPainter painter(this);
    QPen pen;
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);

    // draw the widget border
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(1);
    pen.setBrush(Qt::red);
    painter.setPen(pen);
    painter.drawRect(0,0,839,839);
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(2);
    pen.setBrush(Qt::black);
    painter.setPen(pen);

#ifdef DIJKSTRA
    painter.drawText(10,860,"The Dijkstra Algorithm");
#elif defined (A_STAR)
    painter.drawText(10,860,"The A-star Algorithm");
#elif defined (D_STAR)

#elif defined (RRT)

#elif defined (PRM)

#endif
    // draw the graph
    int n = grid.size();

    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            switch(grid[i][j])
            {
            case 1:
                painter.fillRect(40*j+1,40*i+1,38,38,QBrush(Qt::red));
                break;
            case 2:
                painter.fillRect(40*j+1,40*i+1,38,38,QBrush(Qt::blue));
                break;
            case 3:
                painter.fillRect(40*j+1,40*i+1,38,38,QBrush(Qt::green));
                break;
            case 4:
                painter.fillRect(40*j+1,40*i+1,38,38,QBrush(Qt::yellow));
                break;
            case 5:
                painter.fillRect(40*j+1,40*i+1,38,38,QBrush(Qt::darkYellow));
                grid[i][j] = 2;
                break;
            case 6:
                painter.fillRect(40*j+1,40*i+1,38,38,QBrush(Qt::darkGray));
                break;
            }
        }
    }
    if(path_vector.size()>=2)
    {
        // draw the initial node and goal node
        pen.setWidth(20);
        pen.setBrush(Qt::black);
        painter.setPen(pen);
        painter.drawPoint(40*path_vector.back().y_+20,40*path_vector.back().x_+20);
        pen.setBrush(Qt::magenta);
        painter.setPen(pen);
        painter.drawPoint(40*path_vector.front().y_+20,40*path_vector.front().x_+20);
    }

}

PaintWidget::PaintWidget(QWidget *parent) : QWidget(parent){}
PaintWidget::~PaintWidget(){}


MainWindow::~MainWindow()
{
    delete ui;
    delete graphWidget;
}

