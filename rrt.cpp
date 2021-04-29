// include statements
#include <climits>
#include <iostream>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include "matplotlibcpp.h"

//Obstcale Points- Change points here to edit obstacles
int obst_array[][2] = {{2, 7}, {7, 7}, {6, 4}, {4, 4}, {4, 6}, {2, 6}};

//typedefs to define geometries in boost geometry
typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::polygon<point_xy> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;
typedef boost::geometry::model::linestring<point_xy> linestring_t;

//variable to keep track of goal sampling and goal sampling rate
int g_sampling;
int a;

// Node tructure definiton
struct Node
{
    // x and y define the 2D coordinates of the node
    float x;
    float y;
    // Parent Node to the current Node
    Node *parent;
};

//array of all nodes
Node node_list[500];

// counter to keep track of no. of elements in node_list
int count = 0;

//array to store the path in
Node path[100];

//variable to track elements in path
int track = 0;

//sampler variables- used to set the sampling range
//Change these to use a different search space
int Offset = 0;
int Range = 13;

//Search space is defined from (0,0), (0,12), (12,0) and (12,12)
// function to sample random points in search space and also sample the goal at regular intervals defined by g_sampling
Node sampler(int goal_x, int goal_y)
{
    /*
        Args:
            goal_x(int): x-coordinate of the goal
            goal_y(int): y-coordinate of the goal

        Retuns:
            sample(Node object): The sampled node 
    */
    Node sample = {};
    //samples random points
    if (count < a)
    {
        sample.x = Offset + (rand() % Range);
        sample.y = Offset + (rand() % Range);
        sample.parent = NULL;
    }
    //samples goal as every g_sampling th point
    else
    {
        a += g_sampling;
        sample.x = goal_x;
        sample.y = goal_y;
        sample.parent = NULL;
    }

    return sample;
}

//finds distance between two Node's
float distance(Node node1, Node node2)
{   
    /*
        Args:
            node1(Node object): first node for distance calculation
            node2(Node object): second node for distance calculation

        Retuns:
            dist(float): the distance between the nodes 
    */
    float dist = sqrt(((node1.x - node2.x) * (node1.x - node2.x)) + ((node1.y - node2.y) * (node1.y - node2.y)));
    return dist;
}

//function to find the nearest node to the passed node
int nearest_node(Node current)
{   
    /*
        Args:
            current(Node object):node for which nearest node is to be found

        Retuns:
            near_i(int): the index of the nearest node wrt node_list 
    */
    int near_i;
    float min_dist = 20;
    for (int i = 0; i < count; i++)
    {
        float dist = distance(current, node_list[i]);
        if (dist < min_dist)
        {
            min_dist = dist;
            near_i = i;
        }
    }

    return near_i;
}

//checks intersection between obstacles and the line segment (rand,near) using boost-geometry
bool check_intersection(Node end1, Node end2)
{
    /*
        Args:
            end1(Node object): first node of the line segment
            end2(Node object): second node of the line segment

        Retuns:
            status(bool): 1 if it collides with obstacles, 0 otherwise  
    */

    //define polygons and multi-polygons
    polygon_type obstacle;
    multi_polygon_type obstacles;
    linestring_t line;

    //Adding the line
    // Store points in vector so we can assign them to a polygon
    std::vector<point_xy> pointList;

    point_xy point1(end1.x, end1.y);
    pointList.push_back(point1);

    point_xy point2(end2.x, end2.y);
    pointList.push_back(point2);

    // assign points to polygon line
    boost::geometry::assign_points(line, pointList);

    //Adding the obstacle
    // Store points in vector so we can assign them to a polygon
    std::vector<point_xy> ObstaclePointList;

    for (int i = 0; i < (sizeof(obst_array) / sizeof(obst_array[0])); i++)
    {
        point_xy point(obst_array[i][0], obst_array[i][1]);
        ObstaclePointList.push_back(point);
    }

    // assign points to polygon
    boost::geometry::assign_points(obstacle, ObstaclePointList);

    // Add polygon to multi-polygon
    obstacles.push_back(obstacle);

    //checks if line intersects with obstacles
    bool status = boost::geometry::intersects(line, obstacles);

    return status;
}

//checks if a point is inside an obstacle using boost-geometry
bool is_inside_obstacle(float a, float b)
{
    /*
        Args:
            a(float): x-coordinate of the point to be checked
            b(float): y-coordinate of the point to be checked

        Retuns:
            status(bool): 1 if it is within obstacles, 0 otherwise  
    */
    //define polygons and multi-polygons
    polygon_type obstacle;
    multi_polygon_type obstacles;

    //Adding the point
    point_xy point(a, b);

    //Adding the obstacle
    // Store points in vector so we can assign them to a polygon
    std::vector<point_xy> ObstaclePointList;

    for (int i = 0; i < (sizeof(obst_array) / sizeof(obst_array[0])); i++)
    {
        point_xy point(obst_array[i][0], obst_array[i][1]);
        ObstaclePointList.push_back(point);
    }

    // assign points to polygon
    boost::geometry::assign_points(obstacle, ObstaclePointList);

    // Add polygon to multi-polygon
    obstacles.push_back(obstacle);

    //checks if point is within obstacles
    bool status = boost::geometry::within(point, obstacles);

    return status;
}

//function to backtrack path from goal to start
void backtrack(Node goal, Node start)
{
    /*
        Args:
            goal(Node object): goal
            start(Node object): start
    */
    //initialize current node as the goal
    Node *current = &goal;
    while (current->parent != NULL)
    {
        path[track] = *current;
        track++;
        //assign current node as its parent
        current = current->parent;
    }

    //Add start to the path
    path[track] = start;
    track++;
}

//function to visualise path using matplotlib-cpp
void visualise(bool is_path)
{
    /*
        Args:
            is_path(bool): status of whether path exists or not
    */
    namespace plt = matplotlibcpp;
    //plot obstacle
    std::vector<double> x = {};
    std::vector<double> y = {};

    for (int i = 0; i < (sizeof(obst_array) / sizeof(obst_array[0])); i++)
    {
        x.push_back(obst_array[i][0]);
        y.push_back(obst_array[i][1]);
    }
    x.push_back(obst_array[0][0]);
    y.push_back(obst_array[0][1]);

    plt::plot(x, y);

    //scatter plot if all nodes
    std::vector<double> scatter_x = {};
    std::vector<double> scatter_y = {};

    for (int i = 0; i < count; i++)
    {

        scatter_x.push_back(node_list[i].x);
        scatter_y.push_back(node_list[i].y);
    }

    plt::plot(scatter_x, scatter_y, ".r");

    //plot tree
    for (int i = 0; i < count; i++)
    {
        if (node_list[i].parent != NULL)
        {
            std::vector<double> node_x = {};
            std::vector<double> node_y = {};
            node_x.push_back(node_list[i].x);
            node_y.push_back(node_list[i].y);
            node_x.push_back(node_list[i].parent->x);
            node_y.push_back(node_list[i].parent->y);
            plt::plot(node_x, node_y, "r");
        }
    }

    if (is_path == 1)
    {
        //plot path
        for (int i = 0; i < track; i++)
        {
            if (path[i].parent != NULL)
            {
                std::vector<double> node_x = {};
                std::vector<double> node_y = {};
                node_x.push_back(path[i].x);
                node_y.push_back(path[i].y);
                node_x.push_back(path[i].parent->x);
                node_y.push_back(path[i].parent->y);
                plt::plot(node_x, node_y, ".b-");
            }
        }
    }

    plt::show();
}

// Start of main RRT function
//returns 0 if path not found and 1 if path is found
bool rrt(float start_x, float start_y, float goal_x, float goal_y, int K, float step)
{
    /*
        Args:
            start_x(float):x-coordinate of the start point
            start_y(float):y-coordinate of the start point
            goal_x(float):x-coordinate of the goal point
            goal_y(float):y-coordinate of the goal point
            K(int):maximum number of nodes in the tree
            step(float):step-size for RRT

        Returns:
            1, if path is found
            0, otherwise
    */
    //variable to track if goal has been reached
    bool not_reached;

    // defining start, goal and rand as a Node
    Node start = {start_x, start_y};
    Node goal = {goal_x, goal_y};
    Node rand = {};

    //variable to store index of the nearest neighbour
    int near_i;

    //add start to the node_list
    node_list[count] = start;
    count++;

    for (int j = 0; j < K;)
    {
        //samples a random point
        rand = sampler(goal_x, goal_y);

        // finds the index of the nearest node to the sampled point
        near_i = nearest_node(rand);

        //computing the angle b/w near and rand wrt x-axis
        float theta = atan2(rand.y - node_list[near_i].y, rand.x - node_list[near_i].x);

        //uses step-size to modify rand to a point in its direction, at a step distance from near
        rand.x = node_list[near_i].x + step * cos(theta);
        rand.y = node_list[near_i].y + step * sin(theta);

        //obstacle checking
        bool status = check_intersection(rand, node_list[near_i]);
        if (status == 1)
        {
            //if the line intersects any obstacle
            continue;
        }
        else
        {
            //adds line to node_list if it is collision free
            rand.parent = &node_list[near_i];
            node_list[count] = rand;
            count++;
            j++;
        }

        //variable to track if a direct connection to the goal is possible
        not_reached = check_intersection(rand, goal);

        //variable to store distance between current node and goal
        float goal_dist = distance(rand, goal);

        //adds goal to the node_list and breaks if goal is reachable or justa step-size distance away
        if (not_reached == 0 || goal_dist <= step)
        {
            goal.parent = &rand;
            node_list[count] = goal;
            count++;
            //Goal Reached
            break;
        }
    }
    //returns 0 if path not found
    if (not_reached == 1)
    {
        return 0;
    }

    //calls function to traverse from goal to start using their parent fields
    backtrack(goal, start);

    //returns 1 as path is founs succesfully
    return 1;
}

int main()
{
    // Providing a seed value for rand in sampler function
    srand((unsigned)time(NULL));

    //variables of rrt
    float start_x, start_y, goal_x, goal_y, K, step;

    //Print search space and obstacle end points
    std::cout << "\033[2J\033[1;1H";
    std::cout << "Rapidly Exploring Random Trees" << std::endl;
    std::cout << "The Search space is defined by points (0,0), (0,12), (12,0) and (12,12)" << std::endl;
    std::cout << "The Obstacle is a polygon defined by points (2, 7), (7, 7), (6, 4), (4, 4), (4, 6) and (2, 6)" << std::endl;

    //Taking user input for the rrt variables
    while (1)
    {
        std::cout << "Enter the x-coordinate of start(0-12):";
        std::cin >> start_x;

        std::cout << "Enter the y-coordinate of start(0-12):";
        std::cin >> start_y;

        if (start_x < 0 || start_x > 12 || start_y < 0 || start_y > 12)
        {
            std::cout << "The point is outside the Search space! Retry" << std::endl;
        }
        else if (is_inside_obstacle(start_x, start_y))
        {
            std::cout << "Point inside obstacle! Retry" << std::endl;
        }
        else
        {
            break;
        }
    }

    std::cout << "\033[2J\033[1;1H";
    std::cout << "Rapidly Exploring Random Trees" << std::endl;
    std::cout << "The Search space is defined by points (0,0), (0,12), (12,0) and (12,12)" << std::endl;
    std::cout << "The Obstacle is a polygon defined by points (2, 7), (7, 7), (6, 4), (4, 4), (4, 6) and (2, 6)" << std::endl;

    while (1)
    {
        std::cout << "Enter the x-coordinate of goal(0-12):";
        std::cin >> goal_x;

        std::cout << "Enter the y-coordinate of goal(0-12):";
        std::cin >> goal_y;

        if (goal_x < 0 || goal_x > 12 || goal_y < 0 || goal_y > 12)
        {
            std::cout << "The point is outside the Search space! Retry" << std::endl;
        }
        else if (is_inside_obstacle(goal_x, goal_y))
        {
            std::cout << "Point inside obstacle! Retry" << std::endl;
        }
        else
        {
            break;
        }
    }

    std::cout << "\033[2J\033[1;1H";
    std::cout << "Rapidly Exploring Random Trees" << std::endl;
    std::cout << "The Search space is defined by points (0,0), (0,12), (12,0) and (12,12)" << std::endl;
    std::cout << "The Obstacle is a polygon defined by points (2, 7), (7, 7), (6, 4), (4, 4), (4, 6) and (2, 6)" << std::endl;

    std::cout << "Enter the maximum number of nodes in the tree(0-499):";
    std::cin >> K;

    std::cout << "Enter the step-size for RRT(Preferably between 0.1-2):";
    std::cin >> step;

    std::cout << "Enter the rate at which the goal should be sampled(Enter 0 to generate an unbiased tree)" << std::endl
              << "(Enter 25 if you want every 25th sample to be the goal):";
    std::cin >> g_sampling;

    if (g_sampling == 0)
    {
        g_sampling = K + 1;
    }

    //variable assignment to ensure goal sampling rate is followed
    a = g_sampling;

    std::cout << "\033[2J\033[1;1H";
    std::cout << "Rapidly Exploring Random Trees" << std::endl;
    std::cout << "Finding a path!! Please Wait!" << std::endl;

    //call rrt function- start point is assumed to be the origin
    bool is_path = rrt(start_x, start_y, goal_x, goal_y, K, step);

    //plots the path and the tree
    visualise(is_path);

    if(is_path==0)
    {
        std::cout<<"Sorry,Path Not Found!!";
    }

    //print nodelist
    // for(int i=0;i<count;i++)
    // {
    //     std::cout<<node_list[i].x<<" "<<node_list[i].y<<std::endl;
    // }

    //print path
    // for(int i=0;i<track;i++)
    //     {
    //         std::cout<<path[i].x<<" "<<path[i].y<<std::endl;
    //     }

    return 0;
}