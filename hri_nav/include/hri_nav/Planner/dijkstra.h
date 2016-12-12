#ifndef PLANNER_DIJKISTRA_H
#define PLANNER_DIJKISTRA_H


// #include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
//#include "Timer.h"

/// CONSTANTS
static const int UNKNOWN_CELL  = 58;
static const int FREE_CELL     = 0;
static const int OCCUPIED_CELL = 255;

/// STRUCTURES for easier management.
struct vertex {
    cv::Point2i id_;
    cv::Point2i from_;

    vertex(cv::Point2i id, cv::Point2i from)
    {
        id_ = id;
        from_ = from;
    }
};

/// To be used for finding an element in std::multimap STL.
struct CompareID
{
    CompareID(cv::Point2i val) : val_(val) {}
    bool operator()(const std::pair<double, vertex> & elem) const {
        return val_ == elem.second.id_;
    }
private:
    cv::Point2i val_;
};

/// Some helper functions for dijkstra's algorithm.
uint8_t get_cell_at(const cv::Mat & image, int x, int y)
{
    assert(x < image.rows);
    assert(y < image.cols);
    return image.data[x * image.cols + y];
}

/// Some helper functions for dijkstra's algorithm.
bool checkIfNotOutOfBounds(cv::Point2i current, int rows, int cols)
{
    return (current.x >= 0 && current.y >= 0 &&
            current.x < cols && current.y < rows);
}

/// Brief: Finds the shortest possible path from starting position to the goal position
/// Param gridMap: The stage where the tracing of the shortest possible path will be performed.
/// Param start: The starting position in the gridMap. It is assumed that start cell is a free cell.
/// Param goal: The goal position in the gridMap. It is assumed that the goal cell is a free cell.
/// Param path: Returns the sequence of free cells leading to the goal starting from the starting cell.
bool findPathViaDijkstra(const cv::Mat& gridMap, cv::Point2i start, cv::Point2i goal, std::vector<cv::Point2i>& path)
{
    // Clear the path just in case
    path.clear();
    // Create working and visited set.
    std::multimap<double,vertex> working, visited;

    // Initialize working set. We are going to perform the djikstra's
    // backwards in order to get the actual path without reversing the path.
    working.insert(std::make_pair(0, vertex(goal, goal)));

    // Conditions in continuing
    // 1.) Working is empty implies all nodes are visited.
    // 2.) If the start is still not found in the working visited set.
    // The Dijkstra's algorithm
    while(!working.empty() && std::find_if(visited.begin(), visited.end(), CompareID(start)) == visited.end())
    {

        // Get the top of the STL.
        // It is already given that the top of the multimap has the lowest cost.
        std::pair<double, vertex> currentPair = *working.begin();
        cv::Point2i current = currentPair.second.id_;
        visited.insert(currentPair);
        working.erase(working.begin());

        // Check all arcs
        // Only insert the cells into working under these 3 conditions:
        // 1. The cell is not in visited cell
        // 2. The cell is not out of bounds
        // 3. The cell is free
        for (int x = current.x-1; x <= current.x+1; x++)
            for (int y = current.y-1; y <= current.y+1; y++)
            {

                if (checkIfNotOutOfBounds(cv::Point2i(x, y), gridMap.rows, gridMap.cols) &&
                        get_cell_at(gridMap, x, y) == FREE_CELL &&
                        std::find_if(visited.begin(), visited.end(), CompareID(cv::Point2i(x, y))) == visited.end())
                {
                    vertex newVertex = vertex(cv::Point2i(x,y), current);
                    double cost = currentPair.first + sqrt(2);
                    // Cost is 1
                    if (x == current.x || y == current.y)
                        cost = currentPair.first + 1;
                    std::multimap<double, vertex>::iterator it =
                            std::find_if(working.begin(), working.end(), CompareID(cv::Point2i(x, y)));
                    if (it == working.end())
                        working.insert(std::make_pair(cost, newVertex));
                    else if(cost < (*it).first)
                    {
                        working.erase(it);
                        working.insert(std::make_pair(cost, newVertex));
                    }
                }
            }
    }

    // Now, recover the path.
    // Path is valid!
    if (std::find_if(visited.begin(), visited.end(), CompareID(start)) != visited.end())
    {
        std::pair <double, vertex> currentPair = *std::find_if(visited.begin(), visited.end(), CompareID(start));
        path.push_back(currentPair.second.id_);
        do
        {
            currentPair = *std::find_if(visited.begin(), visited.end(), CompareID(currentPair.second.from_));
            path.push_back(currentPair.second.id_);
        } while(currentPair.second.id_.x != goal.x || currentPair.second.id_.y != goal.y);
        return true;
    }
    // Path is invalid!
    else
        return false;

}

int path_planner_test()
{
    //    cv::Mat image = cv::imread("filteredmap1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat image = cv::Mat(100,100,CV_8UC1);
    std::vector<cv::Point2i> path;

    for (int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            image.data[i*image.cols+j] = FREE_CELL;

            if (j == image.cols/2 && (i > 3 && i < image.rows - 3))
                image.data[i*image.cols+j] = OCCUPIED_CELL;

            //            if (image.data[i*image.cols+j] > 215)
            //                image.data[i*image.cols+j] = FREE_CELL;
            //            else if(image.data[i*image.cols+j] < 100)
            //                image.data[i*image.cols+j] = OCCUPIED_CELL;
            //            else
            //                image.data[i*image.cols+j] = UNKNOWN_CELL;
        }

    // Start top right
    cv::Point2i goal(image.cols-1, 0);
    // Goal bottom left
    cv::Point2i start(0, image.rows-1);

    // Time the algorithm.
    /*
    Timer timer;
    timer.start();
    findPathViaDijkstra(image, start, goal, path);
    std::cerr << "Time elapsed: " << timer.getElapsedTimeInMilliSec() << " ms";
    */

    findPathViaDijkstra(image, start, goal, path);
    // Add the path in the image for visualization purpose.
    cv::cvtColor(image, image, CV_GRAY2BGRA);
    int cn = image.channels();
    for (int i = 0; i < path.size(); i++)
    {
	    image.data[path[i].x*cn*image.cols+path[i].y*cn+0] = 0;
            image.data[path[i].x*cn*image.cols+path[i].y*cn+1] = 255;
            image.data[path[i].x*cn*image.cols+path[i].y*cn+2] = 0;
	    cout << path[i].x << "  " << path[i].y << endl;
    }
    cv::imshow("Map with path", image);
    cv::waitKey();
    return 0;
}

#endif