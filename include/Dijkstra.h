//
// Created by lihao on 19-7-9.
//

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


namespace pathplanning{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

struct Node{
    Point point;  // node coordinate
    int dist;     // distance
    Node* parent; // parent node

    Node(Point _point = Point(0, 0)):point(_point), dist(0), parent(NULL)
    {
    }
};

struct DijkstraConfig{
    bool Diagonal;          // true/false
    int OccupyThresh;       // 0~255
    int InflateRadius;      // integer

    DijkstraConfig(bool _Diagonal = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Diagonal(_Diagonal), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }
};

class Dijkstra{

public:
    // Interface function
    void InitDijkstra(Mat& _Map, DijkstraConfig _config = DijkstraConfig());
    void InitDijkstra(Mat& _Map, Mat& Mask, DijkstraConfig _config = DijkstraConfig());
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path);
    void DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask = noArray(), Scalar color = Scalar(0, 0, 255),
            int thickness = 1, Scalar maskcolor = Scalar(255, 255, 255));

private:
    void MapProcess(Mat& Mask);
    Node* FindPath();
    void GetPath(Node* TailNode, vector<Point>& path);

private:
    //Object
    Mat Map;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    DijkstraConfig config;
    vector<Node*> OpenList;  // open list
    vector<Node*> PathList;  // path list
};

}




#endif //DIJKSTRA_H
