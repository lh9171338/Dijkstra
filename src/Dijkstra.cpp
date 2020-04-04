//
// Created by lihao on 19-7-9.
//

#include "Dijkstra.h"

namespace pathplanning{

void Dijkstra::InitDijkstra(Mat& _Map, DijkstraConfig _config)
{
    Mat Mask;
    InitDijkstra(_Map, Mask, _config);
}

void Dijkstra::InitDijkstra(Mat& _Map, Mat& Mask, DijkstraConfig _config)
{
    char neighbor4[4][2] = {
            {-1, 0},
            {0, -1}, {0, 1},
            {1, 0}
    };
    char neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    Map = _Map;
    config = _config;
    if(config.Diagonal)
    {
        neighbor = Mat(8, 2, CV_8S, neighbor8).clone();
    }
    else
    {
        neighbor = Mat(4, 2, CV_8S, neighbor4).clone();
    }

    MapProcess(Mask);
}

void Dijkstra::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path)
{
    // Get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, path);
}

void Dijkstra::DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask, Scalar color,
        int thickness, Scalar maskcolor)
{
    if(path.empty())
    {
        cout << "Path is empty!" << endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    for(auto it:path)
    {
        rectangle(_Map, it, it, color, thickness);
    }
}

void Dijkstra::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, CV_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, CV_THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, CV_THRESH_BINARY);
    }

    // Inflate
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
}

Node* Dijkstra::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    OpenList.clear();
    OpenList.push_back(new Node(startPoint));
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Node* CurNode = OpenList[0];
        int index = 0;
        int length = OpenList.size();
        for(int i = 0;i < length;i++)
        {
            if(OpenList[i]->dist < CurNode->dist)
            {
                CurNode = OpenList[i];
                index = i;
            }
        }
        int curX = CurNode->point.x;
        int curY = CurNode->point.y;
        OpenList.erase(OpenList.begin() + index);       // Delete CurNode from OpenList
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                if(config.Diagonal)
                {
                    // Determine whether a diagonal line can pass
                    bool walkable = true;
                    if(y == curY - 1 && _LabelMap.at<uchar>(curY - 1, curX) == obstacle)
                    {
                        walkable = false;
                    }
                    else if(y == curY + 1 && _LabelMap.at<uchar>(curY + 1, curX) == obstacle)
                    {
                        walkable = false;
                    }

                    if(x == curX - 1 && _LabelMap.at<uchar>(curY, curX - 1) == obstacle)
                    {
                        walkable = false;
                    }
                    else if(x == curX + 1 && _LabelMap.at<uchar>(curY, curX + 1) == obstacle)
                    {
                        walkable = false;
                    }
                    if(!walkable)
                    {
                        continue;
                    }
                }

                // Calculate distance
                int adddist, dist;
                if(abs(x - curX) == 1 && abs(y - curY) == 1)
                {
                    adddist = 14;
                }
                else
                {
                    adddist = 10;
                }
                dist = CurNode->dist + adddist;

                // Update the dist of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->dist = dist;
                    OpenList.push_back(node);
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    Node* node = NULL;
                    int length = OpenList.size();
                    for(int i = 0;i < length;i++)
                    {
                        if(OpenList[i]->point.x ==  x && OpenList[i]->point.y ==  y)
                        {
                            node = OpenList[i];
                            break;
                        }
                    }
                    if(dist < node->dist)
                    {
                        node->dist = dist;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void Dijkstra::GetPath(Node* TailNode, vector<Point>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
    }
}

}