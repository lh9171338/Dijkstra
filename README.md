[<img height="23" src="https://github.com/lh9171338/Outline/blob/master/icon.jpg"/>'](https://github.com/lh9171338/Outline) Dijkstra
===

# 1. Introduction
>>This repository is a ROS package of Dijkstra algorithm.

# 2. Usage
## 2.1 Subscribed Topics  
map([nav_msgs/OccupancyGrid](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html))    
>>Receive the map via this topic.

initialpose([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))  
>>Receive the start point via this topic.

move_base_simple/goal([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))   
>>Receive the target point via this topic.

## 2.2 Published Topics  
mask([nav_msgs/OccupancyGrid](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html))  
>>Publish the inflation map(mask) via this topic.  

nav_path([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))  
>>Publish the navigation path via this topic.

## 2.3 Parameters  
~Diagonal(bool; default: "true")  
>>Whether can go through the diagonal. When Diagonal is true, serching for 8 neighborhoods, or serching for 4 neighborhoods.

~OccupyThresh(int; default: -1)  
>>Threshold of the image binarization. When OccupyThresh is less than zero(OccupyThresh < 0), using Otsu method to generate threshold.

~InflateRadius(double; defalut: -1)  
>>InflateRadius is the inflation radius(unit: m). When InflateRadius is less than or equal to zero(InflateRadius <= 0), no inflation operation is taken.

~rate(int; default: 10)  
>>The rate of publishing mask topic.

## 2.4 Example  
```
roslaunch dijkstra dijkstra.launch
```

# 3. Result  
![image](https://github.com/lh9171338/Dijkstra/blob/master/results/result.png)
