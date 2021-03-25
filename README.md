## 1. main.py
This file contains provided functions for this project along with implementation of motion planning for each map. You can choose which map and algorithm to use in __main__.

## 2. Planner.py
This file contains searche based A* motion planning algorithm implementation.

## 3. Collision.py
This file contains collision checking alogorithm.

### The following scripts are from rrt-algorithms library.

https://github.com/motion-planning/rrt-algorithms

- 4.geometry.py
- 5.heuristics.py
- 6.obstacle_generation.py
- 7.plotting.py
- 8.rrt.py
- 9.rrt_base.py
- 10.rrt_star.py
- 11.search_space.py
- 12.tree.py


# Results
##	Optimal Path by A* Algorithm
We will take close look and discuss the quality of computed path for each map.

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/3.png)

Fig. 3 Single_Cube Optimal Path in XZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/4.png)
 
Fig. 4 Single_Cube Optimal Path in YZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/5.png)
 
Fig. 5 Single_Cube Optimal Path
 
As we can see the path is truly optimal and will avoid obstacles with 0.1 distance. This is a result of my definition on valid vertex space, preventing the robot to stick to the wall of obstacles and give some space so the robot can feel more comfortable.

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/6.png)
 
Fig. 6 Flappy_Bird Optimal Path in XZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/7.png)
 
Fig. 7 Flappy_Bird Optimal Path in YZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/8.png)
 
Fig. 8 Flappy_Bird Optimal Path in XY Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/9.png)
 
Fig. 9 Flappy_Bird Optimal Path  


Flappy_bird map gives perfect example if there is no coordinate difference, there will be no action in that coordinate. Y coordinate is same for start and goal, so there is no movement in the optimal path. 


![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/10.png)

Fig.10 Maze Optimal Path in XZ Plane
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/11.png)

Fig. 11 Maze Optimal Path in XY Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/12.png)

Fig. 12 Maze Optimal Path


This map show how my algorithm works when working in complex environment. As we can see in fig.10, it still give straight line movements again verify its completeness. What’s interesting is that in fig.11, computed path almost stick to the wall when moving toward exit. This meets our intuition that if we stay as close as possible to the wall, we can have shorter path. Again, it didn’t really “stick on” the wall is a expected result of my non-collision state defining.

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/13.png)

Fig. 13 Monza Optimal Path in XZ Plane 

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/14.png)

Fig. 14 Monza Optimal Path in YZ Plane  

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/15.png)

Fig. 15 Monza Optimal Path in XY Plane  

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/16.png)

Fig. 16 Monza Optimal Path


This is a simple environment, however there is something interesting we should notice. In fig.14 there is some shakes when moving down. This may seem non-optimal, turns out that this is indeed optimal path. It is just that it redistribute motions and total cost remains unchanged, I believe this is acceptable. 

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/17.png)

Fig. 17 Room Optimal Path in XZ Plane
 
 ![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/18.png)
 
Fig. 18 Room Optimal Path in YZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/19.png)

Fig. 19 Room Optimal Path in XY Plane  

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/20.png)
Fig. 20 Room Optimal Path 


Room map may seem complex but its solution is actually straight forward. “Furniture” don’t really block robot’s path, also the Z coordinate are identical, and results give us a beautiful straight line.


![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/21.png)
 
Fig. 21 Tower Optimal Path in XZ Plane 

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/22.png)

Fig. 22 Tower Optimal Path in YZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/23.png)
 
Fig. 23 Room Optimal Path

These two environments are just the similar as other environments. Optimality and correctness are achieved.


![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/24.png)
  
Fig. 24 Window Optimal Path in XZ Plane
 
 ![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/25.png)
 
Fig. 25 Window Optimal Path in YZ Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/26.png)

Fig.26 Window Optimal Path in XY Plane

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/27.png)

Fig. 27 Window Optimal Path



## Comparison of Different Heuristic Weight
 In this part, we will discuss the effect on choosing different weights. As expected, with high weight, we will have faster result but sub-optimal path. However, in some cases, where the environment are simple structures, higher weight can boost substantially while having similar optimal path length. This is also consistent to how A* algorithm should behave.
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/28.png)
 
Fig. 28 Single_cube with ϵ =1 T=6.96s, Path length =8
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/29.png)
 
Fig. 29 Single_cube with ϵ =5c T=0.129s, Path length =8
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/30.png)
 
Fig. 30 Single_cube with ϵ =10 T=0.134s, Path length =8

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/31.png)
  
Fig. 31 Flappy_bird with ϵ =1 T=429s, Path length =26

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/32.png)
 
Fig. 32 Flappy_bird with ϵ =5 T=69.4s, Path length =28

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/33.png)
 
Fig. 33 Flappy_bird with ϵ =10 T=55.6s, Path length =30

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/34.png)

Fig. 34 Maze with h=1 T=3619s, Path length =76

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/35.png)
 
Fig. 35 Maze with ϵ =5 T=2803s, Path length =79
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/36.png)

Fig. 36 Maze with ϵ =10 T=2701s, Path length =80

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/37.png)
 
Fig. 37 Monza with ϵ =1 T=314s, Path length = 76

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/38.png)
 
Fig. 38 Monza with ϵ =5 T=239.2s, Path length =76
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/39.png)

Fig. 39 Monza with ϵ =10 T=241.8s, Path length =76

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/40.png)

Fig. 40 Room with ϵ = 1 T= 95.7s, Path length =11
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/41.png)
 
Fig. 41 Room with ϵ = 5 T=18.79s, Path length =12

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/42.png)

Fig. 42 Room with ϵ = 10 T=18.74s, Path length =12
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/43.png)
 
Fig. 43 Tower with ϵ =1 T= 1148s, Path length = 29

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/44.png)
 
Fig. 44 Tower with ϵ = 5 T=88s, Path length =36

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/45.png)

Fig. 45 Tower with ϵ = 10 T=51.3s, Path length =35

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/46.png)
 
Fig. 46 Window with ϵ = 1 T= 917s, Path length = 26
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/47.png)
 
Fig. 47 Window with ϵ = 5 T=1.56s, Path length =27 

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/48.png)

Fig. 48 Window with ϵ = 10 T=1.33s, Path length =27



## A* vs. RRT* Algorithm
Finally, we compared results of search based and sampling based motion planning. For sampling based algorithm, I am using RRT*.
RRT* algorithm turns out to be extremely time efficient compared to A*. Moreover, it does not give bad result, some path lengths are even close to optimal path length. We need to pay attention on Monza map, RRT* performed worse than A*. This is because sampling based algorithm have trouble squeezing through small gaps and Monza map has relatively small passage compared to other environments.
There are a few things we need to notice, RRT* algorithm does not follows the discrete space I defined previously, it uses continuous space for motion planning. The results can be a slight better than A* with discrete space in some degree.
Given the searching speed and its optimality, it is not hard to see why sampling based planning is currently the most popular motion planning algorithm.

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/49.png)
 
Fig. 49 Single_cube A* T=6s, Path length = 8

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/50.png)
 
Fig. 50 Single_cube RRT* T=0.64s, Path length = 9
 
![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/51.png)

Fig. 51 Flappy_bird A* T=429s, Path length=26

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/52.png)
 
Fig. 52 Flappy_bird RRT* T=2.3s, Path length=27

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/53.png)

Fig. 53 Maze A* T=3619s, Path length=76

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/54.png)
 
Fig. 54 Maze RRT* T=104, Path length=78

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/55.png)
 
Fig. 55 Monza A* T=314s, Path length = 76

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/56.png)
 
Fig. 56 Monza RRT* T=325s, Path length=78

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/57.png)
 
Fig. 57 Room A* T=3s, Path length =11

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/58.png)
 
Fig. 58 Room RRT* T=3.14s, Path length=16.48

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/59.png)

Fig. 59 Tower A* T=1148s, Path length=29

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/60.png)

Fig. 60 Tower RRT* T=11.5s, Path length= 30.8

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/61.png)
 
Fig. 61 Window A* T=917s, Path length= 26

![image](https://github.com/davison0487/Motion-Planning-/blob/main/image/62.png)

Fig. 62 Window RRT* T=18s, Path length= 12
