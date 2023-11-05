# RRT Notes


## Intro
The paper attempts to argue that PRM* and RRT* asymptotically optimal. I wonder if they are trying to say that this creates a cost function that is convex?

RRT and PRM are both sampling based approaches that randomly sample the state space and create a graph that connect these sampled points. 

PRM creates a set of collision-free trajectories and then computes the shortest path through the roadmap. However, this does require a roadmap and that can be challenging.

RRT is an incremental a single query counterpart to PRM and creates the trajectories on the fly.

## Preliminary Material

Configuration space is defined as the state of the environment, usually in 0s and 1s, enumerating the environemnt surrounding the robot. The obstacle region