# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### The Project
This goal of this project is to control a car to drive on a 3-lane highway. It includes path planning and finding the car's trajectory correctly. 

# Architecture
The system consists of two components:
- A simulator which was developed by Udacity in Unity and simulates/displays our car, driving on a 3-lane highway,
- And a Path controller which directs the car by defining the _waypoints_ of the path to move on, in the next couple of frames.
  There is a websocket (http-based data communication protocol) based communication between them. 
The development of the latter component was my project task in the Self-Driving Car Engineer Nanodegree Program. So, in the following part of this document I will describe that.

---

# Design

The application listens on the predefined TCP port 4567, and the simulator connects to it at the beginning. 
Ater the connection is established, the simulator sends JSON encoded messages with the current state of the car (poisition,speed,yaw), state of the other cars in our side of the highway, and the still undriven part of our previously planned path. We need to reply to this with a new path in a similar, JSON formatted message reply. This message exchange repeats frequently, controlled by the simulator logic.      

A path contains route points, which are coordinates on this highway. The simulator advances 1 route point in 0.02 second, which means 50 route points/second. 
We always plan a path for the next 1 second, which means that in a normal scenario we only need to append a couple of route points to the end of the remaining (undriven part of previously generated) path.    

The controlling logic is based on a finite state machine with 3 states:
- KL - In this mode, the car stays in the current the lane
- LCL - Lane change to left, when the car tries to change the lane left
- LCR - Lane change to right, when the car tries to change the lane right.

The change logic of these states is using cost functions. The sum of these cost functions will decide when to change to an other state. 
A cost function will return _infinity_ cost for a _next_state_ when it is dangerous or not safe to change to that state, according to the given cost function. The car's next state will be decided by the minimal total cost in each iteration. 

These are the 5 cost functions:
- lane_stay_cost: Slaloming among the lanes is dangerous ! Changing lanes right after a previous lane change operation is punished with an infinite cost. (Lane change will not be permitted for a few seconds)
- lane_speed_cost: 
  - If it's not safe to change lane to left or right, then give back _infinity_ cost.
  - Otherwise, if nobody is ahead, cost is 0.
  - Otherwise, the cost will be based on the speed of the vehicle in front of us (in the target lane).  
- convenience_cost
  - We are lazy and like to stay in the current lane: Changing lanes will have a prespecified, constant cost. 
- space_ahead_cost
  - This cost is inversely proportional to the distance of the car in front of us (in the target lane). So, we like broader empty places ahead of us.  
- speed_difference_cost
  - There can be a cost from this function when we consider changing lanes. If there is a faster car in the target lane behind us, this cost will be proportional to the speed difference of us, and that faster car. 

For trajectory generation, to create smooth paths even for lane changing, I'm using the spline interpolation library which is free to use ( GPL license ) C++ library, and does not depend on any external things, it only depends on STL.
To work with positions I use the Frenet coordinate system, which is an easy way of representing positions on a road. It's much better than the cartesian X/Y system, because it uses an "s" component which is a distance _along_ the road, and a "d" component with the side-to-side position on the road. s is 0 at the beginning of the road, and it contains the distance in meters. d will be the (perpendicular) distance from the centerline of the road, with negative values on the left side and positive on the right.    
To convert between these coordinate systems we need a map of the highway, which is in the provided data/highway_map.csv file. The conversion itself are implemented in the getFrenet() and getXY() helper functions.

# Implementation details

There is _always_ a target lane (rather, a "d", which is the car's distance from the road centerline). 
When we want to change lane, this will be a target distance, when we will stay in KL state, this will stay our lane's optimal distance from road center. (as 1 lane is 4 meters wide, optimal d is 2+4*[lane index] meters)

Normally, I always plane ahead for 1 sec. (That's 50 coordinates, as the simulation is with 50 frames/sec.) 
 
But there are sometimes unexpected events on a highway when things are not happening as we wanted. Just one example: when we want to change lane from left to middle, and there comes a car at the same moment from the rightmost lane toward out target, center lane. 
I also handle similar cases, because our safetly is the most important: When the distance from an other car it too small, or there is vehicle in our preplanned (1 sec.) path we handle it as an emergency event. (see HighwayState::emergency() method) If that happens, we drop our previously planned path, and create a completely new path from the car's current position toward the center of the lane where we currently are in, considering the speed of the vehicle, which will be in front of us in that lane.   

Bonus: In common.h, there's an extra, special vehicle profile which can be enabled by uncommenting the "#define GT" line. With that, the max. speed wil be 100 MPH, maneuvers will be more dangerous, lane changes more frequent, well it's just not safe, but it's nice :) 

Here is a video file where we are driving 1 lap: [driving_video](./driving_video.mp4)

I had problems with the following, so just a warning: 
- The car's speed is coming from the simulator is in MPH ! 
- The spline generator doesn't like when one line segment is vertical. In that case use a fallback mechanism and use the car's current position, and an other point calculated using the car direction backward. 
- The track is circular, when we reach the end of it s restarts from 0 again ! 