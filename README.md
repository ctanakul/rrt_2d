# **RRT playground**

## **algorithm log**
**2020-09-20**

Algorithm
1. Fix start and end point
2. Save (start:None) to point_list
3. While not find the path from start to endpoint
4. Randomize a new point
5. Find the closest point in the list
    1. If not blocked by an obstacle, connect the line
    2. Else if blocked by obstable, cut the line at obstacle. Reassign the new point as the end of the cut line.
        1. If the line is cut to only start point - the start point is right at the obstacle - consider failure, and quit the loop.
    3. Draw a line from the closest point to the new point
    4. If the new point can connect directly to the endpoint, connect and break the loop
Pros
1. Fast iteration 
   - How many average iterations?
     - Create arg for run number
     - Create a window 
       - to show plot the total iteration of each run
       - to print out the average run number 
     - 