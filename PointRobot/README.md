# A* Algorithm Implementation for Point Robot Path Planning

**Introduction:**
This project implements the A* algorithm for path planning of a point robot. The A* algorithm is a widely-used pathfinding algorithm in robotics and artificial intelligence, capable of finding the shortest path between two points on a graph.

**Libraries**:
* numpy
* pygame - Install using ```pip3 install pygame```

## Instructions to run
1. Open and run the script file corresponding to the A-Star algorithm : 'a_star.py'. You'll be prompted to enter inputs in the below given order.
2. Enter the sum of wall clearance and robot radius as a number (for eg. 10)
3. Enter the step size (for eg. 10)
4. Enter the goal and start state as a list of x, y, theta (in degrees) separated by commas (for eg: 10,10,0)
5. For points 2 and 3, if a non-number input is entered, the program keeps prompting till a number input is entered.
6. For point 4, the program keeps prompting till the start and goal states are entered in the proper manner suggested, and if the goal/start state is inside any of the obstacles' space.

**Features:**
- Utilizes A* algorithm for point robot path planning.
- Allows customization of wall clearance, robot radius, and step size.
- Handles obstacles represented by rectangles, hexagons, and triangles.
- Provides visualization of the generated path using Pygame.

**Key Components:**
1. **Input Handling:**
   - Prompts the user to input wall clearance, robot radius, step size, start position, and goal position.
   - Ensures the feasibility of input values and handles exceptions.

2. **Obstacle Representation:**
   - Defines functions to represent various types of obstacles, including rectangles, hexagons, and triangles.
   - Checks the feasibility of nodes in the obstacle space.

3. **A* Algorithm Execution:**
   - Implements the A* algorithm for path planning, considering the start and goal states.
   - Tracks closed nodes, open nodes, and node visits during the search process.
   - Implements backtracking to find the optimal path.

4. **Visualization:**
   - Utilizes Pygame for visualizing the obstacles, start, goal, closed nodes, and optimal path.
   - Provides real-time rendering of the path planning process.

**Usage:**
1. Input wall clearance, robot radius, and step size.
2. Provide start and goal coordinates along with the angle.
3. Visualize the path planning process and the generated optimal path.

**Conclusion:**
This A* algorithm implementation for point robot path planning offers a flexible and efficient solution for finding the shortest path in environments with obstacles. With its customizable parameters and visualization capabilities, it serves as a valuable tool for robotic path planning applications.


