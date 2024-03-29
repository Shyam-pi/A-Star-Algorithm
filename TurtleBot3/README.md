### A* Algorithm Implementation for TurtleBot3 Path Planning **

**Introduction:**
This project implements the A* algorithm for path planning of TurtleBot3. The A* algorithm is a widely-used pathfinding algorithm in robotics and artificial intelligence, capable of finding the shortest path between two points on a graph.


### Part 1: 2D Path Planning Implementation

- **Objective**: Implement path planning and navigation for the TurtleBot3 in a 2D environment.
- **Algorithm**: Utilizes the A* algorithm to find an optimal path from the start to the goal position while considering obstacles with appropriate clearance.
- **Features**:
  - User input for start and goal positions, as well as wheel angular velocities.
  - A* algorithm implementation with obstacle clearance considerations.
  - Visualization using Pygame to display the planned path and obstacles.
- **Key Components**:
  - A* algorithm for path planning.
  - Functions to check feasibility and generate child nodes.
  - Pygame visualization for better understanding of the planned path and obstacles.

### Part 2: Gazebo Simulation Environment Implementation

- **Objective**: Extend the path planning and navigation capabilities of the TurtleBot3 to a simulated environment in Gazebo.
- **Algorithm**: Utilizes the A* algorithm, similar to Part 1, for path planning in the Gazebo simulation environment.
- **Features**:
  - User input for the goal position.
  - A* algorithm implementation with obstacle clearance considerations.
  - Integration with ROS (Robot Operating System) for communication with the Gazebo simulation environment.
  - Visualization using Pygame to display the planned path and obstacles.
- **Key Components**:
  - A* algorithm for path planning.
  - ROS nodes for communication with Gazebo.
  - Pygame visualization for path and obstacle representation.

Both implementations demonstrate the ability to plan paths for the TurtleBot3 robot in various environments, showcasing the effectiveness of the A* algorithm and its adaptability to different scenarios.

## Video Recording links:

part_1 : https://youtu.be/7MUNlsAgnrA

part_2 : https://youtu.be/KwPj-r8nOog
