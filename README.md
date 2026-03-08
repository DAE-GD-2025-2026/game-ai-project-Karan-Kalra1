
### IMPORTANT

- Please generate the visual studio project files after downloading
- If Flock appears broken after starting the level please recompile the code and run again


# Flocking & Spatial Partitioning

This project implements a flocking simulation using classic steering behaviors and spatial partitioning for efficient neighbor detection

## Features
- Flocking behaviors: Cohesion, Separation, Alignment (Velocity Match)
- Additional steering: Seek, Flee, Wander, Evade
- Blended and Priority steering systems
- Spatial partitioning grid for optimized neighbor search
- Debug visualization for:
  - Neighbor radius
  - Steering directions
  - Cell grid
  - Number of agents per cell

## Controls
- **Left Mouse Button**: Set target position
- **Right Mouse Button**: Move camera
- **Mouse Wheel**: Zoom camera

## Debug Options
Using the ImGui debug menu you can:
- Toggle spatial partitioning
- Enable/disable debug rendering
- Adjust steering behavior weights
- Visualize the partition grid and neighbors

## Performance
Two neighbor detection systems are implemented:
- **Brute Force** – checks all agents (O(n²))
- **Spatial Partitioning** – checks only nearby cells (~O(n))

Spatial partitioning significantly improves performance with larger flocks.
