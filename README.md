
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


# Navigation Mesh & Pathfinding Assignment

## Overview

This assignment implements pathfinding using a **navigation mesh (navmesh)** combined with **graph-based search (A*)** and **path smoothing (SSFA / Funnel Algorithm)**.


## Navigation Graph Construction

A navigation graph is built from the navmesh:

* **Nodes** are created at the **midpoints of shared edges (portals)** between triangles.
* **Connections** are created between nodes that belong to the same triangle:

  * 2 nodes → 1 connection
  * 3 nodes → 3 connections
* Connection costs are set to the **Euclidean distance** between nodes.



## Pathfinding (A*)

To find a path from a start to an end position:

1. Determine the **start and end triangles**.
2. If both positions are in the same triangle → return a direct path.
3. Clone the navigation graph.
4. Add:

   * A **start node** (agent position)
   * An **end node** (target position)

5. Connect these nodes to the graph via their triangle edges.
6. Run **A*** to compute the shortest path.
7. Convert the resulting node path to a list of positions.


## Path Smoothing (SSFA - Funnel Algorithm)

To improve path quality:

* Convert the node path into a sequence of **portals (edges)**.
* Apply the **Simple Stupid Funnel Algorithm (SSFA)**:

  * Maintains a funnel between left/right edges
  * Tightens the path while avoiding unnecessary turns
* Produces a **shorter and smoother path**


## Features

* Graph-based navigation using navmeshes
* A* pathfinding
* Optional path smoothing (SSFA)
* Debug visualization of:

  * Nodes
  * Connections
  * Portals
  * Final path


## Result

The agent successfully:

* Navigates across the navmesh
* Finds shortest paths using A*
* Produces smooth movement using funnel optimization






