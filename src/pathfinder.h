#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include <vector>
#include <queue>
#include "maze.h"

// ============================================================================
// PATHFINDING STRATEGIES
// ============================================================================
enum PathfindingStrategy {
  STRATEGY_BFS = 0,          // Breadth-First Search (guaranteed shortest)
  STRATEGY_DIJKSTRA = 1,     // Dijkstra with cost (distance-optimized)
  STRATEGY_ASTAR = 2         // A* pathfinding (fastest)
};

// ============================================================================
// PATHFINDING FUNCTIONS
// ============================================================================

// Build adjacency graph from junctions
void buildMazeGraph();

// BFS pathfinding - guarantees shortest path in terms of junction count
int findPathBFS(int start, int end, std::vector<int>& outPath);

// Dijkstra pathfinding - finds shortest path by distance
int findPathDijkstra(int start, int end, std::vector<int>& outPath);

// A* pathfinding - heuristic-based fastest search
int findPathAStar(int start, int end, std::vector<int>& outPath);

// Select best pathfinding strategy automatically
int findOptimalPath(int start, int end, std::vector<int>& outPath, 
                    PathfindingStrategy strategy = STRATEGY_BFS);

// Utility functions
int calculateMazeDistance(int junction1, int junction2);
bool areMazeJunctionsAdjacent(int j1, int j2);
void printPathfindingDebug();

#endif // PATHFINDER_H
