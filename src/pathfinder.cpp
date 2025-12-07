#include "pathfinder.h"
#include "maze.h"
#include <Arduino.h>
#include <limits.h>

// ============================================================================
// GRAPH BUILDING
// ============================================================================
void buildMazeGraph() {
  // Connect adjacent junctions discovered in dry run
  Serial.println("[PATHFINDER] Building maze graph...");
  
  for (int i = 0; i < mazeState.junctions.size(); i++) {
    for (int j = i + 1; j < mazeState.junctions.size(); j++) {
      if (areMazeJunctionsAdjacent(i, j)) {
        // Calculate which direction
        int dx = mazeState.junctions[j].x - mazeState.junctions[i].x;
        int dy = mazeState.junctions[j].y - mazeState.junctions[i].y;
        
        Direction dir = DIR_NORTH;
        if (dx > 0) dir = DIR_EAST;
        else if (dx < 0) dir = DIR_WEST;
        else if (dy > 0) dir = DIR_SOUTH;
        
        // Connect neighbors
        if (mazeState.junctions[i].neighbors[dir] == -1) {
          mazeState.junctions[i].neighbors[dir] = j;
        }
        
        Direction oppositeDir = (Direction)((dir + 2) % 4);
        if (mazeState.junctions[j].neighbors[oppositeDir] == -1) {
          mazeState.junctions[j].neighbors[oppositeDir] = i;
        }
      }
    }
  }
  
  Serial.println("[PATHFINDER] Graph building complete");
}

// ============================================================================
// BFS PATHFINDING (GUARANTEED SHORTEST IN JUNCTION COUNT)
// ============================================================================
int findPathBFS(int start, int end, std::vector<int>& outPath) {
  if (start < 0 || start >= mazeState.junctions.size() ||
      end < 0 || end >= mazeState.junctions.size()) {
    Serial.println("[PATHFINDER] ERROR: Invalid start or end junction");
    return -1;
  }
  
  std::vector<int> parent(mazeState.junctions.size(), -1);
  std::vector<bool> visited(mazeState.junctions.size(), false);
  std::queue<int> q;
  
  parent[start] = start;
  visited[start] = true;
  q.push(start);
  
  while (!q.empty()) {
    int current = q.front();
    q.pop();
    
    if (current == end) break;
    
    Junction& junc = mazeState.junctions[current];
    
    // Explore all 4 directions
    for (int dir = 0; dir < 4; dir++) {
      int neighbor = junc.neighbors[dir];
      if (neighbor >= 0 && !visited[neighbor]) {
        visited[neighbor] = true;
        parent[neighbor] = current;
        q.push(neighbor);
      }
    }
  }
  
  // Check if path found
  if (parent[end] == -1 && end != start) {
    Serial.println("[PATHFINDER] No path found!");
    return -1;
  }
  
  // Reconstruct path
  outPath.clear();
  int current = end;
  while (current != start) {
    outPath.insert(outPath.begin(), current);
    current = parent[current];
  }
  outPath.insert(outPath.begin(), start);
  
  Serial.print("[PATHFINDER] BFS path found: ");
  Serial.println(outPath.size());
  
  return outPath.size();
}

// ============================================================================
// DIJKSTRA PATHFINDING (SHORTEST BY DISTANCE)
// ============================================================================
int findPathDijkstra(int start, int end, std::vector<int>& outPath) {
  const float INF = 1e9;
  int n = mazeState.junctions.size();
  
  std::vector<float> dist(n, INF);
  std::vector<int> parent(n, -1);
  std::vector<bool> visited(n, false);
  
  dist[start] = 0;
  parent[start] = start;
  
  for (int i = 0; i < n; i++) {
    int u = -1;
    for (int j = 0; j < n; j++) {
      if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
        u = j;
      }
    }
    
    if (u == -1 || dist[u] == INF) break;
    
    visited[u] = true;
    
    if (u == end) break;
    
    Junction& junc = mazeState.junctions[u];
    
    for (int dir = 0; dir < 4; dir++) {
      int v = junc.neighbors[dir];
      if (v >= 0) {
        float edgeCost = calculateMazeDistance(u, v);
        if (dist[u] + edgeCost < dist[v]) {
          dist[v] = dist[u] + edgeCost;
          parent[v] = u;
        }
      }
    }
  }
  
  if (parent[end] == -1 && end != start) {
    Serial.println("[PATHFINDER] Dijkstra: No path found!");
    return -1;
  }
  
  // Reconstruct path
  outPath.clear();
  int current = end;
  while (current != start) {
    outPath.insert(outPath.begin(), current);
    current = parent[current];
  }
  outPath.insert(outPath.begin(), start);
  
  Serial.print("[PATHFINDER] Dijkstra path found: ");
  Serial.print(outPath.size());
  Serial.print(" junctions, distance: ");
  Serial.println(dist[end]);
  
  return outPath.size();
}

// ============================================================================
// A* PATHFINDING (HEURISTIC-BASED)
// ============================================================================
int findPathAStar(int start, int end, std::vector<int>& outPath) {
  const float INF = 1e9;
  int n = mazeState.junctions.size();
  
  std::vector<float> gScore(n, INF);  // Cost from start
  std::vector<float> fScore(n, INF);  // Estimated total cost
  std::vector<int> parent(n, -1);
  std::vector<bool> closed(n, false);
  
  gScore[start] = 0;
  fScore[start] = calculateMazeDistance(start, end);
  parent[start] = start;
  
  while (true) {
    int current = -1;
    float minF = INF;
    
    for (int i = 0; i < n; i++) {
      if (!closed[i] && fScore[i] < minF) {
        current = i;
        minF = fScore[i];
      }
    }
    
    if (current == -1 || current == end) {
      if (current == end) break;
      else return -1;
    }
    
    closed[current] = true;
    
    Junction& junc = mazeState.junctions[current];
    
    for (int dir = 0; dir < 4; dir++) {
      int neighbor = junc.neighbors[dir];
      if (neighbor >= 0 && !closed[neighbor]) {
        float tentativeG = gScore[current] + calculateMazeDistance(current, neighbor);
        
        if (tentativeG < gScore[neighbor]) {
          parent[neighbor] = current;
          gScore[neighbor] = tentativeG;
          fScore[neighbor] = gScore[neighbor] + calculateMazeDistance(neighbor, end);
        }
      }
    }
  }
  
  if (parent[end] == -1 && end != start) {
    Serial.println("[PATHFINDER] A*: No path found!");
    return -1;
  }
  
  // Reconstruct path
  outPath.clear();
  int current = end;
  while (current != start) {
    outPath.insert(outPath.begin(), current);
    current = parent[current];
  }
  outPath.insert(outPath.begin(), start);
  
  Serial.print("[PATHFINDER] A* path found: ");
  Serial.println(outPath.size());
  
  return outPath.size();
}

// ============================================================================
// OPTIMAL PATH SELECTION
// ============================================================================
int findOptimalPath(int start, int end, std::vector<int>& outPath, 
                    PathfindingStrategy strategy) {
  buildMazeGraph();
  
  switch (strategy) {
    case STRATEGY_BFS:
      return findPathBFS(start, end, outPath);
    case STRATEGY_DIJKSTRA:
      return findPathDijkstra(start, end, outPath);
    case STRATEGY_ASTAR:
      return findPathAStar(start, end, outPath);
    default:
      return findPathBFS(start, end, outPath);
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
int calculateMazeDistance(int junction1, int junction2) {
  if (junction1 < 0 || junction1 >= mazeState.junctions.size() ||
      junction2 < 0 || junction2 >= mazeState.junctions.size()) {
    return INT_MAX;
  }
  
  Junction& j1 = mazeState.junctions[junction1];
  Junction& j2 = mazeState.junctions[junction2];
  
  int dx = abs(j2.x - j1.x);
  int dy = abs(j2.y - j1.y);
  
  // Manhattan distance
  return dx + dy;
}

bool areMazeJunctionsAdjacent(int j1, int j2) {
  if (j1 < 0 || j1 >= mazeState.junctions.size() ||
      j2 < 0 || j2 >= mazeState.junctions.size()) {
    return false;
  }
  
  Junction& junc1 = mazeState.junctions[j1];
  Junction& junc2 = mazeState.junctions[j2];
  
  int dx = abs(junc2.x - junc1.x);
  int dy = abs(junc2.y - junc1.y);
  
  // Adjacent if only differ by 1 in one dimension
  return (dx + dy == 1);
}

void printPathfindingDebug() {
  Serial.println("\n===== PATHFINDER DEBUG =====");
  Serial.print("Total junctions: ");
  Serial.println(mazeState.junctions.size());
  
  for (int i = 0; i < mazeState.junctions.size(); i++) {
    Serial.print("Junction ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(mazeState.junctions[i].x);
    Serial.print(",");
    Serial.print(mazeState.junctions[i].y);
    Serial.print(") Neighbors: ");
    
    for (int dir = 0; dir < 4; dir++) {
      if (mazeState.junctions[i].neighbors[dir] >= 0) {
        Serial.print(mazeState.junctions[i].neighbors[dir]);
        Serial.print("(");
        Serial.print(dir);
        Serial.print(") ");
      }
    }
    Serial.println();
  }
  
  Serial.println("=============================\n");
}
