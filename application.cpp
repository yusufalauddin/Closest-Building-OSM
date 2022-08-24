// Yusuf Alauddin
//
// University of Illinois at Chicago
// CS 251: Fall 2021
// Project #7 - Openstreet Maps
//
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stack>
#include <string>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max();

class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
                  const pair<long long, double>& p2) const {
    if (p1.second == p2.second) {
      return p1.first > p2.first;
    }

    return p1.second > p2.second;
  }
};

//
//Creative Component of Assignment
//
void creative() {}

BuildingInfo searchBuilding(string query, vector<BuildingInfo>& Buildings) {
  BuildingInfo b;

  for (auto building : Buildings) {
    if (building.Abbrev == query) {
      return building;
    }
  }

  for (auto building : Buildings) {
    if (building.Fullname.find(query) != string::npos) {
      return building;
    }
  }

  return b;
}

BuildingInfo nearestBuilding(vector<BuildingInfo>& Buildings,
                             Coordinates midpoint, BuildingInfo b1,
                             BuildingInfo b2,
                             set<string>& unreachableBuildings) {
  BuildingInfo b;
  double min = INF;
  for (auto elem : Buildings) {
    if (unreachableBuildings.count(elem.Fullname) == 1) {
      continue;
    } else {
      double distance = distBetween2Points(midpoint.Lat, midpoint.Lon,
                                           elem.Coords.Lat, elem.Coords.Lon);
      if (distance < min) {
        min = distance;
        b = elem;
      }
    }
  }

  return b;
}

long long nearestNode(map<long long, Coordinates> Nodes, BuildingInfo b,
                      vector<FootwayInfo>& Footways) {
  long long N;
  double min = INF;

  for (auto elem : Footways) {
    for (size_t i = 0; i < elem.Nodes.size(); i++) {
      double long1 = Nodes[elem.Nodes[i]].Lon;
      double lat1 = Nodes[elem.Nodes[i]].Lat;

      double distance =
          distBetween2Points(lat1, long1, b.Coords.Lat, b.Coords.Lon);
      if (distance < min) {
        min = distance;
        N = Nodes[elem.Nodes[i]].ID;
      }
    }
  }
  return N;
}

map<long long, double> DijkstraShortestPath(
    graph<long long, double>& G, long long startV,
    map<long long, long long>& predecessors) {
  long long currV;
  double topW;

  map<long long, double> distances;
  priority_queue<pair<long long, double>, vector<pair<long long, double>>,
                 prioritize>
      unvisitedQueue;

  vector<long long> visited;

  vector<long long> nodes = G.getVertices();

  for (long long v : nodes) {
    distances[v] = INF;
    predecessors[v] = 0;
    unvisitedQueue.push(make_pair(v, INF));
  }

  distances[startV] = 0;
  unvisitedQueue.push(make_pair(startV, 0));

  while (unvisitedQueue.size() > 0) {
    currV = unvisitedQueue.top().first;
    topW = unvisitedQueue.top().second;

    unvisitedQueue.pop();

    if (distances[currV] == INF) {
      break;
    } else {
      bool discovered = false;

      for (long long node : visited) {
        if (currV == node) {
          discovered = true;
        }
      }

      if (!discovered) {
        visited.push_back(currV);
      } else {
        continue;
      }
    }

    set<long long> neighbors = G.neighbors(currV);
    for (long long neighbor : neighbors) {
      G.getWeight(currV, neighbor, topW);

      double alternativePathDistance = topW + distances[currV];

      if (alternativePathDistance < distances[neighbor]) {
        distances[neighbor] = alternativePathDistance;
        predecessors[neighbor] = currV;
        unvisitedQueue.push(make_pair(neighbor, alternativePathDistance));
      }
    }
  }

  return distances;
}

vector<long long> getPath(map<long long, long long> predecessors,
                          long long endVertex) {
  long long currV = endVertex;
  stack<long long> pathway;
  vector<long long> path;

  while (currV != 0) {
    pathway.push(currV);
    currV = predecessors[currV];
  }

  while (!pathway.empty()) {
    currV = pathway.top();
    pathway.pop();
    path.push_back(currV);
  }

  return path;
}

void printPath(vector<long long>& path) {
  cout << "Path: ";
  for (size_t i = 0; i < path.size() - 1; i++) {
    cout << path[i] << "->";
  }
  cout << path[path.size() - 1] << endl;
}

BuildingInfo locateCenterBuilding(BuildingInfo& b1, BuildingInfo& b2,
                                  vector<BuildingInfo>& Buildings,
                                  set<string>& unreachableBuildings) {
  Coordinates midpoint = centerBetween2Points(b1.Coords.Lat, b1.Coords.Lon,
                                              b2.Coords.Lat, b2.Coords.Lon);

  BuildingInfo mid =
      nearestBuilding(Buildings, midpoint, b1, b2, unreachableBuildings);

  return mid;
}

void printBuildings(BuildingInfo& b1, BuildingInfo& b2, BuildingInfo& mid) {
  cout << "Person 1's point:" << endl;
  cout << " " << b1.Fullname << endl;
  cout << " (" << b1.Coords.Lat << ", " << b1.Coords.Lon << ")" << endl;

  cout << "Person 2's point:" << endl;
  cout << " " << b2.Fullname << endl;
  cout << " (" << b2.Coords.Lat << ", " << b2.Coords.Lon << ")" << endl;

  cout << "Destination Building:" << endl;
  cout << " " << mid.Fullname << endl;
  cout << " (" << mid.Coords.Lat << ", " << mid.Coords.Lon << ")" << endl;
}

void printNearestNodes(long long b1Node, long long b2Node, long long midNode,
                       map<long long, Coordinates>& Nodes) {
  cout << "Nearest P1 node:" << endl;
  cout << " " << b1Node << endl;
  cout << " (" << Nodes[b1Node].Lat << ", " << Nodes[b1Node].Lon << ")" << endl;

  cout << "Nearest P2 node:" << endl;
  cout << " " << b2Node << endl;
  cout << " (" << Nodes[b2Node].Lat << ", " << Nodes[b2Node].Lon << ")" << endl;

  cout << "Nearest destination node:" << endl;
  cout << " " << midNode << endl;
  cout << " (" << Nodes[midNode].Lat << ", " << Nodes[midNode].Lon << ")"
       << endl;
}

void application(map<long long, Coordinates>& Nodes,
                 vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings,
                 graph<long long, double>& G) {
  string person1Building, person2Building;
  BuildingInfo b1;
  BuildingInfo b2;
  set<string> unreachableBuildings;

  cout << endl;
  while (person1Building != "#") {
    unreachableBuildings.clear();
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    if (person1Building == "#") {
      break;
    }

    b1 = searchBuilding(person1Building, Buildings);

    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    if (person2Building == "#") {
      break;
    }

    b2 = searchBuilding(person2Building, Buildings);

    if (b1.Fullname == "" && b1.Abbrev == "") {
      cout << "Person 1's building not found" << endl;
      cout << endl;
    } else if (b2.Fullname == "" && b2.Abbrev == "") {
      cout << "Person 2's building not found" << endl;
      cout << endl;
    } else {
      bool found = false;
      bool relocate = false;
      while (!found) {
        cout << endl;

        BuildingInfo mid =
            locateCenterBuilding(b1, b2, Buildings, unreachableBuildings);

        long long b1Node = nearestNode(Nodes, b1, Footways);
        long long b2Node = nearestNode(Nodes, b2, Footways);
        long long midNode = nearestNode(Nodes, mid, Footways);

        if (!relocate) {
          printBuildings(b1, b2, mid);
          cout << endl;
          printNearestNodes(b1Node, b2Node, midNode, Nodes);
          cout << endl;
        } else {
          cout << "New destination building:" << endl;
          cout << " " << mid.Fullname << endl;
          cout << " (" << mid.Coords.Lat << ", " << mid.Coords.Lon << ")"
               << endl;
          cout << "Nearest destination node:" << endl;
          cout << " " << midNode << endl;
          cout << " (" << Nodes[midNode].Lat << ", " << Nodes[midNode].Lon
               << ")" << endl;
          cout << endl;
        }

        map<long long, long long> b1Predecessors;
        map<long long, long long> b2Predecessors;

        map<long long, double> b1Distance =
            DijkstraShortestPath(G, b1Node, b1Predecessors);

        if (b1Distance[b2Node] >= INF) {
          cout << "Sorry, destination unreachable." << endl;
          break;
        } else {
          map<long long, double> b2Distance =
              DijkstraShortestPath(G, b2Node, b2Predecessors);

          if (b1Distance[midNode] >= INF || b2Distance[midNode] >= INF) {
            cout << "At least one person was unable to reach the destination "
                    "building. Finding next closest building..."
                 << endl;
            unreachableBuildings.insert(mid.Fullname);
            relocate = true;
          } else {
            vector<long long> path1 = getPath(b1Predecessors, midNode);

            vector<long long> path2 = getPath(b2Predecessors, midNode);

            cout << "Person 1's distance to dest: " << b1Distance[midNode]
                 << " miles" << endl;
            printPath(path1);
            cout << endl;
            cout << "Person 2's distance to dest: " << b2Distance[midNode]
                 << " miles" << endl;
            printPath(path2);
            found = true;
            cout << endl;
          }
        }
      }
    }
  }
}

int main() {
  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates> Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo> Footways;
  // info about each building, in no particular order
  vector<BuildingInfo> Buildings;
  XMLDocument xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  graph<long long, double> G;

  // add vertices
  for (auto elem : Nodes) {
    G.addVertex(elem.first);
  }

  // add edges

  for (auto elem : Footways) {
    for (size_t i = 0; i < elem.Nodes.size() - 1; i++) {
      double long1 = Nodes[elem.Nodes[i]].Lon;
      double lat1 = Nodes[elem.Nodes[i]].Lat;
      double long2 = Nodes[elem.Nodes[i + 1]].Lon;
      double lat2 = Nodes[elem.Nodes[i + 1]].Lat;

      double distance = distBetween2Points(lat1, long1, lat2, long2);

      G.addEdge(elem.Nodes[i], elem.Nodes[i + 1], distance);
      G.addEdge(elem.Nodes[i + 1], elem.Nodes[i], distance);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //
  // Menu
  //
  string userInput;
  cout << "Enter \"a\" for the standard application or "
       << "\"c\" for the creative component application> ";
  getline(cin, userInput);
  if (userInput == "a") {
    // TO DO: add argument for the graph you make.
    application(Nodes, Footways, Buildings, G);
  } else if (userInput == "c") {
    // TO DO: add arguments
    creative();
  }
  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
