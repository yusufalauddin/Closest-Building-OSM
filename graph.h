// Yusuf Alauddin
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
// University of Illinois at Chicago
// CS 251: Fall 2021
// Project #7 - Openstreet Maps
//

#include <iostream>
#include <set>
#include <stdexcept>
#include <vector>
#include <map>

using namespace std;

template <typename VertexT, typename WeightT>
class graph {
 private:
  typedef pair<VertexT, WeightT> vwPair;
  map<VertexT, vector<vwPair>> adjList{};

  int numVertices;
  int numEdges;

 public:
  //
  // constructor:
  //
  // Constructs an empty graph where n is the max # of vertices
  // you expect the graph to contain.
  //
  // NOTE: the graph is implemented using an adjacency matrix.
  // If n exceeds the dimensions of this matrix, an exception
  // will be thrown to let you know that this implementation
  // will not suffice.
  //

  graph() {
    numVertices = 0;
    numEdges = 0;
  }

  //   copy constructor
  //   graph() {

  //   }

  //   copy operator
  //   graph() {

  //   }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const { return this->numVertices; }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const { return this->numEdges; }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    // check if vertex already exists
    if (adjList.count(v) == 1) {
      return false;
    }

    adjList[v] = {};
    numVertices++;

    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    // checks if vertices exist
    if (adjList.count(from) == 0 || adjList.count(to) == 0) {
      return false;
    }

    // checks if edge already exists for the specific vertex
    for (auto& elem : adjList[from]) {
      if (elem.first == to) {
        elem = make_pair(elem.first, weight);
        return true;
      }
    }

    vwPair p = make_pair(to, weight);
    adjList[from].push_back(p);

    numEdges++;

    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // check if vertices exist
    if (adjList.count(from) == 0 || adjList.count(to) == 0) {
      return false;
    }

    // checks if edge already exists for the specific vertex
    for (auto& elem : adjList.at(from)) {
      if (elem.first == to) {
        weight = elem.second;
        return true;
      }
    }

    return false;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;

    if (adjList.count(v) == 0) {
      return {};
    }

    for (auto elem : adjList.at(v)) {
      S.insert(elem.first);
    }

    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> v;

    for (auto elem : adjList) {
      v.push_back(elem.first);
    }

    return v;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->numVertices << endl;
    output << "**Num edges: " << this->numEdges << endl;

    output << endl;
    output << "**Vertices:" << endl;
    int i = 0;
    for (auto elem : adjList) {
      output << " " << i << ". " << elem.first << endl;
      i++;
    }

    output << endl;
    output << "**Edges:" << endl;

    vector<VertexT> V = getVertices();

    for (auto elem : adjList) {
      output << " row " << elem.first << ": ";
      set<VertexT> S = neighbors(elem.first);
      for (int col = 0; col < numVertices; ++col) {
        if (S.count(V[col]) == 0) {
          output << "F ";
        } else {
          int weight = 0;
          getWeight(elem.first, V[col], weight);
          output << "(T," << weight << ") ";
        }
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }
};
