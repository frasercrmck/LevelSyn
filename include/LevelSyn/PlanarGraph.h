//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-02-28
//  email:  chongyangm@gmail.com
//  info: class declaration of a planar graph
// --------------------------------------------------------------

#ifndef PLANARGRAPH_H
#define PLANARGRAPH_H

#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "tinyxml.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/ref.hpp>

#include "GraphChain.h"
#include "GraphEdge.h"
#include "GraphFace.h"
#include "GraphNode.h"
#include "vec.h"

class CPlanarGraph {
public:
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                boost::property<boost::vertex_index_t, int>,
                                boost::property<boost::edge_index_t, int>>
      Graph;

  typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;
  typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
  typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;
  typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIterator;

  CPlanarGraph();

  void ClearGraph();

  void PrintGraph();

  bool LoadGraphFromXML(const char *fileName, bool flagDetectFaces = true,
                        bool flagIgnoreIndiv = true);

  bool SaveGraphAsXML(const char *fileName);

  void AddGraphNode(CGraphNode &node) { m_nodes.push_back(node); }
  // void AddGraphEdge(CGraphEdge& edge) { m_edges.push_back(edge); }
  bool AddGraphEdge(const CGraphEdge &edge);

  bool CheckDuplicatedEdge(const CGraphEdge &edge);

  void AddGraphNodes(int numOfNodes, int parent = -1);

  void SetNodeNeighbors();

  void MovePickedNode(int dx, int dy);

  void MovePickedNode(v2i dp) { MovePickedNode(dp[0], dp[1]); }

  void UnpickNode() { m_pickedNodeIndex = -1; }

  int GetPickedNodeIndex() { return m_pickedNodeIndex; }

  size_t GetNumOfNodes() { return m_nodes.size(); }

  size_t GetNumOfEdges() { return m_edges.size(); }

  size_t GetNumOfFaces() { return m_faces.size(); }

  v2i GetNodePos(unsigned idx) { return m_nodes[idx].GetPos(); }

  void RandomInitGraph();

  void RandomInitPositions();

  void RandomInitTypes();

  void DetectFaces();

  void SortAdjacentVertices(const Graph &g, VertexDescriptor vert,
                            std::vector<EdgeDescriptor> &adjacentEdges);

  bool CompareEdgeDirections(const v2i &edgePr1, const v2i &edgePr2,
                             const v2i &edgeRef);

  void RemoveTheOutsideFace();

  CGraphNode &GetNode(unsigned idx) { return m_nodes[idx]; }
  CGraphEdge &GetEdge(unsigned idx) { return m_edges[idx]; }
  CGraphFace &GetFace(unsigned idx) { return m_faces[idx]; }
  CGraphChain &GetChain(unsigned idx) { return m_chains[idx]; }

  bool VisitedAllNodes();

  bool VisitedNoNode();

  bool HasFixedNode();

  std::vector<int> GetFixedNodes();

  std::vector<int> GetUnfixedNodes();

  // Step 3: Extract the 'deepest' face or chain not yet inserted (the most
  // included one)
  std::vector<int> ExtractDeepestFaceOrChain(bool &flagCyclic,
                                             bool flagSmallFaceFirst);

  std::vector<int> ExtractDeepestFaceOrChainOld(bool &flagCyclic,
                                                bool flagSmallFaceFirst);

  // Extract the 'deepest' face not yet inserted
  std::vector<int> ExtractDeepestFace(bool flagSmallFaceFirst);

  // Extract the 'deepest' chain not yet inserted
  std::vector<int> ExtractDeepestChainNew();

  std::vector<int> ExtractDeepestChain();

  int CountConstraints(std::vector<int> &indices);

  void SetNumOfTypes(int numOfTypes) { m_numOfTypes = numOfTypes; }

  void GetGraphBoundingBox(v2i &posMin, v2i &posMax);

  void MoveGraphToSceneCenter();

  void ScaleGraphNodePositions(float scaling);

  int FindNodeAccordingToName(const char *str);

  void RemoveIndividualNodes();

private:
  inline int Random2(int max) {
    if (max < 1 || max >= RAND_MAX)
      return 0;
    else
      return static_cast<int>(rand()) / (RAND_MAX / max + 1);
  }

  inline bool OneChanceIn(int a_million) { return (Random2(a_million) == 0); }

  inline bool CoinFlip() { return OneChanceIn(2); }

  std::vector<CGraphNode> m_nodes;
  std::vector<CGraphEdge> m_edges;
  int m_pickedNodeIndex;

  static CGraphFace m_faceTmp;
  static std::vector<CGraphFace> m_faces;

  std::vector<CGraphChain> m_chains;

  int m_numOfTypes; // number of node types

  // Some planar face traversal visitors that will
  // print the vertices and edges on the faces
  struct output_visitor : public boost::planar_face_traversal_visitor {
    void begin_face() {
      m_faceTmp.ClearIndices();
    }

    void end_face() {
      if (m_faceTmp.IsEmpty() == false) {
        m_faces.push_back(m_faceTmp);
      }
    }
  };

  struct vertex_output_visitor : public output_visitor {
    template <typename Vertex> void next_vertex(Vertex v) {
      m_faceTmp.AddIndex(int(v));
    }
  };

  struct edge_output_visitor : public output_visitor {
    template <typename Edge> void next_edge(Edge) {}
  };
};

#endif // PLANARGRAPH_H
