//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-03-07
//  email:  chongyangm@gmail.com
//  info: class declaration of the level synthesis algorithm
// --------------------------------------------------------------

#ifndef LEVELSYNTH_H
#define LEVELSYNTH_H

#include "ConfigSpace.h"
#include "LevelConfig.h"
#include "PlanarGraph.h"
#include "RoomLayout.h"
#include "RoomTemplates.h"
#include "clipperWrapper.h"
#include <stack>
#include <string>

#include <random>

#ifdef __linux__
#include <stdarg.h>
#endif

//#define PRINT_OUT_DEBUG_INFO

// Use to track current solution state
class CurrentState {
public:
  CPlanarGraph m_stateGraph;
  std::vector<v2i> m_stateRoomPositions;
  std::vector<int> myIndices;

  float m_stateEnergy;

  void MoveRoomsToSceneCenter(CPlanarGraph *ptrGraph);

  void Move1DchainToSceneCenter(std::vector<int> &indices);

  float GetStateDifference(CurrentState &otherState, CPlanarGraph *ptrGraph);

  bool InsertToNewStates(std::vector<CurrentState> &newStates,
                         CPlanarGraph *ptrGraph);
};

class CLevelSynth {
public:
  CLevelSynth();

  CLevelSynth(CPlanarGraph *ptrGraph, CRoomTemplates *ptrTemplates);

  void SetGraphAndTemplates(CPlanarGraph *ptrGraph,
                            CRoomTemplates *ptrTemplates);

  void SetGraph(CPlanarGraph *ptrGraph);

  void InitScene();

  CRoomLayout GetLayout(CPlanarGraph *ptrGraph,
                        std::vector<v2i> &roomPositions);

  void SynthesizeScene();

  void UpdateGraphFromLayout();

  bool OpenDoors(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                 bool flagPartial = false);

  std::optional<v2i> OpenDoor(CRoom &room, RoomDoor &door) const;

  bool SaveGraphAsSVG(const char *fileName, CPlanarGraph *ptrGraph,
                      int wd = 800, int ht = 800, float labelRad = 0.25f);

  static bool CompareStateEnergySmallerFirst(const CurrentState &state1,
                                             const CurrentState &state2);

  int GetSolutionCount() { return m_solutionCount; }

  void ResetSolutionCount() { m_solutionCount = 0; }

  int GetIterationCount() { return m_iterationCount; }

  void ResetIterationCount() {
    m_iterationCount = 0;
    m_chainCount = 0;
  }

  inline std::string sprint(const char *fmt, ...) {
    size_t size = 512;
    char *buffer = nullptr;
    buffer = new char[size];
    va_list vl;
    va_start(vl, fmt);
    size_t nsize = vsnprintf(buffer, size, fmt, vl);
    if (size <= nsize) {
      // fail delete buffer and try again
      delete[] buffer;
      buffer = nullptr;
      buffer = new char[nsize + 1]; //+1 for /0
      nsize = vsnprintf(buffer, size, fmt, vl);
    }
    std::string ret(buffer);
    va_end(vl);
    delete[] buffer;
    return ret;
  }

  const CRoomLayout &getLayout(unsigned i) const { return m_layouts[i]; }

private:
  void SynthesizeSceneViaMainLoop();

  bool Solve1Dchain(std::vector<int> &indices, std::vector<int> *tmpIndices,
                    CurrentState &oldState,
                    std::vector<CurrentState> &newStates);

  bool Solve1DchainILS(std::vector<int> &indices, CurrentState &oldState,
                       std::vector<CurrentState> &newStates);

  void SetCurrentState(CurrentState &s);

  void SetSequenceAs1Dchain(const std::vector<int> &indices,
                            CPlanarGraph *ptrGraph);

  void SetVisitedNeighbors(const std::vector<int> &indices);

  void DumpSolutionIntoXML();

  int RandomlyPickOneRoom(CRoomLayout &layout);

  int RandomlyPickOneRoom(std::vector<int> &indices,
                          std::vector<int> *weightedIndices = nullptr);

  int RandomlyPickOneRoom(CRoomLayout &layout, std::vector<int> &indices,
                          std::vector<int> *weightedIndices);

  int RandomlyPickAnotherRoom(CRoomLayout &layout, int pickedIndex);

  std::vector<int> GetConnectedIndices(CPlanarGraph *ptrGraph, int pickedIndex,
                                       bool flagVisitedOnly = true);

  int RandomlyAdjustOneRoom(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                            std::vector<int> &indices,
                            std::vector<int> *weightedIndices);

  int RandomlyAdjustOneRoom03(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                              std::vector<int> &indices,
                              std::vector<int> *weightedIndices);

  void SampleConfigSpaceForPickedRoom(CRoomLayout &layout,
                                      CPlanarGraph *ptrGraph,
                                      std::vector<int> &indices,
                                      int pickedRoomIndex);

  int RandomlyAdjustOneRoom04(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                              std::vector<int> &indices,
                              std::vector<int> *weightedIndices);

  int GradientDescentOneRoom(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                             std::vector<int> &indices);

  float GetLayoutEnergy(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                        int &collideArea, int &connectivity,
                        int roomThatMoved = -1, bool doContact = false,
                        std::vector<int> *indicesForContact = nullptr);

  int CheckRoomConnectivity(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                            bool flagVisitedOnly = false,
                            int roomThatMoved = -1);

  bool isWithinBounds(const CRoomLayout &layout) const;

  int LayoutCollide(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                    bool flagVisitedOnly = false, int roomThatMoved = -1);

  int LayoutCollide(CRoomLayout &layout);

public:
  int RoomCollides(CRoom &room1, CRoom &room2);
private:

  bool TestBoundingBoxCollides(AABB2i &bb1, AABB2i &bb2);

  int LayoutContact(CRoomLayout &layout, CPlanarGraph *ptrGraph,
                    bool flagVisitedOnly = false, bool flagNonOverlap = false,
                    std::vector<int> *indices = nullptr, int roomThatMoved = -1);

  v2i ComputeLabelPosition(int idx, CPlanarGraph *ptrGraph, float labelRad);

  std::vector<int> m_sequence; // 1D chain of instantiated room templates

  CPlanarGraph *m_ptrGraph;
  CRoomTemplates *m_ptrTemplates;
  CRoomLayout m_layout;

  std::vector<CRoomLayout> m_layouts;

  int m_solutionCount;
  std::vector<v2i> m_roomPositions;
  std::vector<std::vector<int>> m_visitedNeighbors;
  int m_pickIndexCount;
  int m_bestSolCount;

  bool m_flagVisitedNoNode;

  int m_iterationCount;
  int m_chainCount;

  int m_backTrackCount;
  int m_backTrackLevel;

  std::mt19937 urng;
};

#endif // LEVELSYNTH_H
