
#include "Room.h"
#include "LevelConfig.h"
#include <algorithm>

CRoom::CRoom() {
  m_centerShift = v2i(0, 0);
  m_templateType = -1;
  m_flagFixed = false;
  m_boundaryType = 0;
  ResetEnergy();
}

CRoomEdge CRoom::GetEdge(size_t idx) const {
  size_t idx1 = idx;
  size_t idx2 = (idx + 1) % GetNumOfVertices();
  CRoomEdge edge;
  edge.SetPos1(GetVertex(idx1));
  edge.SetPos2(GetVertex(idx2));
  edge.SetIdx1(idx1);
  edge.SetIdx2(idx2);
  edge.SetDoorFlag(GetDoorFlag(idx));
  return edge;
}

v2i CRoom::GetRoomCenter() const {
  v2i center(0);
  if (GetNumOfVertices() == 0) {
    return center;
  }
  v2i posMin(std::numeric_limits<int>::max());
  v2i posMax(std::numeric_limits<int>::min());
  for (int i = 0; i < GetNumOfVertices(); i++) {
    v2i pi = m_vertices[i];
    for (int j = 0; j < 2; j++) {
      posMin[j] = std::min(posMin[j], pi[j]);
      posMax[j] = std::max(posMax[j], pi[j]);
    }
  }
  return floor_midpoint(posMin, posMax);
}

v2i CRoom::GetShiftedRoomCenter() {
  v2i center = GetRoomCenter() + m_centerShift;
  return center;
}

void CRoom::TranslateRoom(v2i trans) {
  for (int i = 0; i < GetNumOfVertices(); i++) {
    m_vertices[i] = m_vertices[i] + trans;
  }
}

void CRoom::RotateRoom(float rad) {
  float cv = cos(rad);
  float sv = sin(rad);
  for (int i = 0; i < GetNumOfVertices(); i++) {
    float p0 = m_vertices[i][0];
    float p1 = m_vertices[i][1];
    m_vertices[i][0] = p0 * cv + p1 * sv;
    m_vertices[i][1] = -p0 * sv + p1 * cv;
  }
  float p0 = m_centerShift[0];
  float p1 = m_centerShift[1];
  m_centerShift[0] = p0 * cv + p1 * sv;
  m_centerShift[1] = -p0 * sv + p1 * cv;
}

void CRoom::ScaleRoom(float scaling) {
  v2i center = GetRoomCenter();
  for (int i = 0; i < GetNumOfVertices(); i++) {
    v2i pi = m_vertices[i] - center;
    m_vertices[i] = center + pi * scaling;
  }
  m_centerShift = m_centerShift * scaling;
}

void CRoom::ScaleRoom(v2f scaling) {
  v2i center = GetRoomCenter();
  for (int i = 0; i < GetNumOfVertices(); i++) {
    v2i pi = m_vertices[i] - center;
    pi[0] *= scaling[0];
    pi[1] *= scaling[1];
    m_vertices[i] = center + pi;
  }
  m_centerShift[0] *= scaling[0];
  m_centerShift[1] *= scaling[1];
}

AABB2i CRoom::GetRoomBoundingBox() const {
  v2i pMin(std::numeric_limits<int>::max());
  v2i pMax(std::numeric_limits<int>::min());
  for (int i = 0; i < GetNumOfVertices(); i++) {
    v2i pi = m_vertices[i];
    for (int j = 0; j < 2; j++) {
      pMin[j] = std::min(pMin[j], pi[j]);
      pMax[j] = std::max(pMax[j], pi[j]);
    }
  }
  return AABB2i{pMin, pMax};
}

void CRoom::PrintRoom() const {
  std::cout << "A room with " << GetNumOfVertices() << " vertices...\n";
  for (int i = 0; i < GetNumOfVertices(); i++) {
    std::cout << "  " << i << "th vertex: " << GetVertex(i) << std::endl;
  }
}

void CRoom::InitWalls() {
  m_walls.clear();
  for (int i = 0; i < GetNumOfVertices(); i++) {
    int idx1 = i;
    int idx2 = (i + 1) % GetNumOfVertices();
    v2i pos1 = GetVertex(idx1);
    v2i pos2 = GetVertex(idx2);
    RoomWall wall(pos1, pos2);
    m_walls.push_back(wall);
  }
}

bool CRoom::EraseWall(int idx) {
  if (idx >= GetNumOfWalls()) {
    return false;
  }
  m_walls.erase(m_walls.begin() + idx);
  return true;
}

void CRoom::ResetDoorFlags() {
  if (!m_doorFlags.empty()) {
    return;
  }
  m_doorFlags.resize(GetNumOfEdges(), false);
}

void CRoom::SetDoorFlag(int edgeIdx, bool doorFlag) {
  if (edgeIdx < 0 || edgeIdx >= int(m_doorFlags.size())) {
    return;
  }
  m_doorFlags[edgeIdx] = doorFlag;
}

bool CRoom::GetDoorFlag(int edgeIdx) const {
  if (edgeIdx < 0 || edgeIdx >= int(m_doorFlags.size())) {
    return true;
  }
  return m_doorFlags[edgeIdx];
}

std::vector<bool> CRoom::GetDoorFlags() const { return m_doorFlags; }

bool CRoom::HasRestrictedDoorPosition() const {
  return std::any_of(m_doorFlags.begin(), m_doorFlags.end(),
                     [](bool b) { return !b; });
}
