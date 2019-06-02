
#include "ConfigSpace.h"
#include "LevelConfig.h"

using namespace level_math;

v2i CConfigLine::RandomlySampleConfigLine() {
  float wt1 = rand() / float(RAND_MAX);
  float wt2 = 1.f - wt1;
  v2f pos = wt1 * v2i_to_v2f(m_pos1) + wt2 * v2i_to_v2f(m_pos2);
  v2i diff = m_pos2 - m_pos1;
  assert(diff[0] == 0 || diff[1] == 0);
  int space = diff[0] ? diff[0] : diff[1];
  if (diff[0] == 0) {
    pos = v2f(m_pos1[0], (m_pos1[1] + wt1 * space));
  } else {
    pos = v2f((m_pos1[0] + wt1 * space), m_pos1[1]);
  }
  return v2i(pos[0], pos[1]);
}

v2i CConfigLine::RandomlySampleConfigLineDiscrete() {
  float r = rand() / float(RAND_MAX);
  v2i pos = (r >= 0.5f) ? GetPos1() : GetPos2();
  return pos;
}

int CConfigLine::GetConfigLineLength() const { return mag(m_pos1 - m_pos2); }

int CConfigLine::GetConfigLineSqLength() const { return mag2(m_pos1 - m_pos2); }

void CConfigLine::PrintConfigLine() {
  std::cout << "p1: " << m_pos1 << "; p2: " << m_pos2 << std::endl;
}

void CConfigLine::TranslateConfigLine(v2i trans) {
  m_pos1 = m_pos1 + trans;
  m_pos2 = m_pos2 + trans;
}

std::vector<std::vector<CConfigSpace>> CConfigSpace::m_precomputedTable;
bool CConfigSpace::m_flagPrecomputed = false;

CConfigSpace::CConfigSpace(const CRoom &room1, const CRoom &room2) {
  int type1 = room1.GetTemplateType();
  int type2 = room2.GetTemplateType();
  if (m_flagPrecomputed == true && type1 >= 0 && type2 >= 0) {
    CConfigSpace cs = m_precomputedTable[type1][type2];
    cs.TranslateConfigSpace(room1.GetRoomCenter());
    *this = cs;
    return;
  }
  // cout << "Construct config space on the fly for room pair (" << type1 << ",
  // " << type2 << ")...\n";
  CClipperWrapper wrapper;
  const int contactThresh = CLevelConfig::m_roomContactThresh;
  for (size_t i = 0; i < room1.GetNumOfEdges(); i++) {
    CRoomEdge edge1 = room1.GetEdge(i);
    for (size_t j = 0; j < room2.GetNumOfEdges(); j++) {
      CRoomEdge edge2 = room2.GetEdge(j);
      if (edge1.GetDoorFlag() == false || edge2.GetDoorFlag() == false) {
        continue;
      }
      int cp = cross(edge1.GetDirection(), edge2.GetDirection());
      if (sqrt(cp) != 0) {
        continue;
      }
      std::vector<v2i> vecPr(4);
      vecPr[0] = edge1.GetPos1() - edge2.GetPos1();
      vecPr[1] = edge1.GetPos1() - edge2.GetPos2();
      vecPr[2] = edge1.GetPos2() - edge2.GetPos1();
      vecPr[3] = edge1.GetPos2() - edge2.GetPos2();

      v2i dir = edge1.GetDirection();
      dir = normalize(dir);
      v2i shift = dir * contactThresh;
      for (int k = 0; k < 4; k++) {
        vecPr.push_back(vecPr[k] + shift);
        vecPr.push_back(vecPr[k] - shift);
      }

      SortVecPr(vecPr);
      for (int k = 1; k < int(vecPr.size()); k++) {
        v2i pr1 = vecPr[k];
        v2i pr2 = vecPr[k - 1];
        if (mag2(pr2 - pr1) < 0) {
          continue;
        }
        v2i pr3 = floor_midpoint(pr1, pr2);
        CRoom room2n1 = room2;
        room2n1.TranslateRoom(pr1);
        if (wrapper.ComputeCollideArea(room1, room2n1) > 0) {
          continue;
        }
        int x = MaxRoomContact(room1, room2n1);
        if (MaxRoomContact(room1, room2n1) < contactThresh) {
          continue;
        }
        CRoom room2n2 = room2;
        room2n2.TranslateRoom(pr2);
        if (wrapper.ComputeCollideArea(room1, room2n2) > 0) {
          continue;
        }
        int y = MaxRoomContact(room1, room2n2);
        if (MaxRoomContact(room1, room2n2) < contactThresh) {
          continue;
        }
        CRoom room2n3 = room2;
        room2n3.TranslateRoom(pr3);
        if (wrapper.ComputeCollideArea(room1, room2n3) > 0) {
          continue;
        }
        int z = MaxRoomContact(room1, room2n3);
        if (MaxRoomContact(room1, room2n3) < contactThresh) {
          continue;
        }
        v2i pos1 = room2.GetRoomCenter() + pr1;
        v2i pos2 = room2.GetRoomCenter() + pr2;
        CConfigLine line(pos1, pos2);
        AddConfigLine(line);
      }
    }
  }
  SelfMerge();
#if 0
  room1.PrintRoom();
  room2.PrintRoom();
  PrintConfigSpace();
#endif
}

CConfigSpace::CConfigSpace(std::vector<CConfigLine> &vecConfigLines) {
  m_vecConfigLine = vecConfigLines;
  SelfMerge();
}

v2i CConfigSpace::RandomlySampleConfigSpace() {
  float r = rand() / float(RAND_MAX);
  v2i pos = (r >= 0.5f) ? RandomlySampleConfigSpaceContinuous()
                        : RandomlySampleConfigSpaceDiscrete();
  return pos;
}

v2i CConfigSpace::RandomlySampleConfigSpaceContinuous() {
  unsigned numOfLines = GetNumOfLines();
  int lineIndex = int(rand() / float(RAND_MAX) * numOfLines);
  lineIndex = lineIndex % numOfLines;
  v2i pos = m_vecConfigLine[lineIndex].RandomlySampleConfigLine();
  return pos;
}

v2i CConfigSpace::RandomlySampleConfigSpaceDiscrete() {
  unsigned numOfLines = GetNumOfLines();
  int lineIndex = int(rand() / float(RAND_MAX) * numOfLines);
  lineIndex = lineIndex % numOfLines;
  v2i pos = m_vecConfigLine[lineIndex].RandomlySampleConfigLineDiscrete();
  return pos;
}

std::vector<v2i> CConfigSpace::SmartlySampleConfigSpace() {
  unsigned numOfLines = GetNumOfLines();
  std::vector<v2i> vecPos(numOfLines);
  for (unsigned i = 0; i < numOfLines; i++) {
    CConfigLine &configLine = GetConfigLine(i);
    float r = rand() / float(RAND_MAX);
    v2i pos = (r >= 0.5f) ? configLine.RandomlySampleConfigLine()
                          : configLine.RandomlySampleConfigLineDiscrete();
    vecPos[i] = pos;
  }
  return vecPos;
}

CConfigSpace CConfigSpace::FindIntersection(CConfigSpace &configSpace1,
                                            CConfigSpace &configSpace2) {
  CConfigSpace intersectSpace;
  for (unsigned i1 = 0; i1 < configSpace1.GetNumOfLines(); i1++) {
    CConfigLine &configLine1 = configSpace1.GetConfigLine(i1);
    for (unsigned i2 = 0; i2 < configSpace2.GetNumOfLines(); i2++) {
      CConfigLine &configLine2 = configSpace2.GetConfigLine(i2);
      if (configLine1.GetConfigLineSqLength() == 0 &&
          configLine2.GetConfigLineSqLength() == 0) {
        if (mag2(configLine1.GetPos1() - configLine2.GetPos1()) == 0) {
          intersectSpace.AddConfigLine(configLine1);
        }
        continue;
      } else if (configLine1.GetConfigLineSqLength() == 0) {
        CRoomEdge edge(configLine2.GetPos1(), configLine2.GetPos2());
        if (PointToSegmentSqDistance(configLine1.GetPos1(), edge) == 0) {
          intersectSpace.AddConfigLine(configLine1);
        }
        continue;
      } else if (configLine2.GetConfigLineSqLength() == 0) {
        CRoomEdge edge(configLine1.GetPos1(), configLine1.GetPos2());
        if (PointToSegmentSqDistance(configLine2.GetPos1(), edge) == 0) {
          intersectSpace.AddConfigLine(configLine2);
        }
        continue;
      }
      v2i p11 = configLine1.GetPos1();
      v2i p12 = configLine1.GetPos2();
      v2i p21 = configLine2.GetPos1();
      v2i p22 = configLine2.GetPos2();
      int cp = cross(p12 - p11, p22 - p21);
      if (sqr(cp) != 0) {
        // Not parallel...
        if (std::optional<v2i> pi = SegmentIntersection(p11, p12, p21, p22)) {
          CConfigLine intersectLine(*pi);
          intersectSpace.AddConfigLine(intersectLine);
        }
      } else {
        // Parallel...
        v2i posMin1 = min_union(p11, p12);
        v2i posMax1 = max_union(p11, p12);
        v2i posMin2 = min_union(p21, p22);
        v2i posMax2 = max_union(p21, p22);
        bool flagOverlap = true;
        for (int j = 0; j < 2; j++) {
          if (posMax1[j] < posMin2[j] || posMin1[j] > posMax2[j]) {
            flagOverlap = false;
            break;
          }
        }
        if (flagOverlap == false) {
          continue;
        }
        int d1 = PointToLineSqDistance(p21, configLine1);
        int d2 = PointToLineSqDistance(p22, configLine1);
        if (d1 > 0 || d2 > 0) {
          flagOverlap = false;
        }
        if (flagOverlap == false) {
          continue;
        }
        v2i p1, p2;
        for (int d = 0; d < 2; d++) {
          p1[d] = std::max(std::min(p11[d], p12[d]), std::min(p21[d], p22[d]));
          p2[d] = std::min(std::max(p11[d], p12[d]), std::max(p21[d], p22[d]));
        }
        CConfigLine intersectLine(p1, p2);
        intersectSpace.AddConfigLine(intersectLine);
      }
    }
  }
  return intersectSpace;
}

CConfigSpace CConfigSpace::FindUnion(CConfigSpace &configSpace,
                                     CConfigLine &configLine) {
  CConfigSpace configSpaceNew = configSpace;
  if (configSpace.GetNumOfLines() == 0) {
    configSpaceNew.AddConfigLine(configLine);
    return configSpaceNew;
  }
  bool mergeFlag = false;
  for (unsigned i = 0; i < configSpace.GetNumOfLines(); i++) {
    CConfigLine &line = configSpaceNew.GetConfigLine(i);
    CRoomEdge edge1(line.GetPos1(), line.GetPos2());
    CRoomEdge edge2(configLine.GetPos1(), configLine.GetPos2());
    int sqlength1 = edge1.GetSqLength();
    int sqlength2 = edge2.GetSqLength();
    int cp = cross(edge1.GetDirection(), edge2.GetDirection());
    if (sqr(cp) != 0) {
      continue;
    }
    if (PointToSegmentSqDistance(edge1.GetPos1(), edge2) == 0 ||
        PointToSegmentSqDistance(edge1.GetPos2(), edge2) == 0) {
      v2i posMin1 = min_union(edge1.GetPos1(), edge1.GetPos2());
      v2i posMax1 = max_union(edge1.GetPos1(), edge1.GetPos2());
      v2i posMin2 = min_union(edge2.GetPos1(), edge2.GetPos2());
      v2i posMax2 = max_union(edge2.GetPos1(), edge2.GetPos2());
      v2i posMin = min_union(posMin1, posMin2);
      v2i posMax = max_union(posMax1, posMax2);
      v2i pos1, pos2;
      for (int j = 0; j < 2; j++) {
        pos1[j] = (line.GetPos1()[j] == posMin1[j]) ? posMin[j] : posMax[j];
        pos2[j] = (line.GetPos1()[j] == posMin1[j]) ? posMax[j] : posMin[j];
      }
      line.SetPos1(pos1);
      line.SetPos2(pos2);
      mergeFlag = true;
      break;
    }
  }
  if (mergeFlag == false) {
    configSpaceNew.AddConfigLine(configLine);
  }
  return configSpaceNew;
}

void CConfigSpace::SelfMerge() {
  sort(m_vecConfigLine.begin(), m_vecConfigLine.end(), CompareConfigLineLength);
  CConfigSpace configSpaceNew;
  for (unsigned i = 0; i < GetNumOfLines(); i++) {
    configSpaceNew = FindUnion(configSpaceNew, GetConfigLine(i));
  }
  SetConfigLines(configSpaceNew.GetConfigLines());
}

int CConfigSpace::GetConfigSpaceSize() {
  int sz = 0;
  for (unsigned i = 0; i < GetNumOfLines(); i++) {
    sz += GetConfigLine(i).GetConfigLineLength();
  }
  return sz;
}

void CConfigSpace::PrintConfigSpace() {
  for (unsigned i = 0; i < GetNumOfLines(); i++) {
    std::cout << "The " << i << "th line:\n";
    GetConfigLine(i).PrintConfigLine();
  }
}

void CConfigSpace::TranslateConfigSpace(v2i trans) {
  for (unsigned i = 0; i < GetNumOfLines(); i++) {
    GetConfigLine(i).TranslateConfigLine(trans);
  }
}

bool CConfigSpace::CompareConfigLineLength(const CConfigLine &line1,
                                           const CConfigLine &line2) {
  return (line1.GetConfigLineSqLength() > line2.GetConfigLineSqLength());
}

void CConfigSpace::PrecomputeTable(const std::vector<CRoom> &vecRooms) {
  m_flagPrecomputed = false;
  m_precomputedTable.clear();
  int numOfRooms = int(vecRooms.size());
  m_precomputedTable.resize(numOfRooms);
  // cout << "Pre-compute configuration space table for " << numOfRooms << "
  // rooms...\n";
  for (int i = 0; i < numOfRooms; i++) {
    std::vector<CConfigSpace> vecConfigSpace(numOfRooms);
    CRoom room1 = vecRooms[i];
    v2i centerPos = room1.GetRoomCenter();
    room1.TranslateRoom(-centerPos);
    for (int j = 0; j < numOfRooms; j++) {
      vecConfigSpace[j] = CConfigSpace(room1, vecRooms[j]);
    }
    m_precomputedTable[i] = vecConfigSpace;
  }
  m_flagPrecomputed = true;
}
